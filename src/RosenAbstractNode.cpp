#include "rosen_abstract_node/RosenAbstractNode.h"

#include <boost/bind.hpp>
#include <stdexcept>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>

#include "rosen_abstract_node/NodeState.h"

namespace rosen_abstract_node
{
    namespace
    {
        constexpr double DEFAULT_LOOP_FREQUENCY = 10.0;
        const ros::Duration CURRENT_STATE_FREQUENCY = ros::Duration(1.0);
    }

    RosenAbstractNode::RosenAbstractNode(const unsigned char flags)
    : RosenAbstractNode(ros::this_node::getName(), "~", flags)
    {}

    RosenAbstractNode::RosenAbstractNode(const std::string& nodeName, const std::string& nodeNamespace, const unsigned char flags)
    : RosenAbstractNode(nodeName, std::make_shared<ros::NodeHandle>(nodeNamespace), std::make_shared<diagnostic_updater::Updater>(), flags)
    {}

    RosenAbstractNode::RosenAbstractNode(const std::string& nodeName,
                                         const std::shared_ptr<ros::NodeHandle>& rosNodeHandlePrivate,
                                         const std::shared_ptr<diagnostic_updater::Updater>& rosDiagnosticUpdater,
                                         const unsigned char flags)
    : flags(flags),
      rosNodeName(nodeName),
      nextTrans(NodeTransitionNo::NONE),
      stateTransitionActionServer(nullptr),
      diagUpdater(rosDiagnosticUpdater),
      wrappedPublishers(),
      flagsPublisher(nullptr),
      loopFrequency(DEFAULT_LOOP_FREQUENCY),
      sm(std::bind(&RosenAbstractNode::doInit, this),
         std::bind(&RosenAbstractNode::doConnect, this),
         std::bind(&RosenAbstractNode::doDisconnect, this),
         std::bind(&RosenAbstractNode::doStart, this, std::placeholders::_1),
         std::bind(&RosenAbstractNode::doPause, this),
         std::bind(&RosenAbstractNode::doResume, this),
         std::bind(&RosenAbstractNode::doStop, this)),
      nodeHandlePrivate(rosNodeHandlePrivate)
    {
        if (rosNodeHandlePrivate != nullptr)
        {
            stateTransitionActionServer = std::make_shared<actionlib::SimpleActionServer<StateTransitionAction>>(
              *rosNodeHandlePrivate,
              "state_transition_action",
              [&](const StateTransitionGoalConstPtr& goal) { this->smActionCb(goal); },
              false);
            stateTransitionActionServer->start();

            flagsPublisher = std::make_shared<ros::Publisher>(nodeHandlePrivate->advertise<std_msgs::UInt8>("flags", 1, true));
            publishFlags();
        }
        if (diagUpdater != nullptr)
        {
            diagUpdater->setHardwareID(nodeName);
        }
    }
    
    void RosenAbstractNode::smActionCb(const StateTransitionGoalConstPtr& goal)
    {
        transitionProcessed = false;

        const NodeTransitionNo trans = NodeTransitionNo(goal->transition);
        if (isValid(trans) && trans != NodeTransitionNo::NONE)
        {
            ROS_INFO_STREAM("abstract_node::smActionCb transition: " << toString(trans));

            StateTransitionFeedback stateTransitionFeedback;
            stateTransitionFeedback.current_state = sm.getCurrentState();
            stateTransitionFeedback.transition = trans;
            stateTransitionActionServer->publishFeedback(stateTransitionFeedback);

            initiateTransition(trans);

            std::unique_lock<std::mutex> lock(transitionMutex);
            transitionConditionVariable.wait(lock, [&]{ return transitionProcessed; });
        }
        else
        {
            ROS_WARN("abstract_node::smActionCb received an invalid transition request: %i", trans);
            transitionSuccessful = false;
        }

        StateTransitionResult stateTransitionResult;
        stateTransitionResult.new_state = sm.getCurrentState();
        if (transitionSuccessful)
        {
            stateTransitionActionServer->setSucceeded(stateTransitionResult);
        }
        else
        {
            stateTransitionActionServer->setAborted(stateTransitionResult);
        }
    }

    void RosenAbstractNode::initiateTransition(NodeTransitionNo transition)
    {
        nextTrans = transition;
    }

    void RosenAbstractNode::initiateTransition(unsigned char transition)
    {
        initiateTransition(NodeTransitionNo(transition));
    }

    void RosenAbstractNode::doTransition(const std::unique_ptr<ros::Publisher>& currentStatePublisher)
    {
        if (nextTrans == NodeTransitionNo::NONE)
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(transitionMutex);
            transitionSuccessful = false;

            try
            {
                transitionSuccessful = sm.doTransition(nextTrans);
            }
            catch (const std::out_of_range&)
            {
                const auto message = "abstract_node::doTransition: Received invalid transition. Please check at caller!";
                ROS_FATAL(message);
                throw std::logic_error(message);
            }

            if (nullptr != currentStatePublisher)
            {
                NodeStateInfo nodeStateInfo;
                ros::Time now = ros::Time::now();
                RosenAbstractNode::updateAndPublishNodeStateInfo(*currentStatePublisher, now, nodeStateInfo);
            }
            transitionProcessed = true;
            nextTrans = NodeTransitionNo::NONE;
        }

        transitionConditionVariable.notify_one();
    }

    bool RosenAbstractNode::getTransitionSuccessful() const
    {
        return transitionSuccessful;
    }

    unsigned int RosenAbstractNode::getCurrentState() const
    {
        return sm.getCurrentState();
    }

    void RosenAbstractNode::updateAndPublishNodeStateInfo(const ros::Publisher& currentStatePublisher, const ros::Time now, NodeStateInfo& nodeStateInfo)
    {
        nodeStateInfo.current_state = sm.getCurrentState();
        nodeStateInfo.header.stamp = now;
        currentStatePublisher.publish(nodeStateInfo);
        ++nodeStateInfo.header.seq;
    }

    std::shared_ptr<diagnostic_updater::FrequencyStatus> RosenAbstractNode::createFrequencyStatus()
    {
        auto frequencyParam = diagnostic_updater::FrequencyStatusParam(&loopFrequency, &loopFrequency, 0.1, 10);
        auto frequencyStatus = std::make_shared<diagnostic_updater::FrequencyStatus>(frequencyParam);
        diagUpdater->add(*frequencyStatus);
        return frequencyStatus;
    }

    void RosenAbstractNode::loop()
    {
        loopFrequency = nodeHandlePrivate->param("loop_frequency", DEFAULT_LOOP_FREQUENCY);
        ROS_INFO_STREAM(rosNodeName << " is running with a frequency of " << loopFrequency << " Hz (ROS Time).");

        ros::Time now = ros::Time::now();
        ros::Rate loopRate(loopFrequency);
        auto frequencyStatus = createFrequencyStatus();

        NodeStateInfo nodeStateInfo;
        nodeStateInfo.header.seq = 0;
        std::unique_ptr<ros::Publisher> currentStatePublisher = std::make_unique<ros::Publisher>(nodeHandlePrivate->advertise<NodeStateInfo>("current_state", 10, false));

        ros::Time nextCurrentStateMessage = now;

        while (ros::ok())
        {
            now = ros::Time::now();
            ros::spinOnce();
            doTransition(currentStatePublisher);

            if (now >= nextCurrentStateMessage)
            {
                updateAndPublishNodeStateInfo(*currentStatePublisher, now, nodeStateInfo);
                nextCurrentStateMessage = now + CURRENT_STATE_FREQUENCY;
            }

            doStep();
            frequencyStatus->tick();
            diagUpdater->update();
            loopRate.sleep();
        }
    }

    std::shared_ptr<diagnostic_updater::Updater> RosenAbstractNode::getDiagnosticUpdater()
    {
        return diagUpdater;
    }

    double RosenAbstractNode::getLoopFrequency()
    {
        return loopFrequency;
    }

    unsigned char RosenAbstractNode::getFlags() const
    {
        return flags;
    }

    void RosenAbstractNode::publishFlags() const
    {
        std_msgs::UInt8 flagsMsg;
        flagsMsg.data = flags;
        flagsPublisher->publish(flagsMsg);
    }
}
