#include <algorithm>

#include "rosen_abstract_node/AbstractNodeSm.h"

namespace rosen_abstract_node
{
    AbstractNodeSm::AbstractNodeSm(std::function<bool()> doInit,
                                   std::function<bool()> doConnect,
                                   std::function<bool()> doDisconnect,
                                   std::function<bool(bool&)> doStart,
                                   std::function<bool()> doPause,
                                   std::function<bool()> doResume,
                                   std::function<bool()> doStop)
    : sm(NodeStateNo::STOPPED),
      doCallbacks{
        {NodeTransitionNo::INIT, doInit},
        {NodeTransitionNo::CONNECT, doConnect},
        {NodeTransitionNo::DISCONNECT, doDisconnect},
        {NodeTransitionNo::PAUSE, doPause},
        {NodeTransitionNo::RESUME, doResume},
        {NodeTransitionNo::STOP, doStop},
                   },
      doStart(doStart)
    {
        initSm();
    }

    void AbstractNodeSm::initSm()
    {
        sm.addTransition(NodeTransitionNo::INIT,
                         {NodeStateNo::STOPPED},
                         NodeStateNo::NODE_CONFIGURED,
                         doCallbacks.at(NodeTransitionNo::INIT));
        sm.addTransition(NodeTransitionNo::CONNECT,
                         {NodeStateNo::NODE_CONFIGURED, NodeStateNo::COMPONENT_DISCONNECTED},
                         NodeStateNo::COMPONENT_CONNECTED,
                         doCallbacks.at(NodeTransitionNo::CONNECT));
        sm.addTransition(NodeTransitionNo::DISCONNECT,
                         {NodeStateNo::COMPONENT_PAUSED, NodeStateNo::COMPONENT_RUNNING, NodeStateNo::COMPONENT_CONNECTED},
                         NodeStateNo::COMPONENT_DISCONNECTED,
                         doCallbacks.at(NodeTransitionNo::DISCONNECT));
        sm.addTransition(NodeTransitionNo::PAUSE,
                         {NodeStateNo::COMPONENT_RUNNING},
                         NodeStateNo::COMPONENT_PAUSED,
                         doCallbacks.at(NodeTransitionNo::PAUSE));
        sm.addTransition(NodeTransitionNo::RESUME,
                         {NodeStateNo::COMPONENT_PAUSED},
                         NodeStateNo::COMPONENT_RUNNING,
                         doCallbacks.at(NodeTransitionNo::RESUME));
        sm.addTransition(NodeTransitionNo::STOP,
                         {NodeStateNo::NODE_CONFIGURED, NodeStateNo::COMPONENT_DISCONNECTED},
                         NodeStateNo::STOPPED,
                         doCallbacks.at(NodeTransitionNo::STOP));
    }

    bool AbstractNodeSm::smStart()
    {
        if (sm.getCurrentState() == NodeStateNo::COMPONENT_CONNECTED)
        {
            bool running = false;
            if (doStart(running))
            {
                sm.setCurrentState(running ?    NodeStateNo::COMPONENT_RUNNING : NodeStateNo::COMPONENT_PAUSED);
                return true;
            }
        }
        return false;
    }

    bool AbstractNodeSm::isInNodeState(const NodeStateNo state) const
    {
        return sm.getCurrentState() == state;
    }

    bool AbstractNodeSm::doTransition(NodeTransitionNo nextTrans)
    {
        // The start transition is an edge case, because it can result in different states, running or paused,
        // which depends on running parameter. So the start transition cannot be added to the state machine
        // and has to be implemented outside of it.
        if (nextTrans == NodeTransitionNo::START)
        {
            return smStart();
        }
        else
        {
            return sm.doTransition(nextTrans);
        }
    }

    NodeStateNo AbstractNodeSm::getCurrentState() const
    {
        return sm.getCurrentState();
    }
}
