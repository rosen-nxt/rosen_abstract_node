#include "rosen_abstract_node/DummyNode.h"

#include <std_msgs/Int16.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_node");
    rosen_abstract_node::DummyNode node;
    node.loop();
    return EXIT_SUCCESS;
}

namespace rosen_abstract_node
{

    DummyNode::DummyNode()
     : RosenAbstractNode(),
       testPublisher(nodeHandlePrivate->advertise<std_msgs::Int16>("published_in_loop", 10, false))
    {
    }

    bool DummyNode::doInit()
    {
        return nodeHandlePrivate->param("dummy_node_do_init_succeeds", true);
    }

    bool DummyNode::doStop()
    {
        return nodeHandlePrivate->param("dummy_node_do_stop_succeeds", true);
    }

    bool DummyNode::doConnect()
    {
        return nodeHandlePrivate->param("dummy_node_do_connect_succeeds", true);
    }

    bool DummyNode::doDisconnect()
    {
        return nodeHandlePrivate->param("dummy_node_do_disconnect_succeeds", true);
    }

    bool DummyNode::doPause()
    {
        return nodeHandlePrivate->param("dummy_node_do_pause_succeeds", true);
    }

    bool DummyNode::doStart(bool& running)
    {        
        running = nodeHandlePrivate->param("dummy_node_running", true);
        return nodeHandlePrivate->param("dummy_node_do_start_succeeds", true);
    }

    bool DummyNode::doResume()
    {
        return nodeHandlePrivate->param("dummy_node_do_resume_succeeds", true);
    }

    void DummyNode::doStep()
    {
        std_msgs::Int16 msg;
        msg.data = 42;
        testPublisher.publish(msg);
    }

}
