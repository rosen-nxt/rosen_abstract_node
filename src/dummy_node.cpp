#include "rosen_abstract_node/dummy_node.h"

#include <std_msgs/Int16.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_node");
    rosen_abstract_node::dummy_node node;
    node.loop();
    return EXIT_SUCCESS;
}

namespace rosen_abstract_node
{

    dummy_node::dummy_node()
     : rosen_abstract_node(),
       test_publisher(node_handle_private->advertise<std_msgs::Int16>("published_in_loop", 10, false))
    {
    }

    bool dummy_node::do_init()
    {
        return node_handle_private->param("dummy_node_do_init_succeeds", true);
    }

    bool dummy_node::do_stop()
    {
        return node_handle_private->param("dummy_node_do_stop_succeeds", true);
    }

    bool dummy_node::do_connect()
    {
        return node_handle_private->param("dummy_node_do_connect_succeeds", true);
    }

    bool dummy_node::do_disconnect()
    {
        return node_handle_private->param("dummy_node_do_disconnect_succeeds", true);
    }

    bool dummy_node::do_pause()
    {
        return node_handle_private->param("dummy_node_do_pause_succeeds", true);
    }

    bool dummy_node::do_start(bool& running)
    {        
        running = node_handle_private->param("dummy_node_running", true);
        return node_handle_private->param("dummy_node_do_start_succeeds", true);
    }

    bool dummy_node::do_resume()
    {
        return node_handle_private->param("dummy_node_do_resume_succeeds", true);
    }

    void dummy_node::do_step()
    {
        std_msgs::Int16 msg;
        msg.data = 42;
        test_publisher.publish(msg);
    }

}
