#ifndef ROSEN_ABSTRACT_NODE_DUMMY_NODE_H
#define ROSEN_ABSTRACT_NODE_DUMMY_NODE_H

#include "rosen_abstract_node/RosenAbstractNode.h"

#include <ros/ros.h>

namespace rosen_abstract_node
{
    class dummy_node : public rosen_abstract_node
    {
        public:
            dummy_node();
            virtual ~dummy_node() = default;

        private:
            ros::Publisher test_publisher;

            bool do_init() override;
            bool do_stop() override;
            bool do_connect() override;
            bool do_disconnect() override;
            bool do_pause() override;
            bool do_start(bool& running) override;
            bool do_resume() override;
            void do_step() override;
    };
}

#endif
