#ifndef ROSEN_ABSTRACT_NODE_DUMMY_NODE_H
#define ROSEN_ABSTRACT_NODE_DUMMY_NODE_H

#include "rosen_abstract_node/RosenAbstractNode.h"

#include <ros/ros.h>

namespace rosen_abstract_node
{
    class DummyNode : public RosenAbstractNode
    {
        public:
            DummyNode();
            virtual ~DummyNode() = default;

        private:
            ros::Publisher testPublisher;

            bool doInit() override;
            bool doStop() override;
            bool doConnect() override;
            bool doDisconnect() override;
            bool doPause() override;
            bool doStart(bool& running) override;
            bool doResume() override;
            void doStep() override;
    };
}

#endif
