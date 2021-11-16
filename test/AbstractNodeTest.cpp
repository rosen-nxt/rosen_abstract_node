#include "rosen_abstract_node/NodeTransition.h"
#include "rosen_abstract_node/NodeState.h"
#include "rosen_abstract_node/RosenAbstractNode.h"
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace rosen_abstract_node;
using namespace testing;

namespace node_test
{
    class MockAbstractNode : public rosen_abstract_node::RosenAbstractNode
    {
        public:
            MockAbstractNode() :
                RosenAbstractNode(std::string("node_name"), nullptr, nullptr, 42) {}
            virtual ~MockAbstractNode() = default;

            void doLoopOnce()
            {
                doTransition(nullptr);
                doStep();
            }

            MOCK_METHOD0(doInit, bool());
            MOCK_METHOD0(doStop, bool());
            MOCK_METHOD0(doConnect, bool());
            MOCK_METHOD0(doDisconnect, bool());
            MOCK_METHOD0(doConfigureComponent, void());
            MOCK_METHOD0(doPause, bool());
            MOCK_METHOD0(doResume, bool());
            MOCK_METHOD1(doStart, bool(bool& running));
            
            MOCK_METHOD0(doStep, void());
    };
}

TEST(AbstractNodeTest, flagsHaveBeenSet)
{
    StrictMock<node_test::MockAbstractNode> node;

    ASSERT_EQ(node.getFlags(), 42);
}

TEST(AbstractNodeTest, doFunctionsAreCalledInTheLoop)
{
    StrictMock<node_test::MockAbstractNode> node;

    EXPECT_CALL(node, doInit()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, doStep()).Times(1);

    node.initiateTransition(NodeTransition::INIT);
    node.doLoopOnce();
}
