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
    class mock_abstract_node : public rosen_abstract_node::rosen_abstract_node
    {
        public:
            mock_abstract_node() :
                rosen_abstract_node(std::string("node_name"), nullptr, nullptr, 42) {}
            virtual ~mock_abstract_node() = default;

            void do_loop_once()
            {
                do_transition(nullptr);
                do_step();
            }

            MOCK_METHOD0(do_init, bool());
            MOCK_METHOD0(do_stop, bool());
            MOCK_METHOD0(do_connect, bool());
            MOCK_METHOD0(do_disconnect, bool());
            MOCK_METHOD0(do_configure_component, void());
            MOCK_METHOD0(do_pause, bool());
            MOCK_METHOD0(do_resume, bool());
            MOCK_METHOD1(do_start, bool(bool& running));
            
            MOCK_METHOD0(do_step, void());
    };
}

TEST(abstract_node_test, flags_have_been_set)
{
    StrictMock<node_test::mock_abstract_node> node;

    ASSERT_EQ(node.get_flags(), 42);
}

TEST(abstract_node_test, do_functions_are_called_in_the_loop)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_step()).Times(1);

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
}
