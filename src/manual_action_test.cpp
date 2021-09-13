#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "rosen_abstract_node/StateTransitionAction.h"
#include "rosen_abstract_node/node_state_helper.h"
#include "rosen_abstract_node/node_transition_helper.h"
#include <signal.h>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <menu.h>


namespace rosen_abstract_node
{
    sighandler_t old_handler = nullptr;

    void sighandler(const int param)
    {
        if(param == SIGINT)
        {
            if(old_handler)
            {
                (*old_handler)(SIGINT);
            }
            exit(0);
        }
    }

    void done_cb(const actionlib::SimpleClientGoalState& state,
                 const StateTransitionResultConstPtr& result)
    {
        ROS_INFO("done_cb: server responded with state [%s]", state.toString().c_str());
        ROS_INFO_STREAM("got result: "
                        << node_state_helper::to_string(result->new_state));
    }

    // Called once when the goal becomes active
    void active_cb()
    {
        ROS_INFO("Goal just went active");
    }

    // Called every time feedback is received for the goal
    void feedback_cb(const StateTransitionFeedbackConstPtr& feedback)
    {
        ROS_INFO_STREAM("feedback_cb: current state: " << node_state_helper::to_string(feedback->current_state)
                        << " current transition: " << node_transition_helper::to_string(feedback->transition) );
    }

    void normal_mode(const std::string& topic)
    {
        ROS_INFO("%s", topic.c_str());
        actionlib::SimpleActionClient<StateTransitionAction> ac(topic.c_str(), true);
        ROS_INFO("Waiting for action server to start.");

        ac.waitForServer(); // will wait for infinite time

        ROS_INFO("Action server started");

        StateTransitionGoal goal;

        bool exit = false;
        std::string input;
        old_handler = signal(SIGINT, &sighandler);

        while (ros::ok() && !exit)
        {
            ros::spinOnce();
            for (int t = 0;
                    node_transition_helper::is_valid(t);
                    ++t)
            {
                std::cout << node_transition_helper::to_string(t) << " : " << t << std::endl;
            }
            std::cout << "EXIT : -1 or [CTRL+D] or [CTRL+C]" << std::endl;


            int trans = -1;

            std::cin >> input;
            if(std::cin.fail())
            {
                exit = true;
                continue;
            }

            try
            {
                trans = std::stoi(input);
            }
            catch(const std::invalid_argument&) // no integer
            {
                continue;
            }
            catch(const std::out_of_range&)
            {
                continue;
            }

            if (trans < 0)
            {
                exit = true;
                continue;
            }

            if (node_transition_helper::is_valid(trans))
            {
                goal.transition = trans;
                std::cout << "Sending : " << node_transition_helper::to_string(trans) << std::endl << std::endl;
                ac.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
            }
            else
            {
                std::cout << "Please enter a valid transition." << std::endl;
            }
        }
    }


    using menu_ptr = std::unique_ptr<MENU, decltype(&free_menu)>;
    using item_ptr = std::unique_ptr<ITEM, decltype(&free_item)>;


    class menu
    {
        public:
            menu(const std::vector<std::string>& node_names) :
            nodes(node_names),
            items(create_items(nodes)),
            managed_items(create_managed_items(items)),
            me(menu_ptr(new_menu(managed_items.data()), free_menu))
            {
            }

            ~menu()
            {
                unpost_menu();
            }

            void post_menu()
            {
                ::post_menu(me.get());
            }

            void unpost_menu()
            {
                ::unpost_menu(me.get());
            }

            void menu_driver(const int task)
            {
                ::menu_driver(me.get(), task);
            }

            int get_item_index()
            {
                return item_index(current_item(me.get()));
            }

            std::string select()
            {
                post_menu();
                refresh();

                int ch;

                while( (ch = getch()) != 'q')
                {
                    switch(ch)
                    {
                        case KEY_DOWN:
                            menu_driver(REQ_DOWN_ITEM);
                            break;
                        case KEY_UP:
                            menu_driver(REQ_UP_ITEM);
                            break;
                        case 0xA:    // RETURN key
                            if( static_cast<unsigned int>( get_item_index()) == nodes.size())
                            {
                                exit(0);
                            }

                            int index = get_item_index();
                            return nodes.at(index).append("/state_transition_action");
                    }
                }

                exit(0);
            }

        private:
            std::vector<std::string> nodes;
            std::vector<item_ptr> items;
            // The lifetime of the raw pointers are managed by the the smart pointers from items.
            std::vector<ITEM*> managed_items;
            menu_ptr me;

            std::vector<item_ptr> create_items(const std::vector<std::string>& nodes)
            {
                std::vector<item_ptr> items;

                for(auto & node : nodes)
                {
                    items.push_back(item_ptr(new_item(node.c_str(), ""), free_item));
                }
                items.push_back(item_ptr(new_item("(q)uit", ""), free_item));
                items.push_back(item_ptr(nullptr, free_item));

                return items;
            }

            std::vector<ITEM*> create_managed_items(const std::vector<item_ptr>& items)
            {
                std::vector<ITEM*> managed_items;
                for(auto & item : items)
                {
                    managed_items.push_back(item.get());
                }
                return managed_items;
            }

    };


    class window
    {
        public:
            window()
            {
                initscr();
                old_handler = signal(SIGINT, &sighandler);
                clear();
                noecho();
                curs_set(0);
                cbreak();
                nl();
                keypad(stdscr, TRUE);
                clear();
            }

            ~window()
            {
                endwin();
                old_handler = 0;
            }

            std::string select_from_menu()
            {
                auto nodes = get_nodes();
                rosen_abstract_node::menu me(nodes);
                return me.select();
            }
        
        private:
            std::vector<std::string> nodes;

            std::vector<std::string> get_nodes()
            {
                nodes.clear();
                ros::NodeHandle nh;
                ros::Subscriber diag_subscriber = nh.subscribe("/diagnostics", 100, &window::diag_callback, this);
                const int steps = 20;
                std::cout << std::endl;
                mvaddstr(1, 1, "Searching nodes");
                for(int i = 0; i != steps; ++i)
                {
                    char c='X';
                    switch (i % 4)
                    {
                        case 0:
                            c = '|';
                            break;
                        case 1:
                            c = '/';
                            break;
                        case 2:
                            c = '-';
                            break;
                        case 3:
                            c = '\\';
                            break;
                    }
                    mvaddch(1, 20, c);
                    refresh();
                    ros::spinOnce();
                    usleep(100000);
                }
                
                std::sort(nodes.begin(), nodes.end());
                return nodes;
            }

            void diag_callback(const ros::MessageEvent<const diagnostic_msgs::DiagnosticArray>& event)
            {
                const std::string& identifier = event.getPublisherName();
                if(std::find(nodes.begin(), nodes.end(), identifier) == nodes.end())
                {
                    if(event.getConstMessage()->status[0].name.find(": Frequency Status") != std::string::npos)
                    {
                        nodes.push_back(identifier);
                        mvaddch(1, 25 + nodes.size(), '.');
                        refresh();
                    }
                }
            }
    };

    std::string select_node(int argc, char **argv)
    {
        if(argc == 2 && strncmp(argv[1], "-a", 2) == 0)
        {
                window win;
                return win.select_from_menu();
        }

        std::string target;
        if (argc == 2)
        {
            target = argv[1];
            target += "/state_transition_action";
        }
        else
        {
            target = "/dummy_node/state_transition_action";
        }

        return target;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "actionTest", ros::InitOption::AnonymousName);
    rosen_abstract_node::old_handler = signal(SIGINT, &rosen_abstract_node::sighandler);

    auto target = rosen_abstract_node::select_node(argc, argv);
    rosen_abstract_node::normal_mode(target);
}
