#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "rosen_abstract_node/StateTransitionAction.h"
#include "rosen_abstract_node/NodeStateHelper.h"
#include "rosen_abstract_node/NodeTransitionHelper.h"
#include <signal.h>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <menu.h>


namespace rosen_abstract_node
{
    sighandler_t oldHandler = nullptr;

    void sighandler(const int param)
    {
        if(param == SIGINT)
        {
            if(oldHandler)
            {
                (*oldHandler)(SIGINT);
            }
            exit(0);
        }
    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                 const StateTransitionResultConstPtr& result)
    {
        ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
        ROS_INFO_STREAM("got result: "
                        << toString(NodeStateNo(result->new_state)));
    }

    // Called once when the goal becomes active
    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const StateTransitionFeedbackConstPtr& feedback)
    {
        ROS_INFO_STREAM("feedbackCb: current state: " << toString(NodeStateNo(feedback->current_state))
                        << " current transition: " << toString(NodeTransitionNo(feedback->transition)) );
    }

    void normalMode(const std::string& topic)
    {
        ROS_INFO("%s", topic.c_str());
        actionlib::SimpleActionClient<StateTransitionAction> ac(topic.c_str(), true);
        ROS_INFO("Waiting for action server to start.");

        ac.waitForServer(); // will wait for infinite time

        ROS_INFO("Action server started");

        StateTransitionGoal goal;

        bool exit = false;
        std::string input;
        oldHandler = signal(SIGINT, &sighandler);

        while (ros::ok() && !exit)
        {
            ros::spinOnce();
            for (int t = 0;
                    isValid(NodeTransitionNo(t));
                    ++t)
            {
                std::cout << toString(NodeTransitionNo(t)) << " : " << t << std::endl;
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

            if (isValid(NodeTransitionNo(trans)))
            {
                goal.transition = trans;
                std::cout << "Sending : " << toString(NodeTransitionNo(trans)) << std::endl << std::endl;
                ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
            }
            else
            {
                std::cout << "Please enter a valid transition." << std::endl;
            }
        }
    }


    using MenuPtr = std::unique_ptr<MENU, decltype(&free_menu)>;
    using ItemPtr = std::unique_ptr<ITEM, decltype(&free_item)>;


    class Menu
    {
        public:
            Menu(const std::vector<std::string>& nodeNames) :
            nodes(nodeNames),
            items(createItems(nodes)),
            managedItems(createManagedItems(items)),
            me(MenuPtr(new_menu(managedItems.data()), free_menu))
            {
            }

            ~Menu()
            {
                unpostMenu();
            }

            void postMenu()
            {
                ::post_menu(me.get());
            }

            void unpostMenu()
            {
                ::unpost_menu(me.get());
            }

            void menuDriver(const int task)
            {
                ::menu_driver(me.get(), task);
            }

            int getItemIndex()
            {
                return item_index(current_item(me.get()));
            }

            std::string select()
            {
                postMenu();
                refresh();

                int ch;

                while( (ch = getch()) != 'q')
                {
                    switch(ch)
                    {
                        case KEY_DOWN:
                            menuDriver(REQ_DOWN_ITEM);
                            break;
                        case KEY_UP:
                            menuDriver(REQ_UP_ITEM);
                            break;
                        case 0xA:    // RETURN key
                            if( static_cast<unsigned int>( getItemIndex()) == nodes.size())
                            {
                                exit(0);
                            }

                            int index = getItemIndex();
                            return nodes.at(index).append("/state_transition_action");
                    }
                }

                exit(0);
            }

        private:
            std::vector<std::string> nodes;
            std::vector<ItemPtr> items;
            // The lifetime of the raw pointers are managed by the the smart pointers from items.
            std::vector<ITEM*> managedItems;
            MenuPtr me;

            std::vector<ItemPtr> createItems(const std::vector<std::string>& nodes)
            {
                std::vector<ItemPtr> items;

                for(auto & node : nodes)
                {
                    items.push_back(ItemPtr(new_item(node.c_str(), ""), free_item));
                }
                items.push_back(ItemPtr(new_item("(q)uit", ""), free_item));
                items.push_back(ItemPtr(nullptr, free_item));

                return items;
            }

            std::vector<ITEM*> createManagedItems(const std::vector<ItemPtr>& items)
            {
                std::vector<ITEM*> managedItems;
                for(auto & item : items)
                {
                    managedItems.push_back(item.get());
                }
                return managedItems;
            }

    };


    class Window
    {
        public:
            Window()
            {
                initscr();
                oldHandler = signal(SIGINT, &sighandler);
                clear();
                noecho();
                curs_set(0);
                cbreak();
                nl();
                keypad(stdscr, TRUE);
                clear();
            }

            ~Window()
            {
                endwin();
                oldHandler = 0;
            }

            std::string selectFromMenu()
            {
                auto nodes = getNodes();
                rosen_abstract_node::Menu me(nodes);
                return me.select();
            }
        
        private:
            std::vector<std::string> nodes;

            std::vector<std::string> getNodes()
            {
                nodes.clear();
                ros::NodeHandle nh;
                ros::Subscriber diagSubscriber = nh.subscribe("/diagnostics", 100, &Window::diagCallback, this);
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

            void diagCallback(const ros::MessageEvent<const diagnostic_msgs::DiagnosticArray>& event)
            {
                const std::string& identifier = event.getPublisherName();
                if(std::find(nodes.begin(), nodes.end(), identifier) == nodes.end())
                {
                    if(!event.getConstMessage()->status.empty())
                    {
                        if(event.getConstMessage()->status[0].name.find(": Frequency Status") != std::string::npos)
                        {
                            nodes.push_back(identifier);
                            mvaddch(1, 25 + nodes.size(), '.');
                            refresh();
                        }
                    }
                }
            }
    };

    std::string selectNode(int argc, char **argv)
    {
        if(argc == 2 && strncmp(argv[1], "-a", 2) == 0)
        {
                Window win;
                return win.selectFromMenu();
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
    rosen_abstract_node::oldHandler = signal(SIGINT, &rosen_abstract_node::sighandler);

    auto target = rosen_abstract_node::selectNode(argc, argv);
    rosen_abstract_node::normalMode(target);
}
