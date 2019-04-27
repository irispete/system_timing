//
// Created by irispete on 4/27/19.
//

#ifndef PROJECT_ROS_NODE_H
#define PROJECT_ROS_NODE_H

#include <iris_common/ros.h>
#include <iris_common/log/event_logger.h>

namespace system_timing
{
    class RosNode
    {
        public:
            RosNode(const std::string &node_name);

            void init(int argc, char* argv[]);

            virtual void getParams()
            {}

            void spin();

        protected:
            void notifyWatchdog_(const ros::Publisher &publisher);

            double spin_rate_hz_ = 15;
            ros::NodeHandle node_handle_;
            std::string node_name_;
            iris_common::EventLogger event_logger{};
    };
}

#endif //PROJECT_ROS_NODE_H
