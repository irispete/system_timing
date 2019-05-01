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

            virtual void init(int argc, char* argv[]);

            virtual void getParams()
            {}

            void spin();

        protected:
            void publishHeartbeat_();

            double spin_rate_hz_ = 15;
            std::shared_ptr<ros::NodeHandle> node_handle_;
            ros::Publisher watchdog_publisher_;
            std::string node_name_;
            std::string output_subfolder_;
            iris_common::EventLogger event_logger_{};
    };
}

#endif //PROJECT_ROS_NODE_H
