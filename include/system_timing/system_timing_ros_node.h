//
// Created by irispete on 4/27/19.
//

#ifndef PROJECT_SYSTEM_TIMING_ROS_NODE_H
#define PROJECT_SYSTEM_TIMING_ROS_NODE_H

#include <system_timing/ros_node.h>
#include <system_timing/event_time_manager.h>
#include <iris_common/log/event_logger.h>

namespace system_timing
{
    class SystemTimingRosNode : public RosNode
    {
        public:
            SystemTimingRosNode()
                : RosNode("system_timing"), event_time_manager_(event_logger_)
            {}

            void init(int argc, char* argv[]) override;

        private:
            EventTimeManager event_time_manager_;
            ros::Subscriber event_time_subscriber;    };
}
#endif //PROJECT_SYSTEM_TIMING_ROS_NODE_H
