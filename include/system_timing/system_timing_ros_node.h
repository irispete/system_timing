//
// Created by irispete on 4/27/19.
//

#ifndef PROJECT_SYSTEM_TIMING_ROS_NODE_H
#define PROJECT_SYSTEM_TIMING_ROS_NODE_H

#include <system_timing/ros_node.h>

namespace system_timing
{
    class SystemTimingRosNode : public RosNode
    {
        public:
            SystemTimingRosNode()
                : RosNode("system_timing")
            {}

    };
}
#endif //PROJECT_SYSTEM_TIMING_ROS_NODE_H
