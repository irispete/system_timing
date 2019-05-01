#include <system_timing/system_timing_ros_node.h>
#include <system_timing/event_time_manager.h>

namespace system_timing
{
    void SystemTimingRosNode::init(int argc, char *argv[])
    {
        RosNode::init(argc, argv);
        event_time_subscriber = node_handle_.subscribe<const iris_common::EventTime &>(
                "internal/event_time",            // ros topic name
                32,                               // cache size
                &EventTimeManager::addEvent,      // callback function
                &event_time_manager_              // member object
        );
    }
}