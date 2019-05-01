#include <system_timing/system_timing_ros_node.h>
#include <system_timing/event_time_manager.h>

namespace system_timing
{
    void SystemTimingRosNode::init(int argc, char *argv[])
    {
        RosNode::init(argc, argv);
    }
}