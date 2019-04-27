#include <thread>
#include <chrono>

#include <system_timing/system_timing_ros_node.h>
#include <iris_common/log/console_logging.h>

namespace system_timing
{

    int main(int argc, char* argv[])
    {
        LOG_INFO("Launching system_timing");
        SystemTimingRosNode node{};
        node.init(argc, argv);
        node.spin();
        return 0;
    }
}

int main( int argc, char** args) { return system_timing::main(argc, args); }
