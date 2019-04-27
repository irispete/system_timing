#include <iris_unit_test_framework/iris_unit_test_framework.h>
#include <iris_common/log/event_logger.h>

namespace system_timing
{
    iris_common::EventLogger event_logger{};
}

int main(int argc, char **argv)
{
    return IrisUnitTesting::runAllTests(argc, argv);
}
