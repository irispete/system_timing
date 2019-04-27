#include <thread>
#include <chrono>

#include <system_timing/ros_node.h>
#include <boost/filesystem.hpp>
#include <iris_common/ros_inc.h>
#include <iris_common/Time.h>
#include <iris_common/version.h>
#include <iris_common/log/console_logging.h>

namespace system_timing
{
    RosNode::RosNode(const std::string &node_name)
            : node_name_(node_name)
    {
    }

    void RosNode::notifyWatchdog_(const ros::Publisher &publisher)
    {
        std_msgs::Header message;
        message.stamp = ros::Time::now();
        publisher.publish(message);
    }

    void RosNode::init(int argc, char* argv[])
    {
        LOG_INFO("Launching " + node_name_);
        ros::init(argc, argv, "system_timing");

        //Wait for flight folder param from Iris Common
        while (!node_handle_.hasParam("output_subfolder"))
        {
            ros::spinOnce();
        }
        const std::string output_subfolder = iris_common::fetchRosParam<std::string>(node_handle_,
                                                                                     "output_subfolder");
        const std::string terminal_log_level = iris_common::fetchRosParam<std::string>(node_handle_, node_name_ +
                                                                                                    "_terminal_log_level");
        const std::string event_log_level = iris_common::fetchRosParam<std::string>(node_handle_, node_name_ +
                                                                                                 "_event_log_level");
        spin_rate_hz_ = iris_common::fetchRosParam<double>(node_handle_, node_name_ + "_loop_rate");
        getParams();

        saveToVersionFile(output_subfolder, false);
        event_logger.initialize(node_name_, output_subfolder + "/events/", terminal_log_level, event_log_level);
        event_logger.info("Initializing " + node_name_);
    }

    void RosNode::spin()
    {
        ros::Publisher watchdog_publisher = node_handle_.advertise<std_msgs::Header>("heartbeat/" + node_name_, 1);
        ros::Rate rate(static_cast<double>(spin_rate_hz_));
        while (node_handle_.ok())
        {
            notifyWatchdog_(watchdog_publisher);
            ros::spinOnce();
            rate.sleep();
        }
    }
}

