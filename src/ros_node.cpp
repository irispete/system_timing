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

    void RosNode::publishHeartbeat_()
     {
        std_msgs::Header message;
        message.stamp = ros::Time::now();
        watchdog_publisher_.publish(message);
    }

    void RosNode::init(int argc, char* argv[])
    {
        LOG_INFO("Launching " + node_name_);
        ros::init(argc, argv, "system_timing");
        node_handle_ = std::make_shared<ros::NodeHandle>();

        //Wait for flight folder param from Iris Common
        while (!node_handle_->hasParam("output_subfolder"))
        {
            ros::spinOnce();
        }
        output_subfolder_ = iris_common::fetchRosParam<std::string>(*node_handle_, "output_subfolder");
        const std::string terminal_log_level = iris_common::fetchRosParam<std::string>(*node_handle_, node_name_ + "_terminal_log_level");
        const std::string event_log_level = iris_common::fetchRosParam<std::string>(*node_handle_, node_name_ + "_event_log_level");
        spin_rate_hz_ = iris_common::fetchRosParam<double>(*node_handle_, node_name_ + "_loop_rate");
        getParams();

        saveToVersionFile(output_subfolder_, false);
        event_logger_.initialize(node_name_, output_subfolder_ + "/events/", terminal_log_level, event_log_level);
        event_logger_.info("Initializing " + node_name_);
    }

    void RosNode::spin()
    {
        watchdog_publisher_ = node_handle_->advertise<std_msgs::Header>("heartbeat/" + node_name_, 1);
        ros::Rate rate(static_cast<double>(spin_rate_hz_));
        while (node_handle_->ok())
        {
            publishHeartbeat_();
            ros::spinOnce();
            rate.sleep();
        }
    }
}

