#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>

class ImuClockPublisher
{
public:
    explicit ImuClockPublisher(ros::NodeHandle& nh)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("imu_topic", imu_topic_, "/airsim_node/drone_1/imu/imu");

        clock_pub_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
        imu_sub_ = nh.subscribe(imu_topic_, 100, &ImuClockPublisher::imuCallback, this);
        ROS_INFO("Publishing /clock from IMU topic: %s", imu_topic_.c_str());
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        const ros::Time stamp = msg->header.stamp;
        if (stamp.isZero()) {
            ROS_WARN_THROTTLE(1.0, "Ignore zero IMU stamp for /clock.");
            return;
        }

        if (!last_stamp_.isZero() && stamp < last_stamp_) {
            ROS_WARN_THROTTLE(1.0,
                              "Ignore backward IMU stamp for /clock: current=%.9f, last=%.9f",
                              stamp.toSec(), last_stamp_.toSec());
            return;
        }

        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = stamp;
        clock_pub_.publish(clock_msg);
        last_stamp_ = stamp;
    }

    std::string imu_topic_;
    ros::Publisher clock_pub_;
    ros::Subscriber imu_sub_;
    ros::Time last_stamp_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_clock_publisher");
    ros::NodeHandle nh;
    ImuClockPublisher publisher(nh);
    ros::spin();
    return 0;
}
