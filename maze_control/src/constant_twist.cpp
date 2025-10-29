#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "constant_twist");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped> ("maze/cmd_vel", 1);
    ros::Rate rate(1);
    geometry_msgs::TwistStamped msg;
    msg.header.frame_id = "table_top";
    msg.twist.linear.x = 0.2;
    msg.twist.linear.y = -0.5;
    while(ros::ok())
    {
        pub.publish(msg);
        rate.sleep();
    }
    return 0;
}