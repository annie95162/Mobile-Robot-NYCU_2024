#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cp2");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int16MultiArray>("control", 10);
    ros::Rate loop_rate(1); // 1Hz

    while (ros::ok())
    {
        std_msgs::Int16MultiArray msg;
        int temp;

        std::cout << "user's left is ";
        std::cin >> temp;
        msg.data.push_back(temp);

        std::cout << "user's right is ";
        std::cin >> temp;
        msg.data.push_back(temp);

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}