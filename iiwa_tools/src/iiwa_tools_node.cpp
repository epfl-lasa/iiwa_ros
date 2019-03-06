#include <ros/ros.h>

// Iiwa IK server headers
#include <iiwa_tools/iiwa_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_tools");
    ros::NodeHandle nh;

    iiwa_tools::IiwaTools tools_server(nh);

    ros::spin();

    return 0;
}