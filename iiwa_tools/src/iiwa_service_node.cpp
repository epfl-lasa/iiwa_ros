#include <ros/ros.h>

// Iiwa IK server headers
#include <iiwa_tools/iiwa_service.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_service");
    ros::NodeHandle nh;

    iiwa_tools::IiwaService tools_server(nh);

    ros::spin();

    return 0;
}