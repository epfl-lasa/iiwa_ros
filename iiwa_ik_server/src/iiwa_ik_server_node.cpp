#include <ros/ros.h>

// Iiwa IK server headers
#include <iiwa_ik_server/iiwa_ik_server.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_ik_server");
    ros::NodeHandle nh;

    iiwa_ik_server::IiwaIKServer ik_server(nh);

    ros::spin();

    return 0;
}