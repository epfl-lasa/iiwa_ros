#include <iiwa_driver/iiwa.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_hardware_interface");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    iiwa_ros::Iiwa iiwa(nh);

    spinner.stop();

    return 0;
}