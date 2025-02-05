#include "ladybug_camera.h"
#include <ros/ros.h>
#include <signal.h>

void signalHandler(int) {
    LadybugCamera::LadybugCameraNode::running_ = 0;
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ladybug_camera");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    signal(SIGTERM, signalHandler);

    LadybugCamera::LadybugCameraNode ladybug_node(nh, private_nh);

    if (!ladybug_node.initialize()) {
        ROS_ERROR("Failed to initialize Ladybug camera node.");
        return EXIT_FAILURE;
    }

    ladybug_node.run();
    return EXIT_SUCCESS;
}