#include "ladybug_camera.h"
#include <signal.h>
#include <stdexcept>

namespace LadybugCamera {

volatile int LadybugCameraNode::running_ = 1;

LadybugCameraNode::LadybugCameraNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
    // Initialize default values
    m_dataFormat = LADYBUG_DATAFORMAT_RAW8;
    m_frameRate = 16.0f;
    m_shutterTime = 0.1f;
    m_gainAmount = 10.0f;
    m_isFrameRateAuto = true;
    m_isShutterAuto = true;
    m_isGainAuto = true;
    m_jpegQualityPercentage = 80;

    g_trigger_enabled = false;
    g_trigger_delay = 0.0f;
    g_trigger_polarity = true;
    g_trigger_timeout = 5000;
}

LadybugCameraNode::~LadybugCameraNode() {
    ladybugStop(m_context);
    ladybugDestroyContext(&m_context);
}

bool LadybugCameraNode::initialize() {
    // Read parameters from ROS parameter server
    readParameters();

    // Initialize camera
    if (initCamera() != LADYBUG_OK) {
        ROS_ERROR("Failed to initialize camera.");
        return false;
    }

    // Initialize panoramic stitching
    if (initPanoramic() != LADYBUG_OK) {
        ROS_ERROR("Failed to initialize panoramic stitching.");
        return false;
    }

    // Start camera
    if (startCamera() != LADYBUG_OK) {
        ROS_ERROR("Failed to start camera.");
        return false;
    }

    // Create ROS publishers
    pub_panoramic = nh_.advertise<sensor_msgs::Image>("/ladybug/panoramic/image_raw", 100);
    pub_panoramic_info = nh_.advertise<sensor_msgs::CameraInfo>("/ladybug/panoramic/camera_info", 100);

    if (private_nh_.param<bool>("publish_individual_cameras", false)) {
        for (int i = 0; i < LADYBUG_NUM_CAMERAS; i++) {
            std::string topic = "/ladybug/camera" + std::to_string(i) + "/image_raw";
            individual_pubs.push_back(nh_.advertise<sensor_msgs::Image>(topic, 100));
            ROS_INFO("Publishing individual camera image: %s", topic.c_str());
        }
    }

    return true;
}

void LadybugCameraNode::run() {
    ros::Rate loop_rate(m_frameRate);
    long int count = 0;

    while (running_ && ros::ok()) {
        LadybugImage currentImage;
        LadybugError error = ladybugLockNext(m_context, &currentImage);

        if (error != LADYBUG_OK) {
            ROS_WARN("Failed to acquire image: %s", ladybugErrorToString(error));
            continue;
        }

        ros::Time timestamp = ros::Time::now();

        // Process and publish panoramic image
        processPanoramic(currentImage, timestamp);

        // Unlock the image buffer
        ladybugUnlock(m_context, currentImage.uiBufferIndex);

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}

void LadybugCameraNode::readParameters() {
    private_nh_.param<float>("framerate", m_frameRate, m_frameRate);
    private_nh_.param<bool>("use_auto_framerate", m_isFrameRateAuto, m_isFrameRateAuto);
    private_nh_.param<float>("shutter_time", m_shutterTime, m_shutterTime);
    private_nh_.param<bool>("use_auto_shutter_time", m_isShutterAuto, m_isShutterAuto);
    private_nh_.param<float>("gain_amount", m_gainAmount, m_gainAmount);
    private_nh_.param<bool>("use_auto_gain", m_isGainAuto, m_isGainAuto);
    private_nh_.param<int>("jpeg_percent", m_jpegQualityPercentage, m_jpegQualityPercentage);

    private_nh_.param<bool>("trigger_enabled", g_trigger_enabled, g_trigger_enabled);
    private_nh_.param<float>("trigger_delay", g_trigger_delay, g_trigger_delay);
    private_nh_.param<bool>("trigger_polarity", g_trigger_polarity, g_trigger_polarity);
    private_nh_.param<int>("trigger_timeout", g_trigger_timeout, g_trigger_timeout);
}

// Other member function implementations...

} // namespace LadybugCamera