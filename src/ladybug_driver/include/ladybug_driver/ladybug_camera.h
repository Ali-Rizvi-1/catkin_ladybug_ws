#ifndef LADYBUG_CAMERA_H
#define LADYBUG_CAMERA_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <ladybug.h>
#include <ladybuggeom.h>
#include <ladybugrenderer.h>
#include <string>
#include <vector>

namespace LadybugCamera {

class LadybugCameraNode {
public:
    LadybugCameraNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~LadybugCameraNode();

    bool initialize();
    void run();

    // Other members
    static volatile int running_;

private:
    // Camera initialization
    LadybugError initCamera();
    LadybugError initPanoramic();
    LadybugError startCamera();
    LadybugError configureTrigger();

    // Image processing
    void processPanoramic(const LadybugImage& image, const ros::Time& timestamp);
    void publishImage(const ros::Time& timestamp, const cv::Mat& image, ros::Publisher& pub, long int count, size_t cam_id);

    // ROS helpers
    void readParameters();
    void parseCameraInfo(const cv::Mat& camMat, const cv::Mat& disCoeff, const cv::Size& imgSize, sensor_msgs::CameraInfo& msg);

    // Camera context and settings
    LadybugContext m_context;
    LadybugDataFormat m_dataFormat;
    float m_frameRate, m_shutterTime, m_gainAmount;
    bool m_isFrameRateAuto, m_isShutterAuto, m_isGainAuto;
    int m_jpegQualityPercentage;

    // Trigger settings
    bool g_trigger_enabled;
    float g_trigger_delay;
    bool g_trigger_polarity;
    int g_trigger_timeout;

    // ROS publishers
    ros::Publisher pub_panoramic;
    ros::Publisher pub_panoramic_info;
    std::vector<ros::Publisher> individual_pubs;

    // ROS node handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
};

} // namespace LadybugCamera

#endif // LADYBUG_CAMERA_H