#pragma once

//STD
#include <chrono>
#include <functional>

//ROS2
#include <rclcpp/rclcpp.hpp>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//ArenaSDK
#include "ArenaApi.h"

class CameraCalibration : public rclcpp::Node
{
public:
    CameraCalibration()
    : Node("camera_calibration")
    {   
        parse_parameters_();
        calibrate_();
    }

private:

    //Initialize ROS2 calibration parameters
    std::string calibration_type_,
                camera_name_,
                image_path_,
                extension_,
                camera_output_file_;
    std::vector<double> pattern_size_;
    std::vector<double> frame_size_;
    
    void parse_parameters_();
    void calibrate_();

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraCalibration>();
    rclcpp::shutdown();
    return 0;
}