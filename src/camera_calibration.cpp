//STD
#include <cstring>
#include <string>

//ROS2
#include "rmw/types.h"

//ArenaSDK
#include "camera_calibration.h"
#include "rclcpp_adapter/pixelformat_translation.h"

void CameraCalibration::parse_parameters_()
{   
    //Declare ROS2 calibration parameters
    this->declare_parameter("camera_name");
    this->declare_parameter("image_path");
    this->declare_parameter("extension");
    this->declare_parameter("pattern_size");
    this->declare_parameter("frame_size");
    this->declare_parameter("camera_output_file");

    //Get ROS2 calibration parameters
    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("image_path", image_path_);
    this->get_parameter("extension", extension_);
    this->get_parameter("pattern_size", pattern_size_);
    this->get_parameter("frame_size", frame_size_);
    this->get_parameter("camera_output_file", camera_output_file_);    
}

void CameraCalibration::calibrate_()
{
    //Set double arrays as an integer arrays - Not sure why declaring pattern_size_ as std::vector<int> DOES NOT WORK!
    int pattern_size_int_[2] = {int(pattern_size_[0]), int(pattern_size_[1])};
    int frame_size_int_[2] = {int(frame_size_[0]), int(frame_size_[1])};

    //Generate *. path for glob function
    std::string image_list_ = image_path_ + extension_;

    std::vector<cv::String> file_names_;
    cv::glob(image_list_, file_names_, false);
    cv::Size patternSize(pattern_size_int_[0] - 1, pattern_size_int_[1] - 1);
    std::vector<std::vector<cv::Point2f>> q(file_names_.size());

    //Generate "world" coordinates Q
    std::vector<std::vector<cv::Point3f>> Q;

    //Defining world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i = 1; i < pattern_size_int_[1]; i++)
    {
        for(int j = 1; j < pattern_size_int_[0]; j++)
        {
            objp.push_back(cv::Point3f(j,i,0));
        }
    }

    std::vector<cv::Point2f> imgPoint;
    //Detect feature points
    std::size_t i = 0;

    for (auto const &f : file_names_)
    {   
        //Read in image
        cv::Mat img = cv::imread(f);
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        //Search for chessboard corners
        bool patternFound = cv::findChessboardCorners(
            gray,
            patternSize,
            q[i],
            cv::CALIB_CB_ADAPTIVE_THRESH + 
            cv::CALIB_CB_NORMALIZE_IMAGE +
            cv::CALIB_CB_FAST_CHECK);

        //Refine corner detections
        if(patternFound)
        {
            cv::cornerSubPix(
                gray,
                q[i],
                cv::Size(11,11),
                cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::EPS +
                    cv::TermCriteria::MAX_ITER,
                    30,
                    0.1));
            
            Q.push_back(objp);
        }
        else if (!patternFound)
        {
            RCLCPP_ERROR(this->get_logger(), "\nFailed to detect pattern for %s during calibration for %s\n",
                f.c_str(),
                camera_name_.c_str());
        }

        //Display chessboard pattern detection 
        /*
        cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
        cv::imshow("Chessboard Detection", img);
        int key = cv::waitKey(0);
        if (key == 's')
        {
            cv::imwrite("/home/tahnt/Documents/opencv_cpp_tutorial/ros2_ws/src/camera_calibration_package/images/cv_write_test/draw_chessboard2.jpg", img);
        }
        */

        i++;
    }
    
    cv::Matx33d cameraMatrix(cv::Matx33f::eye()); //Intrinsic camera matrix
    cv::Mat distCoeffs(0, 0, 0, 0, 0);

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinstics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
                cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;

    cv::Size frameSize(frame_size_int_[0], frame_size_int_[1]);

    //Log image list and frame size 
    RCLCPP_INFO(this->get_logger(), "%s Image List: %s", 
        camera_name_.c_str(), 
        image_list_.c_str());
    RCLCPP_INFO(this->get_logger(), "%s Frame Size: %d %d", 
        camera_name_.c_str(), 
        frame_size_int_[0], 
        frame_size_int_[1]);
    RCLCPP_INFO(this->get_logger(), "Calibrating %s ...", camera_name_.c_str());

    //Calibrate camera
    float error = cv::calibrateCamera(Q, q, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags);

    if (!distCoeffs.empty() && !rvecs.empty() && !tvecs.empty())
    {   
        RCLCPP_INFO(this->get_logger(), "Calibration successful");

        //Write camera matrix and distortion coefficients to .yaml file
        cv::FileStorage fs(camera_output_file_, cv::FileStorage::WRITE);

        //Time of calibration 
        time_t tm;
        time( &tm );
        struct tm *t2 = localtime( &tm );
        char buf[1024];
        strftime( buf, sizeof(buf), "%c", t2 );

        //Write to file
        fs << "calibration_time" << buf;
        fs << "camera_name" << camera_name_;
        fs << "frame_width" << frame_size_int_[0];
        fs << "frame_height" << frame_size_int_[1];
        fs << "reprojection_error" << error;
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs << "rotation_vector" << rvecs;
        fs << "translation_vector" << tvecs;

        RCLCPP_INFO(this->get_logger(), "Write successful");
        RCLCPP_INFO(this->get_logger(), "%s parameters written to %s", 
            camera_name_.c_str(), 
            camera_output_file_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Calibration failed");
    }

    //Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Matx33f::eye(), cameraMatrix, frameSize, CV_32FC1, mapX, mapY);

    //Show lens corrected images
    for (auto const &f : file_names_)
    {
        //std::cout << std::string(f) << std::endl;

        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
        cv::Mat imgUndistorted;

        //Remap the image using precomputed interpolation maps
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

        //Display undistorted images
        /*
        cv::imshow("undistorted image", imgUndistorted);
        int key = cv::waitKey(0);
        if (key == 's')
        {
            cv::imwrite("<enter_write_path>", imgUndistorted);
        }
        */
                   
    }
}
