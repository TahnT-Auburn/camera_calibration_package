from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_path = '/home/tahnt/T3_Repos/camera_packages/ros2_ws/src/camera_calibration_package/config/camera_calibration_config.yaml'
    
    return LaunchDescription([
        Node(
            package="camera_calibration_package",
            executable="calibrate",
            namespace="camera1",
            name="camera_calibration",
            output="screen",
            parameters=[config_path]
        ),
        #Node(                                          #New launch nodes must be included to calibrate multiple cameras!
            #package="camera_calibration_package",
            #executable="calibrate",
            #namespace="camera2",
            #name="camera_calibration",
            #output="screen",
            #parameters=[config_path]
        #),
        #Node(                                          #New launch nodes must be included to calibrate multiple cameras!
            #package="camera_calibration_package",
            #executable="calibrate",
            #namespace="camera3",
            #name="camera_calibration",
            #output="screen",
            #parameters=[config_path]
        #)
    ])
       