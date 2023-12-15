# camera_calibration_package

## Description:
ROS2 package to calibrate a camera(s) and write parameters to a YAML file.

This package uses OpenCV libraries and a predefined/populated set of checkerboard images to calibrate cameras and writes their parameters to YAML.

Note that valid sets of checkerboard images taken from each desired camera must be provided prior to successfully run the calibration. 

**For more information on this package, check the Joplin documentation under the _Camera Calibration Package_ page.**

## Requirements:
* Ubuntu Focal Fossa
* ROS2 Foxy Fitzroy
* C++17 or higher

## To Use:
**_Before Use:_** 
* **Make sure ALL PATHS ARE SET CORRECTLY in the launch and config files before use!**
* **These steps assume you have already created a workspace folder and a `/src` directory within it!**

**_Steps_:**
1. Navigate into the `\src` directory of your workspace and clone the repo using `git clone`
2. Navigate back into the workspace directory and source `$ source /opt/ros/foxy/setup.bash`
3. Build package `$ colcon build` or `$ colcon build --packages-select <package_name>`
4. Open a new terminal and source it `$ . install/setup.bash`
5. Run launch file `$ ros2 launch <package_name> <launch_file_name>` in this case it is `$ ros2 launch camera_calibration_package camera_calibration_launch.py`