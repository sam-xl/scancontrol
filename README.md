# micro_epsilon_scancontrol
![CI](https://github.com/sam-xl/scancontrol/workflows/CI/badge.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

ROS device driver for the scanCONTROL series of laser line scanners of Micro Epsilon using the [scanCONTROL Linux C++ SDK 0.2]. The driver allows to connect to a (specific) scanCONTROL device, configure the sensor using predefined settings or at runtime and publishes the sensor data as point clouds. 

**Author: D. Kroezen (GitHub username: dave992)<br />
Affiliation: [SAM|XL](https://samxl.com/), [TU Delft](https://tudelft.nl/)<br />
Maintainer: D. Kroezen, d.kroezen@tudelft.nl**

The micro_epsilon_scancontrol package has been tested under [ROS] Melodic and Ubuntu 18.04. 

## Installation

#### Dependencies

- [Aravis 0.6.x](https://github.com/AravisProject/aravis)
- [scanCONTROL Linux C++ SDK 0.2.4](https://www.micro-epsilon.com/2D_3D/laser-scanner/Software/downloads/) 

<!-- Note: Optional scripts to (un)install the dependencies can be found [here](). [Password required] -->

<!-- # Micro Epsilon scanCONTROL ROS Driver

## Installation instructions 
### NOTE: Installation files are not included in this repository! 
Install dependencies:
`sudo apt install checkinstall` 

Move to api folder:
`cd micro_espilon_scancontrol_api/`

Run the install script and follow the instructions in the command window:
`sudo bash install.sh`

## Uninstal instructions
### NOTE: Uninstallation files are not included in this repository! 
Move to the api folder:
`cd micro_espilon_scancontrol_api/`

Run the uninstall script and follow the instruction in the command window:
`sudo bash uninstall.sh` -->

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using:

	cd catkin_ws/src
	git clone https://github.com/sam-xl/micro_epsilon_scancontrol.git
	cd ../
	catkin build


<!-- ### Unit Tests

Run the unit tests with

	catkin_make run_tests_ros_package_template
 -->

## Usage

Run the main driver node with:

	roslaunch micro_epsilon_scancontrol_driver load_driver.launch

<!-- ## Config files

* **partial_profile.yaml** Configure custom partial profile settings at start-up. The default values only extract the xyz values from the measurement buffer. Adjust at your own risk! 
	- start_point: 
	- start_point_data: 
	- point_count - Number of data points, defaults to -1 (Inherit from resolution)
	- data_width:  -->

## Launch files

* **driver.launch:** Launch a single scanCONTROL driver node and the configuration window. 
<!-- 
     Arguments

     - **`show_rqt_plugin`** Display the rqt plugin to reconfigure the sensor on start-up. Default: `true`. -->

## Nodes

### scancontrol_driver_node

The scancontrol_driver_node connects to the scanCONTROL device and allows control of most settings through the provided services. By default the driver only extracts the xyz data from the measurement buffer to create the point cloud message. For now the additional measurement data such as reflections are discarted. 

#### Published Topics

* **`/scancontrol_pointcloud`** ([sensor_msgs/PointCloud2])

	The laser scan data filtered by the partial profile settings. The last point(s) may get lost, as a timestamp overwrites the last 4 bytes of the measurement buffer.


#### Services
Most servives are wrappers of the scanCONTROL API. For more information on the available settings and values see the documentation as part of the [scanCONTROL Linux C++ SDK 0.2](https://www.micro-epsilon.com/2D_3D/laser-scanner/Software/downloads/). The rqt plugin uses these services to change the settings during runtime. 

* **`~set_feature`** ([micro_epsilon_scancontrol_msgs/SetFeature])

	Set a feature (setting) on the scanCONTROL device. 


* **`~get_feature`** ([micro_epsilon_scancontrol_msgs/GetFeature])

	Get the current feature (setting) from the connected scanCONTROL device. 

* **`~get_resolution`** ([micro_epsilon_scancontrol_msgs/GetResolution])

	Get the current active resolution used by the connected scanCONTROL device.

* **`~set_resolution`** ([micro_epsilon_scancontrol_msgs/SetResolution])

	Set the resultion of the connected scanCONTROL device.

* **`~get_available_resolutions`** ([micro_epsilon_scancontrol_msgs/GetAvailableResolutions])

	Retrieve a list of all available resolutions of the connected scanCONTROL device. 

* **`~invert_x`** ([std_srvs/SetBool])

	Flip the X values around the middle of the laser line of the sensor.  

* **`~invert_z`** ([std_srvs/SetBool]

	Flip the Z values around the middle of the measuring range of the sensor. Factory default value of of this setting is `True`.

#### Parameters
Device Settings
* **`resolution`** (int)

	Define a prefered resolution setting for the laser scan. By default the highest available resolution is selected.

The following parameters are available to allow using multiple scanCONTROL devices.

* **`serial`** (string)

	Define a prefered scanCONTROL device by its serial number. If not defined, the driver will try to connect to the first device it is able to find. 

* **`topic_name`** (string, default: `scancontrol_pointcloud`)

	Define a custom name for the topic to publish the point cloud data on. 

* **`frame_id`** (string, default: `scancontrol`)

	Define a custom name for the measurement frame in which the point clouds are published.



### scancontrol_driver_nodelet

Encapsulates the same driver class as the scancontrol_driver_node, but instead allows for zero-copy data transfer. The Topics, services and paremeters are the same as described for the scancontrol_driver_node above. 


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/sam-xl/scancontrol/issues).


[ROS]: http://www.ros.org
[scanCONTROL Linux C++ SDK 0.2]: (https://www.micro-epsilon.com/2D_3D/laser-scanner/Software/downloads/)
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[micro_epsilon_scancontrol_msgs/GetAvailableResolutions]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/GetAvailableResolutions.srv
[micro_epsilon_scancontrol_msgs/GetFeature]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/GetFeature.srv
[micro_epsilon_scancontrol_msgs/GetResolution]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/GetResolution.srv
[micro_epsilon_scancontrol_msgs/SetFeature]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/SetFeature.srv
[micro_epsilon_scancontrol_msgs/SetResolution]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/SetResolution.srv
[std_srvs/SetBool]: http://docs.ros.org/api/std_srvs/html/srv/SetBool.html