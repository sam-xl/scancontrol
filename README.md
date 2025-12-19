# micro_epsilon_scancontrol
![CI](https://github.com/sam-xl/scancontrol/workflows/CI/badge.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

ROS 2 device driver for the scanCONTROL series of laser line scanners of Micro Epsilon using the [scanCONTROL Linux C++ SDK 1.0.0]. The driver allows to connect to a (specific) scanCONTROL device, configure the sensor using predefined settings or at runtime and publishes the sensor data as point clouds. 

**Author: D. Kroezen (GitHub username: dave992)<br />
Affiliation: [SAM XL](https://samxl.com/), [TU Delft](https://tudelft.nl/)<br />
Maintainer: D. Kroezen, d.kroezen@tudelft.nl**

The `micro_epsilon_scancontrol` packages have been tested under [ROS 2 Jazzy] & Ubuntu 24.04, and [ROS 2 Humble] & Ubuntu 22.04.

## Installation

#### Dependencies

- [Aravis 0.8.x](https://github.com/AravisProject/aravis)
- [scanCONTROL Linux C++ SDK 1.0.0](https://www.micro-epsilon.com/2D_3D/laser-scanner/Software/downloads/) 

#### Building

To build from source, clone the lastest version from this repository into your workspace:
```bash
cd ros_ws/src
git clone https://github.com/sam-xl/scancontrol.git
```

Install the dependencies of the cloned packages using rosdep:
```bash
cd ros_ws
rosdep install --from-paths src --ignore-src -y
```

Finally, build all packages in the workspace:
```bash
cd ros_ws
colcon build [--merge-install] [--symlink-install]
```

## Usage

Run the main driver node with:
```bash
ros2 launch micro_epsilon_scancontrol_driver load_driver.launch
```

## Launch files

### `micro_epsilon_scancontrol_driver`
* **load_driver.launch:** Launch a scanCONTROL driver node that connect to a single device.
* **test_driver.launch:** Visualize the live measurement data from the driver using RViz. By default uses the `26x0_29x0_25` model.

### `micro_epsilon_scancontrol_description`
* **load_scancontrol_26x0_29x0_25.launch:** Launch a `robot_state_publisher` with the `robot_description` loaded for the `26x0_29x0_25` sensor series.
* **load_scancontrol_27x0_100.launch:** Launch a `robot_state_publisher` with the `robot_description` loaded for the `27x0_100` sensor series.
* **load_scancontrol_30xx_25.launch:** Launch a `robot_state_publisher` with the `robot_description` loaded for the `30xx_25` sensor series.
* **test_scancontrol_26x0_29x0_25.launch:** Visialize the `robot_description` for the `26x0_29x0_25` sensor series.
* **test_scancontrol_27x0_100.launch:** Visialize the `robot_description` for the `27x0_100` sensor series.
* **test_scancontrol_30xx_25.launch:** Visialize the `robot_description` for the `30xx_25` sensor series.

### Modifying launch arguments
To see arguments that may be given to the launch file, run the following command:
```bash
ros2 launch <package> <file.launch> --show-args
```

## Nodes

### `scancontrol_driver_node`

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


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/sam-xl/scancontrol/issues).

[ROS 2 Humble]: https://docs.ros.org/en/humble/index.html
[ROS 2 Jazzy]: https://docs.ros.org/en/jazzy/index.html
[scanCONTROL Linux C++ SDK 1.0.0]: https://www.micro-epsilon.com/2D_3D/laser-scanner/Software/downloads/
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[micro_epsilon_scancontrol_msgs/GetAvailableResolutions]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/GetAvailableResolutions.srv
[micro_epsilon_scancontrol_msgs/GetFeature]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/GetFeature.srv
[micro_epsilon_scancontrol_msgs/GetResolution]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/GetResolution.srv
[micro_epsilon_scancontrol_msgs/SetFeature]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/SetFeature.srv
[micro_epsilon_scancontrol_msgs/SetResolution]: https://github.com/sam-xl/scancontrol/blob/master/micro_epsilon_scancontrol_msgs/srv/SetResolution.srv
[std_srvs/SetBool]: http://docs.ros.org/api/std_srvs/html/srv/SetBool.html
