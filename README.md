# micro_epsilon_scancontrol
![CI](https://github.com/sam-xl/scancontrol/workflows/CI/badge.svg) 
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview
ROS device driver for the scanCONTROL series of laser line scanners of Micro Epsilon using the [scanCONTROL Linux C++ SDK 1.0]. The driver allows to connect to a (specific) scanCONTROL device, configure the sensor using predefined settings or at runtime and publishes the sensor data as point clouds. 

Author: **D. Kroezen** \
Affiliation: [SAM XL](https://samxl.com/), [TU Delft](https://tudelft.nl/) \
Maintainers: 
**D. Kroezen, d.kroezen@tudelft.nl**, **E. Bernardi, e.bernardi@tudelft.nl**

> [!WARNING]  
> This micro_epsilon_scancontrol package branch **is under development** for [ROS2 Humble](https://docs.ros.org/en/humble/index.html) and Ubuntu 22.04 Jammy!


## Installation

#### Dependencies
- [Aravis 0.8.x](https://github.com/AravisProject/aravis/releases) [[docs]](https://aravisproject.github.io/aravis/)
- [scanCONTROL Linux C++ SDK 1.0.x](https://www.micro-epsilon.com/2D_3D/laser-scanner/Software/downloads/) 

#### Building
To build from source, clone the latest version from this repository into your workspace and compile the package:

```bash
mkdir -p ros2_ws/src && cd ros2_ws
git clone git@github.com:sam-xl/scancontrol.git -b ros2-devel src
```
To build locally, after installing Aravis and the SDK:
```bash
sudo apt-get update
rosdep update 
rosdep install --from-paths src --ignore-src -y
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" -Wall -Wextra -Wpedantic
```
And source the installation
```bash
. install/setup.bash
```
## Development setup
For anyone having access to scanCONTROL hardware and not wanting to install dependencies locally, follow the setup below to work with docker compose or in a devcontainer environment.

#### Docker compose:
- `cd src/scancontrol/.docker`
- `docker compose up -d --build`
- If using VSCode, you can attach a shell or VSCode and work from there.
- Or you can use other dev IDEs? **TODO**: 
  - Add [watch](https://docs.docker.com/compose/file-watch/) sections to one or more services in compose.yaml
  - Run docker compose watch to build and launch a Compose project and start the file watch mode.
  - Edit service source files using your preferred IDE or editor
#### devcontainer:
- Link the `.devcontainer` folder
  - cd to your `ros2_ws` 
  - `cp src/scancontrol/.devcontainer/ .`
  - Hit `F1` in VSCode and select `build and reaopen in container`

## Nodes
### scancontrol_description_node
Visualise the scanCONTROL models
```bash
ros2 launch micro_epsilon_scancontrol_description load_scancontrol.launch.py
```

Possible models passed as arguments with `scancontrol_type:=`
- `scancontrol_26x0_29x0_25.xacro` (default)
- `scancontrol_27x0_100.xacro`
- `scancontrol_30xx_25.xacro`

### scancontrol_driver_node

The scancontrol_driver_node connects to the scanCONTROL device and allows control of most settings through the provided services. By default the driver only extracts the xyz data from the measurement buffer to create the point cloud message. For now the additional measurement data such as reflections are discarted. 

#### Published Topics

* **`/scancontrol_pointcloud`** ([sensor_msgs/PointCloud2])

	The laser scan data filtered by the partial profile settings. The last point(s) may get lost, as a timestamp overwrites the last 4 bytes of the measurement buffer.


#### Services
Most servives are wrappers of the scanCONTROL API.
The rqt plugin uses these services to change the settings during runtime. 

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

## Docker Images

This repository provides a `Dockerfile` (in the `.docker` folder) with multiple stages for images with Aravis and the scanCONTROL SDK installed.

### scancontrol-core
Extends: `[ros-humble-ros-core]`

- Installed Aravis.
- Installed scanCONTROL SDK.

```bash
cd .docker
docker build . --target scancontrol-core -t scancontrol:humble-ros-core
```

### scancontrol-base
Extends: `[scancontrol-core]`

- Installed `ros-humble-ros-base`
- Installed `rviz2`. 

```bash
cd .docker
docker build . --target scancontrol-base -t scancontrol:humble-ros-base
```

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