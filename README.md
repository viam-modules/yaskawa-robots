# [`yaskawa-robots` module](https://app.viam.com/module/viam/yaskawa-robots)

This repo is a [module](https://docs.viam.com/registry/#modular-resources) that implements the [`rdk:component:arm` API](https://docs.viam.com/components/arm/) resource to allow control over [Yaskawa Robots](https://www.yaskawa.com/) arms. Currently the following models are supported:

- GP12, as `viam:yaskawa-robots:gp12`
- GP180-120, as `viam:yaskawa-robots:gp180-120`

## Configuration and Usage

This model can be used to control a yaskawa robots arm from a machine running a viam-server. We recommend that the machine running the viam-server is using a wired ethernet connection for the best performance of this module. The following attribute template can be used to configure this model:

```json
{
    "host": <arm ip address string>,
    "speed_rad_per_sec": <float or array>,
    "acceleration_rad_per_sec2": <float or array>
}
```

### Attributes

The following attributes are available for `viam:yaskawa-robots` arms:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `host` | string | **Required** | The IP address of the robot arm on your network. |
| `speed_rad_per_sec` | float or array | **Required** | Set the maximum desired speed of the arm joints in radians per second. Can be a single value (applied to all joints) or an array for per-joint limits. |
| `acceleration_rad_per_sec2` | float or array | **Required** | Set the maximum desired acceleration of the arm joints in radians per second squared. Can be a single value (applied to all joints) or an array for per-joint limits. |
| `reject_move_request_threshold_rad` | float | Optional | Rejects move requests when the difference between the current position and first waypoint is above threshold. **Range: 0 - 2&pi; radians** |
| `trajectory_sampling_freq_hz` | float | Optional | Sampling frequency for trajectory generation in Hz. Higher values produce smoother trajectories but require more computation. **Default 3 Hz** (range: 1 - 250) |
| `waypoint_deduplication_tolerance_deg` | float | Optional | Tolerance in degrees for deduplicating consecutive waypoints. Waypoints within this tolerance of each other are considered identical. **Default ~0.057 degrees** (1e-3 radians) (range: 0 - 10) |
| `path_tolerance_rad` | float | Optional | Path tolerance for trajectory planning in radians. **Default 0.1 rad** (range: 0.0 - 3.0) |
| `segmentation_threshold_rad` | float | Optional | Threshold for detecting direction reversals in waypoint paths. Lower values detect only sharp reversals; higher values are more sensitive. **Default 0.005 rad** (range: > 0, <= 0.1) |
| `collinearization_ratio` | float | Optional | Waypoint collinearization parameter for trajectory smoothing. (range: 0.0 - 2.0) |
| `enable_new_trajectory_planner` | bool | Optional | Enables the new trajectory planning algorithm. **Default true** |
| `telemetry_output_path` | string | Optional | Path for writing telemetry data files. **Default: VIAM_MODULE_DATA environment variable** |
| `group_index` | int | Optional | Define which control group this arm represents. **Default 0** |

### Example configurations:

#### Basic configuration with wired ethernet connection
```json
{
    "host": "10.1.10.84",
    "speed_rad_per_sec": 2.09,
    "acceleration_rad_per_sec2": 0.14
}
```

#### Per-joint velocity and acceleration limits
```json
{
    "host": "10.1.10.84",
    "speed_rad_per_sec": [2.09, 2.09, 2.09, 2.09, 2.09, 2.09],
    "acceleration_rad_per_sec2": [0.14, 0.14, 0.14, 0.14, 0.14, 0.14]
}
```

#### Custom trajectory generation parameters
```json
{
    "host": "10.1.10.84",
    "speed_rad_per_sec": 2.09,
    "acceleration_rad_per_sec2": 0.14,
    "trajectory_sampling_freq_hz": 10,
    "path_tolerance_rad": 0.05,
    "enable_new_trajectory_planner": true
}
```

#### Telemetry output
```json
{
    "host": "10.1.10.84",
    "speed_rad_per_sec": 2.09,
    "acceleration_rad_per_sec2": 0.14,
    "telemetry_output_path": "/var/log/arm_telemetry"
}
```

### Interacting with the Arm
First ensure that your machine is displaying as **Live** on the Viam App. Then you can interact with your Yaskawa Robots arm in a couple ways:
- To simply view data from and manipulate your arm, use the **CONTROL** tab of the Viam App.
For more information, see [Control Machines](https://docs.viam.com/fleet/control/).
- More advanced control of the arm can be achieved by using one of [Viam's client SDK libraries](https://docs.viam.com/components/arm/#control-your-arm-with-viams-client-sdk-libraries)

## Building and Running

### Building for Development

For development purposes, you can either build directly on your development machine, or within a docker container, or with `conan`. The advantage of the docker container is that it comes with the required runtime libraries preinstalled. However, the `module.tar.gz` that is produced will only run on an Ubuntu Jammy (22.04) machine which already has the necessary runtime libraries installed, and it is unlikely that this matches your development machine. The advantage to building directly on your development machine is that the `module.tar.gz` that is built will work on that machine. However, you are required to install the necessary support libraries first. The advantage of `conan` is that it will build you a self-contained binary and will handle installing all of the dependencies. The resulting `module.tar.gz` should run on any system offering a system ABI compatible with the machine on which you built it (for instance, build on Debian 12 Bookworm, and it should run on Debian 13 Trixie).

#### Build With Docker

Clone this repository to your machine and from the newly created folder start a new Docker container using the following commands:

```
git submodule update --init
docker pull ghcr.io/viam-modules/yaskawa-robots:amd64
docker run --net=host --volume .:/src -it ghcr.io/viam-modules/yaskawa-robots:amd64
```

If you are working on an ARM based machine, use the arm64 images:
```
docker pull ghcr.io/viam-modules/yaskawa-robots:arm64
docker run --net=host --volume .:/src -it ghcr.io/viam-modules/yaskawa-robots:arm64
```

Once inside the docker container build the binary using:

```
cd /src && make module.tar.gz
```

#### Build For Local System

Install the prerequisite support libraries. These include, but are not limited to:
- Abseil
- Boost
- Eigen
- Protobuf
- Viam C++ SDK
- Xtensor
- gRPC

#### Build for Remote Deployment

First, if you don't already have it, you will need to install `conan`. On macOS this is as easy as `brew install conan`. On systems where the system package manager does not provide it, you will most likely need to create a virtualenv and then run `python -m pip install conan`. Or use your python project/package management system of choice.

Next, you need to have conan build and install the prerequisite package by running the `./bin/setup.sh` script. This will take a while since it needs to compile Boost, gRPC, Eigen, and the Viam C++ SDK from source.

Finally, you can run the `bin/build.sh` script, which will build the Yaskawa Robots driver. This should be relatively quick, and you can run this repeatedly to re-run the build of the driver and produce `module.tar.gz`.

## Running

Use one of the above techniques to produce `module.tar.gz`. Then, follow the instructions to [add a local module](https://docs.viam.com/registry/configure/#add-a-local-module) to add the local instance of the `yaskawa-robots` module to your machine.

> [!NOTE]
> Simply running `make install` inside the docker image is a faster way to build but creates an executable that must be run from within the Docker container, per the above notes.  If you are interested in making and testing many quick changes to this module it will likely be faster to only build this way and then run `viam-server` from within the Docker container, pointing to a config file that has a local instance of this module with an **executable path** of `build/install/bin/yaskawa-robots`.  To download and run `viam-server` this way run:
> ```
> wget https://storage.googleapis.com/packages.viam.com/apps/viam-server/viam-server-stable-x86_64
> chmod +x ./viam-server-stable-x86_64
> ./viam-server-stable-x86_64 -config {PATH_TO_VIAM_CONFIG}
> ```

## Kinematics

Our kinematics representation of the arm can be found at `src/kinematics`. Right now we only support the `gp12` model.
We would like to call out that `link_4_r` is modeled as a smaller geometry than necessary and might lead to collisions when there should be none. If a user runs into this please raise immediately and we will issue an appropriate fix.
The motivation for shrinking the geometry is described next. Suppose a user has attached an end effector attachment. If it is long or wide enough and `joint_4_r` has a rotation of `-90` degrees this will result in a collision between the attachment and `link_4_r`.
Below is an image to show exactly how undermodeled the geometry is at this point in time.
<img width="1143" height="556" alt="image" src="https://github.com/user-attachments/assets/c2ea200c-ff88-4c65-833b-da59421dadab" />
