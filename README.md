# [`yaskawa-robots` module](https://app.viam.com/module/viam/yaskawa-robots)

This repo is a [module](https://docs.viam.com/registry/#modular-resources) that implements the [`rdk:component:arm` API](https://docs.viam.com/components/arm/) resource to allow control over [Yaskawa Robots](https://www.yaskawa.com/) arms. Currently the following models are supported:

- GP12, as `viam:yaskawa-robots:gp12`

## Configuration and Usage

This model can be used to control a yaskawa robots arm from a machine running a viam-server. We recommend that the machine running the viam-server is using a wired ethernet connection for the best performance of this module. The following attribute template can be used to configure this model:

```json
{
    "host": <arm ip address string>,
    "speed_radss_per_sec": <float>,
    "acceleration_radss_per_sec2": <float>
}
```

### Attributes

The following attributes are available for `viam:yaskawa-robots` arms:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `host` | string | **Required** | The IP address of the robot arm on your network. Find this when setting up your UR5e. |
| `speed_rads_per_sec` | float | **Required** | Set the maximum desired speed of the arm joints in degrees per second. |
| `acceleration_rads_per_sec2` | float | **Required** | Set the maximum desired acceleration of the arm joints. |
| `reject_move_request_threshold_deg` | float | Not Required | Rejects move requests when the difference between the current position and first waypoint is above threshold |

### Example configuration:
```json
{
    "host": "10.1.10.84",
    "speed_degs_per_sec": 120,
    "acceleration_degs_per_sec2": 8
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

Next, you need to have conan build and install the prerequsite package by running the `./bin/setup.sh` script. This will take a while since it needs to compile Boost, gRPC, Eigen, and the Viam C++ SDK from source.

Finally, you can run the `bin/build.sh` script, which will build the Yaskawa Robots driver. This should be relatively quick, and you can run this repeatedly to re-run the build of the driver and produce `module.tar.gz`.

## Running

Use one of the above techniques to produce `module.tar.gz`. Then, follow the instructions to [add a local module](https://docs.viam.com/registry/configure/#add-a-local-module) to add the local instance of the `yaskawa-robots` module to your machine.

> [!NOTE]
> Simply running `make install` inside the docker image is a faster way to build but creates an executable that must be run from within the Docker container, per the above notes.  If you are interested in making and testing many quick changes to this module it will likely be faster to only build this way and then run `viam-server` from within the Docker container, pointing to a config file that has a local instance of this module wih an **executable path** of `build/install/bin/yaskawa-robots`.  To download and run `viam-server` this way run:
> ```
> wget https://storage.googleapis.com/packages.viam.com/apps/viam-server/viam-server-stable-x86_64
> chmod +x ./viam-server-stable-x86_64
> ./viam-server-stable-x86_64 -config {PATH_TO_VIAM_CONFIG}
> ```
