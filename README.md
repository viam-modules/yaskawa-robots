# [`yaskawa-robots` module](https://app.viam.com/module/viam/yaskawa-robots)

This repo is a [module](https://docs.viam.com/registry/#modular-resources) that implements the [`rdk:component:arm` API](https://docs.viam.com/components/arm/) resource to allow control over [Yaskawa Robots](https://www.yaskawa.com/) arms. Currently the following models are supported:

- GP12, as `viam:yaskawa-robots:gp12`

## Configuration and Usage

This model can be used to control a yaskawa robots arm from a machine running a viam-server. We recommend that the machine running the viam-server is using a wired ethernet connection for the best performance of this module. The following attribute template can be used to configure this model:

```json
{
    "host": <arm ip address string>,
    "speed_rads_per_sec": <float>,
    "acceleration_rads_per_sec2": <float>
}
```

### Attributes

The following attributes are available for `viam:yaskawa-robots` arms:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `host` | string | **Required** | The IP address of the robot arm on your network. |
| `speed_rads_per_sec` | float | **Required** | Set the maximum desired speed of the arm joints in degrees per second. |
| `acceleration_rads_per_sec2` | float | **Required** | Set the maximum desired acceleration of the arm joints. |
| `reject_move_request_threshold_deg` | float | Not Required | Rejects move requests when the difference between the current position and first waypoint is above threshold |

### Example configuration:
```json
{
    "host": "10.1.10.84",
    "speed_rads_per_sec": 2.09,
    "acceleration_rads_per_sec2": 0.14
}
```
### Interacting with the Arm
First ensure that your machine is displaying as **Live** on the Viam App. Then you can interact with your Yaskawa Robots arm in a couple ways:
- To simply view data from and manipulate your arm, use the **CONTROL** tab of the Viam App.
For more information, see [Control Machines](https://docs.viam.com/fleet/control/).
- More advanced control of the arm can be achieved by using one of [Viam's client SDK libraries](https://docs.viam.com/components/arm/#control-your-arm-with-viams-client-sdk-libraries)

## Building and Running

### Build Configuration

Prerequisites: CMake 3.25+, C++20 compiler, and dependencies (Abseil, Boost, Eigen, Protobuf, Viam C++ SDK, Xtensor, gRPC).

Configure and build:
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=RelWithDebInfo -GNinja
ninja -C build
```

### Building the Module Package

To create the deployable `module.tar.gz` archive for use with Viam:

```bash
ninja -C build install
ninja -C build package
```

This installs binaries to `build/install/` and packages them into `build/module.tar.gz`. The archive contains the `yaskawa-robots` module binary, metadata, and kinematics files needed for deployment.

### Building the Example

To build the standalone example program for direct robot testing:

```bash
ninja -C build yaskawa-example
```

This creates `build/yaskawa-example`, a test binary that connects directly to the robot without Viam server infrastructure.

### Docker Build

For containerized builds with pre-installed dependencies:

```bash
git submodule update --init
docker pull ghcr.io/viam-modules/yaskawa-robots:amd64
docker run --net=host --volume .:/src -it ghcr.io/viam-modules/yaskawa-robots:amd64
```

For ARM64: use `ghcr.io/viam-modules/yaskawa-robots:arm64`

Inside the container:
```bash
cd /src
cmake -B build -S . -GNinja
ninja -C build install
ninja -C build package
```

## Running

### Deploying the Module

After building `module.tar.gz`, follow the [add a local module](https://docs.viam.com/registry/configure/#add-a-local-module) instructions to deploy it to your Viam machine.

### Quick Development Testing

For rapid iteration without packaging, point your Viam config directly to the built binary at `build/install/bin/yaskawa-robots`. For Docker-based testing:

```bash
wget https://storage.googleapis.com/packages.viam.com/apps/viam-server/viam-server-stable-x86_64
chmod +x ./viam-server-stable-x86_64
./viam-server-stable-x86_64 -config {PATH_TO_VIAM_CONFIG}
```
