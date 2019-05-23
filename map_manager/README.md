# map_manager

## Installation

Look at the included dockerfile.

## Include in your project

### Manually

```bash
git clone https://git.web.boeing.com/robotics/ros/map_manager
```

### With a .rosinstall file

Add the following to a .rosinstall file:

```yaml
- git: {local-name: map_manager, uri: 'https://git.web.boeing.com/robotics/ros/map_manager'}
```

```bash
wstool update --target-workspace=/path/to/workspace/src/with/.rosinstall/file
```

For more information about using rosinstall and wstool see the [ROS Wiki - wstool](http://wiki.ros.org/wstool)

## Get dependencies

Once the source is downloaded to your workspace run:

```bash
rosdep update
rosdep install --from-paths /path/to/your/workspace --ignore-src
```

If using as part of a Dockerfile that command will run as root and so must be modified to

```bash
rosdep install --from-paths /path/to/your/workspace --ignore-src --as-root apt:false -y
```