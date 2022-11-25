# Modular Navigation

## Motivation

A mobile robot needs to be able to navigate a complex and dynamic environment and be able to plan and drive its way
to a given goal. This package aims to be a flexible, modular system for real-time navigation and path planning for
a holonomic robot equipped with a number of sensors (namely laser and depth).


## Usage

The navigation stack contains many packages but runs as a single ROS node with independent threads for each of the
layers.

Important!: This package needs to be built with compiler optimisations to be able to run in real-time. See section
*How to Build* for instructions.

Most of the configurations is done via a YAML file which is read *once* directly by the node on startup. The ROS
parameter server is **NOT** used except to pass the path of the YAML file.

An example launch script:
```xml
<node pkg="autonomy" type="autonomy" respawn="true" name="autonomy">
    <param name="navigation_config" value="/path/to/config/file" />
</node>
```

`autonomy` will then listen to `autonomy/goal` for new goals and publish feedback on `autonomy/feedback`

## Goals

- Create a map that is representative of the robot environment and the obstacles in it
- Plan a path from the robot's current position to some provided goal
- Follow the path and update the map/path along the way
- Send velocity commands to move the robot

## Requirements

1. Find a path from start to goal if physically possible
2. Path must avoid obstacles
3. Path must update in real-time to avoid newly detected obstacles
4. Control robot by sending velocity commands
5. Robot must be able to drive very close (up to 5cm) to obstacles

## Definitions

| Definition   | Description                                                                               |
| ------------ | ----------------------------------------------------------------------------------------- |
| Pose         | A 3D state of the robot (X, Y and orientation)                                            |
| Goal         | The final desired pose of the robot. Usually w.r.t. the map origin                        |
| Cost         | A measure of how undesireable it is for the robot to be at or move through a particular region |
| Costmap      | A map showing the costs at each point on the map                                          |
| Node         | A single point on a trajectory                                                            |
| Trajectory   | An ordered set of nodes that links the start to the goal.                                 |
| Path Planning| The process of finding a continuous path from start to goal                               |
| Trajectory optimisation| The process of refining a trajectory to smooth it and avoid obstacles           |
| Control | The process of commanding the robot to accurately follow the trajectory                        |

## Design

### Assumptions

- Robot knows its location relative to the map (robot is localised) using something like Cartographer
- Odometry is published
- There are sufficient sensors to detect any potential obstacles and the sensor data is accurate
- `cmd_vel` will be obeyed reasonably faithfully and that the robot can physically achieve the velocities
and accelerations
- Robot drives reasonably slowly (<0.25 m/s)

### Limitations

- Path planning updates take approximately 1 second for a map of size 150 m^2, so there will
likely be performance issues when scaling up.
- Robot cannot drive too fast or the map/trajectory cannot update fast enough.

### Description

Meta-package for navigation stack. Contains:
- `autonomy` - Main package for navigation
- `astar_planner` - Hybrid A* path planner
- `sim_band_planner` - Sim band (trajectory optimiser)
- `pure_pursuit_controller` - Pure Pursuit (controller)
- `gridmap` - Layered probability grid
- `map_manager` - map database interface

#### Overview
Autonomy is a multi-layered system for 3D robot navigation (X,Y and rotation).
Conceptually, navigation can be broken into three layers:
1. Path planning - produce a rough path from A to B
2. Trajectory optimisation - Smooth out the path and swerve around immediate obstacles
3. Control - Follow the path and perform last-second collision checking

Each layer can in theory be swapped out with different algorithms without
affecting the next. Therefore, the interface for each layer is separately
defined in `navigation_interface`.

For now, there is only one of each layer:

#### Path planning (Hybrid A*)
Hybrid A* divides the map into a grid and "explores" it by computing the
cost required to get to each cell.

The cost is computed as the cost to get from Start to the cell, plus a
heuristic. The heuristic is an estimate of the cost to get from that cell
to the goal. Starting with the Start cell, the algorithm always explores
the next cheapest cell. Because of the heuristic, the search may be "guided"
towards the goal without having to explore all the cells (as in Dijkstra),
significantly reducing the time required to find a valid path.

A well tuned heuristic is extremely important. If we underestimate the
heuristic, it means each time we take a step towards the goal, the traversal
is more expensive than what the heuristic predicted, so we will stop exploring
down that path and end up doing a wide search (slow). However, an overestimating
heuristic will quickly guide you down a narrow path that could end up being
invalid (since it doesn't check collisions properly). Currently, the heuristic
is calculated using a 2D Dijkstra (ignoring rotation) and estimating the
robot as a single circle. Interestingly, the Dijkstra is cached and done
in reverse (goal to robot) to be able to re-use the cached costs as the
robot drives towards the goal.

The cost is also calculated differently for different types of motion.
For example, driving backwards is more expensive than forwards,
allowing us to tune the behaviour of the robot.

The robot's footprint is approximated using a few small circles. This makes
collision checking fast. For the heursitic, the robot uses a single conservative circle.

#### Sim Band planner (trajectory optimisation)

Trajectory optimisation does two things: smooth out the path generated by the planner and
nudge the path to swerve around obstacles. The internally developed Sim Band Planner does this.

Sim Band was inspired largely by the [E-Band planner](http://www8.cs.umu.se/research/ifor/dl/Control/elastic%20bands.pdf),
which treats the path given by the path planner as an elastic band with "bubbles" along the trajectory.

Sim band's approach also treats the trajectory like an elastic band with nodes.
Obstacles exert forces on the nodes and we iteratively shift the nodes to achieve "equilibrium" like a physics
simulation.

Instead of circular bubbles, the footprint of the robot is approximated using many small circles like of the
path planner. This allows us to calculate the force on each small circle individually and from that,
calculate the overall torque on the robot, allowing us to optimise rotation as well as translation. Note that
the footprint can be different from the planner's but in general, you should have
a footprint that is equal to or smaller than the planner so that the planner will
not generate a path that causes Sim Band to be in collision.

Summary of differences between E-band and Sim-band
- No "bubbles"
- Robot footprint is approximated as multiple circles
- Torque is applied to nodes as well (e-band uses as single circle, so rotation is ignored)


#### Pure Pursuit Controller
The controller's job is to track the trajectory while performing last-second, thorough
collision checking. [Pure pursuit](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
is one of the simplest control algorithms. The vehicle finds a point on the path that is
a certain distance ahead (the lookahead distance) and chases it. This method is popular
for Ackerman steered vehicles, but our robot can also pivot and strafe.

The main change to the traditional approach is to treat rotation as a third axis and
distance is measured as an L2 norm with an adjustable weight for the rotation.
Currently the weight is 1.0, meaning 1 rad is equivalent to 1m. We use this distance function
to find the point on the path that is closest to the robot as well as for calculating
the lookahead.

First, we find the closest node to the robot, then based on the robot's current speed,
we calculate the required lookahead distance by multiplying the speed with a lookahead time.
We then calculate the target by finding the first point on the path that is just ahead of
the required lookahead distance. The robot the chases this point with a PD controller.

When the robot approaches the final node, the integral term is enabled, allowing the
robot to completely close the gap.

#### How to build
`astar_planner` and `sim_band_planner` need to be built with compiler
optimisation to work in real time:
```bash
catkin build modular_navigation --cmake-args -DCMAKE_CXX_FLAGS="-O2 -Wall -Werror"
```

#### Tests and debugging
##### Testing the entire stack
Run the simulation and use `goal.py`. This script sends goal commands to
autonomy in a loop.

##### astar_planner tests
Build astar_planner with the test arg:
```bash
catkin build astar_planner --make-args tests
```
The `unit_test` will be in `devel/.private/astar_planner/lib/astar_planner`

## Technologies

- Mobile robot utilising ROS

## Requirements Evaluation

| Requirement | Met? | Comments |
| ------------| ------- | ---------- |
| Find a path from start to goal if physically possible            | Yes | If goal is in collision, can resample to a nearby pose |
| Path must avoid obstacles                                        | Yes | Depth sensor noise means it can not see obstacles lower than a configurable threshold |
| Path must update in real-time to avoid newly detected obstacles  | Yes | Large maps may slow down planning |
| Control robot by sending velocity commands                       | Yes | |
| Robot must be able to drive very close (up to 5cm) to obstacles  | Yes | Can sometimes drive itself into collision and get stuck |
