# DXF Specification

## Motivation

We need tools to draw maps containing information from potentially a variety of sources (site drawings, empirical map).
Ideally, the map should be a single source of truth for everything (localisation, navigation, graph-based planners,
behaviours). This specification details how each map feature should be defined in CAD/DXF for it to be parsed and
used in the map pipeline.

## Executive Summary

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

Disambiguations:

* Paths in map_manger/autonomy are lines with low-cost that the robot will preferentially use
* Currently, "Nodes" in map_manager refers to the points at the end and intersections of paths
* Zones refer to polygons with an enumerated type (avoid, exclusion, driveable etc.). This is only used to modify the cost in astar planner. In the new system, zones may be an extension of this.

## Goal

* Define how to draw a DXF map to be used in the map pipeline. Features include:
  * Walls
  * Nodes
  * Edges (undirected and directed)
  * Areas in La(0)
  * Areas in La(n) (n >=1)
  * Zones

## Requirements

In theory, any CAD software that can create and edit DXF can be used but different CAD programs may do certain things
in slightly different ways. Also, some CAD softwares do not support the full feature set of DXF (eg. FreeCAD).
The process has been verified using **ProgeCAD only** and some of its quirks have been noted below.

## DXF Features

* Layers - entities can be grouped by layers and layers can be named. Layers cannot be nested
* Elements - standard drawing elements. Examples include.
  * Polyline - a collection of lines and points (vertices). Can be closed or open.
  * Circle
  * Text
  * Insert (also known as block reference)
* Blocks - blocks are templates that can be defined once and inserted multiple times as block references
  * Attributes - blocks can have attributes (key-value pair) that can be set with a default value and modified for each individual block reference
  * Scale + rotate - Each block reference can be scaled and rotated individually
* Groups - entities that are tied together
  * Groups cannot be nested but an element can be in 2 or more groups.
* Boundary - this is not really a feature of DXF but a tool in ProgeCAD where you can select a closed area and it will
    automatically generate a polyline around it. This is useful when you have a bunch of walls and you can just
    draw a line across the doorway and it will create a perfect polygon filling the room with no missed spots.
* Hatches - A DXF element that renders a hatch pattern in a region defined by a path (can be polyline or an edgePath,
            which can contain arcs and splines as well)

There is no way to associate attributes with individual elements. It must be in a block.

## Specification

### DXF Version

The DXF format has undergone many revisions. During export, ProgeCAD (and likely other CAD programs) allow you to
select which version. This specification has been tested to work with DXF versions AutoCAD 2000 up to AutoCAD 2017.

AutoCAD 2017 file sizes are smaller, so it is recommended. Even though both binary or ASCII DXFs work, ASCII is
recommended so it can be inspected and version controlled more easily.

Things that fail with older versions:

* Square brackets (for layer tags) get converted into underscores
* No true_color attribute
* No layer descriptions
* In newer version,s polylines are saved as Light Weight Polylines (both types are handled in the script)

### Units

DXF drawings are unitless and the DXF specification does not define how to handle units. To make things consistent
with the rest of the system, all drawings should be in **metres**.

> **Note:** The old system (used in mobile toolclean) uses millimetres in the DXF and scaling is handled in the import script.
The import script now no longer does any scaling.

### Tags

Layer names can have optional tags that tell the parser how to process that layer. Tags are always at the beginning
of the layer name string and enclosed in square brackets. Eg.

```yaml
[TAG1][TAG2]layer_name
```

Spaces are not allowed in tags. Spaces in the name are allowed but spaces between the last tag and the layer name are
ignored but spaces in the name are allowed. The following are valid formats.

* Example 1: `[TAG1]layer_name`
* Example 2: `[TAG1][TAG2]layer_name`
* Example 3: `[TAG1][TAG2]  layer name`

When parsed, these become:

* Example 1: Tags: `TAG1`, Name: `layer_name`
* Example 2: Tags: `TAG1, TAG2`, Name: `layer_name`
* Example 3: Tags: `TAG1, TAG2`, Name: `layer name`

Currently, valid tags include:

* `[AREA]`: Layer represents an area
* `[ZONE]`: Layer represents a zone class
* `[OBSTACLE]`: Layer represents an obstacle class

### Layer Attributes

Layers can have attributes defined in the description. They must be in the form:

```yaml
<key1=value1><key2=value1,value2,value3>
```

Depending on the type of layer (based on the tags), different attributes are relevant.

Some examples of attributes:

```yaml
<HEIGHT=1.2>  # Sets the height of objects in an OBSTACLE layer
<NAME=Pretty name>  # Sets the pretty name of a ZONE or AREA
```

### Obstacles (Walls, doors etc.)

Obstacles of a given class should be drawn as polylines on a layer, with the `[OBSTACLE]` tag in the layer name.
Closed polylines will become polygons extruded along the Z axis by the `HEIGHT` attribute in metres while unclosed
polylines will have a thickness built around the line. Thickness is determined by the `THICKNESS` attribute also
in metres.

The colour of the link in Gazebo can be set by setting either:

1. The colour of the individual entity
1. The colour of the entire layer, and choose "BY LAYER" for the entity's colour

The colour will be used to change the appearance of the link in the resulting `.world` file.

### Bollards

Bollards are treated specially and should be drawn as circles on a layer nammed "BOLLARDS". The radius should be th

### Nodes (traversal nodes)

Nodes shall be inserted as block references of the `Node` block. The `Node` block is defined in a separate file and
can be inserted by selecting the `Node.dxf` file when inserting a block reference.
The first time this is done, the block definition will be copied and embedded into the map file, allowing it to be
reused without having to re-import it. If the `Node` definition is updated,
the embedded definition can be updated by inserting a new `Node` reference and choosing the file again.

The `Node` block contains the following attributes to be filled out when inserting:
    *ID (string) - Node ID, must be unique
    * NAME (string) - Display name for the node (optional)

### Undirected edges (traversal layer)

Undirected edges shall be inserted as block references of the `Edge` block.
The `Edge` block is defined in a separate file and
can be inserted by selecting the `Edge.dxf` file when inserting a block reference.

The `Edge` block definition contains a single unit-length polyline. When inserting the block reference,
the scale/rotation of the **entire** block reference should be used to manipulate this line. This can be done
in ProgeCAD by first selecting the "start" node, then selecting the "to" node **twice**
(first time specifies scale, second time specifies rotation).

The connection to nodes is determined by the distance from the endpoint of the edge to the centre of the node's circle.
The circles nominally have a radius of 100mm (although they can be scaled when inserted),
which is the same as the threshold used to determine if an edge
connects to a node. This means if in the CAD, the edge is within the circle, it will be detected as a connection.
A useful tip is to use snapping when drawing the edges to make it snap perfectly to the circle centres.

The `Edge` block contains the following attributes to be filled out when inserting:

* BEHAVIOUR (string) - Name of the desired behaviour to activate when the agent is traversing this edge

### Directed edges (traversal layer)

Directed edges are inserted exactly the same way as undirected edges. The only difference is that the "DirectedEdge"
block should be used. This block contains a line that is thicker at the start and thinner at the end to make it
clear what direction it is.

### Area

Only La1 areas should have hatches. LaN (N >= 2) layers are only unions of La0 areas and therefore should be empty.

Areas should be defined as one or more hatches in separate DXF layers, with each DXF layer being an La0 area.
DXF layers that are areas **must** include the `[AREA]` tag in the layer name followed by the area's ID.

```yaml
[AREA]your_area_id spaces allowed
```

An La1 area can have more than 1 hatch, but each hatch can have **only one boundary** (ie. no islands/holes).
If the area has a hole, it should be split into two separate hatches. This is to prevent unexpected behaviour
when creating the area polygons.

Area layers have the following attributes, which can be added as layer attributes in the description of the layer:

* NAME (string, optional) - Display name to use. If not specified or empty, the area's ID will be used.\
* LEVEL (int, optional for level 1 areas, mandatory for 2+) - The level of the area
* CHILDREN (list(str), ignored for level 1 areas, mandatory for 2+) - IDs of children areas

### Higher level areas

Higher level areas can be defined using groups:

* Group Name becomes the new area's ID
* Entities should be the hatches of the included areas

The following attributes (in the group description) are used

* LEVEL (int >=2, required) - The level of the area
* NAME (str, optional) - display name of the area

#### Hatch boundary types

Hatch boundaries (paths) can be one of two types: `PolylinePath` or `EdgePath`. A polylinepath is just a closed
polyline while an edge path can consist of a combination of lines, arcs and splines. The CAD may generate either but
currently, only `PolylinePaths` are supported because areas and zones are saved as polygons.

ProgeCAD generates `PolylinePaths` by default when you use the Hatch tool if the boundary only consists of straight
lines. However, if you first generate a boundary using the boundary tool, then generate a hatch from the boundary,
it becomes an `EdgePath`, even if all the edges are lines. There is no way to tell if a hatch is a `PolylinePath`
or `EdgePath` in CAD but the import script will tell you if an `EdgePath` is detected.

### Zone

Zones are defined similarly to areas, except a [ZONE] tag should be added and a few extra attributes are
available.

```yaml
<COST=int> # Value from 0 to 255 specifying the cost of the zone on the costmap
<BEHAVIOUR=str> # String referring to the behaviour that should be used on this zone
```

As with areas, there can be multiple hatches in a zone layer, but the same attributes will be applied to all the
hatches.
