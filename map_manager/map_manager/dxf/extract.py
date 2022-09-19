
# This script contains functions to extract entities of interest from a DXF floorplan

import logging
import os
import re
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union

import ezdxf
import numpy as np
from ezdxf.colors import DXF_DEFAULT_COLORS, int2rgb
from ezdxf.entities.boundary_paths import BoundaryPathType
from ezdxf.entities.circle import Circle
from ezdxf.entities.dxfgfx import DXFGraphic
from ezdxf.entities.hatch import Hatch
from ezdxf.entities.insert import Insert
from ezdxf.entities.layer import Layer
from ezdxf.entities.dxfgroups import DXFGroup, GroupCollection
from shapely.geometry import Polygon
from shapely.geometry.base import CAP_STYLE, JOIN_STYLE
from shapely.geometry.linestring import LineString

from graph_map.node import Edge, Node
from graph_map.area import Area, Region, Zone, Color

import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)


errors: List[str] = list()
warnings: List[str] = list()


def get_logs() -> Tuple[List, List]:
    global errors, warnings

    return errors, warnings


class Group:
    def __init__(self, id: str, level: int, children: Iterable[str], display_name: Optional[str] = None):
        self.id: str = id
        self.level: int = level
        self.children: Iterable[str] = children

        self.display_name: str = id if (display_name is None) else display_name

    def __repr__(self) -> str:
        return self.id


def get_attribute_with_default_warn(attributes_dict, attribute_key, default, type=Any, warning_prefix=''):
    global warnings

    if attribute_key in attributes_dict.keys():
        if isinstance(attributes_dict[attribute_key], type):
            return attributes_dict[attribute_key]
        else:
            msg = warning_prefix + \
                'Attribute {} must be a {}, found {} (type: {}). Using default of {}.'.format(
                    attribute_key, type, attributes_dict[attribute_key],
                    type(attributes_dict[attribute_key]), default)
            logger.warning(msg)
            warnings.append(msg)
            return default
    else:
        logger.info(warning_prefix + 'Attribute {} not provided. Using default of {}.'
                    .format(attribute_key, default))
        return default


def get_color(entity: DXFGraphic, layer: Layer) -> Tuple[float, float, float]:
    true_color: Optional[Tuple[int, int, int]] = entity.rgb
    if true_color is not None:
        return true_color

    else:
        color_code = entity.get_dxf_attrib('color')

        # Get color from layer if color is set to BYLAYER
        if color_code is None or color_code == 256:
            true_color = layer.rgb
            if true_color is not None:
                return true_color

            else:
                color_code = layer.get_dxf_attrib('color')
                return int2rgb(DXF_DEFAULT_COLORS[color_code])
        else:
            return int2rgb(DXF_DEFAULT_COLORS[color_code])


def get_transparency(entity: DXFGraphic, layer: Layer) -> float:
    if entity.dxf.hasattr('transparency'):
        return entity.transparency
    else:
        return layer.transparency


def thicken_polyline(points: List[Tuple[float, float]], thickness: float) -> List[Tuple[float, float]]:
    linestring: LineString = LineString(points)

    thickened = linestring.buffer(distance=thickness / 2.0, resolution=1,
                                  cap_style=CAP_STYLE.square, join_style=JOIN_STYLE.mitre)

    return [point for point in thickened.exterior.coords]


def find_closest_node(nodes: Iterable[Node], point: Tuple[float, float], threshold: float) -> Optional[Node]:
    closest_node: Node
    closest_dist = np.inf
    for node in nodes:
        dist_to_node = np.linalg.norm([point[0] - node.x, point[1] - node.y])
        if dist_to_node < closest_dist:
            closest_dist = dist_to_node
            closest_node = node

    if closest_dist <= threshold:
        return closest_node
    else:
        return None


# Layer name in the form "[Tag1][Tag2][Tag3]layer_name"
def split_layer_name(string: str) -> Tuple[str, List[str]]:
    tag_pattern = r"\[([\w ]+)\]"  # Any text in square brackets
    name_pattern = r"\[[\w ]+\]*\s*"  # Remove anthying in square brackets
    return re.sub(name_pattern, "", string), re.findall(tag_pattern, string)


# Matches anything of the form <key=value> in a string and return a dictionary of the attributes.
# Keys are all converted to lower case
def get_attributes(string: str) -> Dict:
    pattern = r"<([^=>]+=[^=>]+)>"
    attributes = re.findall(pattern, string)

    result: Dict[str, Any] = dict()
    for att in attributes:
        split = att.split('=')
        key = split[0]
        key = key.lower()
        value_str: str = split[1]

        try:
            value: Union[int, float, Any] = int(value_str)
        except ValueError:
            try:
                value = float(value_str)
            except ValueError:
                if value_str.lower() == 'true':
                    value = True
                elif value_str.lower() == 'false':
                    value = False
                else:
                    value = value_str

        # Turn repeated keys into a list
        if key in result.keys():
            if not isinstance(result[key], list):
                result[key] = [result[key]]

            result[key].append(value)
        else:
            result[key] = value

    return result


def load_doc(dxf_file: str):
    assert os.path.isfile(dxf_file)
    global errors

    logger.info('Loading DXF: {}'.format(dxf_file))
    try:
        return ezdxf.readfile(dxf_file)
    except IOError:
        msg = 'Not a DXF file or a generic I/O error.'
        logger.error(msg)
        errors.append(msg)
        return None
    except ezdxf.DXFStructureError:
        msg = 'Invalid or corrupted DXF file.'
        logger.error(msg)
        errors.append(msg)
        return None


def extract_obstacles(doc) -> List[Dict]:
    global warnings

    logger.info('Extracting obstacles...')
    msp = doc.modelspace()

    obstacles: List[Dict] = list()

    layers: Iterable[Layer] = doc.layers
    for layer in layers:
        height: float = 2.5
        thickness: float = 0.1

        obstacle_id, tags = split_layer_name(layer.get_dxf_attrib('name'))
        attributes = get_attributes(layer.description)

        if 'obstacles' in (tag.lower() for tag in tags):
            msg = 'Ignoring layer "{}" with "OBSTACLES" tag. Did you mean "OBSTACLE"?'.format(
                obstacle_id)
            logger.warning(msg)
            warnings.append(msg)

        if 'obstacle' in (tag.lower() for tag in tags):
            logger.info('Found obstacle layer: {}, Tags: {}, Attributes: {}'.format(
                obstacle_id, tags, attributes))

            # Get height from attributes
            height = get_attribute_with_default_warn(
                attributes, 'height', default=height, type=float,
                warning_prefix='Obstacle layer {}: '.format(obstacle_id))

            # Get thickness from attributes
            thickness = get_attribute_with_default_warn(
                attributes, 'thickness', default=height, type=float,
                warning_prefix='Obstacle layer {}: '.format(obstacle_id))

            # Get material from attributes
            material = ''
            if 'material' in attributes.keys():
                if isinstance(attributes['material'], str):
                    material = attributes['material']
                    logger.info('Obstacle layer {}: Using material: {}.'.format(
                        obstacle_id, material))
                else:
                    msg = 'Obstacle layer {}: material attribute must be a string.'.format(
                        obstacle_id)
                    logger.warning(msg)
                    warnings.append(msg)

            polylines: Iterable = msp.query('POLYLINE LWPOLYLINE [layer == "{}"]i'
                                            .format(layer.get_dxf_attrib('name')))
            if len(tuple(polylines)) == 0:
                msg = 'Obstacle layer {} has no POLYLINE or LWPOLYLINE entities'
                logger.warning(msg)
                warnings.append(msg)

            links = list()
            for i, line in enumerate(polylines):
                rgb = get_color(line, layer)
                transparency = get_transparency(line, layer)

                # Polyline may be POLYLINE or LWPOLYLINE depending on DXF version
                points: List[Tuple[float, float]]
                if line.dxftype() == 'POLYLINE':
                    points = [(p[0], p[1]) for p in line.points()]
                else:
                    points = [(p[0], p[1]) for p in line.get_points()]

                # If not closed then build a thickness around the polyline
                if not line.is_closed:
                    msg = 'Line {} is not closed. Adding thickness of {}m'.format(i, thickness)
                    logger.warning(msg)
                    warnings.append(msg)
                    points = thicken_polyline(points, thickness)

                # Check for bad polygons
                # simplify can fix some issues like duplicate points
                polygon = Polygon(points).simplify(1e-4)
                if not polygon.is_valid:
                    logger.warning('Obstacle {} contains an invalid polygon'.format(obstacle_id))
                    logger.warning(list(polygon.exterior.coords))
                    plt.plot(*polygon.exterior.xy)
                    plt.gca().set_aspect('equal', adjustable='box')
                    plt.show()
                else:
                    links.append({
                        'points': [
                            '{} {}'.format(p[0], p[1]) for p in points
                        ],
                        'material': material,
                        'color': {
                            'r': rgb[0] / 255.0,
                            'g': rgb[1] / 255.0,
                            'b': rgb[2] / 255.0,
                            'a': 1.0 - transparency
                        }
                    })

            obstacles.append({
                'id': obstacle_id,
                'height': height,
                'collision': True,
                'links': links
            })

            logger.info('Found {} obstacles in layer: {}'.format(
                len(links), obstacle_id))

    return obstacles


def extract_bollards(doc) -> List[Dict]:
    msp = doc.modelspace()
    bollards: List[Dict] = list()

    # Get all block references to Node blocks. i option ignores case.
    circle: Circle
    for circle in msp.query('CIRCLE[layer=="bollards"]i'):
        center = circle.get_dxf_attrib('center')
        radius = circle.get_dxf_attrib('radius')
        bollards.append({
            'center': '{} {}'.format(center[0], center[1]),
            'radius': radius
        })

    logger.info('Found {} bollards'.format(len(bollards)))
    return bollards


def extract_nodes(doc) -> Dict[str, Node]:
    global errors

    msp = doc.modelspace()
    nodes: Dict[str, Node] = dict()

    # Get all block references to Node blocks. i option ignores case.
    obj: Insert
    for obj in msp.query('INSERT[name=="node"]i'):
        id = obj.get_attrib_text('ID')
        name = obj.get_attrib_text('NAME')
        attributes_str = obj.get_attrib_text('ATTRIBUTES')
        attributes_dict = get_attributes(attributes_str)

        node_pos = None

        # Get the coordinates (in metres) of the circle
        node_pos = obj.dxf.insert[0], obj.dxf.insert[1]
        node_theta = obj.dxf.rotation * np.pi / 180.0

        if id is None or id == '':
            msg = 'Found node with no ID attribute'
            logger.error(msg)
            errors.append(msg)
        else:
            if name is None or name == '':
                name = id

            logger.debug('Found node, ID: {}, Display Name: {}, position: {}'.format(
                id, name, node_pos))

            if id in nodes.keys():
                msg = 'Duplicate node ID: {}, ignoring.'.format(id)
                logger.error(msg)
                errors.append(msg)
            else:
                node = Node(id=id, x=node_pos[0], y=node_pos[1], theta=node_theta, display_name=name,
                            attr=attributes_dict)
                nodes[id] = (node, attributes_dict)

    return nodes


def extract_edges(doc, nodes) -> Dict[Tuple[Node, Node], Tuple[Edge, Dict]]:
    global warnings
    msp = doc.modelspace()

    edges: Dict[Tuple[Node, Node], Tuple[Edge, Dict]] = dict()
    undirected_count: int = 0
    directed_count: int = 0

    # For edges, we check the endpoints to see if they are close to any nodes. Note that depending on the DXF version,
    # polylines may be saved as Light Weight Polylines instead (LWPOLYLINE)
    # Undirected edges
    for obj in msp.query('INSERT[name=="edge" | name=="directededge"]i'):
        attributes_str = obj.get_attrib_text('ATTRIBUTES')
        attributes_dict = get_attributes(attributes_str)

        # Get the endpoints of the polyline
        for entity in obj.virtual_entities():
            if entity.dxftype() == 'LWPOLYLINE':
                start = (entity.get_points()[0][0], entity.get_points()[0][1])
                end = (entity.get_points()[1][0], entity.get_points()[1][1])

            elif entity.dxftype() == 'POLYLINE':
                points = [(p[0], p[1]) for p in entity.points()]
                start = points[0]
                end = points[1]

            else:
                # Not the line element, ignore
                break

            # Compare start and end to nodes to find which one it is connected to
            node_finding_threshold = 0.05
            closest_start_node = find_closest_node(
                nodes, start, threshold=node_finding_threshold)
            if closest_start_node is None:
                msg = 'Could not find node within {}m of edge start point'
                logger.warning(msg)
                warnings.append(msg)
                break

            closest_end_node = find_closest_node(
                nodes, end, threshold=node_finding_threshold)
            if closest_end_node is None:
                msg = 'Could not find node within {}m of edge end point'
                logger.warning(msg)
                warnings.append(msg)
                break

            # Calculate weight
            length = closest_start_node.dist(closest_end_node)
            if 'weight' in attributes_dict.keys():
                # Absolute weight override provided, no need to do anything
                attributes_dict['weight'] = float(attributes_dict['weight'])
                logger.info('Edge ({} -> {}): Using weight override of: {:.3f}'
                            .format(closest_start_node.id, closest_end_node.id, attributes_dict['weight']))
            elif 'weight_mult' in attributes_dict.keys():
                logger.info('Edge ({} -> {}): Using weight multiplier: weight_mult ({:.3f}) * length ({:.3f}) = {:.3f}'
                            .format(closest_start_node.id, closest_end_node.id,
                                    attributes_dict['weight_mult'], length, attributes_dict['weight_mult'] * length))
                attributes_dict['weight'] = attributes_dict['weight_mult'] * length
            else:
                logger.info('Edge ({} -> {}): Using length as weight: {:.3f}'
                            .format(closest_start_node.id, closest_end_node.id, length))
                attributes_dict['weight'] = length

            # Undirected edge, so add two
            if obj.get_dxf_attrib('name').lower() == 'edge':
                undirected_count += 1
                logger.debug(
                    'Found undirected edge between nodes {} and {}'.format(closest_start_node.id, closest_end_node.id))
                edge = Edge(start=closest_start_node, end=closest_end_node)
                edges[(closest_start_node, closest_end_node)] = (edge, attributes_dict)
                edge = Edge(start=closest_end_node, end=closest_start_node)
                edges[(closest_end_node, closest_start_node)] = (edge, attributes_dict)
            # Directed edge
            else:
                directed_count += 1
                logger.debug(
                    'Found directed edge from {} to {}'.format(closest_start_node.id, closest_end_node.id))
                edge = Edge(start=closest_start_node, end=closest_end_node)
                edges[(closest_start_node, closest_end_node)] = (edge, attributes_dict)
            break

    logger.info('Found {} undirected edges and {} directed edges'.format(
        undirected_count, directed_count))

    return edges


def extract_areas(doc) -> Dict[str, Area]:
    global warnings, errors
    msp = doc.modelspace()

    areas: Dict[str, Area] = dict()

    layers: Iterable[Layer] = doc.layers
    for layer in layers:
        area_id, tags = split_layer_name(layer.get_dxf_attrib('name'))
        attributes = get_attributes(layer.description)

        if 'areas' in (tag.lower() for tag in tags):
            msg = 'Ignoring layer "{}" with "AREAS" tag. Did you mean "AREA"?'.format(
                area_id)
            logger.warning(msg)
            warnings.append(msg)

        if 'area' in (tag.lower() for tag in tags):
            logger.info('Found area layer: {}, Tags: {}, Attributes: {}'.format(
                area_id, tags, attributes))
            if 'name' in attributes.keys():
                display_name = attributes['name']
            else:
                display_name = area_id

            hatches: Iterable[Hatch] = msp.query(
                'HATCH [layer == "{}"]i'.format(layer.get_dxf_attrib('name')))
            if len(tuple(hatches)) == 0:
                msg = 'Area layer {} has no hatch entities. Ignoring.'.format(
                    area_id)
                logger.warning(msg)
                warnings.append(msg)
                break

            regions = list()
            for hatch in hatches:
                logger.info('Found HATCH in layer {}'.format(
                    layer.get_dxf_attrib('name')))
                rgb = get_color(hatch, layer)
                transparency = get_transparency(hatch, layer)

                if len(hatch.paths) > 1:
                    msg = 'Hatch contains multiple boundary paths. Only the first will be considered.'
                    logger.warning(msg)
                    warnings.append(msg)

                if hatch.paths[0].type == BoundaryPathType.POLYLINE:
                    points = hatch.paths[0].vertices
                    for point in points:
                        if point[2] != 0.0:
                            msg = 'Segment in region boundary has non-zero bulge factor. Bulge will be ' +\
                                'ignored and the segment treated as a straight line'
                            logger.warning(msg)
                            warnings.append(msg)

                    # An area may contain more than 1 region. Find it and add the region.
                    regions.append(Region(points=[(p[0], p[1]) for p in points],
                                   color=Color(r=rgb[0] / 255.0, g=rgb[1] / 255.0, b=rgb[2] / 255.0,
                                   a=1.0 - transparency)))

                # EdgePath
                else:
                    msg = 'Area {} contains an EdgePath. Only PolylinePaths are supported for hatch boundaries'.format(
                        area_id)
                    logger.error(msg)
                    errors.append(msg)

            # Check
            for region in regions:
                if not region.polygon.is_valid:
                    print('Area {} contains an invalid polygon'.format(area_id))
                    plt.plot(*region.polygon.exterior.xy)
                    plt.show()

            logger.info('Found Area {} with {} hatches'.format(
                area_id, len(regions)))
            areas[area_id] = Area(
                id=area_id, display_name=display_name, regions=regions)

    return areas


# Zone layers are marked with the [ZONE] tag. These layers may contain hatches or polylines.
# Zones will be added to the SDF has semi transparent colored links so they show up in sim
def extract_zones(doc) -> Dict[str, Zone]:
    global warnings, errors

    msp = doc.modelspace()

    zones: Dict[str, Zone] = dict()

    layers: Iterable[Layer] = doc.layers
    for layer_count, layer in enumerate(layers):
        thickness: float = 0.2

        zone_id, tags = split_layer_name(layer.get_dxf_attrib('name'))
        attributes = get_attributes(layer.description)

        if 'zones' in (tag.lower() for tag in tags):
            msg = 'Ignoring layer "{}" with "ZONES" tag. Did you mean "ZONE"?'.format(
                zone_id)
            logger.warning(msg)
            warnings.append(msg)

        if 'zone' in (tag.lower() for tag in tags):
            logger.info('Found zone layer: {}, Tags: {}, Attributes: {}'.format(
                zone_id, tags, attributes))

            # Get thickness from attributes
            if 'thickness' in attributes.keys():
                if isinstance(attributes['thickness'], float):
                    thickness = attributes['thickness']
                else:
                    msg = 'Zone layer {}: thickness attribute must be a float. \
                                Using default thickness of {}m'.format(zone_id, thickness)
                    logger.warning(msg)
                    warnings.append(msg)
            else:
                msg = 'Zone layer {}: No thickness provided. Using default thickness of {}m'.format(
                    zone_id, thickness)
                logger.warning(msg)
                warnings.append(msg)

            if 'name' in attributes.keys():
                display_name = str(attributes['name'])
            else:
                display_name = zone_id

            drivable = get_attribute_with_default_warn(
                attributes, 'drivable', default=True, type=bool,
                warning_prefix='Zone layer {}: '.format(zone_id))

            cost = get_attribute_with_default_warn(
                attributes, 'cost', default=0.0, type=float,
                warning_prefix='Zone layer {}: '.format(zone_id))

            if 'thickness' in attributes.keys():
                attributes.pop('thickness')
            if 'name' in attributes.keys():
                attributes.pop('name')
            if 'drivable' in attributes.keys():
                attributes.pop('drivable')
            if 'cost' in attributes.keys():
                attributes.pop('cost')

            regions = list()

            # Look for any HATCH entities
            hatches: Iterable[Hatch] = msp.query(
                'HATCH [layer == "{}"]i'.format(layer.get_dxf_attrib('name')))
            for hatch in hatches:
                logger.info('Found HATCH in layer {}'.format(
                    layer.get_dxf_attrib('name')))
                rgb = get_color(hatch, layer)
                transparency = get_transparency(hatch, layer)

                if len(hatch.paths) > 1:
                    msg = 'Hatch contains multiple boundary paths. Only the first will be considered.'
                    logger.warning(msg)
                    warnings.append(msg)

                if hatch.paths[0].type == BoundaryPathType.POLYLINE:
                    points = hatch.paths[0].vertices
                    for point in points:
                        if point[2] != 0.0:
                            msg = 'Segment in boundary has non-zero bulge factor. ' +\
                                'Bulge will be ignored and the segment treated as a straight line'
                            logger.warning(msg)
                            warnings.append(msg)

                    regions.append(Region(points=[(p[0], p[1]) for p in points],
                                          color=Color(r=rgb[0] / 255.0, g=rgb[1] / 255.0, b=rgb[2] / 255.0,
                                          a=1.0 - transparency)))

                # EdgePath
                else:
                    msg = 'Zone {} contains an EdgePath. Only PolylinePaths are supported for hatch boundaries'.format(
                        zone_id)
                    logger.error(msg)
                    errors.append(msg)

            # Now look for any POLYLINES or LWPOLYLINES
            polylines: Iterable = msp.query('POLYLINE LWPOLYLINE [layer == "{}"]i'
                                            .format(layer.get_dxf_attrib('name')))

            for line in polylines:
                rgb = get_color(line, layer)
                transparency = get_transparency(line, layer)

                # Polyline may be POLYLINE or LWPOLYLINE depending on DXF version
                if line.dxftype() == 'POLYLINE':
                    points = [([p[0], p[1]]) for p in line.points()]
                else:
                    points = [([p[0], p[1]]) for p in line.get_points()]

                # If not closed then build a thickness around the polyline
                if not line.is_closed:
                    msg = 'Line is not closed. Adding thickness of {}m'.format(
                        thickness)
                    logger.warning(msg)
                    warnings.append(msg)
                    points = thicken_polyline(points, thickness)

                regions.append(Region(points=[(p[0], p[1]) for p in points],
                                      color=Color(r=rgb[0] / 255.0, g=rgb[1] / 255.0, b=rgb[2] / 255.0,
                                      a=1.0 - transparency)))

            # Check
            for region in regions:
                if not region.polygon.is_valid:
                    print('Area {} contains an invalid polygon'.format(zone_id))
                    plt.plot(*region.polygon.exterior.xy)
                    plt.show()

            logger.info('Found {} polygons in layer {}'.format(
                len(regions), zone_id))
            zones[zone_id] = Zone(id=zone_id, display_name=display_name, regions=regions, drivable=drivable,
                                  cost=cost, attr=attributes)

    logger.info('Found a total of {} zones'.format(len(zones)))
    return zones


# Extract groups from the DXF (higher level areas) as Dicts
def extract_groups(doc) -> List[Group]:
    global errors

    group_collection: GroupCollection = doc.groups

    groups: List[Group] = list()
    group_id: str
    group: DXFGroup
    # mypy will complain about "group" being a DXFObject but it is also a DXFGroup
    for group_id, group in group_collection:    # type: ignore
        attributes = get_attributes(group.get_dxf_attrib('description'))
        logger.info('Found group: {}, attributes: {}'.format(
            group_id, attributes))

        # Check the level key is present
        if 'level' not in attributes.keys():
            msg = 'Group {} does not have a "level" attribute. Ignoring this group.'
            logger.error(msg)
            errors.append(msg)

            break
        else:
            if not isinstance(attributes['level'], int):
                msg = 'Group {} "level" is not an int. Ignoring this group.'
                logger.error(msg)
                errors.append(msg)

                break

        children = set()
        for entity in group:
            if entity.dxftype() == 'HATCH':
                area_id, _ = split_layer_name(entity.get_dxf_attrib('layer'))
                children.add(area_id)

        if 'name' in attributes.keys():
            display_name = str(attributes['name'])
        else:
            display_name = group_id

        groups.append(Group(
            id=group_id, level=attributes['level'], children=children, display_name=display_name))

    logger.info('Found {} groups'.format(len(groups)))
    return groups
