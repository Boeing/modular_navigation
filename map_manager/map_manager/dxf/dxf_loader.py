#!/usr/bin/env python3

# This script converts a DXF into a world file (SDF)

import argparse
import logging
import os
from typing import Dict, List, Optional, Tuple

import ezdxf.document
import jinja2

from graph_map.area import Area, Zone
from graph_map.area_manager import AreaManager
from graph_map.node import Edge, Node
from graph_map.node_graph_manager import NodeGraphManager
from graph_map.util import draw_node_graph, draw_area_graph
from map_manager.dxf.extract import extract_obstacles, extract_bollards, extract_nodes, extract_edges, extract_areas, \
    extract_zones, extract_groups, load_doc, Group

logger = logging.getLogger(__name__)
logging.getLogger('map_manager.dxf.extract').setLevel(logging.INFO)

DEFAULT_SDF_TEMPLATE = """<sdf version='1.6'>
    <world name='default'>

        <plugin name="disable_physics" filename="libgazebo_no_physics_plugin.so"></plugin>
        <plugin name="model_attachment" filename="libgazebo_model_attachment_plugin.so"></plugin>

        <light name='sun' type='directional'>
            <cast_shadows>1</cast_shadows>
            <pose frame=''>0 0 10 0 -0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.5 -1</direction>
        </light>

        <model name='ground_plane'>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>1000 1000</size>
                        </plane>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>100</mu>
                                <mu2>50</mu2>
                            </ode>
                            <torsional>
                                <ode/>
                            </torsional>
                        </friction>
                        <contact>
                            <ode/>
                        </contact>
                        <bounce/>
                    </surface>
                    <max_contacts>10</max_contacts>
                </collision>
                <visual name='visual'>
                    <cast_shadows>0</cast_shadows>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>1000 1000</size>
                        </plane>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Grey</name>
                        </script>
                    </material>
                    <transparency>0.5</transparency>
                </visual>
                <self_collide>0</self_collide>
                <kinematic>0</kinematic>
                <gravity>1</gravity>
            </link>
        </model>

        <gravity>0 0 -9.8</gravity>
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        <atmosphere type='adiabatic'/>
        <physics name='default_physics' default='1' type='ode'>
            <max_step_size>0.02</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>50</real_time_update_rate>
        </physics>
        <scene>
            <ambient>0.4 0.4 0.4 1</ambient>
            <background>0.7 0.7 0.7 1</background>
            <shadows>1</shadows>
        </scene>
        <spherical_coordinates>
            <surface_model>EARTH_WGS84</surface_model>
            <latitude_deg>0</latitude_deg>
            <longitude_deg>0</longitude_deg>
            <elevation>0</elevation>
            <heading_deg>0</heading_deg>
        </spherical_coordinates>

        {% for obstacle in obstacles %}
        <model name='{{ obstacle.id }}'>
            <static>1</static>
            <pose frame=''>0 0 0 0 0 0</pose>

            {% for link in obstacle.links %}
            <link name='{{ obstacle.id }}_{{ loop.index0 }}'>
                <visual name="{{ obstacle.id }}_{{ loop.index0 }}_visual">
                    <geometry>
                        <polyline>
                            {% for point in link.points %}
                            <point>{{ point }}</point>
                            {% endfor %}
                            <height>{{ obstacle.height }}</height>
                        </polyline>
                    </geometry>
                    <material>
                        {% if link.material != '' %}
                        <script>
                            <uri>file://media/materials/scripts</uri>
                            <uri>file://media/materials/textures</uri>
                            <name>{{ link.material }}</name>
                        </script>
                        {% else %}
                        <ambient>{{ link.color.r }} {{ link.color.g }} {{ link.color.b }} 1.0</ambient>
                        <diffuse>{{ link.color.r }} {{ link.color.g }} {{ link.color.b }} 1.0</diffuse>
                        {% endif %}
                    </material>
                    <transparency>{{ 1.0 - link.color.a }}</transparency>
                </visual>
                {% if obstacle.collision %}
                <collision name="{{ obstacle.id }}_{{ loop.index0 }}_collision">
                    <geometry>
                        <polyline>
                            {% for point in link.points %}
                            <point>{{ point }}</point>
                            {% endfor %}
                            <height>{{ obstacle.height }}</height>
                        </polyline>
                    </geometry>
                </collision>
                {% endif %}
            </link>
            {% endfor %}
        </model>
        {% endfor %}

        {% for zone in zones %}
        <model name='{{ zone.id }}'>
            <static>1</static>
            <pose frame=''>0 0 0 0 0 0</pose>

            {% for region in zone.regions %}
            <link name='{{ zone.id }}_{{ loop.index0 }}'>
                <visual name='{{ zone.id }}_{{ loop.index0 }}_visual'>
                    <geometry>
                        <polyline>
                            {% for point in region.points %}
                            <point>{{ point[0] }} {{ point[1] }}</point>
                            {% endfor %}
                            <height>0.0</height>
                        </polyline>
                    </geometry>
                    <material>
                        <ambient>{{ region.color.r }} {{ region.color.g }} {{ region.color.b }} 1.0</ambient>
                        <diffuse>{{ region.color.r }} {{ region.color.g }} {{ region.color.b }} 1.0</diffuse>
                    </material>
                    <transparency>{{ 1.0 - region.color.a }}</transparency>
                </visual>
            </link>

            {% endfor %}
        </model>
        {% endfor %}

        <model name='bollards'>
            <static>1</static>
            <pose frame=''>0 0 0 0 0 0</pose>

            {% for bollard in bollards %}
            <link name='bollard_{{ loop.index0 }}'>
                <pose frame=''>{{ bollard.center }} 0 0 0 0</pose>
                <visual name='bollard_{{ loop.index0 }}_visual'>
                    <pose frame=''>0 0 0.3 0 0 0</pose>
                    <geometry>
                        <cylinder>
                            <radius>{{ bollard.radius }}</radius>
                            <length>0.6</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Yellow</name>
                        </script>
                    </material>
                </visual>
                <collision name='bollard_{{ loop.index0 }}_collision'>
                    <pose frame=''>0 0 0.3 0 0 0</pose>
                    <geometry>
                        <cylinder>
                            <radius>{{ bollard.radius }}</radius>
                            <length>0.6</length>
                        </cylinder>
                    </geometry>
                    <laser_retro>9000.0</laser_retro>
                </collision>
            </link>
            {% endfor %}
        </model>

        <state world_name='default'>

            {% for obstacle in obstacles %}
            <model name='{{ obstacle.id }}'>
                <pose frame=''>0 0 0 0 0 0</pose>
            </model>
            {% endfor %}

            {% for zone in zones %}
            <model name='{{ zone.id }}'>
                <pose frame=''>0 0 0 0 0 0</pose>
            </model>
            {% endfor %}

            <model name='bollards'>
                <pose frame=''>0 0 0 0 0 0</pose>
            </model>

            <model name='ground_plane'>
                <pose frame=''>0 0 0 0 -0 0</pose>
            </model>

            <light name='sun'>
                <pose frame=''>0 0 10 0 -0 0</pose>
            </light>

        </state>

        <gui fullscreen='0'>
            <camera name='user_camera'>
                <pose frame=''>35.2381 -80.333 45.8663 0 0.641796 1.64418</pose>
                <view_controller>orbit</view_controller>
                <projection_type>perspective</projection_type>
            </camera>
        </gui>
    </world>
</sdf>
"""


def build_area_tree(areas: Dict[str, Area], groups: List[Group], filename: str = 'root') -> AreaManager:
    am = AreaManager()
    am.add_areas(areas.values())

    highest_level: int = 1
    group_levels = dict()
    # First add all groups as nodes
    for group in groups:
        if group.level > highest_level:
            highest_level = group.level

        group_levels[group.id] = group.level
        logger.info('Adding group area: {}'.format(group.id))
        am.add_area(Area(id=group.id, display_name=group.display_name))

    # Strategy
    # 1. Go through each level in ascending order starting from level 2.
    # 2. Check each group (explicit nodes) for children and create those edges
    for i in range(2, highest_level + 1):
        for group in groups:
            if group.level == i:
                for child in group.children:
                    # The groups only store the leaf nodes, so we need to the find the child that is exactly 1 level
                    # lower than the current node/group
                    child_area = am.area_by_id(child)
                    immediate_child = child_area.ancestor(generations=i - 2)
                    immediate_child.parent = am.area_by_id(group.id)

        # Now look for any nodes in the layer below that don't have parents and create a parent for it
        area: Area
        for id, area in am.area_ids.items():
            # Get the true level of groups because these will have a level of 1 until they are linked up.
            if id in group_levels.keys():
                level = group_levels[id]
            else:
                level = area.level

            if level == i - 1:
                if id in group_levels.keys():
                    if group_levels[id] != i - 1:
                        break

                if area.parent is None:
                    pseudo_area = Area(id='{}*'.format(id), display_name=area.display_name)
                    logger.info('Adding pseudo_area: {}'.format(pseudo_area.id))
                    am.add_area(pseudo_area)
                    area.parent = pseudo_area

    # Now add a root node if necessary
    if len(am.get_areas(level=highest_level)) > 1:
        root_area = Area(id=filename, display_name=filename)
        am.add_area(root_area)

        print('ADDING ROOT AREA NODE')
        for area in am.get_areas(level=highest_level).keys():
            if area is not root_area:
                area.parent = root_area

    # Validate the tree
    issues: Dict[str, List] = am.validate()
    for error in issues['Error']:
        logger.error(error)
    for warning in issues['Warning']:
        logger.warning(warning)

    return am


class DxfLoader:
    def __init__(self, dxf_file: Optional[str] = None):
        self.obstacles: List[Dict] = list()
        self.bollards: List[Dict] = list()
        self.nodes: Dict[str, Node] = dict()
        self.edges: Dict[Tuple[Node, Node], Tuple[Edge, Dict]] = dict()
        self.areas: Dict[str, Area] = dict()
        self.zones: Dict[str, Zone] = dict()
        self.groups: List[Group] = list()

        self.dxf_name = str()

        self.doc: Optional[ezdxf.document.Drawing] = None

        if dxf_file is not None:
            self.load(dxf_file=dxf_file)

    def load(self, dxf_file: str):
        self.dxf_name = os.path.splitext(os.path.basename(dxf_file))[0]
        self.doc = load_doc(dxf_file)

        if self.doc is not None:
            self.obstacles = extract_obstacles(self.doc)
            self.bollards = extract_bollards(self.doc)
            self.nodes = extract_nodes(self.doc)
            # Each value is a tuple of (Node, attributes_dict)
            self.edges = extract_edges(self.doc, [node_tuple[0] for node_tuple in self.nodes.values()])
            self.areas = extract_areas(self.doc)
            self.zones = extract_zones(self.doc)
            self.groups = extract_groups(self.doc)

    def write_sdf(self, output_file: str, template_file: Optional[str] = None):
        if template_file is not None:
            template_loader = jinja2.FileSystemLoader(searchpath=os.path.dirname(template_file))
            template_env = jinja2.Environment(loader=template_loader)
            template = template_env.get_template(os.path.basename(template_file))
        else:
            template = jinja2.Environment(
                loader=Optional[jinja2.BaseLoader]  # type: ignore
            ).from_string(DEFAULT_SDF_TEMPLATE)

        zones_dicts = [zone.to_simple_dict() for zone in self.zones.values()]
        zones_dicts += [area.to_simple_dict() for area in self.areas.values()]

        out = template.render(bollards=self.bollards, obstacles=self.obstacles, zones=zones_dicts)

        with open(output_file, 'w') as f:
            f.write(out)

        logger.info('Wrote SDF to {}'.format(output_file))

    def parse_dxf(self):
        gm = NodeGraphManager()
        gm.add_nodes(list(self.nodes.values()))
        gm.add_edges(list(self.edges.values()))
        gm.set_area_manager(self.__get_area_tree())
        return gm

    def __get_area_tree(self):
        am = build_area_tree(areas=self.areas, groups=self.groups, filename=self.dxf_name)
        return am


if __name__ == '__main__':
    logger.addHandler(logging.StreamHandler())
    logger.setLevel(logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument('dxf', help='DXF file')
    parser.add_argument('--sdf_template', help='SDF template file')
    parser.add_argument('--sdf', default='out.world', help='SDF output file')

    args = parser.parse_args()

    loader = DxfLoader(dxf_file=args.dxf)

    gm = loader.parse_dxf()
    draw_node_graph(gm.nodes, positions=True, show=True)

    am = gm.area_manager
    draw_area_graph(gm.area_manager.tree, show=True)
