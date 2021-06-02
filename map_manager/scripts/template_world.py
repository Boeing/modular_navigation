#!/usr/bin/env python3

import argparse
import logging
import math
import os
import uuid
from matplotlib.path import Path

import ezdxf
import jinja2
import numpy
from ezdxf.legacy.graphics import Circle
from ezdxf.legacy.polyline import Polyline
from numpy import dot, empty_like

logger = logging.getLogger(__name__)


def make_path(x1, y1, x2, y2):
    return Path([[x1, y1], [x1, y2], [x2, y2], [x2, y1]])


def perp(a):
    b = empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return
def seg_intersect(a1, a2, b1, b2):
    da = a2 - a1
    db = b2 - b1
    dp = a1 - b1
    dap = perp(da)
    denom = dot(dap, db)
    num = dot(dap, dp)

    x3 = ((num / denom.astype(float)) * db + b1)[0]
    y3 = ((num / denom.astype(float)) * db + b1)[1]
    p1 = make_path(a1[0], a1[1], a2[0], a2[1])
    p2 = make_path(b1[0], b1[1], b2[0], b2[1])
    if p1.contains_point([x3, y3], radius=0.1) and p2.contains_point([x3, y3], radius=0.1):
        return x3, y3
    else:
        return False


def get_offset_points(p1, p2, thickness):
    vec = p2 - p1
    norm = numpy.array([vec[1], -vec[0]])
    norm /= numpy.linalg.norm(norm)
    return (p1 + norm * thickness), (p1 - norm * thickness)


def run(dxf_file, world_template_file, output_sdf_file):
    # type: (str, str, str) -> None

    assert os.path.isfile(dxf_file)
    assert os.path.isfile(world_template_file)

    logger.info('Loading DXF: {}'.format(dxf_file))
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()

    bollards = []
    walls = []
    raw_paths = []
    markers = []

    thickness = 20

    for obj in msp.query('*'):
        layer = obj.get_dxf_attrib('layer')

        # Convert all strings to lowercase. Sometimes CAD packages save as uppercase.
        layer = layer.lower()

        if isinstance(obj, Polyline):

            if layer == 'paths':
                points = [p for p in obj.points()]
                raw_paths.append(points)

            elif layer.startswith('pose_'):
                points = [(p[0] / 1000.0, p[1] / 1000.0) for p in obj.points()]
                assert (len(points) == 2)
                dx = points[1][0] - points[0][0]
                dy = points[1][1] - points[0][1]
                markers.append({
                    'name': '{}'.format(layer[5:]),
                    'center': '{} {}'.format(points[0][0], points[0][1]),
                    'rotation': math.atan2(dy, dx)
                })

            else:
                points = [numpy.array([p[0], p[1]]) for p in obj.points()]
                points.reverse()

                # If not closed then build a thickness around the polyline
                if not obj.is_closed:

                    top_points = []
                    bottom_points = []

                    start_top, start_bottom = get_offset_points(points[0], points[1], thickness)
                    end_bottom, end_top = get_offset_points(points[-1], points[-2], thickness)

                    if len(points) == 2:

                        points = list()
                        points.append(start_bottom)
                        points.append(start_top)
                        points.append(end_top)
                        points.append(end_bottom)
                    else:
                        loop_top = (start_top + end_bottom) / 2.0
                        loop_bottom = (start_bottom + end_top) / 2.0

                        top_points.append(loop_top)
                        bottom_points.append(loop_bottom)

                        for i in range(1, len(points) - 1):
                            bw_top, bw_bottom = get_offset_points(points[i], points[i - 1], thickness)
                            fw_bottom, fw_top = get_offset_points(points[i], points[i + 1], thickness)

                            top = (bw_top + fw_top) / 2.0
                            btm = (bw_bottom + fw_bottom) / 2.0

                            top_points.append(top)
                            bottom_points.append(btm)

                        top_points.append(loop_top)
                        bottom_points.append(loop_bottom)

                        bottom_points.reverse()

                        points = top_points
                        points += bottom_points

                height = 2.5
                has_collision = True

                if layer == 'walls':
                    material_name = 'Gazebo/Blue'
                elif layer == 'doors':
                    material_name = 'Gazebo/Orange'
                elif layer == 'roller_doors':
                    material_name = 'Gazebo/Black'
                elif layer == 'fence':
                    material_name = 'Gazebo/Orange'
                    height = 1.2
                elif layer == 'free_space':
                    material_name = 'Gazebo/White'
                    height = 0.01
                    has_collision = False
                elif layer == 'drivable_zone':
                    material_name = 'Gazebo/Green'
                    height = 0.02
                    has_collision = False
                elif layer == 'avoid_zone':
                    material_name = 'Gazebo/Orange'
                    height = 0.03
                    has_collision = False
                elif layer == 'exclusion_zone':
                    material_name = 'Gazebo/Red'
                    height = 0.03
                    has_collision = False
                else:
                    material_name = 'Gazebo/Grey'

                walls.append({
                    'name': '{}_{}'.format(layer, len(walls)),
                    'layer': layer,
                    'points': [
                        '{} {}'.format(p[0] / 1000.0, p[1] / 1000.0) for p in points
                    ],
                    'material_name': material_name,
                    'height': height,
                    'collision': has_collision
                })

        elif isinstance(obj, Circle):
            center = obj.get_dxf_attrib('center')
            radius = obj.get_dxf_attrib('radius')
            bollards.append({
                'name': 'bollard_{}'.format(len(bollards)),
                'center': '{} {}'.format(center[0] / 1000.0, center[1] / 1000.0),
                'radius': radius / 1000.0
            })
        else:
            pass

    logger.info('Found {} paths'.format(len(raw_paths)))
    logger.info('Found {} bollards'.format(len(bollards)))
    logger.info('Found {} walls'.format(len(walls)))
    logger.info('Found {} markers'.format(len(markers)))

    nodes = {}
    paths = []

    # Create nodes for all paths
    # Update paths using node ids
    for path in raw_paths:
        new_path = []
        for point in path:
            node_id = str(uuid.uuid4())
            nodes[node_id] = (point[0] / 1000.0, point[1] / 1000.0)
            new_path.append(node_id)
        paths.append(new_path)

    # Determine unique overlapping pairs
    overlapping_pairs = set()
    for path_i, path in enumerate(paths):
        for n_path_i, n_path in enumerate(paths):
            if path_i == n_path_i:
                continue
            for node_id in path:
                for n_node_id in n_path:
                    distance = numpy.linalg.norm(numpy.array(nodes[node_id]) - numpy.array(nodes[n_node_id]))
                    if distance < 0.1:
                        ol_pair = tuple(sorted((node_id, n_node_id)))
                        overlapping_pairs.add(ol_pair)

    # Compute an overlapping dictionary
    top_nodes = {}
    olp = sorted(list(overlapping_pairs))
    for ol_pair in olp:
        f_is_top = ol_pair[0] in top_nodes
        s_is_top = ol_pair[1] in top_nodes
        if f_is_top and s_is_top:
            top_nodes[ol_pair[0]].update(top_nodes[ol_pair[1]])
            top_nodes[ol_pair[0]].add(ol_pair[1])
            del top_nodes[ol_pair[1]]
        elif s_is_top:
            top_nodes[ol_pair[1]].add(ol_pair[0])
        elif f_is_top:
            top_nodes[ol_pair[0]].add(ol_pair[1])
        else:
            top_nodes[ol_pair[0]] = {ol_pair[1]}
    for ol_pair in olp:
        f_is_top = ol_pair[0] in top_nodes
        s_is_top = ol_pair[1] in top_nodes
        if f_is_top and s_is_top:
            top_nodes[ol_pair[0]].update(top_nodes[ol_pair[1]])
            top_nodes[ol_pair[0]].add(ol_pair[1])
            del top_nodes[ol_pair[1]]

    # Remove overlapping nodes
    for top, child_node_ids in top_nodes.items():
        for node_id in child_node_ids:
            del nodes[node_id]
            paths = [
                [ni if ni != node_id else top for ni in path]
                for path in paths
            ]

    for i, path in enumerate(paths):
        paths[i] = {
            'nodes': path
        }

    nodes = [
        {
            'id': n,
            'x': point[0],
            'y': point[1]
        }
        for n, point in nodes.items()
    ]

    template_loader = jinja2.FileSystemLoader(searchpath=os.path.dirname(world_template_file))
    template_env = jinja2.Environment(loader=template_loader)
    template = template_env.get_template(os.path.basename(world_template_file))

    out = template.render(bollards=bollards, walls=walls, nodes=nodes, paths=paths, markers=markers)

    with open(output_sdf_file, 'w') as f:
        f.write(out)

    logger.info('Done!')


if __name__ == '__main__':
    logger.addHandler(logging.StreamHandler())
    logger.setLevel(logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument('dxf', help='DXF file')
    parser.add_argument('world_template', help='SDF template')
    parser.add_argument('--sdf', default='world.sdf', help='SDF output location')

    args = parser.parse_args()

    run(dxf_file=args.dxf, world_template_file=args.world_template, output_sdf_file=args.sdf)
