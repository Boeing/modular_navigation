import matplotlib.pyplot as plt
import numpy as np

from graph_map.node import Node
from graph_map.node_graph_manager import NodeGraphManager
from graph_map.util import draw_node_graph, PoseLike
from graph_planner.navigation_behavior.behavior_selector import BehaviourSelector
from graph_planner.navigation_behavior.behaviors import NodeFollow
from graph_planner.navigation_behavior.behaviors.behavioral_policy import SharedData
from graph_planner.navigation_behavior.behaviors.node_follow import lin_interp, pose_dist
from graph_planner.navigation_behavior.behaviors.policy.status import Status
from map_manager.dxf.dxf_loader import DxfLoader

if __name__ == '__main__':
    fname = './south_prep_x4_graph.dxf'

    # fname = '../../../../map_manager/dxf/sample.dxf'  # TODO: temp
    loader = DxfLoader(dxf_file=fname)

    gm: NodeGraphManager = loader.parse_dxf()

    zones = list(loader.zones.values())

    for zone in zones:
        print(zone.display_name)

    # draw_node_graph(gm.graph, positions=True, show=True)

    start: Node = gm.node_ids['west_annex_2']
    start_pos = PoseLike(9, 31)
    end: Node = gm.node_ids['south_prep_east_3']
    end_pos = PoseLike(54, 12)

    path_nodes = gm.shortest_path(start, end)  # using nodes to calc
    path_pos = gm.shortest_path(start_pos, end_pos)

    robot_poss = list()
    goal_poss = list()

    print('start area: {} | {}, end area: {} | {}'.format(start.area, start.pos, end.area, end.pos))

    print('equiv: {}'.format(path_nodes == path_pos))

    ds = SharedData(gm=gm, zones=zones, path=path_nodes, goal=end_pos)

    # Play with NextNode
    nn = NodeFollow(shared_data=ds)
    cur_pos = start_pos

    success = False
    while not success:
        for zone in zones:
            if zone.is_in(cur_pos):
                print(zone.display_name)

        outcome = nn.calculate_waypoint(cur_pos)

        success = outcome.status == Status.SUCCESS
        new_goal = outcome.waypoint

        if not success:
            robot_poss.append((cur_pos.x, cur_pos.y))
            goal_poss.append((new_goal.pose.x, new_goal.pose.y))
            if new_goal is None:
                print('Failed to calc path')
                break
            print('Cur_pos: {}, new_goal: {}'.format(str(cur_pos), str(new_goal)))
            max_dist = 0.1
            if pose_dist(cur_pos, new_goal.pose) > max_dist:
                cur_pos = lin_interp(cur_pos, new_goal.pose, max_dist)
            else:
                cur_pos = new_goal.pose

    print('Finished next node chasing')
    robo = np.array(robot_poss)
    goalss = np.array(goal_poss)

    draw_node_graph(gm.graph, positions=True, show=False)

    plt.plot(goalss[:, 0], goalss[:, 1], 'go-', label='goal position')
    plt.plot(robo[:, 0], robo[:, 1], 'ro-', label='robot position')
    plt.axis('equal')
    plt.show()

    # Play with behavior selector
    behavior_selector = BehaviourSelector()

    behavior_selector.update_map_data(gm, zones)
