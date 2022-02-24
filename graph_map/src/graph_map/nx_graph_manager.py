from typing import Dict

import networkx as nx
from abc import abstractmethod
import json

from graph_map.util import SimpleDictProtocol, NxGraphManagerNode


class NxGraphManager:
    @abstractmethod
    def get_nodes(self) -> NxGraphManagerNode:
        raise NotImplementedError

    @abstractmethod
    def set_graph(self, graph: nx.Graph):
        raise NotImplementedError

    @abstractmethod
    def get_graph(self) -> nx.Graph:
        raise NotImplementedError

    def _to_json(self, node_key):
        g = nx.relabel_nodes(self.get_graph(), lambda x: x.id, copy=True)  # type: nx.DiGraph
        # Keep all of the Node data in a single dict
        g.add_nodes_from([(node.id, {node_key: node.to_simple_dict()}) for node in self.get_graph()])

        data = nx.adjacency_data(g)
        return json.dumps(data)

    def _write_graph(self, filepath, node_key):
        g = nx.relabel_nodes(self.get_graph(), lambda x: x.id, copy=True)  # type: nx.DiGraph
        # Keep all of the Node data in a single dict
        g.add_nodes_from([(node.id, {node_key: node.to_simple_dict()}) for node in self.get_graph()])

        data = nx.adjacency_data(g)
        with open(filepath, 'w') as fp:
            json.dump(data, fp, indent=4)

    @classmethod
    def _read_graph(cls, filepath: str, node_key: str, node_type: SimpleDictProtocol):
        """
        Reads a networkX graph saved using json_graph.adjacency_data
        :param filepath: Path to graph data
        :param node_key: key for the node data to be dumped into
        :param node_type: class of the node that implements from_dict
        :return:
        """

        gm = cls()

        with open(filepath, 'r') as fp:
            data = json.load(fp)
            g = nx.adjacency_graph(data)

        # gm.graph = g

        node_dict = dict()

        for node_name in list(g.nodes):
            # Need to add ID back into the dict because networkX removes it
            # NX assumes ID is used for node name
            node_data: Dict = g.nodes[node_name].pop(node_key)  # Remove the Node data
            new_node = node_type.from_dict({**node_data, 'id': node_name})
            node_dict[node_name] = new_node

        new_g = nx.relabel_nodes(g, node_dict)
        gm.set_graph(new_g)

        # Set the tree in each node to be the new one
        for node in gm.get_nodes().keys():
            node.set_graph(new_g)

        return gm

    @classmethod
    def _from_jsons(cls, json_str: str, node_key: str, node_type: SimpleDictProtocol):
        """
        Reads a networkX graph saved using json_graph.adjacency_data as a json string
        :param json_str: JSON string of the graph
        :param node_key: key for the node data to be dumped into
        :param node_type: class of the node that implements from_dict
        :return:
        """

        gm = cls()

        data = json.loads(json_str)
        g = nx.adjacency_graph(data)

        # gm.graph = g

        node_dict = dict()

        for node_name in list(g.nodes):
            # Need to add ID back into the dict because networkX removes it
            # NX assumes ID is used for node name
            node_data: Dict = g.nodes[node_name].pop(node_key)  # Remove the Node data
            new_node = node_type.from_dict({**node_data, 'id': node_name})
            node_dict[node_name] = new_node

        new_g = nx.relabel_nodes(g, node_dict)
        gm.set_graph(new_g)

        # Set the tree in each node to be the new one
        for node in gm.get_nodes().keys():
            node.set_graph(new_g)

        return gm
