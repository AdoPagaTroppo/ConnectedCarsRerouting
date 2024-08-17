from graph_util import build_graph
from node_file_parser import parse_file_for_nodes
from node_file_parser import parse_file_for_work
from node_file_parser import parse_file_for_checkpoints
import traci

class MapData():
    
    def __init__(self,scenario):
        self.scenario = scenario
        traci.start(['sumo','-c','osm_'+str(scenario)+'.sumocfg','--step-length','1.0'])
        self.graphdict, self.graphmap, self.net, self.connections, self.edgegraph = build_graph(scenario)
        self.destinations, self.sources = parse_file_for_nodes(str(scenario)+'ScenarioData/config.txt')
        self.target_weights = []
        self.targets = {}
        for t in self.destinations:
            self.target_weights.append(t[2])
            self.targets[t[1]] = t[0]
        edge_list = self.net.getEdges()
        self.edgelist = []
        self.edge2target = {}
        for e in edge_list:
            if e.allows('passenger'):
                self.edgelist.append(e)
                self.edge2target[e.getID()] = {}
                for t in self.targets:
                    self.edge2target[e.getID()][self.targets[t]] = (False,False,[])
        self.edge_dict = {}

        for e in self.edgelist:
            self.edge_dict[e.getID()] = self.edgelist.index(e)
        self.works = parse_file_for_work(str(scenario)+'ScenarioData/works.txt')
        traci.close()
    