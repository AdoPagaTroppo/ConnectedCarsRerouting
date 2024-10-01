from graph_util import build_graph
from node_file_parser import parse_file_for_nodes
from node_file_parser import parse_file_for_work
from node_file_parser import parse_file_for_checkpoints
import traci
import os

class Roundabout():
    
    def __init__(self,id,roads,mapdata=None,exits=None):
        self.id = id
        self.roads = roads
        self.exits = [] if exits is None else exits
        if exits is None:
            net = mapdata.net
            for edge in self.roads:
                o_outs = net.getEdge(edge).getOutgoing()
                outs = []
                if len(o_outs)>1:
                    for e in o_outs:
                        if net.getEdge(e.getID()).allows('passenger'):
                            outs.append(e)
                    for e in outs:
                        if e.getID() not in self.roads:
                            self.exits.append(e.getID())
    
    def exit_order(self,mapdata,path):
        net = mapdata.net
        starting_edge = None
        exiting_edge = None
        exit_id = 1
        for e in path:
            if e in self.roads and starting_edge is None:
                starting_edge = e
            if starting_edge is not None and e in self.roads:
                o_outs = net.getEdge(e).getOutgoing()
                outs = []
                if len(o_outs)>1:
                    for e in o_outs:
                        if net.getEdge(e.getID()).allows('passenger'):
                            outs.append(e)
                    if len(outs)>1:
                        for e in outs:
                            if e.getID() in self.exits:
                                if e.getID() not in path:
                                    exit_id += 1
                                else:
                                    break
        return exit_id
        
        

class MapData():
    
    def __init__(self,scenario):
        self.maxprio = 0
        self.minprio = 100
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
                prio = self.net.getEdge(e.getID()).getPriority()
                if prio > self.maxprio:
                    self.maxprio = prio
                if prio < self.minprio:
                    self.minprio = prio
                self.edgelist.append(e)
                self.edge2target[e.getID()] = {}
                for t in self.targets:
                    self.edge2target[e.getID()][self.targets[t]] = (False,False,[])
        print(len(self.edgelist)) 
        self.edge_dict = {}

        for e in self.edgelist:
            self.edge_dict[e.getID()] = self.edgelist.index(e)
        self.works = parse_file_for_work(str(scenario)+'ScenarioData/works.txt')
        self.roundabouts = None
        create_roundabouts = False
        if os.path.exists(str(scenario)+'ScenarioData/roundabouts.txt'):
            self.roundabouts = self.read_roundabouts(scenario)
        elif create_roundabouts:
            self.roundabouts = self.find_roundabouts()
        self.streets_in_roundabouts = {}
        for r in self.roundabouts:
            for road in r.roads:
                self.streets_in_roundabouts[road] = r.id
        traci.close()
        
    def check_roundabouts(self,calling_edge,current_edge,len_up_to_now):
        net = self.net
        start_length = net.getEdge(current_edge).getLength()
        if start_length+len_up_to_now>75 or net.getEdge(calling_edge).getPriority()!=net.getEdge(current_edge).getPriority(): 
            return None
        o_outs = net.getEdge(current_edge).getOutgoing()
        outs = []
        ret_val = None
        for e in o_outs:
            if net.getEdge(e.getID()).allows('passenger'):
                outs.append(e)
        for edge in outs:
            if edge.getID() == calling_edge and net.getEdge(calling_edge).getPriority()==net.getEdge(edge.getID()).getPriority():
                return [current_edge]
            ret_val = self.check_roundabouts(calling_edge,edge.getID(),start_length+len_up_to_now)
            if ret_val is not None:
                ret_new = [current_edge]
                ret_new.extend(ret_val)
                return ret_new
        return ret_val
    
    def find_roundabouts(self):
        roundabouts = []
        roads_in_roundabouts = []
        rs_index = 0
        for edge in self.edgelist:
            print('checking edge '+str(self.edgelist.index(edge)))
            if edge.getID() not in roads_in_roundabouts:
                ret_value = self.check_roundabouts(edge.getID(),edge.getID(),0)
                if ret_value is not None:
                    roads_in_roundabouts.extend(ret_value)
                    r = Roundabout('roundabout'+str(rs_index),ret_value,mapdata=self)
                    roundabouts.append(r)
                    self.write_roundabout(r)
                    rs_index += 1
        return roundabouts
    
    def write_roundabout(self,r):
        f = open('roundabouts_Unisa.txt','a')
        f.write(str(len(r.roads))+';')
        for road in r.roads:
            f.write(road+';')
        f.write(str(len(r.exits)))
        for ex in r.exits:
            f.write(';'+str(ex))
        f.write('\n')
        f.close()
            
    def read_roundabouts(self,scenario):
        f = open(str(scenario)+'ScenarioData/roundabouts.txt','r')
        content = f.readlines()
        f.close()
        roundabouts = {}
        roundabout_list = []
        index = 0
        for r in content:
            rid = 'r'+str(index)
            roundabouts[rid] = {}
            roundabouts[rid]['roads'] = []
            roundabouts[rid]['exits'] = []
            split_content = r.split(';')
            r_len = int(split_content[0])
            for i in range(r_len):
                roundabouts[rid]['roads'].append(split_content[1+i].replace('\n',''))
            e_len = int(split_content[1+r_len])
            for i in range(e_len):
                roundabouts[rid]['exits'].append(split_content[2+r_len+i].replace('\n',''))
            rb = Roundabout(rid,roundabouts[rid]['roads'],exits=roundabouts[rid]['exits'])
            roundabout_list.append(rb)
            index += 1
        return roundabout_list
        
        
    