import numpy as np
import traci
import os
import sys
import sumolib
from sumolib.net import readNet
from algorithms import connection_exists
from node_file_parser import parse_file_for_nodes
from graph_util import build_graph
from algorithms import build_path

def valid_neighbors(edge,graphdict,connections,target=None):
    #get only valid turns for the given edge
    res = []
    n1 = edge.getToNode()
    for e in n1.getOutgoing():
        if e.getID()!=edge.getID() and e.allows('passenger') and connection_exists(edge.getFromNode().getID(),e.getFromNode().getID(),e.getToNode().getID(),graphdict,connections):
            if (target is not None and len(traci.simulation.findRoute(e.getID(),target).edges)>0) or target is None:
                res.append(e)
    return(res)

def valid_neighbors_lengths(edges,target=None):
    #get only valid turns for the given edge
    res = {}
    for e in edges:
        res[e] = traci.simulation.findRoute(e.getID(),target).length
    return(res)

def index2alg(index):
    if index==0:
        return 'dijkstra'
    if index==1:
        return 'astar'
    if index==2:
        return 'bfs'
    if index==3:
        return 'greedybfs'
    
def create_behaviours(num_algs):

    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")


    # SALERNO NET ------------
    net_name = "fiscianobaronissi.net.xml"
    sumocfg_name = "osm4.sumocfg"
    behaviors_name = "test_behaviours.npy"
    dests, sources = parse_file_for_nodes('config.txt')
    destnodes, s = parse_file_for_nodes('config_nodes.txt')
    targets = []
    for d in dests:
        targets.append(d[0])
    # targets = ['330222144','-40567772#0','330224632','1084284613','84679693','620934337#2']
    # targets = ['1084284613']
    #targets are parking spaces
    # targets = ['672732730', '-672732731','-672732728', '401420254#0','-565381618#0', '401420254#4', '-1001198066',
    #           '744932253', '670934106', '670934108', '160821659#2', '671983008', '92961457#3', '92961457#4', 
    #           '92961462', '672273418#3', '673737658#3', '673737658#6', '-565381618#0', '163563222', 'E5', 'E6'] 

    #Choose between these lines for GUI or not
    #sumoBinary = sumolib.checkBinary('sumo-gui')
    sumoBinary = sumolib.checkBinary('sumo')

    sumoCmd = [sumoBinary, "-c", sumocfg_name,"--step-length","0.05"] #The last parameter is the step size, has to be small
    traci.start(sumoCmd)
    print("Starting SUMO...")
    graphdict, graphmap, net, connections = build_graph()
    # print(graphdict['932314595'])
    #read road network
    edgelist = net.getEdges()
    edge_list = []
    for edge in edgelist:
        if edge.allows('passenger'):
            edge_list.append(edge)
    #Dictionary of list IDs
    edge_dict = {}

    for e in edge_list:
        edge_dict[e.getID()] = edge_list.index(e)

    behaviors = np.zeros((len(targets)*num_algs, len(edge_list), len(edge_list)))

    i=0
    for t in targets:
        pmfs = np.zeros((len(edge_list), len(edge_list)))
        endnode = net.getEdge(t).getToNode().getID()
        print(endnode)
        for j in range(num_algs):
            for e in edge_list: 
                route = []
                if j == 0:
                    route = traci.simulation.findRoute(e.getID(), t).edges
                else:
                    route = traci.simulation.findRoute(e.getID(), t, "routerByDistance").edges
                pmf = [0]*((len(edge_list)))
                
                if route is None or len(route) ==0 : # edges are not connected 
                    for x in range(len(pmf)):
                        pmf[x] = 0
                
                elif(len(route) == 1):
                    e_prime = e.getID()
                    pmf[edge_dict[e_prime]] = 1.0
                    # prob = 0.95 
                    
                    # pmf[edge_dict[e_prime]] = 0.95  
                    # valid_edges = valid_neighbors(e,graphdict,connections)
                        
                    # for v in valid_edges:
                    #     if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                    #         pmf[edge_dict[v.getID()]] = (1-prob) / len(valid_edges)
                else:
                    
                    e_prime = route[1] # e' is the edge just after e 

                    prob = 0.95
                    pmf[edge_dict[e_prime]] = prob

                    valid_edges = valid_neighbors(e,graphdict,connections,t)

                    for v in valid_edges:
                        if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                            pmf[edge_dict[v.getID()]] = (1-prob) / len(valid_edges)

                pmf = np.array(pmf)
                        
                if np.sum(pmf) !=0:
                    pmf = pmf/np.sum(pmf)
                        
                pmfs[edge_dict[e.getID()]] = pmf
                    
            behaviors[i] = pmfs


            i+=1
    print("Behaviors generated!")
    traci.close()
    np.save(behaviors_name, behaviors)
    
def online_make_behaviours(num_algs,target_edge,state_space,edgelist,graphdict,connections,net):
    # print(graphdict['932314595'])
    #read road network
    edge_list = []
    for s in state_space:
        edind = s
        print(edind)
        edge_list.append(edind)
        for v in valid_neighbors(edind,graphdict,connections,target_edge):
            edge_list.append(v)
    #Dictionary of list IDs
    edge_dict = {}

    for e in edge_list:
        edge_dict[e.getID()] = edge_list.index(e)

    behaviors = np.zeros((num_algs, len(edge_list), len(edge_list)))

    i=0
    pmfs = np.zeros((len(edge_list), len(edge_list)))
    for j in range(num_algs):
        for e in state_space: 
            route = []
            if j == 0:
                route = traci.simulation.findRoute(e.getID(), target_edge).edges
            else:
                route = traci.simulation.findRoute(e.getID(), target_edge, "routerByDistance").edges
            pmf = [0]*((len(edge_list)))
            
            if route is None or len(route) ==0 : # edges are not connected 
                for x in range(len(pmf)):
                    pmf[x] = 0
            
            elif(len(route) == 1):
                e_prime = e.getID()
                pmf[edge_dict[e_prime]] = 1.0
                # prob = 0.95 
                
                # pmf[edge_dict[e_prime]] = 0.95  
                # valid_edges = valid_neighbors(e,graphdict,connections)
                    
                # for v in valid_edges:
                #     if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                #         pmf[edge_dict[v.getID()]] = (1-prob) / len(valid_edges)
            else:
                
                e_prime = route[1] # e' is the edge just after e 

                prob = 0.95 
                pmf[edge_dict[e_prime]] = prob

                valid_edges = valid_neighbors(e,graphdict,connections,target_edge)

                for v in valid_edges:
                    if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                        pmf[edge_dict[v.getID()]] = (1-prob) / len(valid_edges)
                

            pmf = np.array(pmf)
                    
            if np.sum(pmf) !=0:
                pmf = pmf/np.sum(pmf)
                    
            pmfs[edge_dict[e.getID()]] = pmf
                
        behaviors[i] = pmfs


        i+=1
    print("Behaviors generated!")
    return behaviors
    
def online_create_behaviours(edge_list,targets,num_algs,target_edge,graphdict,connections,ss_edges,graphmap):
    #Dictionary of list IDs
    edge_dict = {}

    for e in edge_list:
        edge_dict[e.getID()] = edge_list.index(e)

    behaviors = np.zeros((len(targets)*num_algs, len(edge_list), len(edge_list)))

    i=0
    for t in targets.values():
        pmfs = np.zeros((len(edge_list), len(edge_list)))
        for j in range(num_algs):
            if t == target_edge:
                for e in ss_edges:
                    route = []
                    if j == 0:
                        route = traci.simulation.findRoute(e.getID(), t).edges
                    elif j == 1:
                        route = traci.simulation.findRoute(e.getID(), t, "routerByDistance").edges
                    else:
                        route = build_path(graphdict,ss_edges[0].getFromNode().getID(),edge_list[edge_dict[target_edge]].getToNode().getID(),index2alg(j),graphmap=graphmap,connections=connections)
                    pmf = [0]*((len(edge_list)))
                    
                    if route is None or len(route) ==0 : # edges are not connected 
                        for x in range(len(pmf)):
                            pmf[x] = 0
                    
                    elif(len(route) == 1):
                        e_prime = e.getID()
                        pmf[edge_dict[e_prime]] = 1.0
                        # prob = 0.95 
                        
                        # pmf[edge_dict[e_prime]] = 0.95  
                        # valid_edges = valid_neighbors(e,graphdict,connections)
                            
                        # for v in valid_edges:
                        #     if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                        #         pmf[edge_dict[v.getID()]] = (1-prob) / len(valid_edges)
                    else:
                        
                        e_prime = route[1] # e' is the edge just after e 

                        prob = 0.97 
                        pmf[edge_dict[e_prime]] = prob

                        valid_edges = valid_neighbors(e,graphdict,connections,t)
                        # valid_lengths = valid_neighbors_lengths(valid_edges,t)
                        # totlength = sum(valid_lengths.values())
                        for v in valid_edges:
                            if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                                pmf[edge_dict[v.getID()]] = (1-prob) / len(valid_edges)
                            # pmf[edge_dict[v.getID()]] = (totlength-valid_lengths[v])/totlength if totlength!=valid_lengths[v] else 1.0
                            # print(str(v)+" "+str(pmf[edge_dict[v.getID()]]))

                    pmf = np.array(pmf)
                            
                    if np.sum(pmf) !=0:
                        pmf = pmf/np.sum(pmf)
                            
                    pmfs[edge_dict[e.getID()]] = pmf
                                
            behaviors[i] = pmfs


            i+=1
    return behaviors