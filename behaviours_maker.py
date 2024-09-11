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
from colors import *

def valid_neighbors(edge,mapdata,target=None,forbid=None):
    #get only valid turns for the given edge
    graphdict = mapdata.graphdict
    connections = mapdata.connections
    edgegraph = mapdata.edgegraph
    res = []
    n1 = edge.getToNode()
    for e in n1.getOutgoing():
        # if e.getID()!=edge.getID() and e.allows('passenger') and connection_exists(edge.getFromNode().getID(),e.getFromNode().getID(),e.getToNode().getID(),graphdict,connections):
        if e.getID()!=edge.getID() and e.allows('passenger') and e.getID() in edgegraph[edge.getID()]:
            if target is None:
                res.append(e)
            else:
                if not mapdata.edge2target[e.getID()][target][0]:
                    mapdata.edge2target[e.getID()][target] = (True,False,[])
                    path = traci.simulation.findRoute(e.getID(),target).edges
                    if len(path)>0:
                        mapdata.edge2target[e.getID()][target] = (True,True,list(path))
                        if forbid is None:
                            res.append(e)
                        else:
                            if forbid not in path:
                                res.append(e)
                    else:
                        mapdata.edge2target[e.getID()][target] = (True,False,[])
                else:
                    if mapdata.edge2target[e.getID()][target][1]:
                        if forbid is None:
                            res.append(e)
                        else:
                            if forbid not in mapdata.edge2target[e.getID()][target][2]:
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
        return 'e_dijkstra'
    if index==1:
        return 'e_astar'
    if index==2:
        return 'e_bfs'
    if index==3:
        return 'e_greedybfs'
    
def index2path(j,target_edge,e,mapdata,dijkstrabased=None,works=None):
        route = [] # if decomment return 1 tab
    # if j == 0:
    #     route = traci.simulation.findRoute(e.getID(), target_edge).edges
    # elif j == 1:
    #     route = traci.simulation.findRoute(e.getID(), target_edge, "routerByDistance").edges
    # else:
        ext = None
        # route.append(e.getID())
        # ext = build_path(graphdict,e.getToNode().getID(),endnode,index2alg(j),graphmap=graphmap,connections=connections,edge=e.getID())
        # ext = build_path(mapdata,e.getToNode().getID(),endnode,index2alg(j),edge=e.getID())
        # if e.getID() in dijkstrabased and e.getID() in works:
        #     ext1 = build_path(mapdata,e.getID(),target_edge,index2alg(j),forbidnode=dijkstrabased[e.getID()])
        #     if ext1 is None:
        #         ext2 = build_path(mapdata,e.getID(),target_edge,index2alg(j))
        #         ext = ext2
        #     else:
        #         ext = ext1
        # else:
        if not works:
            ext = build_path(mapdata,e.getID(),target_edge,index2alg(j))
        else:
            print('you sure no work?')
            if works:
                if j<2:
                    ext = build_path(mapdata,e.getID(),target_edge,index2alg(j))
                else:
                    ext = build_path(mapdata,e.getID(),target_edge,index2alg(j-2),forbidnode=works)
                    if ext is None:
                        ext = build_path(mapdata,e.getID(),target_edge,index2alg(j))
        if ext is None:
            route = None
        else:
            if 'SUCC' not in ext:
                route.extend(ext)
        return route # if decomment return 1 tab
    
def create_behaviours(num_algs,mapdata,consider_works=False):

    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")
    targets = mapdata.targets
    edge_list = mapdata.edgelist
    net = mapdata.net
    edge_dict = mapdata.edge_dict
    

    sumocfg_name = 'osm_'+str(mapdata.scenario)+'.sumocfg'
    sumoBinary = sumolib.checkBinary('sumo')

    sumoCmd = [sumoBinary, "-c", sumocfg_name,"--step-length","0.05"] #The last parameter is the step size, has to be small
    traci.start(sumoCmd)
    print("Starting SUMO...")

    behaviors = np.zeros((len(targets)*num_algs, len(edge_list), len(edge_list)))
    works = mapdata.works if consider_works else {}
    i=0
    paths = {}
    for t in targets:
        pmfs = np.zeros((len(edge_list), len(edge_list)))
        dijkstrabased = {}
        target_edge = targets[t]
        for j in range(num_algs):
            pmfs = np.zeros((len(edge_list), len(edge_list)))
            for e in edge_list:
                if (e.getID() not in paths):
                    paths[e.getID()] = []
                route = index2path(j,target_edge,e,mapdata,dijkstrabased=dijkstrabased,works=works)
                if len(paths[e.getID()])<num_algs*len(targets):
                    if route is None:
                        paths[e.getID()].append(None)
                    else:    
                        paths[e.getID()].append(list(route))
                pmf = [0]*((len(edge_list)))
                if route is None or len(route) ==0 : # edges are not connected 
                    for x in range(len(pmf)):
                        pmf[x] = 0
                elif(len(route) == 1):
                    e_prime = e.getID()
                    pmf[edge_dict[e_prime]] = 1.0
                else:
                    e_prime = route[1] # e' is the edge just after e 

                    prob = 0.99 
                    pmf[edge_dict[e_prime]] = prob

                    valid_edges = valid_neighbors(e,mapdata,target_edge)
                    # valid_lengths = valid_neighbors_lengths(valid_edges,t)
                    # totlength = sum(valid_lengths.values())
                    for v in valid_edges:
                        if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                            pmf[edge_dict[v.getID()]] = (1-prob) / (len(valid_edges)-1)
                        # pmf[edge_dict[v.getID()]] = (totlength-valid_lengths[v])/totlength if totlength!=valid_lengths[v] else 1.0
                        # print(str(v)+" "+str(pmf[edge_dict[v.getID()]]))
                    if len(valid_edges)>1 and j==0:
                        dijkstrabased[e.getID()] = e_prime
                    
                pmf = np.array(pmf)
                        
                if np.sum(pmf) !=0:
                    pmf = pmf/np.sum(pmf)
                        
                pmfs[edge_dict[e.getID()]] = pmf
                # print([x for x in pmf if x!=0])
            
            for k in range(len(pmfs)):
                behaviors[i,k] = behaviors[i,k]+pmfs[k]
                if np.sum(behaviors[i,k])>1:
                    print('WARNING HERE AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')        
            i+=1
            #     route = []
            #     # if j == 0:
            #     #     route = traci.simulation.findRoute(e.getID(), t).edges
            #     # else:
            #     #     route = traci.simulation.findRoute(e.getID(), t, "routerByDistance").edges
            #     if j == 0:
            #             route = traci.simulation.findRoute(e.getID(), t).edges
            #     elif j == 1:
            #         route = traci.simulation.findRoute(e.getID(), t, "routerByDistance").edges
            #     else:
            #         route = []
            #         route.append(e.getID())
            #         ext = build_path(graphdict,e.getToNode().getID(),endnode,index2alg(j),graphmap=graphmap,connections=connections,edge=e.getID())
            #         if ext is None:
            #             route = None
            #         else:
            #             if 'SUCC' not in ext:
            #                 route.extend(ext)
            #     pmf = [0]*((len(edge_list)))
                
            #     if route is None or len(route) ==0 : # edges are not connected 
            #         for x in range(len(pmf)):
            #             pmf[x] = 0
                
            #     elif(len(route) == 1):
            #         e_prime = e.getID()
            #         pmf[edge_dict[e_prime]] = 1.0
            #         print(str(route)+' from edge '+str(e.getID()))
            #         # prob = 0.95 
                    
            #         # pmf[edge_dict[e_prime]] = 0.95  
            #         # valid_edges = valid_neighbors(e,graphdict,connections)
                        
            #         # for v in valid_edges:
            #         #     if(edge_dict[v.getID()]!= edge_dict[e_prime]):
            #         #         pmf[edge_dict[v.getID()]] = (1-prob) / len(valid_edges)
            #     else:
                    
            #         e_prime = route[1] # e' is the edge just after e 

            #         prob = 0.95
            #         pmf[edge_dict[e_prime]] = prob

            #         valid_edges = valid_neighbors(e,graphdict,connections,t)

            #         for v in valid_edges:
            #             if(edge_dict[v.getID()]!= edge_dict[e_prime]):
            #                 pmf[edge_dict[v.getID()]] = (1-prob) / len(valid_edges)

            #     pmf = np.array(pmf)
                        
            #     if np.sum(pmf) !=0:
            #         pmf = pmf/np.sum(pmf)
                        
            #     pmfs[edge_dict[e.getID()]] = pmf
                    
            # behaviors[i] = pmfs


            # i+=1
    print("Behaviors generated!")
    traci.close()
    np.save('behaviors_'+str(mapdata.scenario)+'_'+str(num_algs)+('_wip' if consider_works else ''), behaviors)
    np.save('paths_'+str(mapdata.scenario)+'_'+str(num_algs)+('_wip' if consider_works else ''), paths)
    
def online_create_behaviours(mapdata,num_algs,target_edge,ss_edges,behaviors,behavior_created,works,toupdate=False,colorpaths=False,colorprobs=False):
    targets = mapdata.targets
    net = mapdata.net
    edge_list = mapdata.edgelist
    graphdict = mapdata.graphdict
    graphmap = mapdata.graphmap
    connections = mapdata.connections
    edge_dict = mapdata.edge_dict
    # behaviors = np.zeros((len(targets)*num_algs, len(edge_list), len(edge_list)))
    colorvalue = getIfromRGB(list(car_color(list(targets.keys())[list(targets.values()).index(target_edge)]))[0:3])
                            
    # i=0
    i = list(targets.values()).index(target_edge)*num_algs
    # for t in targets.values():
        # pmfs = np.zeros((len(edge_list), len(edge_list)))
        # pmfs = behaviors[i]
    endnode = net.getEdge(target_edge).getToNode().getID()
    if colorprobs:
        for edg in edge_list:
            traci.edge.setParameter(edg.getID(),'color',0)
    dijkstrabased = {}
    paths = {}
    for j in range(num_algs):
        pmfs = np.zeros((len(edge_list), len(edge_list)))
        for e in ss_edges:
            if (e.getID(),target_edge) not in behavior_created or toupdate:
                if (e.getID() not in paths) or (j==0):
                    paths[e.getID()] = []
                route = index2path(j,target_edge,e,mapdata,dijkstrabased=dijkstrabased,works=works)
                if len(paths[e.getID()])<num_algs:
                    paths[e.getID()].append(list(route))
                pmf = [0]*((len(edge_list)))
                if route is None or len(route) ==0 : # edges are not connected 
                    for x in range(len(pmf)):
                        pmf[x] = 0
                elif(len(route) == 1):
                    e_prime = e.getID()
                    pmf[edge_dict[e_prime]] = 1.0
                else:
                    e_prime = route[1] # e' is the edge just after e 

                    prob = 0.99 
                    pmf[edge_dict[e_prime]] = prob

                    valid_edges = valid_neighbors(e,mapdata,target_edge)
                    # valid_lengths = valid_neighbors_lengths(valid_edges,t)
                    # totlength = sum(valid_lengths.values())
                    for v in valid_edges:
                        if(edge_dict[v.getID()]!= edge_dict[e_prime]):
                            pmf[edge_dict[v.getID()]] = (1-prob) / (len(valid_edges)-1)
                        # pmf[edge_dict[v.getID()]] = (totlength-valid_lengths[v])/totlength if totlength!=valid_lengths[v] else 1.0
                        # print(str(v)+" "+str(pmf[edge_dict[v.getID()]]))
                    if len(valid_edges)>1 and j==0:
                        dijkstrabased[e.getID()] = e_prime
                    if colorpaths:
                        if j == 2:
                            for edg in edge_list:
                                traci.edge.setParameter(edg.getID(),'color',0)
                            
                            for re in route:
                                traci.edge.setParameter(re,'color',colorvalue-j*100000)
                    if colorprobs:
                        if j == 2:
                            for r in valid_edges:
                                traci.edge.setParameter(r.getID(),'color',colorvalue*pmf[edge_dict[r.getID()]])
                            
                    
                pmf = np.array(pmf)
                        
                if np.sum(pmf) !=0:
                    pmf = pmf/np.sum(pmf)
                        
                pmfs[edge_dict[e.getID()]] = pmf
                # print([x for x in pmf if x!=0])
        for k in range(len(pmfs)):
            if toupdate and edge_list[k] in ss_edges:
                behaviors[i,k] = np.zeros(len(edge_list))
            behaviors[i,k] = behaviors[i,k]+pmfs[k]
            su = np.sum(behaviors[i,k])
            if su>1:
                print('WARNING HERE AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA '+str(su))        
        i+=1
    return behaviors,paths