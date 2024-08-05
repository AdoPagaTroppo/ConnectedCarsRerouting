import traci
import traci._vehicle
import traci.constants as tc
import traci.domain
import sumolib
from algorithms import connection_exists
from algorithms import build_path
from vehicledata import VehicleData
from node_file_parser import parse_file_for_nodes
import random
import randomTrips as rt
from graph_util import build_graph
from agent import Agent
from crowdsourcing import Crowdsourcing
import numpy as np
from bus_parser import bus_parser

NUM_VEHICLES = 10
USE_DESIRED_AGENTS = False
DESIRED_AGENTS = 2
PERC_UNI_CARS = 1.0
PERC_AGENT_CARS = DESIRED_AGENTS/NUM_VEHICLES if USE_DESIRED_AGENTS else 1.0*PERC_UNI_CARS
NUM_AGENTS = NUM_VEHICLES*PERC_AGENT_CARS
DIJKSTRA_BASED_REROUTING = False
SHOW_GUI = True
T_HORIZON = 3
STEP_SIZE = 0.1
INCLUDE_BUS = True
INCLUDE_RANDOM = True
START_TIME = 12

def car_color(type):
    if type=='NAPOLI':
        return 255,0,255,1 # fucsia
    elif type=='AVELLINO':
        return 0,255,0,1 # green
    elif type=='SALERNO':
        return 0,255,255,1 # azure
    elif type=='BENEVENTO':
        return 255,255,0,1 # yellow
    elif type=='FISCIANO':
        return 255,0,0,1 # red
    elif type=='PENTA':
        return 0,0,255,1 # blue
    return 255,255,255,1

def neighbor_edges(edge, graphdict, connections, net, edgelist):
    #Find neighbor edges
    res = []
    #get all the incoming and outgoing edges of the end node
    n1 = edge.getToNode()
    for e in (n1.getIncoming()+n1.getOutgoing()):
    # for e in (n1.getOutgoing()):
        if e.allows('passenger') and not e.getID().__contains__(':'):
            if e.getID()!=edge.getID() and connection_exists(edge.getFromNode().getID(),e.getFromNode().getID(),e.getToNode().getID(),graphdict,connections):
                res.append(edgelist.index(e))
    #get all the incoming and outgoing edges of the starting node
    n2 = edge.getFromNode()
    for e in (n2.getIncoming()+n2.getOutgoing()):
    # for e in (n2.getOutgoing()):
        if e.allows('passenger') and not e.getID().__contains__(':'):
            if e.getID()!=edge.getID() and connection_exists(edge.getFromNode().getID(),e.getFromNode().getID(),e.getToNode().getID(),graphdict,connections):
                res.append(edgelist.index(e))
    return(res)

def state_space(n_node, tHor, net, connections, graphdict,edgelist):
    #State space for a given horizon: iteratively check every neighbor and add it to the state space
    stsp = [n_node]
    for t in range(tHor):
        newSpace = stsp.copy()
        for n in stsp:
            ed = edgelist[n]
            for x in neighbor_edges(ed,graphdict,connections,net,edgelist):
                if x not in newSpace:
                    newSpace.append(x)
        stsp = newSpace
    for s in stsp:
        print(edgelist[s].getID())
    return(np.array(stsp))

def compute_reward(ss,vehicle,edgelist,time):
    r = [0]*len(edgelist)
    for i in ss:
        edge = edgelist[i]
        edid = edge.getID()
        # if edid in occupied_edges:
        #     r[i] = -occupied_edges[edid]
        # else:
        #     r[i] = 0
        # r[i] += -traci.edge.getTraveltime(edid)-traci.edge.getEffort(edid,time)
        r[i] = -traci.edge.getTraveltime(edid)-traci.edge.getLastStepMeanSpeed(edid)
    return np.array(r)

def spawnBus(busdata,currenttime):
    spawned = False
    spawnedBuss = []
    for tup in busdata:
        if tup[0]==currenttime:
            routedata = busdata[tup]
            vbusid = 'bus_'+str(tup[1]).replace(' ','_')+'_'+str(tup[0])
            if len(routedata)==2:
                r = traci.simulation.findRoute(routedata[0][0],routedata[1][0]).edges
                traci.route.add(vbusid+'route',r)
                traci.vehicle.add(vbusid,vbusid+'route','bus')
            else:
                r1 = list(traci.simulation.findRoute(routedata[0][0],routedata[1][0]).edges)
                r2 = list(traci.simulation.findRoute(routedata[1][0],routedata[2][0]).edges)
                r1.__delitem__(-1)
                r1.extend(r2)
                traci.route.add(vbusid+'route',r1)
                traci.vehicle.add(vbusid,vbusid+'route','bus')
                traci.vehicle.setSpeed(vbusid,-1)
            spawned = True
            spawnedBuss.append(vbusid)
    return spawned,spawnedBuss
                
def spawnRandom(graphdict):
    sourcenode = random.choice(list(graphdict.keys()))
    if graphdict[sourcenode]=={}:
        return False,None
    sourcenoded = random.choice(list(graphdict[sourcenode].keys()))
    source = graphdict[sourcenode][sourcenoded][0]
    sourcenode = random.choice(list(graphdict.keys()))
    if graphdict[sourcenode]=={}:
        return False,None
    sourcenoded = random.choice(list(graphdict[sourcenode].keys()))
    dest = graphdict[sourcenode][sourcenoded][0]
    route = list(traci.simulation.findRoute(source,dest).edges)
    if len(route)>3:
        rid = 'random_'+str(source)+'_'+str(dest)
        traci.route.add(rid,route)
        traci.vehicle.add(rid+'_vehicle',rid,'Car_RANDOM')
        traci.vehicle.setSpeed(rid+'_vehicle',-1)
        insim.append(rid)
        return True,rid+'_vehicle'
    return False,None
    

traci.start(['sumo'+('-gui' if SHOW_GUI else ''),'-c','osm4.sumocfg','--step-length',str(STEP_SIZE)])
# routealg = {'bfs': 'route01','greedybfs': 'route02','dijkstra': 'route03','astar': 'route04'}
# algindex = ['bfs','greedybfs','dijkstra','astar']
graphdict, graphmap, net, connections = build_graph()
# source = '6045581080'
# dest = '3801519365'
paths = {}
# paths['bfs'] = build_path(graphdict,source,dest,'bfs',connections=connections)
# paths['greedybfs'] = build_path(graphdict,source,dest,'greedybfs',graphmap,connections=connections)
# paths['dijkstra'] = build_path(graphdict,source,dest,'dijkstra',connections=connections)
# paths['astar'] = build_path(graphdict,source,dest,'astar',graphmap,connections=connections)
destinations, sources = parse_file_for_nodes('config.txt')
target_weights = []
targets = {}
for t in destinations:
    target_weights.append(t[2])
    targets[t[1]] = t[0]
vehs = {}
for c in range(int((PERC_UNI_CARS-PERC_AGENT_CARS)*NUM_VEHICLES)):
    source = random.choices(sources)[0]
    dest = random.choices(destinations,target_weights)[0]
    id = 'veh'+str(c+1)+'_'+str(dest[1])
    pathbuilt=build_path(graphdict,source,net.getEdge(dest[0]).getToNode().getID(),'dijkstra',graphmap,connections=connections)
    paths['route'+str(c+1)] = traci.simulation.findRoute(net.getEdge(pathbuilt[0]).getID(),net.getEdge(pathbuilt[-1]).getID()).edges
    vehs[id] = VehicleData(id,'route'+str(c+1),dest[1])
i = 0
j = 0
start_edge = '-579690548#1'
end_edge = '1084284613'
for vehicle in vehs:
    traci.route.add('route'+str(j+1),paths['route'+str(j+1)])
    traci.vehicle.add(vehicle,'route'+str(j+1),'Car_'+vehs[vehicle].dest,str(i))
    traci.vehicle.setSpeed(vehicle,-1)
    i += 5
    j += 1
agents = {}
for i in range(int(NUM_AGENTS)):
    destt = random.choices(destinations,target_weights)[0]
    print(destt)
    dest = destt[0]
    agentid = 'agent'+str(i)+'_'+str(destt[1])
    agrouteid = agentid+'_route'
    agent = Agent(start_edge,list(targets.values()).index(dest))
    crowds_controller = Crowdsourcing(agent)
    agents[agentid] = crowds_controller
    vehs[agentid] = VehicleData(agentid,agrouteid,destt[1])
    traci.route.add(agrouteid,(start_edge,start_edge))
    traci.vehicle.add(agentid,agrouteid,'Car_AGENT',str(i*10))
    traci.vehicle.setSpeed(agentid,-1)
print('loaded vehicles')
# for vehicle in vehs:
#     traci.vehicle.subscribeContext(vehicle,tc.CMD_GET_VEHICLE_VARIABLE,1,[tc.VAR_SPEED,tc.VAR_DISTANCE,tc.VAR_ROAD_ID,tc.VAR_ROUTE_ID])
# print('subscribed to vehicles')
totarrived = 0
rerouting_occurred = {}
for vehicle in vehs:
    rerouting_occurred[vehicle] = False
forbid_rerouting = []
# forbid_rerouting.append('veh1')
# forbid_rerouting.append('veh2')
forbid_rerouting.append('agent')
prev_edge = {}
occupied_edges = {}
next_edge = {}
secondtot = 1/STEP_SIZE
secondcounter = 0
mcounter = -1
scounter = -1
busdata = bus_parser(START_TIME)
tottoarrive = PERC_UNI_CARS*NUM_VEHICLES
insim = []
edge_list = net.getEdges()
edgelist = []
for e in edge_list:
    if e.allows('passenger'):
        edgelist.append(e)
print('ready to go')
while True:
    totarrived += traci.simulation.getArrivedNumber()
    if totarrived == tottoarrive:
        break
    traci.simulationStep()
    for vehicle in vehs:
        if(vehicle in traci.simulation.getDepartedIDList()):
            insim.append(vehicle)
        elif vehicle in traci.simulation.getArrivedIDList():
            insim.remove(vehicle)
        if vehicle in insim:
            vehs[vehicle].speeds.append(traci.vehicle.getSpeed(vehicle))
            vehs[vehicle].dist = traci.vehicle.getDistance(vehicle)
            roadid = traci.vehicle.getRoadID(vehicle)
            if vehicle in prev_edge:
                occupied_edges[prev_edge[vehicle]] -= 1
            if roadid in occupied_edges:
                occupied_edges[roadid] += 1
            else:
                occupied_edges[roadid] = 1
            # veh_route_id = traci.vehicle.getRouteID(vehicle)
            # veh_route = traci.vehicle.getRoute(vehicle)
            # if (not roadid.__contains__(':')) and vehicle in prev_edge and roadid!=prev_edge[vehicle] and veh_route.index(roadid)<len(veh_route)-2 and DIJKSTRA_BASED_REROUTING and vehicle not in forbid_rerouting:
            #     wait_times = {}
            #     min_wait_t = occupied_edges[veh_route[veh_route.index(roadid)+1]] if veh_route[veh_route.index(roadid)+1] in occupied_edges else 0,veh_route[veh_route.index(roadid)+1]
            #     for otheredges in net.getEdge(roadid).getToNode().getOutgoing():
            #         if net.getEdge(otheredges.getID()).allows('passenger') and not otheredges.getID().__contains__(':'):
            #             outid = otheredges.getID()
            #             if connection_exists(net.getEdge(roadid).getFromNode().getID(),net.getEdge(roadid).getToNode().getID(),net.getEdge(outid).getToNode().getID(),graphdict,connections):
            #                 extendedpath = build_path(graphdict, net.getEdge(outid).getToNode().getID(), dest, 'astar',graphmap,connections=connections)
            #                 if extendedpath is not None and connection_exists(net.getEdge(outid).getFromNode().getID(),net.getEdge(outid).getToNode().getID(),net.getEdge(extendedpath[0]).getToNode().getID(),graphdict,connections):
            #                     wait_times[outid] = occupied_edges[outid] if outid in occupied_edges else 0
            #                     if wait_times[outid] < min_wait_t[0]:
            #                         min_wait_t = wait_times[outid],outid
            #     if len(wait_times)>1 and veh_route[veh_route.index(roadid)+1]!=min_wait_t[1]:
            #         newpath = []
            #         newpath.append(roadid)
            #         newpath.append(min_wait_t[1])
            #         extendedpath = build_path(graphdict, net.getEdge(min_wait_t[1]).getToNode().getID(), dest, 'astar',graphmap,connections=connections)
            #         newpath.extend(extendedpath)
            #         traci.vehicle.setRoute(vehicle,newpath)
            #         rerouting_occurred[vehicle] = True
            #         print('REROUTING OCCURRED for '+str(vehicle))
            if vehicle.__contains__('agent') and (vehicle not in prev_edge or vehicle in prev_edge and prev_edge[vehicle]!=roadid):
            # if vehicle=='agent' and not roadid.__contains__(':'):
                currentid = roadid if not roadid.__contains__(':') else next_edge[vehicle]
                statecars = edgelist.index(net.getEdge(currentid))
                ss = state_space(statecars,T_HORIZON,net,connections,graphdict,edgelist)
                # print(vehicle+' state space: '+str(ss))
                # if len(net.getEdge(currentid).getToNode().getOutgoing())==1:
                #     prox_edge = edgelist[ss[0]]
                # else:
                r = compute_reward(ss,vehicle,edgelist,scounter+1)
                new_state = agents[vehicle].receding_horizon_DM(statecars,T_HORIZON,ss,r)
                prox_edge = edgelist[new_state]
                next_edge[vehicle] = prox_edge.getID()
                # print(vehicle+' next edge: '+next_edge[vehicle])
                # statecars = edgelist.index(net.getEdge(next_edge))
                # ss = state_space(statecars,T_HORIZON,net,connections,graphdict)
                # r = compute_reward(ss,occupied_edges,edgelist)
                # new_state = crowds_controller.receding_horizon_DM(statecars,T_HORIZON-1 if T_HORIZON>1 else T_HORIZON,ss,r)
                # prox_edge = edgelist[new_state]
                # print(vehicle+" on edge "+str(roadid)+" with prox edge "+prox_edge.getID())
                if prox_edge.getID() == end_edge:
                    print(vehicle+' ARRIVED')
                traci.vehicle.changeTarget(vehicle,prox_edge.getID())
            prev_edge[vehicle] = roadid
    if secondcounter%secondtot == 0:
        scounter += 1
        if scounter%60 == 0:
            mcounter += 1
            if INCLUDE_RANDOM and tottoarrive+1<NUM_VEHICLES:
                spawned, spawnedRand = spawnRandom(graphdict)
                if spawned:
                    print("Spawned random vehicle")
                    tottoarrive += 1
                    vehs[spawnedRand] = VehicleData(spawnedRand,spawnedRand.replace('_vehicle',''),'')
            if INCLUDE_BUS and tottoarrive+1<NUM_VEHICLES:
                spawned, spawnedBuss = spawnBus(busdata,mcounter)
                if spawned:
                    print("Spawned bus")
                    tottoarrive += len(spawnedBuss)
                    for el in spawnedBuss:
                        vehs[el] = VehicleData(el,el+'route','')
    secondcounter += 1
    
traci.close()
# print("VEHICLE || ALGORITHM || AVG_SPEED || TRAVELED_DISTANCE || WITH_REROUTING || DESTINATION")
print("VEHICLE || ALGORITHM || AVG_SPEED || TRAVELED_DISTANCE || DESTINATION")
for vehicle in vehs:
    print(str(vehicle)+" || "+str(vehs[vehicle].alg)+" || "+str(sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds))+" || "+str(vehs[vehicle].dist)+" || "+str(vehs[vehicle].dest))
    # print(str(vehicle)+" || "+str(vehs[vehicle].alg)+" || "+str(sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds))+" || "+str(vehs[vehicle].dist)+" || "+str(rerouting_occurred[vehicle])+" || "+str(vehs[vehicle].dest))