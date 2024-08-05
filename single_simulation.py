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
from behaviours_maker import valid_neighbors
from behaviours_maker import online_create_behaviours
from intercar_comm import tweet

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
        if e.allows('passenger') and not e.getID().__contains__(':'):
            if e.getID()!=edge.getID() and connection_exists(edge.getFromNode().getID(),e.getFromNode().getID(),e.getToNode().getID(),graphdict,connections):
                res.append(edgelist.index(e))
    #get all the incoming and outgoing edges of the starting node
    n2 = edge.getFromNode()
    for e in (n2.getIncoming()+n2.getOutgoing()):
        if e.allows('passenger') and not e.getID().__contains__(':'):
            if e.getID()!=edge.getID() and connection_exists(edge.getFromNode().getID(),e.getFromNode().getID(),e.getToNode().getID(),graphdict,connections):
                res.append(edgelist.index(e))
    return(res)

def state_space(n_node, tHor, net, connections, graphdict,edgelist,target):
    #State space for a given horizon: iteratively check every neighbor and add it to the state space
    stsp = [n_node]
    for t in range(tHor):
        newSpace = stsp.copy()
        for n in stsp:
            ed = edgelist[n]
            for x in valid_neighbors(ed,graphdict,connections,target):
            # for x in neighbor_edges(ed,graphdict,connections,net,edgelist):
                xid = edgelist.index(x)
                if xid not in newSpace:
                    newSpace.append(xid)
        stsp = newSpace
    return(np.array(stsp))

def compute_reward(ss,passed,edgelist,time,works=None):
    fullen = 0
    # if online:
    #     fullen = len(ss)
    # else:
    fullen = len(edgelist)
    r = [0]*fullen
    for i in ss:
        edge = edgelist[i]
        edid = edge.getID()
        r[i] = -traci.edge.getAdaptedTraveltime(edid,time)+traci.edge.getLastStepMeanSpeed(edid)-traci.edge.getLastStepVehicleNumber(edid)-(1000000 if edid in works else 0)-(1000000 if edid in passed else 0)
        # da aggiungere un aggiornamento di costo tenendo conto di comunicazione e interfacciamento con social + fiducia + nnumero di persone che twittano stessa cosa
        # aggiungere costi su incidenti + lavori (simulazioni con stessi lavori e scenari significativi)
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
        return True,rid+'_vehicle'
    return False,None
    
def conv_ss2edges(state_space,edgelist):
    ss = []
    for s in state_space:
        edind = edgelist[s]
        ss.append(edind)
    return ss

DIJKSTRA_BASED_REROUTING = False


def single_sim(NUM_VEHICLES, PERC_UNI_CARS, SHOW_GUI, T_HORIZON, STEP_SIZE, INCLUDE_BUS, INCLUDE_RANDOM, START_TIME, PERC_AGENT_CARS, USE_DESIRED_AGENTS=False, DESIRED_AGENTS=0, NUM_ALGS=1, ONLINE=False):
    PERC_AGENT_CARS = DESIRED_AGENTS/NUM_VEHICLES if USE_DESIRED_AGENTS else PERC_AGENT_CARS*PERC_UNI_CARS
    NUM_AGENTS = NUM_VEHICLES*PERC_AGENT_CARS
    traci.start(['sumo'+('-gui' if SHOW_GUI else ''),'-c','osm4.sumocfg','--step-length',str(STEP_SIZE)])
    graphdict, graphmap, net, connections = build_graph()
    paths = {}
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
    end_edge = {}
    for vehicle in vehs:
        traci.route.add('route'+str(j+1),paths['route'+str(j+1)])
        traci.vehicle.add(vehicle,'route'+str(j+1),'Car_AGENT',str(i))
        traci.vehicle.setSpeed(vehicle,-1)
        i += 5
        j += 1
    agents = {}
    for i in range(int(NUM_AGENTS)):
        # destt = random.choices(destinations,target_weights)[0]
        destt = destinations[i%len(destinations)]
        print(destt)
        dest = destt[0]
        agentid = 'agent'+str(i)+'_'+str(destt[1])
        agrouteid = agentid+'_route'
        end_edge[agentid] = dest
        print(list(targets.values()).index(dest)*NUM_ALGS)
        agent = Agent(start_edge,list(targets.values()).index(dest)*NUM_ALGS)
        crowds_controller = Crowdsourcing(agent,NUM_ALGS)
        agents[agentid] = crowds_controller
        vehs[agentid] = VehicleData(agentid,agrouteid,destt[1])
        traci.route.add(agrouteid,(start_edge,start_edge))
        traci.vehicle.add(agentid,agrouteid,'Car_'+str(destt[1]),str(i*10))
        traci.vehicle.setSpeed(agentid,-1)
    print('loaded vehicles')
    totarrived = 0
    rerouting_occurred = {}
    for vehicle in vehs:
        rerouting_occurred[vehicle] = False
    forbid_rerouting = []
    forbid_rerouting.append('agent')
    prev_edge = {}
    occupied_edges = {}
    next_edge = {}
    secondtot = 1/STEP_SIZE
    secondcounter = 0
    mcounter = -1
    scounter = -1
    busdata = bus_parser(START_TIME)
    tottoarrive = int(PERC_UNI_CARS*NUM_VEHICLES if NUM_AGENTS==0 else NUM_AGENTS)
    insim = []
    edge_list = net.getEdges()
    edgelist = []
    for e in edge_list:
        if e.allows('passenger'):
            edgelist.append(e)
    print('ready to go')
    correctlyarrived = []
    works = []
    # works.append('951974692#1')
    passed = {}
    while True:
        for v in traci.simulation.getArrivedIDList():
            if NUM_AGENTS != 0:
                if v.__contains__('agent'):
                    totarrived += 1
        if totarrived == tottoarrive:
            break
        traci.simulationStep()
        for vehicle in vehs:
            if(vehicle in traci.simulation.getDepartedIDList()):
                insim.append(vehicle)
            elif vehicle in traci.simulation.getArrivedIDList():
                insim.remove(vehicle)
                if NUM_AGENTS == 0:
                    totarrived += 1
            if vehicle in insim:
                vehs[vehicle].speeds.append(traci.vehicle.getSpeed(vehicle))
                vehs[vehicle].dist = traci.vehicle.getDistance(vehicle)
                vehs[vehicle].fuelconsumption += traci.vehicle.getFuelConsumption(vehicle)
                vehs[vehicle].waitingtime += traci.vehicle.getWaitingTime(vehicle)
                vehs[vehicle].co2emission += traci.vehicle.getCO2Emission(vehicle)
                vehs[vehicle].noiseemission += traci.vehicle.getNoiseEmission(vehicle)
                roadid = traci.vehicle.getRoadID(vehicle)
                if vehicle in prev_edge:
                    occupied_edges[prev_edge[vehicle]] -= 1
                if roadid in occupied_edges:
                    occupied_edges[roadid] += 1
                else:
                    occupied_edges[roadid] = 1
                # if vehicle.__contains__('agent') and (vehicle not in prev_edge or vehicle in prev_edge and prev_edge[vehicle]!=roadid and not prev_edge[vehicle].__contains__(':')):
                if vehicle.__contains__('agent') and (vehicle not in prev_edge or vehicle in prev_edge and prev_edge[vehicle]!=roadid) and (vehicle in next_edge and next_edge[vehicle]!=end_edge[vehicle] or vehicle not in next_edge):
                    currentid = roadid if not roadid.__contains__(':') else next_edge[vehicle]
                    statecars = edgelist.index(net.getEdge(currentid))
                    vn = []
                    vn.append(net.getEdge(currentid))
                    vn.extend(valid_neighbors(net.getEdge(currentid),graphdict,connections,end_edge[vehicle]))
                    # print(vn)
                    # print(len(vn))
                    prox_edge = None
                    if end_edge[vehicle] in vn:
                        prox_edge = end_edge[vehicle]
                    else:
                        if len(vn)>2:
                            # print('calculating')
                            ss = state_space(statecars,T_HORIZON,net,connections,graphdict,edgelist,end_edge[vehicle])
                            ss_edges = conv_ss2edges(ss,edgelist)
                            # print(ss_edges)
                            r = compute_reward(ss,passed[vehicle],edgelist,scounter+1,works)
                            # print([x for x in r if x!=0])
                            # print(ss)
                            behav = online_create_behaviours(edgelist,targets,NUM_ALGS,end_edge[vehicle],graphdict,connections,vn,graphmap)
                            new_state = agents[vehicle].receding_horizon_DM(statecars,T_HORIZON,ss,r,ONLINE,behav)
                            prox_edge = edgelist[new_state]
                        elif len(vn)==2:
                            # print('immediate next edge')
                            prox_edge = vn[1]
                        else:
                            # print('no out edge')
                            prox_edge = vn[0]
                    next_edge[vehicle] = prox_edge.getID()
                    # if prox_edge.getID() in works:
                    #     tweet(net.getEdge(roadid),prox_edge,vehs,graphdict,connections)
                    if prox_edge.getID() == end_edge[vehicle]:
                        print(vehicle+' ARRIVED')
                        if vehicle not in correctlyarrived:
                            correctlyarrived.append(vehicle)
                    traci.vehicle.changeTarget(vehicle,prox_edge.getID())
                    traci.edge.setParameter(prox_edge.getID(),'color',100)
                    if vehicle in prev_edge:
                        traci.edge.setParameter(prev_edge[vehicle],'color',0)
                    traci.edge.setParameter(roadid,'color',0)
                    # else:
                    #     prox_edge = vn[0]
                    #     next_edge[vehicle] = prox_edge.getID()
                    #     if prox_edge.getID() == end_edge[vehicle]:
                    #         print(vehicle+' ARRIVED')
                    #     traci.vehicle.changeTarget(vehicle,prox_edge.getID())
                prev_edge[vehicle] = roadid
                if vehicle not in passed:
                    passed[vehicle] = []
                if roadid not in passed[vehicle]:
                    passed[vehicle].append(roadid)
        if secondcounter%secondtot == 0:
            scounter += 1
            if scounter%60 == 0:
                mcounter += 1
                if INCLUDE_RANDOM and tottoarrive+1<NUM_VEHICLES:
                    spawned, spawnedRand = spawnRandom(graphdict)
                    if spawned:
                        print("Spawned random vehicle")
                        tottoarrive += 1 if NUM_AGENTS==0 else 0
                        vehs[spawnedRand] = VehicleData(spawnedRand,spawnedRand.replace('_vehicle',''),'')
                if INCLUDE_BUS and tottoarrive+1<NUM_VEHICLES:
                    spawned, spawnedBuss = spawnBus(busdata,mcounter)
                    if spawned:
                        print("Spawned bus")
                        tottoarrive += len(spawnedBuss) if NUM_AGENTS==0 else 0
                        for el in spawnedBuss:
                            vehs[el] = VehicleData(el,el+'route','')
        secondcounter += 1
        
    traci.close()
    # print("VEHICLE || ALGORITHM || AVG_SPEED || TRAVELED_DISTANCE || DESTINATION")
    retds = []
    for vehicle in vehs:
        spedlen = len(vehs[vehicle].speeds)
        avgspeed = 0
        if spedlen != 0:
            avgspeed = sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds)
        retds.append((vehicle,vehs[vehicle].alg,avgspeed,vehs[vehicle].dist,vehs[vehicle].fuelconsumption,vehs[vehicle].waitingtime,vehs[vehicle].noiseemission,vehs[vehicle].co2emission))
        # print(str(vehicle)+" || "+str(vehs[vehicle].alg)+" || "+str(sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds))+" || "+str(vehs[vehicle].dist)+" || "+str(vehs[vehicle].dest))
    return retds,NUM_AGENTS,correctlyarrived