
import sumolib
from algorithms import connection_exists
from algorithms import build_path
from vehicledata import VehicleData
import random
import traci
from graph_util import build_graph
from agent import Agent
import numpy as np
from bus_parser import bus_parser
from behaviours_maker import valid_neighbors
from behaviours_maker import online_create_behaviours
from intercar_comm import tweet
from spawners import *
from colors import *
from text2speech_handler import playAudio
from node_file_parser import parse_file_for_checkpoints
from mpl_toolkits import mplot3d

WORKS_THRESHOLD = 1
PLAY_AUDIO = False

def randomSignal(prob,vehicle,roadid,works):
    signal = random.random()<prob
    if signal:
        if roadid in works:
            vehicle.influence = vehicle.influence*2
            if vehicle.influence>1:
                vehicle.influence = 1
        else:
            vehicle.influence = vehicle.influence/2

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

def state_space(n_node, tHor,mapdata,target):
    edgelist = mapdata.edgelist
    #State space for a given horizon: iteratively check every neighbor and add it to the state space
    stsp = [n_node]
    for t in range(tHor):
        newSpace = stsp.copy()
        for n in stsp:
            ed = edgelist[n]
            for x in valid_neighbors(ed,mapdata,target):
            # for x in neighbor_edges(ed,graphdict,connections,net,edgelist):
                xid = edgelist.index(x)
                if xid not in newSpace:
                    newSpace.append(xid)
        stsp = newSpace
    return(np.array(stsp))

def compute_reward(ss,passed,mapdata,target_edge,works,vehicle):
    worksweight = 1000
    edgelist = mapdata.edgelist
    net = mapdata.net
    fullen = len(edgelist)
    r = [0]*fullen
    totrepeat = 0
    for i in ss:
        if edgelist[i].getID() in passed:
            totrepeat += 1
    for i in ss:
        edge = edgelist[i]
        edid = edge.getID()
        veh_list = traci.edge.getIDList()
        busnum = 0
        for id in veh_list:
            if id.__contains__('bus'):
                busnum += 1
        path = traci.simulation.findRoute(edid,target_edge,'routerByDistance').edges
        pathlen = 0
        for j in path:
            pathlen += net.getEdge(j).getLength()
        # print('edge '+str(edid)+' distance '+str(pathlen))
        traveltime = -traci.edge.getTraveltime(edid)
        veh_num = -traci.edge.getLastStepVehicleNumber(edid)
        carnum = veh_num-busnum
        density = (5*carnum+7*busnum)/net.getEdge(edid).getLength()
        if density>1:
            density = 1
        road_in_wip = -(worksweight*net.getEdge(edid).getLength() if edid in works else 0)
        road_repeat = -(70/totrepeat+100*(passed[edid]-1) if edid in passed else 0)
        tail_in_wip = 0
        if edid in vehicle.weightened:
            tail_in_wip = -worksweight*veh_num/vehicle.weightened[edid]
        pred_works = 0
        for e in works:
            if e in path and works[e]>WORKS_THRESHOLD:
                pred_works += -1000
        roadspeed = -(net.getEdge(edid).getSpeed()-traci.edge.getLastStepMeanSpeed(edid))
        r[i] = traveltime+veh_num+road_in_wip+road_repeat+tail_in_wip-pathlen-10*density+roadspeed+pred_works
        # r[i] = -pathlen-(1000000 if edid in works else 0)-(70/totrepeat+100*(passed[edid]-1) if edid in passed else 0)
        # print('edge '+str(edid)+' reward '+str(r[i]))
        # da aggiungere un aggiornamento di costo tenendo conto di comunicazione e interfacciamento con social + fiducia + nnumero di persone che twittano stessa cosa
        # aggiungere costi su incidenti + lavori (simulazioni con stessi lavori e scenari significativi)
    return np.array(r)
    
def conv_ss2edges(state_space,edgelist):
    ss = []
    for s in state_space:
        edind = edgelist[s]
        ss.append(edind)
    return ss

DIJKSTRA_BASED_REROUTING = False


def single_sim(NUM_VEHICLES, PERC_UNI_CARS, SHOW_GUI, T_HORIZON, STEP_SIZE, INCLUDE_BUS, INCLUDE_RANDOM, START_TIME, PERC_AGENT_CARS, mapdata, USE_DESIRED_AGENTS=False, DESIRED_AGENTS=0, NUM_ALGS=1, ONLINE=False, CONSIDER_WORKS=False):
    LANG = 'it'
    SCENARIO = mapdata.scenario
    PERC_AGENT_CARS = DESIRED_AGENTS/NUM_VEHICLES if USE_DESIRED_AGENTS else PERC_AGENT_CARS*PERC_UNI_CARS
    NUM_AGENTS = NUM_VEHICLES*PERC_AGENT_CARS
    traci.start(['sumo'+('-gui' if SHOW_GUI else ''),'-c','osm_'+str(SCENARIO)+'.sumocfg','--step-length',str(STEP_SIZE)])
    graphdict = mapdata.graphdict
    net = mapdata.net
    targets = mapdata.targets
    edgelist = mapdata.edgelist
    destinations = mapdata.destinations
    checkpoints = parse_file_for_checkpoints(str(SCENARIO)+'ScenarioData/checkpoints.txt')
    vehs = spawnUncontrolledCars(int((PERC_UNI_CARS-PERC_AGENT_CARS)*NUM_VEHICLES),mapdata)
    agents,end_edge = spawnControlledCars(NUM_AGENTS,mapdata,NUM_ALGS,vehs,ONLINE)
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
    busdata = bus_parser(START_TIME,SCENARIO)
    tottoarrive = int(PERC_UNI_CARS*NUM_VEHICLES if NUM_AGENTS==0 else NUM_AGENTS)
    insim = []
    print('ready to go')
    correctlyarrived = []
    works = mapdata.works if CONSIDER_WORKS else {}
    passed = {}
    behaviour_created = []
    behaviour_db = np.zeros((len(targets)*NUM_ALGS, len(edgelist), len(edgelist)))
    colorstreet = True
    colorreward = False
    rewards4colors = {}
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
                vehs[vehicle].arrived = True
                if NUM_AGENTS == 0:
                    totarrived += 1
            if vehicle in insim:
                vehicle_speed = traci.vehicle.getSpeed(vehicle)
                vehs[vehicle].speeds.append(traci.vehicle.getSpeed(vehicle))
                vehs[vehicle].dist = traci.vehicle.getDistance(vehicle)
                vehs[vehicle].fuelconsumption += traci.vehicle.getFuelConsumption(vehicle)
                vehs[vehicle].waitingtime += traci.vehicle.getWaitingTime(vehicle)
                vehs[vehicle].co2emission += traci.vehicle.getCO2Emission(vehicle)
                vehs[vehicle].noiseemission += traci.vehicle.getNoiseEmission(vehicle)
                vehs[vehicle].traveltime = (scounter+1)-vehs[vehicle].depart
                if vehicle not in passed:
                    passed[vehicle] = {}
                roadid = traci.vehicle.getRoadID(vehicle)
                if secondcounter%secondtot == 0:
                    randomSignal(0.1,vehs[vehicle],roadid,works)
                if roadid in works and not roadid.__contains__(':'):
                    if secondcounter%secondtot == 0:
                        tweet(works,roadid,vehs,end_edge,mapdata,False,vehs[vehicle],LANG,WORKS_THRESHOLD)
                    traci.vehicle.setSpeed(vehicle,net.getEdge(roadid).getSpeed()/10)
                    print(str(vehicle)+" TWEETED: hey, there's work in progress at "+str(roadid))
                elif roadid not in works:
                    traci.vehicle.setSpeed(vehicle,-1)
                if roadid in checkpoints:
                    if secondcounter%secondtot==0:
                        checkpoints[roadid]['speed'].append(vehicle_speed)
                        checkpoints[roadid]['flow'] += 1
                if vehicle in prev_edge:
                    occupied_edges[prev_edge[vehicle]] -= 1
                if roadid in occupied_edges:
                    occupied_edges[roadid] += 1
                else:
                    occupied_edges[roadid] = 1
                if vehicle.__contains__('agent') and (vehicle not in prev_edge or vehicle in prev_edge and prev_edge[vehicle]!=roadid) and (vehicle in next_edge and next_edge[vehicle]!=end_edge[vehicle] or vehicle not in next_edge) and len(roadid)>2:
                    currentid = roadid if not roadid.__contains__(':') else next_edge[vehicle]
                    statecars = edgelist.index(net.getEdge(currentid))
                    vn = []
                    vn.append(net.getEdge(currentid))
                    vn.extend(valid_neighbors(net.getEdge(currentid),mapdata,end_edge[vehicle]))
                    vn2 = []
                    vn2.append(net.getEdge(currentid))
                    vn2.extend(valid_neighbors(net.getEdge(currentid),mapdata))
                    prox_edge = None
                    paths = None
                    indmin = -1
                    selpath = None
                    if end_edge[vehicle] in vn:
                        prox_edge = end_edge[vehicle]
                    else:
                        if len(vn)>2:
                            # print(vehicle)
                            ss = state_space(statecars,T_HORIZON,mapdata,end_edge[vehicle])
                            # print(ss)
                            # print(statecars)
                            ss_edges = conv_ss2edges(ss,edgelist)
                            # print(ss_edges)
                            r = compute_reward(ss,passed[vehicle],mapdata,end_edge[vehicle],works,vehs[vehicle])
                            if ONLINE and (currentid,end_edge[vehicle]) not in behaviour_created:
                                behaviour_db,paths = online_create_behaviours(mapdata,NUM_ALGS,end_edge[vehicle],ss_edges,behaviour_db,behaviour_created)
                                for v in ss_edges:
                                    behaviour_created.append((v.getID(),end_edge[vehicle]))
                                    # print('added '+str(v.getID())+' towards '+str(end_edge[vehicle]))
                            # print([x for x in behaviour_db[agents[vehicle].targetIndex,edgelist.index(net.getEdge(currentid))] if x!=0])
                            if colorreward:
                                colorMap(r,mapdata,rewards4colors,end_edge[vehicle])
                                # for e in edgelist:
                                #     if e not in ss_edges:
                                #         traci.edge.setParameter(e.getID(),'color',0)
                                #     else:
                                #         traci.edge.setParameter(e.getID(),'color',1000000)
                            new_state,indmin = agents[vehicle].receding_horizon_DM(statecars,T_HORIZON,ss,r,ONLINE,behaviour_db,edgelist)
                            prox_edge = edgelist[new_state]
                            if paths is not None and indmin>=0:
                                selpath = paths[currentid][indmin]
                        elif len(vn)==2:
                            prox_edge = vn[1]
                            selpath = traci.simulation.findRoute(currentid,end_edge[vehicle]).edges
                        else:
                            prox_edge = vn[0]
                            selpath = traci.simulation.findRoute(currentid,end_edge[vehicle]).edges
                    if len(vn2)>2:
                        if NUM_AGENTS==1 and currentid==roadid and PLAY_AUDIO:
                            playAudio(mapdata,currentid,prox_edge,LANG)
                    next_edge[vehicle] = prox_edge.getID()
                    for s in vn:
                        if s.getID() in works:
                            tweet(works,s.getID(),vehs,end_edge,mapdata,True,vehs[vehicle],LANG,WORKS_THRESHOLD)
                            print(str(vehicle)+" TWEETED: hey, there's work in progress at "+str(s.getID()))
                    if prox_edge.getID() == end_edge[vehicle]:
                        print(vehicle+' ARRIVED')
                        if vehicle not in correctlyarrived:
                            correctlyarrived.append(vehicle)
                    # print(str(vehicle)+'on edge '+str(currentid)+' target '+str(prox_edge.getID())+' current target '+str(traci.vehicle.getRoute(vehicle)[-1]))
                    # traci.vehicle.changeTarget(vehicle,prox_edge.getID())
                    traci.vehicle.setRoute(vehicle,traci.simulation.findRoute(roadid,prox_edge.getID()).edges)
                    # colorvalue = sum(list(car_color(list(targets.keys())[list(targets.values()).index(end_edge[vehicle])]))[0:3])
                    if colorstreet and selpath is not None:
                        colorvalue = getIfromRGB(list(car_color(list(targets.keys())[list(targets.values()).index(end_edge[vehicle])]))[0:3])
                        for stre in edgelist:
                            if stre.getID() in selpath:
                                traci.edge.setParameter(stre.getID(),'color',colorvalue)
                            else:
                                traci.edge.setParameter(stre.getID(),'color',0)
                        # traci.edge.setParameter(prox_edge.getID(),'color',colorvalue)
                        # if vehicle in prev_edge:
                        #     traci.edge.setParameter(prev_edge[vehicle],'color',10)
                prev_edge[vehicle] = roadid
                if roadid not in passed[vehicle]:
                    passed[vehicle][roadid] = 1
                else:
                    passed[vehicle][roadid] += 1
        if secondcounter%secondtot == 0:
            scounter += 1
            if scounter%60 == 0:
                mcounter += 1
                if INCLUDE_RANDOM and tottoarrive+1<NUM_VEHICLES:
                    spawned, spawnedRand = spawnRandom(graphdict)
                    if spawned:
                        print("Spawned random vehicle")
                        tottoarrive += 1 if NUM_AGENTS==0 else 0
                        vehs[spawnedRand] = VehicleData(spawnedRand,spawnedRand.replace('_vehicle',''),scounter*(mcounter+1),'')
                if INCLUDE_BUS and tottoarrive+1<NUM_VEHICLES:
                    spawned, spawnedBuss = spawnBus(busdata,mcounter)
                    if spawned:
                        print("Spawned bus")
                        tottoarrive += len(spawnedBuss) if NUM_AGENTS==0 else 0
                        for el in spawnedBuss:
                            vehs[el] = VehicleData(el,el+'route',scounter*(mcounter+1),'')
        secondcounter += 1
        
    traci.close()
    # print("VEHICLE || ALGORITHM || AVG_SPEED || TRAVELED_DISTANCE || DESTINATION")
    retds = []
    for vehicle in vehs:
        if vehs[vehicle].arrived:
            spedlen = len(vehs[vehicle].speeds)
            avgspeed = 0
            if spedlen != 0:
                avgspeed = sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds)
            retds.append((vehicle,vehs[vehicle].alg,avgspeed,vehs[vehicle].dist,vehs[vehicle].fuelconsumption,vehs[vehicle].waitingtime,vehs[vehicle].noiseemission,vehs[vehicle].co2emission,vehs[vehicle].traveltime))
        # print(str(vehicle)+" || "+str(vehs[vehicle].alg)+" || "+str(sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds))+" || "+str(vehs[vehicle].dist)+" || "+str(vehs[vehicle].dest))
    speed_time_averages = []
    speed_space_averages = []
    flow_averages = []
    for t in destinations:
        checkpoint = None
        for c in checkpoints:
            if t[1] in checkpoints[c]['place']:
                checkpoint = checkpoints[c]
        if len(checkpoint['speed'])>0:
            speed_time_averages.append(sum(checkpoint['speed'])/len(checkpoint['speed'])*t[2])
            inversesum = 0
            for sp in checkpoint['speed']:
                inversesum += 1/sp
            speed_space_averages.append(len(checkpoint['speed'])/inversesum*t[2])
            flow_averages.append(checkpoint['flow']*3600/scounter*t[2])
    speed_tavg = sum(speed_time_averages)/100
    speed_savg = sum(speed_space_averages)/100
    flow_avg = sum(flow_averages)/100
    # plt.matshow(behaviour_db[0])
    # plt.colorbar()
    # plt.show()
    return retds,NUM_AGENTS,correctlyarrived,speed_tavg,speed_savg,flow_avg