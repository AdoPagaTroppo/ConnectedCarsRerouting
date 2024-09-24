
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
import subprocess

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
            # print('Fake signal by '+str(vehicle.id))

def neighbor_edges(edge, graphdict, connections, net, edgelist):
    #Find neighbor edges
    res = []
    #get all the incoming and outgoing edges of the end node
    n1 = edge.getToNode()
    for e in (n1.getIncoming()+n1.getOutgoing()):
        if e.allows('passenger') and not e.getID().__contains__(':'):
            if e.getID()!=edge.getID() and connection_exists(edge.getFromNode().getID(),e.getFromNode().getID(),e.getToNode().getID(),graphdict,connections):
                res.append(e)
                # res.append(edgelist.index(e))
    #get all the incoming and outgoing edges of the starting node
    n2 = edge.getFromNode()
    for e in (n2.getIncoming()+n2.getOutgoing()):
        if e.allows('passenger') and not e.getID().__contains__(':'):
            if e.getID()!=edge.getID() and connection_exists(edge.getFromNode().getID(),e.getFromNode().getID(),e.getToNode().getID(),graphdict,connections):
                res.append(e)
                # res.append(edgelist.index(e))
    return(res)

def state_space(n_node, tHor,mapdata,target):
    edgelist = mapdata.edgelist
    #State space for a given horizon: iteratively check every neighbor and add it to the state space
    stsp = [n_node]
    for t in range(tHor):
        newSpace = stsp.copy()
        for n in stsp:
            ed = edgelist[n]
            for x in valid_neighbors(ed,mapdata,target,checkIngoing=True):
            # for x in neighbor_edges(ed,graphdict,connections,net,edgelist):
                xid = edgelist.index(x)
                if xid not in newSpace:
                    newSpace.append(xid)
        stsp = newSpace
    return(np.array(stsp))

def compute_reward(ss,passed,mapdata,target_edge,works,vehicle):
    worksweight = 20 # tochange, work weight must depend on work area travel time
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
        # path = traci.simulation.findRoute(edid,target_edge,'routerByDistance').edges
        path = build_path(mapdata,edid,target_edge,'e_dijkstra')
        pathlen = 0
        if path is not None and len(path)!=0:
            pathlen = calculate_pathlen(path,works,mapdata)
        else:
            pathlen = math.inf
        if edid not in works and path is not None:
            path2 = build_path(mapdata,edid,target_edge,'e_dijkstra',forbidnode=works)
            pathlen2 = 0
            if path2 is not None and len(path2)!=0:
                pathlen2 = calculate_pathlen(path2,works,mapdata)
                if pathlen2<=pathlen:
                    path = path2
                    pathlen = pathlen2
        # print('edge '+str(edid)+' distance '+str(pathlen))
        traveltime = -traci.edge.getTraveltime(edid)
        veh_num = -traci.edge.getLastStepVehicleNumber(edid)
        carnum = veh_num-busnum
        density = (5*carnum+7*busnum)/net.getEdge(edid).getLength()
        if density>1:
            density = 1
        # road_in_wip = -(worksweight*net.getEdge(edid).getLength() if edid in works else 0)
        road_in_wip = -(worksweight*traci.edge.getTraveltime(edid) if edid in works else 0)
        road_repeat = -(70/totrepeat+100*(passed[edid]-1) if edid in passed else 0) # to avoid going on the same streets, useful for roundabouts
        repeat_in_path = 0 # to avoid selecting paths with already-traversed streets, useful for roundabouts and dodging signalled wip areas without going suboptimal
        for id in passed:
            if id in path:
                repeat_in_path += -100
        tail_in_wip = 0
        if edid in vehicle.weightened:
            tail_in_wip = -worksweight*veh_num/vehicle.weightened[edid]
        lastspeed = traci.edge.getLastStepMeanSpeed(edid)
        roadspeed = -(net.getEdge(edid).getSpeed()-lastspeed)
        waitingtime = -traci.edge.getWaitingTime(edid)
        # r[i] = traveltime+veh_num+road_in_wip+road_repeat+tail_in_wip-pathlen-10*density+roadspeed+pred_works
        # r[i] = 1
        r[i] = -pathlen+roadspeed*density+repeat_in_path+road_repeat+10*waitingtime # product between speed and density to define dependency between density and car speeds
        # r[i] = -pathlen+density+repeat_in_path+road_repeat+waitingtime # product between speed and density to define dependency between density and car speeds
        # r[i] = 5*roadspeed*density+road_repeat+road_in_wip # product between speed and density to define dependency between density and car speeds
        # print('reward edge '+str(edid)+': '+str(pathlen)+' + '+str(roadspeed)+' + '+str(5*roadspeed*density)+str(repeat_in_path)+' + '+str(road_repeat)+' = '+str(r[i]))
        # print('reward edge '+str(edid)+': '+str(pathlen)+' + '+str(roadspeed)+' + '+str(5*roadspeed*density)+str(repeat_in_path)+' + '+str(road_repeat)+' = '+str(r[i]))
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

def update_works(works):
    coll_list = traci.simulation.getCollidingVehiclesIDList()
    ret_works = []
    for l in coll_list:
        id = traci.vehicle.getRoadID(l)
        if id not in works and id not in ret_works and len(id)>1:
            ret_works.append(id)
    return ret_works

def load_offline_paths(scenario,num_algs,consider_works):
    data_structure = np.load('paths_'+str(scenario)+'_'+str(num_algs)+('_wip' if consider_works else '')+'.npy',allow_pickle=True)
    # print(data_structure)
    paths = {}
    for i in data_structure.item():
        # print(i)
        paths[i] = data_structure.item()[i]
        # print(data_structure.item()[i])
        # print('end')
    return paths

def calculate_pathlen(path,works,mapdata):
    worksweight = 20
    net = mapdata.net
    pathlen = 0
    for j in path:
        if j!='SUCC':
            # pathlen += traci.edge.getTraveltime(j)*(1 if j not in works else 10) # keep track of signalled wip areas
            pathlen += net.getEdge(j).getLength()/net.getEdge(j).getSpeed()*(1 if j not in works else worksweight)
    return pathlen
    

DIJKSTRA_BASED_REROUTING = False


def single_sim(NUM_VEHICLES, PERC_UNI_CARS, SHOW_GUI, T_HORIZON, STEP_SIZE, INCLUDE_BUS, INCLUDE_RANDOM, START_TIME, PERC_AGENT_CARS, mapdata, USE_DESIRED_AGENTS=False, DESIRED_AGENTS=0, NUM_ALGS=1, ONLINE=False, CONSIDER_WORKS=False):
    LANG = 'it'
    SCENARIO = mapdata.scenario
    PERC_AGENT_CARS = DESIRED_AGENTS/NUM_VEHICLES if USE_DESIRED_AGENTS else PERC_AGENT_CARS*PERC_UNI_CARS
    NUM_AGENTS = NUM_VEHICLES*PERC_AGENT_CARS
    # subprocess.run(["D:\\Users\\Principale\\Downloads\\eclipse-mosaic-24.1\\mosaic.bat","-c","D:\\Users\\Principale\\Desktop\\Uni\\M2 Anno\\Tesi\\ThesisProjectFolder\\ConnectedCarsRerouting\\scenario_config_json"])
    
    # stdout, stderr = p.communicate()
    traci.start(['sumo'+('-gui' if SHOW_GUI else ''),'-c','osm_'+str(SCENARIO)+'.sumocfg','--step-length',str(STEP_SIZE)])
    graphdict = mapdata.graphdict
    net = mapdata.net
    targets = mapdata.targets
    edgelist = mapdata.edgelist
    destinations = mapdata.destinations
    checkpoints = parse_file_for_checkpoints(str(SCENARIO)+'ScenarioData/checkpoints.txt')
    NUM_FOES = NUM_VEHICLES-NUM_AGENTS
    vehs = spawnUncontrolledCars(int(NUM_FOES),mapdata)
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
    # tottoarrive = int(PERC_UNI_CARS*NUM_VEHICLES if NUM_AGENTS==0 else NUM_AGENTS)
    tottoarrive = PERC_UNI_CARS*NUM_VEHICLES
    insim = []
    print('ready to go')
    correctlyarrived = []
    works = mapdata.works if CONSIDER_WORKS else {}
    signalled_works = []
    for w in works:
        traci.edge.setParameter(w,'color',10)
    passed = {}
    behaviour_created = []
    behaviour_db = np.zeros((len(targets)*NUM_ALGS, len(edgelist), len(edgelist)))
    checkfromfile = True
    colorpars = []
    try:
        f = open('sim_config.txt','r')
        colorparams = f.readline()
        f.close()
        colorpars = colorparams.split(';')
    except:
        checkfromfile = False
    colorstreet = False if not checkfromfile else eval(colorpars[2])
    colorreward = False if not checkfromfile else eval(colorpars[4])
    colorpaths = False if not checkfromfile else eval(colorpars[3])
    print(str(colorstreet)+' '+str(colorreward)+' '+str(colorpaths))
    rewards4colors = {}
    paths_db = {} if ONLINE else load_offline_paths(SCENARIO,NUM_ALGS,CONSIDER_WORKS)
    agent_co2s = []
    agent_fuels = []
    agent_noises = []
    foes_co2s = []
    foes_fuels = []
    foes_noises = []
    while True:
        # for v in traci.simulation.getArrivedIDList():
        # for v in traci.simulation.getArrivedIDList():
        #     # if NUM_AGENTS != 0:
        #         if not v.__contains__('bus') or not v.__contains__('random'):
        #             totarrived += 1
        # if totarrived >= tottoarrive:
        if len(insim)==0 and totarrived>=1:
            break
        traci.simulationStep()
        vehicles_in_sim = traci.vehicle.getIDList()
        print(vehicles_in_sim)
        temp_agent_co2s = []
        temp_agent_fuels = []
        temp_agent_noises = []
        temp_foes_co2s = []
        temp_foes_fuels = []
        temp_foes_noises = []
        prev_signalled_works = len(signalled_works)
        for w in update_works(works):
            works[w] = 0
        for vehicle in vehs:
            if vehicle in vehicles_in_sim:
                if vehicle not in insim:
                    insim.append(vehicle)
            else:
                if vehicle in insim:
                    insim.remove(vehicle)
                    vehs[vehicle].arrived = True
                    if not vehicle.__contains__('bus') and not vehicle.__contains__('random'):
                        totarrived += 1
            # if(vehicle in traci.simulation.getDepartedIDList()):
            #     insim.append(vehicle)
            # elif vehicle in traci.simulation.getArrivedIDList():
            #     insim.remove(vehicle)
            #     vehs[vehicle].arrived = True
                # if NUM_AGENTS == 0:
                #     totarrived += 1
            if vehicle in insim:
                vehicle_speed = traci.vehicle.getSpeed(vehicle)
                vehicle_fuel = traci.vehicle.getFuelConsumption(vehicle)*STEP_SIZE
                vehicle_noise = traci.vehicle.getNoiseEmission(vehicle)
                vehicle_co2 = traci.vehicle.getCO2Emission(vehicle)*STEP_SIZE
                if vehicle.__contains__('agent'):
                    temp_agent_co2s.append(vehicle_co2)
                    temp_agent_noises.append(vehicle_noise)
                    temp_agent_fuels.append(vehicle_fuel)
                elif vehicle.__contains__('veh'):
                    temp_foes_co2s.append(vehicle_co2)
                    temp_foes_fuels.append(vehicle_fuel)
                    temp_foes_noises.append(vehicle_noise)
                vehs[vehicle].speeds.append(vehicle_speed)
                vehs[vehicle].dist = traci.vehicle.getDistance(vehicle)
                vehs[vehicle].fuelconsumption.append(vehicle_fuel)
                vehs[vehicle].waitingtime += traci.vehicle.getWaitingTime(vehicle)
                vehs[vehicle].co2emission.append(vehicle_co2)
                vehs[vehicle].noiseemission.append(vehicle_noise)
                vehs[vehicle].traveltime += (1 if secondcounter%secondtot==0 else 0) # (scounter+1)-vehs[vehicle].depart
                if vehicle not in passed:
                    passed[vehicle] = {}
                roadid = traci.vehicle.getRoadID(vehicle)
                if secondcounter%secondtot == 0 and not roadid.__contains__(':'):
                    vehs[vehicle].currentroad = roadid
                    randomSignal(0.005,vehs[vehicle],roadid,works)
                if roadid in works and not roadid.__contains__(':'):
                    if secondcounter%secondtot == 0:
                        if tweet(works,roadid,vehs,end_edge,mapdata,False,vehs[vehicle],LANG,WORKS_THRESHOLD):
                            if roadid not in signalled_works:
                                signalled_works.append(roadid)
                                traci.edge.setParameter(roadid,'color',12)
                    traci.vehicle.setSpeed(vehicle,net.getEdge(roadid).getSpeed()/20)
                    # print(str(vehicle)+" TWEETED: hey, there's work in progress at "+str(roadid))
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
                        ss = state_space(statecars,T_HORIZON,mapdata,end_edge[vehicle])
                        # print(ss)
                        # print(statecars)
                        ss_edges = conv_ss2edges(ss,edgelist)
                        
                        for s in ss_edges:
                            if s.getID() in works:
                                if tweet(works,s.getID(),vehs,end_edge,mapdata,True,vehs[vehicle],LANG,WORKS_THRESHOLD):
                                    if s.getID() not in signalled_works:
                                        signalled_works.append(s.getID())
                                        traci.edge.setParameter(s.getID(),'color',12)
                        if ONLINE and ((currentid,end_edge[vehicle]) not in behaviour_created or len(signalled_works)!=prev_signalled_works):
                                behaviour_db,paths = online_create_behaviours(mapdata,NUM_ALGS,end_edge[vehicle],ss_edges,behaviour_db,behaviour_created,signalled_works,len(signalled_works)!=prev_signalled_works)
                                for p in paths:
                                    if p not in paths_db:
                                        paths_db[p] = [None]*(len(targets)*NUM_ALGS)
                                    for i in range(NUM_ALGS):
                                        paths_db[p][agents[vehicle].targetIndex+i] = paths[p][i]
                                for v in ss_edges:
                                    if (v.getID(),end_edge[vehicle]) not in behaviour_created or len(signalled_works)!=prev_signalled_works:
                                        behaviour_created.append((v.getID(),end_edge[vehicle]))
                                prev_signalled_works = len(signalled_works)
                              
                        if len(vn)>2:
                            # print(vehicle)
                                    # print(str(vehicle)+" TWEETED: hey, there's work in progress at "+str(s.getID()))
                            # print(ss_edges)
                            # print(signalled_works)
                            r = compute_reward(ss,passed[vehicle],mapdata,end_edge[vehicle],signalled_works,vehs[vehicle])
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
                            if ONLINE:
                                if paths is not None and indmin>=0:
                                    selpath = paths[currentid][indmin]
                                    vehs[vehicle].selected_id = indmin
                            else:
                                if indmin>=0:
                                    selpath = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                                    vehs[vehicle].selected_id = indmin
                        elif len(vn)==2:
                            prox_edge = vn[1]
                            indmin = vehs[vehicle].selected_id
                            selpath = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                        else:
                            prox_edge = vn[0]
                            indmin = vehs[vehicle].selected_id
                            selpath = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                    if len(vn2)>2:
                        if NUM_AGENTS==1 and currentid==roadid and PLAY_AUDIO:
                            playAudio(mapdata,currentid,prox_edge,LANG)
                    next_edge[vehicle] = prox_edge.getID()
                    if prox_edge.getID() == end_edge[vehicle]:
                        print(vehicle+' ARRIVED')
                        if vehicle not in correctlyarrived:
                            correctlyarrived.append(vehicle)
                    print('time ['+str(int(secondcounter/secondtot))+':'+str(int(secondcounter%secondtot))+'] - '+str(vehicle)+' on edge '+str(currentid)+' target '+str(prox_edge.getID())+' current target '+str(traci.vehicle.getRoute(vehicle)[-1]))
                    # traci.vehicle.changeTarget(vehicle,prox_edge.getID())
                    traci.vehicle.setRoute(vehicle,traci.simulation.findRoute(roadid,prox_edge.getID()).edges)
                    print('selected edge '+str(prox_edge.getID())+' selected index '+str(vehs[vehicle].selected_id))
                    # colorvalue = sum(list(car_color(list(targets.keys())[list(targets.values()).index(end_edge[vehicle])]))[0:3])
                    # if colorstreet and selpath is not None:
                    if colorstreet and ((colorpaths) or (selpath is not None and not colorpaths)):
                        # print('selected alg '+index2alg(indmin))
                        colorvalue = getIfromRGB(list(car_color(list(targets.keys())[list(targets.values()).index(end_edge[vehicle])]))[0:3])
                        for stre in edgelist:
                            if stre.getID() not in works:
                                traci.edge.setParameter(stre.getID(),'color',0)
                        if colorpaths:
                            for selpat in range(NUM_ALGS): # range(len(paths_db[currentid])):
                                colorv = getIfromRGB(list(alg_color(index2alg(selpat)))[0:3])
                                # print(paths[currentid][selpat])
                                # print(paths_db[currentid][agents[vehicle].targetIndex][selpat])
                                for e in paths_db[currentid][agents[vehicle].targetIndex+selpat]:
                                    # pathlen += net.getEdge(e).getLength()
                                    # pathlen += traci.edge.getTraveltime(e)
                                    if e not in works:
                                        traci.edge.setParameter(e,'color',colorv)
                                # print('pathlen '+str(pathlen))
                        else:
                            colorv = getIfromRGB(list(alg_color(index2alg(indmin)))[0:3])
                            for e in selpath:
                                if e not in works:
                                    traci.edge.setParameter(e,'color',colorv)
                        # traci.edge.setParameter(prox_edge.getID(),'color',colorvalue)
                        if vehicle in prev_edge:
                            traci.edge.setParameter(prev_edge[vehicle],'color',0)
                print('time ['+str(int(secondcounter/secondtot))+':'+str(int(secondcounter%secondtot))+'] - '+str(vehicle)+' on edge '+str(roadid))
                    
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
                        # tottoarrive += 1 if NUM_AGENTS==0 else 0
                        vehs[spawnedRand] = VehicleData(spawnedRand,spawnedRand.replace('_vehicle',''),scounter*(mcounter+1),'')
                if INCLUDE_BUS and tottoarrive+1<NUM_VEHICLES:
                    spawned, spawnedBuss = spawnBus(busdata,mcounter)
                    if spawned:
                        print("Spawned bus")
                        # tottoarrive += len(spawnedBuss) if NUM_AGENTS==0 else 0
                        for el in spawnedBuss:
                            vehs[el] = VehicleData(el,el+'route',scounter*(mcounter+1),'')
        secondcounter += 1
        agent_co2s.append(0 if len(temp_agent_co2s)==0 else np.mean(temp_agent_co2s))
        agent_noises.append(0 if len(temp_agent_noises)==0 else np.mean(temp_agent_noises))
        agent_fuels.append(0 if len(temp_agent_fuels)==0 else np.mean(temp_agent_fuels))
        foes_co2s.append(0 if len(temp_foes_co2s)==0 else np.mean(temp_foes_co2s))
        foes_noises.append(0 if len(temp_foes_noises)==0 else np.mean(temp_foes_noises))
        foes_fuels.append(0 if len(temp_foes_fuels)==0 else np.mean(temp_foes_fuels))
        
    # print("VEHICLE || ALGORITHM || AVG_SPEED || TRAVELED_DISTANCE || DESTINATION")
    retds = []
    for vehicle in vehs:
        if vehs[vehicle].arrived and (not vehicle.__contains__('bus') and not vehicle.__contains__('random')):
            # spedlen = len(vehs[vehicle].speeds)
            # avgspeed = 0
            # if spedlen != 0:
            #     avgspeed = sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds)
            retds.append((vehicle,vehs[vehicle].alg,(0 if len(vehs[vehicle].speeds)==0 else np.mean(vehs[vehicle].speeds)),vehs[vehicle].dist,sum(vehs[vehicle].fuelconsumption),vehs[vehicle].waitingtime,sum(vehs[vehicle].noiseemission),sum(vehs[vehicle].co2emission),vehs[vehicle].traveltime))
        # elif not vehs[vehicle].arrived and (not vehicle.__contains__('bus') or not vehicle.__contains__('random')) and vehs[vehicle].route is not None:
        #     counting = False
        #     for ed in vehs[vehicle].route:
        #         vehs[vehicle].traveltime += (net.getEdge(ed).getLength()/net.getEdge(ed).getSpeed()*(20 if ed in works else 1) if counting else 0)
        #         if ed == vehs[vehicle].currentroad:
        #             counting = True
        #     retds.append((vehicle,vehs[vehicle].alg,(0 if len(vehs[vehicle].speeds)==0 else np.mean(vehs[vehicle].speeds)),vehs[vehicle].dist,sum(vehs[vehicle].fuelconsumption),vehs[vehicle].waitingtime,sum(vehs[vehicle].noiseemission),sum(vehs[vehicle].co2emission),vehs[vehicle].traveltime))
        # print(str(vehicle)+" || "+str(vehs[vehicle].alg)+" || "+str(sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds))+" || "+str(vehs[vehicle].dist)+" || "+str(vehs[vehicle].dest))
    traci.close()
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
                if sp>0:
                    inversesum += 1/sp
            if inversesum>0:
                speed_space_averages.append(len(checkpoint['speed'])/inversesum*t[2])
                flow_averages.append(checkpoint['flow']*3600/scounter*t[2])
    speed_tavg = sum(speed_time_averages)/100
    speed_savg = sum(speed_space_averages)/100
    flow_avg = sum(flow_averages)/100
    
    agent_time_data = {}
    foes_time_data = {}
    agent_time_data['co2'] = agent_co2s
    agent_time_data['fuel'] = agent_fuels
    agent_time_data['noise'] = agent_noises
    foes_time_data['co2'] = foes_co2s
    foes_time_data['fuel'] = foes_fuels
    foes_time_data['noise'] = foes_noises
            
    # plt.matshow(behaviour_db[0])
    # plt.colorbar()
    # plt.show()
    return retds,NUM_AGENTS,correctlyarrived,speed_tavg,speed_savg,flow_avg,agent_time_data,foes_time_data