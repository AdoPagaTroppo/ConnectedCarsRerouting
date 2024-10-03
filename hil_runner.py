from threading import Thread
from pynmeagps import NMEAReader
import serial
from time import sleep

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
from traci import TraCIException
from single_simulation import calculate_pathlen_meters
from single_simulation import compute_reward
from text2speech_handler import search_next

class Message():

    def __init__(self,lat,lon):
        self.lat = lat
        self.lon = lon

def reading_thread(ser):
    global msg
    global stop
    stop = False
    print('thread is on')
    msg = None
    try:
        nmr = NMEAReader(ser)
        while True:
            (raw_data, msg2) = nmr.read() #msg will be global variable that main will read
            # block for a moment
            msgstr = str(msg2)
            # print(msgstr)
            if msgstr.__contains__('lat=') and msgstr.__contains__('lon=') and (not msgstr.__contains__('lat=,') and not msgstr.__contains__('lon=,')):
                f = open('log_gps_vil.txt','a')
                msg = msg2
                # print(f"lat: {msg.lat} - lon: {msg.lon}")
                f.write(str(msg.lat)+':'+str(msg.lon)+'\n')
                f.close()
            # while not proceed:
            #     pass    
            # sleep(0.10)
    except KeyboardInterrupt:
        ser.close()

def reading_thread_file(mapdata):
    global msg
    global stop
    # global initial_edge
    # global x
    # global y
    print('thread is on')
    msg = None
    stop = False
    # initial_edge = None
    coords = []
    f = open('log_gps_vil.txt','r')
    dats = f.readlines()
    for dat in dats:
        cs = dat.split(':')
        coords.append([float(cs[0]),float(cs[1])])
    i = 0
    sleep(3.0)
    for i in range(len(coords)):
        msg = Message(coords[i][0],coords[i][1])
        print(coords[i])
        # initial_edge,x,y = find_closest_edge(mapdata)
        while not proceed:
            pass
        sleep(0.1)
    stop = True
    exit()



def take_first(elem):
    return elem[0]

def find_closest_edge(mapdata,path=None,start_edge=None):
    print('FINDING CLOSEST EDGE')
    net = mapdata.net
    edgelist = mapdata.edgelist
    radius = 20
    x, y = net.convertLonLat2XY(msg.lon, msg.lat) 
    ne_edges = net.getNeighboringEdges(x, y, radius,includeJunctions=False)
    # while len(ne_edges)==0:
    #     x, y = net.convertLonLat2XY(msg.lon, msg.lat) 
    #     ne_edges = net.getNeighboringEdges(x, y, radius,includeJunctions=True)
    #     print('not found yet, coords '+str(msg.lat)+','+str(msg.lon))
    #     pass
    distancesAndEdges_full = sorted([(dist, edge) for edge, dist in ne_edges], key=take_first)
    distancesAndEdges = []
    print(path)
    for dae in distancesAndEdges_full:
        print(dae[1])
        if dae[1].allows('passenger'):
            if path is None:
            # dist, closestEdge = dae
            # print(closestEdge)
            # break
                distancesAndEdges.append(dae)
            else:
                if dae[1].getID() in path:
                    distancesAndEdges.append(dae)
                else:
                    if start_edge is not None:
                        p2 = traci.simulation.findRoute(start_edge,dae[1].getID()).edges
                        if len(p2)>0 and len(p2)<4:
                            distancesAndEdges.append(dae)
    print(distancesAndEdges)
    if len(distancesAndEdges)==0:
        return None,0,0
    dist, closestEdge = distancesAndEdges[0]
    initial_edge = closestEdge.getID()
    print('x: '+str(x)+' y: '+str(y)+' '+str(net.getEdge(initial_edge)))
    return initial_edge,x,y

def check_audio_from_params():
    f = open('sim_config.txt','r')
    c = f.readline().split(';')[5]
    c = eval(c)
    f.close()
    return c

WORKS_THRESHOLD = 1

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

# def compute_reward(ss,passed,mapdata,target_edge,works,vehicle):
#     worksweight = 20 # tochange, work weight must depend on work area travel time
#     edgelist = mapdata.edgelist
#     net = mapdata.net
#     fullen = len(edgelist)
#     r = [0]*fullen
#     totrepeat = 0
#     for i in ss:
#         if edgelist[i].getID() in passed:
#             totrepeat += 1
#     for i in ss:
#         edge = edgelist[i]
#         edid = edge.getID()
#         veh_list = traci.edge.getIDList()
#         busnum = 0
#         for id in veh_list:
#             if id.__contains__('bus'):
#                 busnum += 1
#         # path = traci.simulation.findRoute(edid,target_edge,'routerByDistance').edges
#         path = build_path(mapdata,edid,target_edge,'e_dijkstra')
#         pathlen = 0
#         for j in path:
#             if j!='SUCC':
#                 # pathlen += traci.edge.getTraveltime(j)*(1 if j not in works else 10) # keep track of signalled wip areas
#                 pathlen += net.getEdge(j).getLength()/net.getEdge(j).getSpeed()*(1 if j not in works else worksweight)
#         if edid not in works:
#             path2 = build_path(mapdata,edid,target_edge,'e_dijkstra',forbidnode=works)
#             pathlen2 = 0
#             for j in path2:
#                 if j!='SUCC':
#                     # pathlen += traci.edge.getTraveltime(j)*(1 if j not in works else 10) # keep track of signalled wip areas
#                     pathlen2 += net.getEdge(j).getLength()/net.getEdge(j).getSpeed()
#             # print('pathlen1 '+str(pathlen)+' pathlen2 '+str(pathlen2))
#             if pathlen2<=pathlen:
#                 path = path2
#                 pathlen = pathlen2
#         # print('edge '+str(edid)+' distance '+str(pathlen))
#         traveltime = -traci.edge.getTraveltime(edid)
#         veh_num = -traci.edge.getLastStepVehicleNumber(edid)
#         carnum = veh_num-busnum
#         density = (5*carnum+7*busnum)/net.getEdge(edid).getLength()
#         if density>1:
#             density = 1
#         # road_in_wip = -(worksweight*net.getEdge(edid).getLength() if edid in works else 0)
#         road_in_wip = -(worksweight*traci.edge.getTraveltime(edid) if edid in works else 0)
#         road_repeat = -(70/totrepeat+100*(passed[edid]-1) if edid in passed else 0) # to avoid going on the same streets, useful for roundabouts
#         repeat_in_path = 0 # to avoid selecting paths with already-traversed streets, useful for roundabouts and dodging signalled wip areas without going suboptimal
#         for id in passed:
#             if id in path:
#                 repeat_in_path += -100
#         tail_in_wip = 0
#         if edid in vehicle.weightened:
#             tail_in_wip = -worksweight*veh_num/vehicle.weightened[edid]
#         lastspeed = traci.edge.getLastStepMeanSpeed(edid)
#         roadspeed = -(net.getEdge(edid).getSpeed()-lastspeed)
#         # r[i] = traveltime+veh_num+road_in_wip+road_repeat+tail_in_wip-pathlen-10*density+roadspeed+pred_works
#         r[i] = 1
#         # r[i] = -pathlen+roadspeed+5*roadspeed*density+repeat_in_path+road_repeat # product between speed and density to define dependency between density and car speeds
#         # r[i] = 5*roadspeed*density+road_repeat+road_in_wip # product between speed and density to define dependency between density and car speeds
#         # print('reward edge '+str(edid)+': '+str(pathlen)+' + '+str(roadspeed)+' + '+str(5*roadspeed*density)+str(repeat_in_path)+' + '+str(road_repeat)+' = '+str(r[i]))
#         # r[i] = -pathlen-(1000000 if edid in works else 0)-(70/totrepeat+100*(passed[edid]-1) if edid in passed else 0)
#         # print('edge '+str(edid)+' reward '+str(r[i]))
#         # da aggiungere un aggiornamento di costo tenendo conto di comunicazione e interfacciamento con social + fiducia + nnumero di persone che twittano stessa cosa
#         # aggiungere costi su incidenti + lavori (simulazioni con stessi lavori e scenari significativi)
#     return np.array(r)
    
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

DIJKSTRA_BASED_REROUTING = False


def single_sim(NUM_VEHICLES, PERC_UNI_CARS, SHOW_GUI, T_HORIZON, STEP_SIZE, INCLUDE_BUS, INCLUDE_RANDOM, START_TIME, PERC_AGENT_CARS, mapdata, USE_DESIRED_AGENTS=False, DESIRED_AGENTS=0, NUM_ALGS=1, ONLINE=False, CONSIDER_WORKS=False):
    LANG = 'it'
    SCENARIO = mapdata.scenario
    PERC_AGENT_CARS = DESIRED_AGENTS/NUM_VEHICLES if USE_DESIRED_AGENTS else PERC_AGENT_CARS*PERC_UNI_CARS
    NUM_AGENTS = NUM_VEHICLES*PERC_AGENT_CARS
    PLAY_AUDIO = check_audio_from_params()
    # open('log_gps_vil.txt','w').close()
    # subprocess.run(["D:\\Users\\Principale\\Downloads\\eclipse-mosaic-24.1\\mosaic.bat","-c","D:\\Users\\Principale\\Desktop\\Uni\\M2 Anno\\Tesi\\ThesisProjectFolder\\ConnectedCarsRerouting\\scenario_config_json"])
    # ser = serial.Serial('/dev/rfcomm0', 9600)
    global proceed
    proceed = False
    # create a thread
    # thread = Thread(target=reading_thread, args=[ser])
    thread = Thread(target=reading_thread_file, args=[mapdata])
    # run the thread
    thread.daemon = True
    thread.start()
    # sleep(1.0)
    traci.start(['sumo'+('-gui' if SHOW_GUI else ''),'-c','osm_'+str(SCENARIO)+'.sumocfg','--step-length',str(STEP_SIZE)])
    graphdict = mapdata.graphdict
    net = mapdata.net
    targets = mapdata.targets
    edgelist = mapdata.edgelist
    destinations = mapdata.destinations
    checkpoints = parse_file_for_checkpoints(str(SCENARIO)+'ScenarioData/checkpoints.txt')
    while msg is None:
        pass
    print('message read')
    # x, y = net.convertLonLat2XY(msg.lon, msg.lat) 
    # ne_edges = net.getNeighboringEdges(x, y, radius)
    # while(len(ne_edges)==0):
    #     ne_edges = net.getNeighboringEdges(x, y, radius)
    #     x, y = net.convertLonLat2XY(msg.lon, msg.lat)
    # print(ne_edges)
    print('looking for initial edge')
    initial_edge,x,y = find_closest_edge(mapdata)
    while initial_edge is None:
        initial_edge, x, y = find_closest_edge(mapdata)
        proceed = True
    proceed = False
    out_of_course_counter = 0
    if initial_edge is not None:
        initial_edge,x,y = find_closest_edge(mapdata)
        vehs = spawnUncontrolledCars(int((PERC_UNI_CARS-PERC_AGENT_CARS)*NUM_VEHICLES),mapdata)
        agents,end_edge = spawnControlledCars(NUM_AGENTS,mapdata,NUM_ALGS,vehs,ONLINE,initial_edge)
        print('loaded vehicles')
        agent_car = list(agents.keys())[0]
        
        traci.vehicle.moveToXY(agent_car,initial_edge,0,x,y)
        # traci.vehicle.setSpeed(agent_car,0.101)
        totarrived = 0
        rerouting_occurred = {}
        for vehicle in vehs:
            rerouting_occurred[vehicle] = False
        forbid_rerouting = []
        forbid_rerouting.append('agent')
        prev_edge = {}
        cl_prev_edge = {}
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
        rewards4colors = {}
        paths_db = {} if ONLINE else load_offline_paths(SCENARIO,NUM_ALGS,CONSIDER_WORKS)
        agent_co2s = []
        agent_fuels = []
        agent_noises = []
        foes_co2s = []
        foes_fuels = []
        foes_noises = []
        current_edge = None
        played_audio_edge = None
        played_for_crossing = False
        indmin = -1
        while True:
            proceed = False
            move = False
                            
            for v in traci.simulation.getArrivedIDList():
                # if NUM_AGENTS != 0:
                    if not v.__contains__('bus') or not v.__contains__('random'):
                        totarrived += 1
            if totarrived == tottoarrive:
                break
            traci.simulationStep()
            proceed = True
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
                if(vehicle in traci.simulation.getDepartedIDList()):
                    insim.append(vehicle)
                elif vehicle in traci.simulation.getArrivedIDList():
                    insim.remove(vehicle)
                    vehs[vehicle].arrived = True
                    # if NUM_AGENTS == 0:
                    #     totarrived += 1
                if vehicle in insim:
                    played_for_crossing = False
                    roadid = traci.vehicle.getRoadID(vehicle)
                    try:
                        if vehicle.__contains__('agent') and not stop:
                            # ADD CHECK TO KEEP ROAD WHILE NOT MOVING OR SIMILAR THINGS
                            # ADDITIONALLY, FIX AUDIO
                            old_x = x
                            old_y = y
                            checkr = None
                            if agent_car in cl_prev_edge:
                                if not roadid.__contains__(':'):
                                    checkr = roadid
                                else:
                                    checkr = cl_prev_edge[agent_car]
                            else:
                                checkr = roadid
                            print(checkr)
                            kR = 1
                            current_edge,x,y = find_closest_edge(mapdata,path=(None if indmin<0 else paths_db[checkr][agents[vehicle].targetIndex+indmin]),start_edge=(None if indmin<0 or agent_car not in cl_prev_edge else cl_prev_edge[agent_car]))
                            if current_edge is None:
                                current_edge,x,y = find_closest_edge(mapdata)
                                kR = 0    
                            out_of_course_counter,move = move_car_on_road(mapdata,roadid,agent_car,cl_prev_edge,current_edge,None if indmin<0 else paths_db[checkr][agents[vehicle].targetIndex+indmin],x,y,old_x,old_y,out_of_course_counter,kR=kR)
                    except TraCIException as e:
                        move = False
                        traci.vehicle.setSpeed(agent_car,-1)
                        print('exception here, '+str(e))
                            
                    vehicle_speed = traci.vehicle.getSpeed(vehicle)
                    vehicle_fuel = traci.vehicle.getFuelConsumption(vehicle)*STEP_SIZE
                    vehicle_noise = traci.vehicle.getNoiseEmission(vehicle)
                    vehicle_co2 = traci.vehicle.getCO2Emission(vehicle)*STEP_SIZE
                    vehicle_waiting = traci.vehicle.getWaitingTime(vehicle)
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
                    if vehs[vehicle].waitingtime[-1]==0:
                        if vehicle_waiting>0:
                            vehs[vehicle].waitingtime[-1] = vehicle_waiting
                    else:
                        if vehicle_waiting==0:
                            vehs[vehicle].waitingtime.append(0)
                        else:
                            vehs[vehicle].waitingtime[-1] = vehicle_waiting
                    vehs[vehicle].co2emission.append(vehicle_co2)
                    vehs[vehicle].noiseemission.append(vehicle_noise)
                    vehs[vehicle].traveltime += (1 if secondcounter%secondtot==0 else 0) # (scounter+1)-vehs[vehicle].depart
                    if vehicle not in passed:
                        passed[vehicle] = {}
                    print('roadid '+str(roadid))
                    if not roadid.__contains__(':'):
                        if vehicle not in cl_prev_edge or roadid!=cl_prev_edge[vehicle]:
                            if vehicle.__contains__('agent'):
                                if move:
                                    cl_prev_edge[vehicle] = current_edge
                                else:
                                    cl_prev_edge[vehicle] = roadid
                            else:
                                cl_prev_edge[vehicle] = roadid
                    print('cl prev edge '+str(cl_prev_edge[vehicle]))
                    if agent_car==vehicle and current_edge is not None and move and not roadid.__contains__(':'):
                        roadid = cl_prev_edge[agent_car]
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
                    elif roadid not in works and vehicle!=agent_car:
                        traci.vehicle.setSpeed(vehicle,-1)
                    # traci.vehicle.setSpeed(agent_car,0.101)
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
                        prevrt = None if agent_car not in cl_prev_edge else paths_db[cl_prev_edge[agent_car]][agents[vehicle].targetIndex+vehs[vehicle].selected_id]
                        # currentid = list(traci.vehicle.getRoute(agent_car))[0]
                        print('currentid '+str(currentid))
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
                                            print('new works at '+str(s.getID()))
                            if len(signalled_works)!=prev_signalled_works:
                                print('new works detected')
                            if ONLINE and ((currentid,end_edge[vehicle]) not in behaviour_created or len(signalled_works)!=prev_signalled_works):
                                    print('producing behaviours ')
                                    behaviour_db,paths = online_create_behaviours(mapdata,NUM_ALGS,end_edge[vehicle],ss_edges,behaviour_db,behaviour_created,signalled_works,len(signalled_works)!=prev_signalled_works)
                                    for p in paths:
                                        if p not in paths_db:
                                            paths_db[p] = [None]*len(targets)
                                        paths_db[p][agents[vehicle].targetIndex] = paths[p]
                                    for v in ss_edges:
                                        if (v.getID(),end_edge[vehicle]) not in behaviour_created or len(signalled_works)!=prev_signalled_works:
                                            behaviour_created.append((v.getID(),end_edge[vehicle]))
                                    prev_signalled_works = len(signalled_works)
                                
                            if len(vn)>2:
                                # print(vehicle)
                                        # print(str(vehicle)+" TWEETED: hey, there's work in progress at "+str(s.getID()))
                                # print(ss_edges)
                                # print(signalled_works)
                                r = compute_reward(ss,passed[vehicle],mapdata,end_edge[vehicle],signalled_works,vehs[vehicle],paths_db,agents)
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
                                # if prox_edge.getID()!=played_audio_edge:
                                playAudio(mapdata,currentid,prox_edge,LANG,roundabout=(currentid in mapdata.streets_in_roundabouts))
                                    # played_audio_edge = prox_edge.getID()
                                # vn3 = []
                                # vn3.append(net.getEdge(prox_edge.getID()))
                                # vn3.extend(valid_neighbors(net.getEdge(prox_edge.getID()),mapdata,end_edge[vehicle]))
                                # if len(vn3)>2:
                                #     even_next = paths_db[prox_edge.getID()][agents[vehicle].targetIndex+indmin][1]
                                #     dist = int(net.getEdge(currentid).getLength()+net.getEdge(prox_edge.getID()).getLength())                                    
                                #     playAudio(mapdata,currentid,net.getEdge(even_next),LANG,dist,prox_edge)
                        else:
                            if NUM_AGENTS==1 and PLAY_AUDIO and selpath is not None:
                                print('enter here')
                                true_play_audio, in_roundabout, dist, audio_edge, prev_audio_edge = search_next(mapdata,paths_db[currentid][agents[vehicle].targetIndex+indmin])
                                if currentid in mapdata.streets_in_roundabouts:
                                    true_play_audio = False
                                if true_play_audio and audio_edge!=played_audio_edge:
                                    print('true play audio')
                                    playAudio(mapdata,currentid,net.getEdge(audio_edge),LANG,dist,net.getEdge(prev_audio_edge),in_roundabout,path=(None if not in_roundabout or paths_db[currentid] is None else paths_db[currentid][agents[vehicle].targetIndex+indmin]))
                                    played_audio_edge = audio_edge
                        next_edge[vehicle] = prox_edge.getID()
                        if prox_edge.getID() == end_edge[vehicle]:
                            print(vehicle+' ARRIVED')
                            if vehicle not in correctlyarrived:
                                correctlyarrived.append(vehicle)
                        # rt = traci.simulation.findRoute(roadid,prox_edge.getID()).edges
                        # if len(rt)>0:
                            # traci.vehicle.setRoute(agent_car,traci.simulation.findRoute(roadid,prox_edge.getID()).edges)
                        # traci.vehicle.setRoute(agent_car,paths_db[initial_edge][agents[vehicle].targetIndex+indmin])
                        # print(paths_db[currentid][agents[vehicle].targetIndex+indmin])
                        rt = paths_db[currentid][agents[vehicle].targetIndex+indmin]
                        if rt is not None:
                            rt2 = []
                            # if vehicle in prev_edge:
                            #     print('time to chekc')
                            #     if prev_edge[vehicle] not in paths_db[currentid][agents[vehicle].targetIndex+indmin]:
                            #         rt2.append(prev_edge[vehicle])
                            # if agent_car in cl_prev_edge and move:
                            #     if cl_prev_edge[agent_car] not in rt:
                            #         rt2.extend(traci.simulation.findRoute(cl_prev_edge[agent_car],currentid).edges)
                            print('changing path')
                            rt2.extend(rt)
                            try:
                                traci.vehicle.setRoute(agent_car,rt2)
                            except TraCIException:
                                # prologue_rt = list(traci.simulation.findRoute(currentid,prevrt[1]).edges)
                                # prologue_rt.extend(prevrt[2:])
                                # traci.vehicle.setRoute(agent_car,prologue_rt)
                                pass
                        # traci.vehicle.setRoute(agent_car,traci.simulation.findRoute(currentid,prox_edge.getID()).edges)
                        print('route '+str(traci.vehicle.getRoute(agent_car)))
                        # traci.vehicle.moveToXY(agent_car,current_edge,-1,x,y) # 2 check da fare: lane=0 o lane=-1, inoltre cambiare moveToXY in moveTo
                        # # traci.vehicle.moveTo(agent_car,net.getEdge(initial_edge).getLanes()[0].getID(),x,y) # 2 check da fare: lane=0 o lane=-1, inoltre cambiare moveToXY in moveTo
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
                                    pathlen = 0
                                    # print(paths_db[currentid][agents[vehicle].targetIndex][selpat])
                                    for e in paths_db[currentid][agents[vehicle].targetIndex+selpat]:
                                        # pathlen += net.getEdge(e).getLength()
                                        # pathlen += traci.edge.getTraveltime(e)
                                        pathlen += net.getEdge(e).getLength()/net.getEdge(e).getSpeed()
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
                    # if vehicle.__contains__('agent'):
                    #     if NUM_AGENTS==1 and PLAY_AUDIO and selpath is not None:
                    #         print('enter here')
                    #         true_play_audio, in_roundabout, dist, audio_edge, prev_audio_edge = search_next(mapdata,paths_db[currentid][agents[vehicle].targetIndex+indmin])
                    #         if true_play_audio and audio_edge!=played_audio_edge and not played_for_crossing:
                    #             print('true play audio')
                    #             playAudio(mapdata,currentid,net.getEdge(audio_edge),LANG,dist,net.getEdge(prev_audio_edge),in_roundabout,path=(None if not in_roundabout or paths_db[currentid] is None else paths_db[currentid][agents[vehicle].targetIndex+indmin]))
                    #             played_audio_edge = audio_edge
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
        proceed = True
        # print("VEHICLE || ALGORITHM || AVG_SPEED || TRAVELED_DISTANCE || DESTINATION")
        retds = []
        for vehicle in vehs:
            if vehs[vehicle].arrived and (not vehicle.__contains__('bus') or not vehicle.__contains__('random')):
                # spedlen = len(vehs[vehicle].speeds)
                # avgspeed = 0
                # if spedlen != 0:
                #     avgspeed = sum(vehs[vehicle].speeds)/len(vehs[vehicle].speeds)
                retds.append((vehicle,vehs[vehicle].alg,(0 if len(vehs[vehicle].speeds)==0 else np.mean(vehs[vehicle].speeds)),vehs[vehicle].dist,sum(vehs[vehicle].fuelconsumption),vehs[vehicle].waitingtime,sum(vehs[vehicle].noiseemission),sum(vehs[vehicle].co2emission),vehs[vehicle].traveltime))
            elif not vehs[vehicle].arrived and (not vehicle.__contains__('bus') or not vehicle.__contains__('random')) and vehs[vehicle].route is not None:
                counting = False
                for ed in vehs[vehicle].route:
                    vehs[vehicle].traveltime += (net.getEdge(ed).getLength()/net.getEdge(ed).getSpeed()*(20 if ed in works else 1) if counting else 0)
                    if ed == vehs[vehicle].currentroad:
                        counting = True
                retds.append((vehicle,vehs[vehicle].alg,(0 if len(vehs[vehicle].speeds)==0 else np.mean(vehs[vehicle].speeds)),vehs[vehicle].dist,sum(vehs[vehicle].fuelconsumption),vehs[vehicle].waitingtime,sum(vehs[vehicle].noiseemission),sum(vehs[vehicle].co2emission),vehs[vehicle].traveltime))
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
    else:
        return None

def move_car_on_road(mapdata,roadid,agent_car,cl_prev_edge,current_edge,path,x,y,old_x,old_y,out_of_course_counter,kR=1):
    conn = mapdata.edgegraph
    print('roadid '+str(roadid)+' current_edge '+str(current_edge))
    print('current route '+str(list(traci.vehicle.getRoute(agent_car))))
    # if not current_edge.__contains__(':') and current_edge not in list(traci.vehicle.getRoute(agent_car)):
    #     print('resetting')
    #     traci.vehicle.setRoute(agent_car,paths_db[current_edge][agents[vehicle].targetIndex+indmin])
    keepRoute = 1
    tolerance = 15
    # checkr = None
    # if agent_car in cl_prev_edge:
    #     if not roadid.__contains__(':'):
    #         checkr = roadid
    #     else:
    #         checkr = cl_prev_edge[agent_car]
    #     if current_edge.__contains__('-'):
    #         c_temp = current_edge.replace('-','')
    #         for con in conn[cl_prev_edge[agent_car]]:
    #             print(con)
    #             if con == c_temp:
    #                 current_edge = con
    #                 break
    #     else:
    #         for con in conn[cl_prev_edge[agent_car]]:
    #             if con.replace('-','') == current_edge:
    #                 current_edge = con
    #                 break
    #     if current_edge not in path:
    #         out_of_course_counter += 1
    #         if out_of_course_counter>1 and math.sqrt((x-old_x)**2+(y-old_y)**2)<5 and roadid!=current_edge:
    #             out_of_course_counter -= 1
    #     else:
    #         out_of_course_counter = 0
    #         pathfromprev = list(traci.simulation.findRoute(cl_prev_edge[agent_car],current_edge).edges)
    #         pathlen2 = 100 if len(pathfromprev)==0 else calculate_pathlen_meters(pathfromprev,mapdata)
    #         if len(pathfromprev)>5 and roadid!=current_edge:
    #             out_of_course_counter += 1
        
    # if not roadid.__contains__(':'):
    #     if roadid in conn:
    #         pathfromroadid = list(traci.simulation.findRoute(current_edge,roadid).edges)
    #         maxlen = 10
    #         pathlen = 100 if len(pathfromroadid)==0 else calculate_pathlen_meters(pathfromroadid,mapdata)
            
    #         shouldmove = True
            
    #         if out_of_course_counter>0 and out_of_course_counter<tolerance:
    #             shouldmove = False
    #         # if roadid.__contains__('-'):
    #         #     if current_edge == roadid.replace('-',''):
    #         #         shouldmove = False
    #         # else:
    #         #     if current_edge.replace('-','') == roadid:
    #         #         shouldmove = False
    #         # if shouldmove:
    #         #     shouldmove = len(pathfromroadid)>0
    #         # if shouldmove:
    #         #     shouldmove = abs(traci.edge.getAngle(current_edge)-traci.edge.getAngle(roadid))<30
    #             # shouldmove = abs(traci.edge.getAngle(current_edge,))
            
    #         if shouldmove or current_edge == roadid:
    #             move = True
    #             keepRoute = 0
    #             # if abs(traci.edge.getAngle(current_edge)-traci.edge.getAngle(roadid))>30:
    #             #     keepRoute = 1
    #             # if agent_car in cl_prev_edge:
    #             #     pathfromprev = list(traci.simulation.findRoute(cl_prev_edge[agent_car],current_edge).edges)
    #             #     pathlen2 = 100 if len(pathfromprev)==0 else calculate_pathlen_meters(pathfromprev,mapdata)
    #             #     if (pathlen2<maxlen) or current_edge == cl_prev_edge[agent_car]:
    #             #         keepRoute = 0
    #             #     else:
    #             #         keepRoute = 1
    #             print('moving')
    #             print(out_of_course_counter)
    #         else:
    #             move = False
    #     # if not move and roadid!=current_edge and out_of_course_counter==0:
    #     #     move = True
    #     #     keepRoute = 0
    # else:
    #     # if agent_car in cl_prev_edge:
    #     #     if cl_prev_edge[agent_car] in conn:
    #     #         if current_edge in conn[cl_prev_edge[agent_car]] or current_edge == cl_prev_edge[agent_car]:
    #     move = True
    #     if out_of_course_counter>0 and out_of_course_counter<tolerance:
    #         move = False
    #     keepRoute = 0
    # print('out of course counter '+str(out_of_course_counter))
    # print('old and new x and y: '+str(old_x)+', '+str(old_y)+'  '+str(x)+', '+str(y))
    # if x == old_x and y == old_y:
    #     traci.vehicle.setSpeed(agent_car,0)
    # else:
    #     traci.vehicle.setSpeed(agent_car,-1)
    # if move:
    #     # route = list(traci.vehicle.getRoute(agent_car))
    #     # if agent_car in cl_prev_edge:
    #     #     if current_edge!=route(route.index(cl_prev_edge[agent_car]+1)) and current_edge!=cl_prev_edge[agent_car]:
    #     #         print('entered')    
    #     #         traci.vehicle.changeTarget(agent_car,current_edge)
    #     traci.vehicle.moveToXY(agent_car,current_edge,-1,x,y,keepRoute=keepRoute)
    #     print('MOVING HERE')
    r_prev_edge = agent_car in cl_prev_edge
    a_prev_edge = None if not r_prev_edge else cl_prev_edge[agent_car]
    move = False
    crossing = roadid.__contains__(':')
    if not r_prev_edge:
        move = True
        keepRoute = 0
        out_of_course_counter = 0
    else:
        if x == old_x and y == old_y:
            traci.vehicle.setSpeed(agent_car,0)
            if r_prev_edge:
                if current_edge!=a_prev_edge and not crossing:
                    out_of_course_counter = 1
                else:
                    out_of_course_counter = 0
                    move = True
                    keepRoute = 0
        else:
            traci.vehicle.setSpeed(agent_car,-1)
            if crossing:
                move = True
                keepRoute = 1
            else:
                # if current_edge.__contains__('-'):
                #     c_temp = current_edge.replace('-','')
                #     for con in conn[a_prev_edge]:
                #         print('negedge '+str(con))
                #         if con == c_temp:
                #             current_edge = con
                #             break
                # else:
                #     for con in conn[a_prev_edge]:
                #         print('posedge '+str(con))
                #         if con.replace('-','') == current_edge:
                #             current_edge = con
                #             break
                if current_edge==roadid:
                        
                    move = True
                    keepRoute = 0
                else:
                    if path is not None:
                        if current_edge in path:
                            if len(traci.simulation.findRoute(a_prev_edge,current_edge).edges)<5:
                                move = True
                                keepRoute = 0
                                out_of_course_counter = 0
                            else:
                                if out_of_course_counter<tolerance:
                                    move = False
                                    out_of_course_counter += 1
                                else:
                                    move = True
                                    out_of_course_counter = 0
                        else:
                            if out_of_course_counter<tolerance:
                                move = False
                                out_of_course_counter += 1
                            else:
                                move = True
                                keepRoute = 0
                                out_of_course_counter = 0
                    else:
                        move = True
                        keepRoute = 0
                        out_of_course_counter = 0
    print('move: '+str(move))
    print('oocc: '+str(out_of_course_counter))
    print('current_edge: '+str(current_edge))
    print('x,y:         '+str(x)+','+str(y))
    print('old_x,old_y: '+str(old_x)+','+str(old_y))
    if move:
        print('trying to move')
        traci.vehicle.moveToXY(agent_car,current_edge if not crossing else roadid,-1,x,y,keepRoute=kR)
        print('MOVING HERE')
    return out_of_course_counter,move