import traci
import random
from vehicledata import *
from algorithms import build_path
from agent import Agent
from online_crowdsourcing import *
from crowdsourcing import Crowdsourcing

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

def spawnBus(busdata,currenttime):
    spawned = False
    spawnedBuss = []
    for tup in busdata:
        if tup[0]==currenttime:
            routedata = busdata[tup]
            vbusid = 'bus_'+str(tup[1]).replace(' ','_')+'_'+str(tup[0])
            if routedata[0][0] is not None and routedata[1][0]:
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

def spawnUncontrolledCars(num_uncontrolled,mapdata):
    # paths = {}
    destinations = mapdata.destinations
    sources = mapdata.sources
    target_weights = mapdata.target_weights
    net = mapdata.net
    vehs = {}
    i = 0
    j = 0
    for c in range(int(num_uncontrolled)):
        source = random.choices(sources)[0]
        # dest = random.choices(destinations,target_weights)[0]
        dest = destinations[0]
        id = 'veh'+str(c+1)+'_'+str(dest[1])
        routeid = 'route'+str(c+1)
        # pathbuilt=build_path(graphdict,source,net.getEdge(dest[0]).getToNode().getID(),'dijkstra',graphmap,connections=connections)
        # paths['route'+str(c+1)] = traci.simulation.findRoute(net.getEdge(pathbuilt[0]).getID(),net.getEdge(pathbuilt[-1]).getID()).edges
        # route = traci.simulation.findRoute(net.getEdge(pathbuilt[0]).getID(),net.getEdge(pathbuilt[-1]).getID()).edges
        route = traci.simulation.findRoute(source,dest[0]).edges
        pathlen = 0
        for e in route:
            pathlen += net.getEdge(e).getLength()/net.getEdge(e).getSpeed()
        print('uncontr pathlen '+str(pathlen)+' source '+str(source))
        vehs[id] = VehicleData(id,'route'+str(c+1),i,dest[1])
        traci.route.add(routeid,route)
        traci.vehicle.add(id,routeid,'Car_AGENT',str(i))
        traci.vehicle.setSpeed(id,-1)
        i += 5
        j += 1
    # i = 0
    # j = 0
    # for vehicle in vehs:
    #     traci.route.add('route'+str(j+1),paths['route'+str(j+1)])
    #     traci.vehicle.add(vehicle,'route'+str(j+1),'Car_AGENT',str(i))
    #     traci.vehicle.setSpeed(vehicle,-1)
    #     i += 5
    #     j += 1
    return vehs

def spawnControlledCars(NUM_AGENTS,mapdata,NUM_ALGS,vehs,online):
    destinations = mapdata.destinations
    targets = mapdata.targets
    target_weights = mapdata.target_weights
    scenario = mapdata.scenario
    start_edge = '-579690548#1' if scenario=='Unisa' else '766350967'
    # start_edge = '93216752#1'
    # start_edge = '486603222'
    # start_edge = '50702261#2'
    end_edge = {}
    agents = {}
    for i in range(int(NUM_AGENTS)):
        # destt = random.choices(destinations,target_weights)[0]
        # destt = destinations[i%len(destinations)]
        destt = destinations[0]
        dest = destt[0]
        agentid = 'agent'+str(i)+'_'+str(destt[1])
        agrouteid = agentid+'_route'
        end_edge[agentid] = dest
        agent = Agent(start_edge,list(targets.values()).index(dest)*NUM_ALGS)
        crowds_controller = OnlineCrowdsourcing(agent,NUM_ALGS,mapdata) if online else Crowdsourcing(agent,NUM_ALGS)
        agents[agentid] = crowds_controller
        vehs[agentid] = VehicleData(agentid,agrouteid,i*10,destt[1])
        traci.route.add(agrouteid,(start_edge,start_edge))
        traci.vehicle.add(agentid,agrouteid,'Car_'+str(destt[1]),str(i*10))
        traci.vehicle.setSpeed(agentid,-1)
        print(agentid+" loaded")
    return agents,end_edge