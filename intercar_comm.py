import traci
from behaviours_maker import valid_neighbors
from queue import PriorityQueue

THRESHOLD = 1

# def tweet(source,road,vehicles,graphdict,connections):
    # for v in vehicles:
    #     route = traci.vehicle.getRoute(v)
    #     if road.getID() in route and route.index(vehicles[v].currentroad)<route.index(road.getID()):
    #         vehicles[v].influenced += 1
    #         if vehicles[v].influenced >= 5:
    #             vehicles[v].influenced = 0
    #             vn = valid_neighbors(source,graphdict,connections,route[-1])
    #             if len(vn)>1:
    #                 r1 = traci.simulation.findRoute(source.getID(),vn[1].getID()).edges
    #                 r2 = traci.simulation.findRoute(r1[-1],route[-1]).edges
    #                 r1.extend(r2)
    #                 traci.vehicle.setRoute(v,r1)

def find_alternative(road,target,mapdata):
    net = mapdata.net
    edgegraph = mapdata.edgegraph
    explore = PriorityQueue()
    explore.put(road)
    returnroads = []
    while not explore.empty():
        edge = net.getEdge(explore.get())
        fromnode_edges = edge.getFromNode().getIncoming()
        for inced in fromnode_edges:
            edid = inced.getID()
            if net.getEdge(edid).allows('passenger'):
                if edge.getID() in edgegraph[edid]:
                    vn = valid_neighbors(inced,mapdata,target)
                    if len(vn)==1:
                        returnroads.append(edid)
                        explore.put(edid)
    return returnroads
    

def tweet(works,road,vehs,end_edge,mapdata,agent):
    prevval = works[road]
    works[road] += 10 if agent else 1
    if works[road]>=THRESHOLD and prevval<THRESHOLD:
        for v in vehs:
            if v.__contains__('agent'):
                roadsequence = find_alternative(road,end_edge[v],mapdata)
                for r in roadsequence:
                    vehs[v].weightened[r] = len(roadsequence)
                # vehs[v].weightened.extend(roadsequence)
                # for r in roadsequence:
                #     traci.edge.setParameter(r,'color',100000)
                print(roadsequence)