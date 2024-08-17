import traci
from behaviours_maker import valid_neighbors
from queue import PriorityQueue
from text2speech_handler import playWarningAudio

PLAY_AUDIO = True
workAudioPlayed = []

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
                    # else:
                    #     vn2 = valid_neighbors(inced,mapdata,target,road)
                    #     if len(vn2)==1:
                    #         returnroads.append(edid)
                    #         explore.put(edid)
        # print('exploring')
    return returnroads
    

def tweet(works,road,vehs,end_edge,mapdata,agent,vehicle,LANG,THRESHOLD):
    prevval = works[road]
    works[road] += (10 if agent else 2)*vehicle.influence
    colorworks = True
    if works[road]>=THRESHOLD and prevval<THRESHOLD:
        for v in vehs:
            if v.__contains__('agent'):
                roadsequence = find_alternative(road,end_edge[v],mapdata)
                for r in roadsequence:
                    vehs[v].weightened[r] = len(roadsequence)
                # vehs[v].weightened.extend(roadsequence)
                if colorworks:
                    traci.edge.setParameter(road,'color',100000)
                    for r in roadsequence:
                        traci.edge.setParameter(r,'color',100000)
                if PLAY_AUDIO and road not in workAudioPlayed:
                    playWarningAudio(mapdata,road,roadsequence,LANG)
                    workAudioPlayed.append(road)
                print(roadsequence)