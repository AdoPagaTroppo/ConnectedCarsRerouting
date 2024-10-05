# Script for quickly checking random things

from graph_util import build_graph
from algorithms import build_path
from behaviours_maker import index2alg
import traci
from mapdata import MapData
import matplotlib.pyplot as plt
import math
import numpy as np
from mapdata import Roundabout

# sumoCmd = ['sumo', "-c", 'osm4.sumocfg',"--step-length","0.05"] #The last parameter is the step size, has to be small
# traci.start(sumoCmd)
# graphdict, graphmap, net, connections = build_graph()
# s = '587489968#3'
# t = '330222144'
# j = 2
# route = []
# route.append(s)
# ext = build_path(graphdict,net.getEdge(s).getToNode().getID(),net.getEdge(t).getToNode().getID(),index2alg(j),graphmap=graphmap,connections=connections,edge=s)
# if ext is None:
#     route = None
# else:
#     route.extend(ext)
# print(route)
# traci.close()

# def car_color(type):
#     if type=='NAPOLI':
#         return 255,0,255,1 # fucsia
#     elif type=='AVELLINO':
#         return 0,255,0,1 # green
#     elif type=='SALERNO':
#         return 0,255,255,1 # azure
#     elif type=='BENEVENTO':
#         return 255,255,0,1 # yellow
#     elif type=='FISCIANO':
#         return 255,0,0,1 # red
#     elif type=='PENTA':
#         return 0,0,255,1 # blue
#     return 255,255,255,1

# def getIfromRGB(rgb):
#     red = rgb[0]
#     green = rgb[1]
#     blue = rgb[2]
#     RGBint = (red<<16) + (green<<8) + blue
#     return RGBint

# colorvalue = getIfromRGB(list(car_color('PENTA'))[0:3])
# print(colorvalue)

# md = MapData()
# start = '951974693#2'
# goal = '330222144'
# print(build_path(md,start,goal,'e_bfs'))

# import numpy as np

# b = np.load('behaviors_Unisa_4.npy')
# map = MapData('Unisa')
# # plt.matshow(b[0])
# # plt.colorbar()
# id = 1965
# ed = map.edgelist[id]
# plt.title("PMF for edge "+str(ed.getID())+", destination Naples, Dijkstra algorithm")
# plt.plot(b[0][1965])
# plt.show()
# from single_simulation import calculate_pathlen

# SCENARIO = 'Salerno'
# map = MapData(SCENARIO)
# pathlens = [[0]*len(map.targets)]*len(map.edgelist)
# print(map.targets)
# for i in range(len(map.edgelist)):
#     ed = map.edgelist[i]
#     print('Analyzing edge '+str(ed.getID()))
#     for j in range(len(list(map.targets.values()))):
#         print('\t for target '+str(list(map.targets.keys())[j]))
#         path = build_path(map,ed.getID(),list(map.targets.values())[j],'e_dijkstra')
#         plen = 0
#         if path is not None:
#             plen = calculate_pathlen(path,[],map)
#         else:
#             plen = math.inf
#         pathlens[i][j] = plen
# np.save('pathlens_'+str(SCENARIO),pathlens)
    
    
# DA FINIRE DOPO, AGGIUNGERE MECCANISMO EFFICIENTE DI MEMORIZZAZIONE

# coords = []
# f = open('log_gps.txt','r')
# dats = f.readlines()
# for dat in dats:
#     cs = dat.split(':')
#     coords.append([float(cs[0]),float(cs[1])])
# print(coords)

# m = MapData('Unisa')
f = open('roundabouts_Unisa.txt','r')
content = f.readlines()
roundabouts = {}
index = 0
for r in content:
    rid = 'r'+str(index)
    roundabouts[rid] = {}
    roundabouts[rid]['roads'] = []
    roundabouts[rid]['exits'] = []
    split_content = r.split(';')
    r_len = int(split_content[0])
    for i in range(r_len):
        roundabouts[rid]['roads'].append(split_content[1+i].replace('\n',''))
    e_len = int(split_content[1+r_len])
    for i in range(e_len):
        roundabouts[rid]['exits'].append(split_content[2+r_len+i].replace('\n',''))
    print(roundabouts[rid])
    index += 1
f.close()

traci.start(['sumo-gui','-c','osm_'+str('Unisa')+'.sumocfg','--step-length',str(0.05)])

roundabouts2 = roundabouts
# for r in roundabouts:
#     path = traci.simulation.findRoute(roundabouts[r]['roads'][0],roundabouts[r]['roads'][-1]).edges
#     if len(path)>0:
#         roundabouts2[r] = roundabouts[r]


for r in roundabouts2:
    for roads in range(len(roundabouts2[r]['roads'])):
        traci.edge.setParameter(roundabouts2[r]['roads'][roads],'color',12)
    for roads in range(len(roundabouts2[r]['exits'])):
        traci.edge.setParameter(roundabouts2[r]['exits'][roads],'color',12000)
    traci.simulationStep()
traci.close()
    