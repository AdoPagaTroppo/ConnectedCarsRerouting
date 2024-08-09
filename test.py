from graph_util import build_graph
from algorithms import build_path
from behaviours_maker import index2alg
import traci
from mapdata import MapData

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

md = MapData()
start = '951974693#2'
goal = '330222144'
print(build_path(md,start,goal,'e_bfs'))