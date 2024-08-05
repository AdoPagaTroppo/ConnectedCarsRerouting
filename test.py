from graph_util import build_graph
from algorithms import build_path
from behaviours_maker import index2alg
import traci

sumoCmd = ['sumo', "-c", 'osm4.sumocfg',"--step-length","0.05"] #The last parameter is the step size, has to be small
traci.start(sumoCmd)
graphdict, graphmap, net, connections = build_graph()
s = '587489968#3'
t = '330222144'
j = 2
route = []
route.append(s)
ext = build_path(graphdict,net.getEdge(s).getToNode().getID(),net.getEdge(t).getToNode().getID(),index2alg(j),graphmap=graphmap,connections=connections,edge=s)
if ext is None:
    route = None
else:
    route.extend(ext)
print(route)
traci.close()