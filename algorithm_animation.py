import traci
from algorithms import *
from mapdata import *
from behaviours_maker import index2path
from behaviours_maker import index2alg
from behaviours_maker import alg_color
from colors import getIfromRGB
import time
from single_simulation import calculate_pathlen

# start = '766350967'
# start = '50706083#1'
start = '-79867013#1'
# end = '50827474' # avellino
end = '-40567772#0' # avellino da Unisa
# end = '93450829' # napoli
SCENARIO = 'Unisa'
dijkstrabased = {}
mapdata = MapData(SCENARIO)
works = mapdata.works
start = mapdata.net.getEdge(start)
traci.start(['sumo-gui','-c','osm_'+str(SCENARIO)+'.sumocfg','--step-length',str(1.0)])
paths = [None]*4
for i in range(4):
    paths[i] = index2path(i,end,start,mapdata,dijkstrabased=dijkstrabased,works=works)
    print('pathlen: '+str(calculate_pathlen(paths[i],works,mapdata)))
    if i == 0:
        dijkstrabased[start.getID()] = paths[i][1]
    colorv = getIfromRGB(list(alg_color(index2alg(i)))[0:3])               
    for e in paths[i]:
        # if e.getID() not in paths[i]:
        traci.edge.setParameter(e,'color',colorv)
        time.sleep(0.01)
        traci.simulationStep()
    time.sleep(5.0)
    traci.simulationStep()
traci.close()
    