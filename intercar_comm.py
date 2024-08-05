import traci
from behaviours_maker import valid_neighbors

def tweet(source,road,vehicles,graphdict,connections):
    for v in vehicles:
        route = traci.vehicle.getRoute(v)
        if road.getID() in route and route.index(vehicles[v].currentroad)<route.index(road.getID()):
            vehicles[v].influenced += 1
            if vehicles[v].influenced >= 5:
                vehicles[v].influenced = 0
                vn = valid_neighbors(source,graphdict,connections,route[-1])
                if len(vn)>1:
                    r1 = traci.simulation.findRoute(source.getID(),vn[1].getID()).edges
                    r2 = traci.simulation.findRoute(r1[-1],route[-1]).edges
                    r1.extend(r2)
                    traci.vehicle.setRoute(v,r1)