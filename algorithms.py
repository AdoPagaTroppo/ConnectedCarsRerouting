from queue import PriorityQueue
from queue import Queue
import math
import traci
from colors import alg_color
from colors import getIfromRGB
import time

ANIMATION = False
TSTEP = 0.01

def connection_exists(node1,node2,node3,graphdict,connections,edge=None):
    # print(node1+" "+node2+" "+node3)
    try:
        edge1 = None
        if node1 is None:
            edge1=(edge,0)
        else:
            edge1 = graphdict[node1][node2]
        edge2 = graphdict[node2][node3]
        if edge2[0] in connections[edge1[0]] :
        # or edge1[0] in connections[edge2[0]]:
            return True
    except:
        return False
    return False

def build_path(mapdata,start,goal,type,forbidnode=None,edge=None):
    graphdict = mapdata.graphdict
    connections = mapdata.connections
    graphmap = mapdata.graphmap
    edgegraph = mapdata.edgegraph
    if type=='bfs':
        return bfs(graphdict,start,goal,connections,edge)
    elif type=='dijkstra':
        return dijkstra(graphdict,start,goal,forbidnode,connections,edge)
    elif type=='greedybfs':
        return greedybfs(graphdict,start,goal,graphmap,connections,edge)
    elif type=='astar':
        return astar(graphdict,start,goal,graphmap,forbidnode,connections,edge)
    elif type=='e_bfs':
        return edge_bfs(start,goal,edgegraph,forbidnode)
    elif type=='e_greedybfs':
        return edge_greedybfs(mapdata,start,goal,forbidnode)
    elif type=='e_dijkstra':
        return edge_dijkstra(mapdata,start,goal,forbidnode)
    elif type=='e_astar':
        return edge_astar(mapdata,start,goal,forbidnode)
    return None

def bfs(graphdict,start,goal,connections,edge):
    if start == goal:
        return ['SUCC']
    frontier = PriorityQueue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if dests not in came_from and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                frontier.put(dests)
                came_from[dests] = current
    if goal_found:
        print('I actually found the goal')
        path = []
        path.append(graphdict[came_from[goal]][goal][0])
        current = came_from[goal]
        while current!=start:
            path.append(graphdict[came_from[current]][current][0])
            current = came_from[current]
        path.reverse()
        return path
    else:
        return None

def dijkstra(graphdict,start,goal,forbidnode=None,connections=None,edge=None):
    frontier = PriorityQueue()
    frontier.put(start,0)
    came_from = {}
    came_from[start] = None
    cost_so_far = {}
    cost_so_far[start] = 0
    i = 0
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if (dests!=forbidnode and current == start) or current!=start:
                new_cost = float(cost_so_far[current])+float(graphdict[current][dests][1])
                if (dests not in cost_so_far or new_cost<cost_so_far[dests]) and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                # if (dests not in cost_so_far or new_cost<cost_so_far[dests]):
                    frontier.put(dests,new_cost)
                    i += 1
                    came_from[dests] = current
                    cost_so_far[dests] = new_cost
    if goal_found:
        path = []
        if came_from[goal] is None:
            return ['SUCC']
        path.append(graphdict[came_from[goal]][goal][0])
        current = came_from[goal]
        while current!=start:
            path.append(graphdict[came_from[current]][current][0])
            current = came_from[current]
        path.reverse()
        return path
    else:
        return None

def heuristic(n0,n1,graphmap):
    n0x = float(graphmap[n0][0])
    n1x = float(graphmap[n1][0])
    n0y = float(graphmap[n0][1])
    n1y = float(graphmap[n1][1])
    hvalue = math.sqrt((n0x-n1x)**2+(n0y-n1y)**2)
    return hvalue

def greedybfs(graphdict,start,goal,graphmap,connections,edge=None):
    if start == goal:
        return ['SUCC']
    frontier = PriorityQueue()
    frontier.put(start,0)
    came_from = {}
    came_from[start] = None
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if dests not in came_from and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                frontier.put(dests,heuristic(dests,goal,graphmap))
                came_from[dests] = current
    if goal_found:
        path = []
        if came_from[goal] is None:
            return ['SUCC']
        path.append(graphdict[came_from[goal]][goal][0])
        current = came_from[goal]
        while current!=start:
            path.append(graphdict[came_from[current]][current][0])
            current = came_from[current]
        path.reverse()
        return path
    else:
        return None

def astar(graphdict,start,goal,graphmap,forbidnode=None,connections=None,edge=None):
    frontier = PriorityQueue()
    frontier.put(start,0)
    came_from = {}
    came_from[start] = None
    cost_so_far = {}
    cost_so_far[start] = 0
    i = 0
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if (dests!=forbidnode and current == start) or current!=start:
                new_cost = float(cost_so_far[current])+float(graphdict[current][dests][1])
                if (dests not in cost_so_far or new_cost<cost_so_far[dests]) and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                    frontier.put(dests,new_cost+heuristic(dests,goal,graphmap))
                    i += 1
                    came_from[dests] = current
                    cost_so_far[dests] = new_cost
    # print(source+" "+dest)
    # print(goal_found)
    if goal_found:
        path = []
        if came_from[goal] is None:
            return ['SUCC']
        path.append(graphdict[came_from[goal]][goal][0])
        current = came_from[goal]
        while current!=start:
            path.append(graphdict[came_from[current]][current][0])
            current = came_from[current]
        path.reverse()
        return path
    else:
        return None

def edge_bfs(start,goal,edgegraph,forbidnode=None):
    if start == goal:
        return ['SUCC']
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None
    goal_found = False
    colorv = getIfromRGB(list(alg_color('e_bfs'))[0:3])
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break               
        if ANIMATION and not goal_found:
            traci.edge.setParameter(current,'color',colorv)
            time.sleep(TSTEP)
            traci.simulationStep()
        for dests in edgegraph[current]:
            if dests not in came_from and (forbidnode is None or (forbidnode is not None and dests not in forbidnode)):
                frontier.put(dests)
                came_from[dests] = current
    if goal_found:
        path = []
        current = goal
        while current!=start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    else:
        return None
    
def edge_greedybfs(mapdata,start,goal,forbidnode=None):
    edgegraph = mapdata.edgegraph
    graphmap = mapdata.graphmap
    net = mapdata.net
    if start == goal:
        return ['SUCC']
    frontier = PriorityQueue()
    frontier.put((0,start))
    came_from = {}
    came_from[start] = None
    goal_found = False
    colorv = getIfromRGB(list(alg_color('e_greedybfs'))[0:3])
    while not frontier.empty():
        current = frontier.get()[1]
        if current == goal:
            goal_found = True
            break
        if ANIMATION and not goal_found:
            traci.edge.setParameter(current,'color',colorv)
            time.sleep(TSTEP)
            traci.simulationStep()
        for dests in edgegraph[current]:
            if dests not in came_from and (forbidnode is None or (forbidnode is not None and dests not in forbidnode)):
                frontier.put((heuristic(net.getEdge(dests).getToNode().getID(),net.getEdge(goal).getToNode().getID(),graphmap),dests))
                came_from[dests] = current
    if goal_found:
        path = []
        current = goal
        while current!=start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    else:
        return None
    
def edge_dijkstra(mapdata,start,goal,forbidnode=None):
    if start == goal:
        return ['SUCC']
    edgegraph = mapdata.edgegraph
    net = mapdata.net
    frontier = PriorityQueue()
    frontier.put((0,start))
    came_from = {}
    came_from[start] = None
    cost_so_far = {}
    cost_so_far[start] = edge_cost(mapdata,start)
    goal_found = False
    colorv = getIfromRGB(list(alg_color('e_dijkstra'))[0:3])
    while not frontier.empty():
        popel = frontier.get()
        current = popel[1]
        if current == goal:
            # print('GOAL FOUND')
            goal_found = True
            break
        if ANIMATION and not goal_found:
            traci.edge.setParameter(current,'color',colorv)
            time.sleep(TSTEP)
            if not goal_found:
                traci.simulationStep()
        for dests in edgegraph[current]:
            if forbidnode is None or (forbidnode is not None and dests not in forbidnode):
                # new_cost=float(cost_so_far[current])+float(net.getEdge(dests).getLength())
                # new_cost=float(cost_so_far[current])+float(traci.edge.getTraveltime(dests))
                new_cost=float(cost_so_far[current])+edge_cost(mapdata,dests)
                if dests not in cost_so_far or new_cost<cost_so_far[dests]:
                    frontier.put((new_cost,dests))
                    came_from[dests] = current
                    cost_so_far[dests] = new_cost
    if goal_found:
        path = []
        current = goal
        while current!=start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    else:
        return None
    
def edge_astar(mapdata,start,goal,forbidnode=None):
    if start == goal:
        return ['SUCC']
    edgegraph = mapdata.edgegraph
    net = mapdata.net
    graphmap = mapdata.graphmap
    frontier = PriorityQueue()
    frontier.put((0,start))
    came_from = {}
    came_from[start] = None
    cost_so_far = {}
    cost_so_far[start] = edge_cost(mapdata,start)
    goal_found = False
    colorv = getIfromRGB(list(alg_color('e_astar'))[0:3])
    while not frontier.empty():
        current = frontier.get()[1]
        if current == goal:
            goal_found = True
            break
        if ANIMATION and not goal_found:
            traci.edge.setParameter(current,'color',colorv)
            time.sleep(TSTEP)
            traci.simulationStep()
        for dests in edgegraph[current]:
            if forbidnode is None or (forbidnode is not None and dests not in forbidnode):
                # new_cost=float(cost_so_far[current])+float(net.getEdge(dests).getLength())
                # new_cost=float(cost_so_far[current])+float(net.getEdge(dests).getLength()/net.getEdge(dests).getSpeed())
                new_cost=float(cost_so_far[current])+edge_cost(mapdata,dests)
                if dests not in cost_so_far or new_cost<cost_so_far[dests]:
                    frontier.put((new_cost+heuristic(net.getEdge(dests).getToNode().getID(),net.getEdge(goal).getToNode().getID(),graphmap),dests))
                    came_from[dests] = current
                    cost_so_far[dests] = new_cost
    if goal_found:
        path = []
        current = goal
        while current!=start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    else:
        return None


def edge_cost(mapdata,edge):
    maxprio = mapdata.maxprio
    minprio = mapdata.minprio
    net = mapdata.net
    return float(net.getEdge(edge).getLength()/net.getEdge(edge).getSpeed())
    # return traci.edge.getTraveltime(edge)
    