from queue import PriorityQueue
import math

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

def build_path(graphdict,start,goal,type,graphmap=None,forbidnode=None,connections=None,edge=None):
    if type=='bfs':
        return bfs(graphdict,start,goal,connections,edge)
    elif type=='dijkstra':
        return dijkstra(graphdict,start,goal,forbidnode,connections,edge)
    elif type=='greedybfs':
        return greedybfs(graphdict,start,goal,graphmap,connections,edge)
    elif type=='astar':
        return astar(graphdict,start,goal,graphmap,forbidnode,connections,edge)
    return None

def bfs(graphdict,start,goal,connections,edge):
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    i = 0
    goal_found = False
    while len(frontier)>0:
        current = frontier.pop(0)
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if dests not in came_from and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                frontier.append(dests)
                i += 1
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
    frontier = PriorityQueue()
    frontier.put(start,0)
    came_from = {}
    came_from[start] = None
    i = 0
    goal_found = False
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            goal_found = True
            break
        for dests in graphdict[current]:
            if dests not in came_from and connection_exists(came_from[current],current,dests,graphdict,connections,edge=edge):
                frontier.put(dests,heuristic(dests,goal,graphmap))
                i += 1
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
