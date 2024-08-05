import sumolib
import traci

def build_graph():
    net = sumolib.net.readNet('fiscianobaronissi.net.xml')
    nodes = traci.junction.getIDList()
    graphdict = {}
    graphmap = {}
    for n in nodes:
        graphdict[n] = {}
        try:
            graphmap[n] = net.getNode(n).getCoord()
            for neighbour in net.getNode(n).getNeighboringNodes(outgoingNodes=True,incomingNodes=False):
                for edge in net.getNode(n).getOutgoing():
                    if net.getEdge(edge.getID()).getToNode().getID() == neighbour.getID() and net.getEdge(edge.getID()).allows('passenger'):
                        graphdict[n][neighbour.getID()] = edge.getID(),edge.getLength()
        except:
            pass
    connects = sumolib.xml.parse_fast('fiscianobaronissi.net.xml','connection',['from','to'])
    connections = {}
    for c in connects:
        key = c[0]
        if key not in connections:
            connections[key] = []
        val = c[1]
        connections[key].append(val)
    return graphdict, graphmap, net, connections