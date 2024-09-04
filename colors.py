import numpy as np
import traci

def car_color(type):
    if type=='NAPOLI':
        return 255,0,255,1 # fucsia
    elif type=='AVELLINO':
        return 0,255,0,1 # green
    elif type=='SALERNO':
        return 0,255,255,1 # azure
    elif type=='BENEVENTO':
        return 255,255,0,1 # yellow
    elif type=='FISCIANO':
        return 255,0,0,1 # red
    elif type=='PENTA':
        return 0,0,255,1 # blue
    return 255,255,255,1

def alg_color(type):
    if type=='e_bfs':
        return 255,0,255,1 # fucsia
    elif type=='e_dijkstra':
        return 0,255,0,1 # green
    elif type=='e_greedybfs':
        return 0,255,255,1 # azure
    elif type=='e_astar':
        return 255,255,0,1 # yellow
    return 255,255,255,1

def getIfromRGB(rgb):
    red = rgb[0]
    green = rgb[1]
    blue = rgb[2]
    RGBint = (red<<16) + (green<<8) + blue
    return RGBint

def colorMap(r,mapdata,rewards4colors,end_edge):
    edgelist = mapdata.edgelist
    targets = mapdata.targets
    maxrew = np.max(r[r!=0])
    maxre4=0
    minrew = np.min(r[r!=0])
    minre4=0
    vals = list(rewards4colors.values())
    sumvals = np.sum(vals)
    if sumvals!=0:
        maxre4 = np.max(vals[vals!=0])
        minre4 = np.min(vals[vals!=0])
    maxr = 0
    minr = 0
    if sumvals!=0:
        maxr = maxrew if maxrew>maxre4 else maxre4
        minr = minrew if minrew<minre4 else minre4
    else:
        maxr = maxrew
        minr = minrew
        
    
    print('maxr '+str(maxr)+' minr '+str(minr)+' maxre4 '+str(maxre4)+' minre4 '+str(minre4)+' maxrew '+str(maxrew)+' minrew '+str(minrew))
    for edgeindex in range(len(edgelist)):
        colors = list(car_color(list(targets.keys())[list(targets.values()).index(end_edge)]))[0:3]
        ed = edgelist[edgeindex].getID()
        rvalue = r[edgeindex] if r[edgeindex]!=0 else minr
        if r[edgeindex]!=0:
            rewards4colors[ed] = r[edgeindex]
        rvalue = rewards4colors[ed] if ed in rewards4colors and rewards4colors[ed]!=0 else minr
        for c in range(len(colors)):
            colors[c] = int(colors[c]*abs(rvalue-minr)/abs(maxr-minr))
        colorvalue = getIfromRGB(colors)
        traci.edge.setParameter(ed,'color',colorvalue)