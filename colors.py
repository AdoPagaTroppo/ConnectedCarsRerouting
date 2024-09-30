import numpy as np
import traci
import math

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
    maxrew = np.max(r)
    maxre4=0
    minrew = np.min(r)
    minre4=0
    vals = list(rewards4colors.values())
    sumvals = np.sum(vals)
    if sumvals!=0:
        maxre4 = np.max(vals)
        minre4 = np.min(vals)
    maxr = 0
    minr = 0
    # if sumvals!=0:
    #     maxr = maxrew if maxrew>maxre4 else maxre4
    #     minr = minrew if minrew<minre4 else minre4
    # else:
    maxr = maxrew
    minr = minrew
    # move by 1 tab if decommenting before
    if sumvals == 0:
        maxr = 1000
    print('maxr '+str(maxr)+' minr '+str(minr)+' maxre4 '+str(maxre4)+' minre4 '+str(minre4)+' maxrew '+str(maxrew)+' minrew '+str(minrew))
    for edgeindex in range(len(edgelist)):
        colors = list(car_color(list(targets.keys())[list(targets.values()).index(end_edge)]))[0:3]
        ed = edgelist[edgeindex].getID()
        rvalue = minr
        colorvalue = 0
        if r[edgeindex]!=0:
            rewards4colors[ed] = r[edgeindex]
            rvalue = rewards4colors[ed]
            colorvalue = int(abs(rvalue-minr)/abs(maxr-minr)*1000)+10
        # else:
            # if ed in rewards4colors:
            #     if sumvals!=0 and maxre4!=minre4:
            #         diffvalue = abs(rewards4colors[ed]-minre4)/abs(maxre4-minre4)*abs(maxr-minr)
            #         rewards4colors[ed] = diffvalue+minr
            #         colorvalue = int(diffvalue/abs(maxr-minr)*1000)+10
        # for c in range(len(colors)):
        #     colors[c] = int(colors[c]*abs(rvalue-minr)/abs(maxr-minr))
        # colorvalue = getIfromRGB(colors)
        traci.edge.setParameter(ed,'color',colorvalue)