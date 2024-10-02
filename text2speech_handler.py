
from gtts import gTTS
from tempfile import TemporaryFile
import pygame
from behaviours_maker import valid_neighbors
from mapdata import Roundabout

def convertDir2String(dir,lang):
    if dir=='s':
        return 'straight' if lang=='en' else 'dritto'
    if dir=='l':
        return 'left' if lang=='en' else 'a sinistra'
    if dir=='r':
        return 'right' if lang=='en' else 'a destra'
    if dir=='R':
        return 'right' if lang=='en' else 'a destra'
    if dir=='L':
        return 'left' if lang=='en' else 'a sinistra'
    if dir=='t':
        return 'back' if lang=='en' else 'indietro'
    
def approx10(value):
    v = int(value/10)
    return v*10

def playAudio(mapdata,current,prox,lan,dist=None,prox_edge=None,roundabout=False,path=None):
    net = mapdata.net
    connections = mapdata.connections
    roadname = net.getEdge(prox.getID()).getName()
    c_roadname = net.getEdge(current).getName()
    roadlen = approx10(int(net.getEdge(current).getLength())) if dist is None else approx10(dist)
    keepGoing = roadname==c_roadname
    lentext = ("In "+str(roadlen)+" meters, " if lan=='en' else "Tra "+str(roadlen)+" metri, ") if roadlen>=20 else ""
    aud_text = lentext
    play = True
    if not roundabout:
        aud_text = aud_text+("go " if lan=='en' else "procedi ")
        direction = None
        if prox_edge is None:
            if prox.getID() in connections[current]:
                direction = connections[current][prox.getID()]
        else:
            if prox.getID() in connections[prox_edge.getID()]:
                direction = connections[prox_edge.getID()][prox.getID()]
        if direction is not None:
            directionstring = convertDir2String(direction,lan)
            aud_text += directionstring
            if len(roadname)>2:
                aud_text += ((' towards ' if not keepGoing else ' along ') if lan=='en' else (' verso ' if not keepGoing else ' lungo '))+str(roadname)
        else:
            play = False
    else:
        if prox_edge is None:
            if prox.getID() not in mapdata.streets_in_roundabouts:
                aud_text = ("exit the roundabout." if lan=='en' else "esci dalla rotonda.")
            else:
                play = False
        else:
            aud_text = aud_text+("in the roundabout, take the " if lan=='en' else "alla rotonda, prendi la ")
            val = mapdata.roundabouts[int(mapdata.streets_in_roundabouts[prox.getID()].replace('r',''))].exit_order(mapdata,path)
            aud_text = aud_text+order_number_string(val,lan)+(" exit." if lan=='en' else " uscita.")
    if play:
        aud = gTTS(text=aud_text,lang=lan,slow=False)
        f = TemporaryFile()
        aud.write_to_fp(f)
        f.seek(0)
        pygame.mixer.init()
        pygame.mixer.music.load(f)
        pygame.mixer.music.play()

def playWarningAudio(mapdata,road,roadsequence,lan):
    net = mapdata.net
    roadname = net.getEdge(road).getName()
    nonameroad = len(roadname)<=2
    if nonameroad:
        for r in roadsequence:
            rname = net.getEdge(r).getName()
            if len(rname)>2:
                roadname = rname
                break
    aud_text = 'Warning, work in progress ' if lan=='en' else 'Attenzione, lavori in corso '
    if nonameroad:
        aud_text += 'close to '+str(roadname) if lan=='en' else 'vicino a '+str(roadname)
    else:
        aud_text += 'at '+str(roadname) if lan=='en' else 'a '+str(roadname)
    aud = gTTS(text=aud_text,lang=lan,slow=False)
    f = TemporaryFile()
    aud.write_to_fp(f)
    f.seek(0)
    pygame.mixer.init()
    pygame.mixer.music.load(f)
    pygame.mixer.music.play()

def order_number_string(value,lan):
    if value==1:
        return 'first' if lan=='en' else 'prima'
    if value==2:
        return 'second' if lan=='en' else 'seconda'
    if value==3:
        return 'third' if lan=='en' else 'terza'
    if value==4:
        return 'fourth' if lan=='en' else 'quarta'
    if value==5:
        return 'fifth' if lan=='en' else 'quinta'

def search_next(mapdata,path):
    net = mapdata.net
    warning_edge = None
    prev_warning_edge = None
    for e in path:
        neighbours = valid_neighbors(net.getEdge(e),mapdata)
        if len(neighbours)>=2:
            warning_edge = e
            break
        else:
            prev_warning_edge = e
    
    print('warning edge is '+str(warning_edge))
    if warning_edge is not None:
        if prev_warning_edge is None:
            prev_warning_edge = warning_edge
            warning_edge = path[path.index(warning_edge)+1]
        pathlen = 0
        for e in path:
            pathlen += net.getEdge(e).getLength()
            if e==warning_edge:
                break
        if pathlen<200:
            if e in mapdata.streets_in_roundabouts:
                return True,True,pathlen,warning_edge,prev_warning_edge
            else:
                return True,False,pathlen,warning_edge,prev_warning_edge
    return False,False,0,None,None