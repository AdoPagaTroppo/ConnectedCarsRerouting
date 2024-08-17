
from gtts import gTTS
from tempfile import TemporaryFile
import pygame

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

def playAudio(mapdata,current,prox,lan):
    net = mapdata.net
    connections = mapdata.connections
    roadname = net.getEdge(prox.getID()).getName()
    c_roadname = net.getEdge(current).getName()
    roadlen = approx10(int(net.getEdge(current).getLength()))
    keepGoing = roadname==c_roadname
    lentext = ("In "+str(roadlen)+" meters, " if lan=='en' else "Tra "+str(roadlen)+" metri, ") if roadlen>=20 else ""
    aud_text = lentext+("go " if lan=='en' else "procedi ")
    direction = connections[current][prox.getID()]
    directionstring = convertDir2String(direction,lan)
    aud_text += directionstring
    if len(roadname)>2:
        aud_text += ((' towards ' if not keepGoing else ' along ') if lan=='en' else (' verso ' if not keepGoing else ' lungo '))+str(roadname)
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