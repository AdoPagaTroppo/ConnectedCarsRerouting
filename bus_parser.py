import os

def bus_parser(starttime):
    data = []
    buslines = {}
    with open('oraripullman.csv','r') as f:
        data = f.readlines()
    currentline = ''
    valindexes = []
    index = 0
    for el in data:
        stringels = el.split(';')
        if stringels[0].__contains__('Linea'):
            index = 0
            e = stringels[0].replace('Linea:   ','')
            eels = e.split(' ')
            currentline = eels[0]+" "+eels[1]
            buslines[currentline] = {}
        elif stringels[0].__contains__('Validità'):
            valindexes.clear()
            for i in range(len(stringels)):
                if stringels[i].__contains__('15'):
                    valindexes.append(i)
        elif len(stringels[0])>1:
            index = 0
            for i in range(len(stringels)):
                if i in valindexes:
                    if index not in buslines[currentline]:
                        buslines[currentline][index] = []
                    if len(stringels[i])>1:
                        buslines[currentline][index].append((stringels[0],stringels[i]))
                    index += 1
        
    usefulbuslines = {}
            
    for p in buslines:
        # print(p)
        for i in buslines[p]:
            # print('Route '+str(i+1))
            for j in buslines[p][i]:
                # print(buslines[p][i])
                # for k in range(len(buslines[p][i])):
                try:
                    hour = int(j[1].split(':')[0])
                    if j[0].__contains__('UNIVERSITA') and (hour<starttime+4 and hour>starttime-1):
                        if p not in usefulbuslines:
                            usefulbuslines[p] = {}
                        if i not in usefulbuslines[p]:
                            usefulbuslines[p][i] = []
                        usefulbuslines[p][i] = buslines[p][i]
                except:
                    pass

    buss = {}
    for p in usefulbuslines:
        # print(p)    
        for i in usefulbuslines[p]:
            source = None
            l = []
            # print('Route '+str(i+1))
            for j in usefulbuslines[p][i]:
                if source is None:
                    if j[0].__contains__('UNIVERSITA'):
                        source = source2edge(j[0]),calculate_counter(starttime,j[1])    
                    else:
                        source = lineend(p,'from',j[0]),calculate_counter(starttime,j[1],True)
                    l.append(source)
                else:
                    dest = source2edge(j[0])
                    if dest!='unk':
                        l.append((dest,calculate_counter(starttime,j[1])))
                    else:
                        if j==usefulbuslines[p][i][-1]:
                            l.append((lineend(p,'to',j[0]),calculate_counter(starttime,j[1],True)))
                # print(j)
            buss[(l[0][1],p)] = l
    return buss
                
def source2edge(source):
    if source == "FISCIANO UNIVERSITA'":
        return "-392822665#5"           
    elif source == "LANCUSI UNIVERSITA'":
        return "707344764#2"
    return 'unk'

def lineend(source,direction,additional=None):
    if source.__contains__('017'):
        return "1084284613" if direction=='to' else "1084284608"
    if source.__contains__('027'):
        return "1084284613" if direction=='to' else "1084284608"
    elif source.__contains__('035'):
        return "1084284613" if direction=='to' else "1084284608"
    elif source.__contains__('036'):
        return "1084284613" if direction=='to' else "1084284608"
    elif source.__contains__('057'):
        if additional.__contains__('COPERCHIA'):
            return "112230000" if direction=='from' else "-112230000"
        return "-40567772#0" if direction=='to' else "40567772#0"
    elif source.__contains__('081'):
        return "330222144" if direction=='to' else "84945288#0"
    elif source.__contains__('082'):
        return "330222144" if direction=='to' else "84945288#0"
    elif source.__contains__('083'):
        return "330222144" if direction=='to' else "84945288#0"
    elif source.__contains__('084'):
        return "330222144" if direction=='to' else "84945288#0"

def calculate_counter(starttime,giventime,notuni=False):
    givsplit = giventime.split(':')
    h = int(givsplit[0])-starttime
    m = int(givsplit[1])
    val = h*60+m+(-5 if notuni else 0)
    return 0 if val<0 else val
