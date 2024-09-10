import os

def bus_parser(starttime,scenario):
    searchstring = 'UNIVERSITA' if scenario=='Unisa' else 'VINCIPROVA'
    code = '15' if scenario=='Unisa' else '02'
    data = []
    buslines = {}
    with open('oraripullman.csv','r',errors='ignore') as f:
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
                if stringels[i].__contains__(code):
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
                    if j[0].__contains__(searchstring) and (hour<starttime+4 and hour>starttime-1):
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
                    if j[0].__contains__(searchstring):
                        source = source2edge(j[0],scenario),calculate_counter(starttime,j[1])    
                    else:
                        source = lineend(p,'from',j[0]),calculate_counter(starttime,j[1],True)
                    l.append(source)
                else:
                    dest = source2edge(j[0],scenario)
                    if dest!='unk':
                        l.append((dest,calculate_counter(starttime,j[1])))
                    else:
                        if j==usefulbuslines[p][i][-1]:
                            l.append((lineend(p,'to',j[0]),calculate_counter(starttime,j[1],True)))
                # print(j)
            if source is None:
                print("source is still none for "+str(i+1))
            buss[(l[0][1],p)] = l
    return buss
                
def source2edge(source,scenario):
    if source == "FISCIANO UNIVERSITA'":
        return "-392822665#5"           
    elif source == "LANCUSI UNIVERSITA'":
        return "707344764#2" if scenario=='Unisa' else 'unk'
    elif source == 'VINCIPROVA':
        return "398058811#0"
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
    elif source.__contains__('002'):
        return "671510078#4" if direction=='to' else "-671510078#4"
    elif source.__contains__('003'):
        return "671037653" if direction=='to' else "-124115234#0"
    elif source.__contains__('004'):
        return "102235300#2" if direction=='to' else "-102235300#2"
    elif source.__contains__('006'):
        return "-69364147#1" if direction=='to' else "69364147#1"
    elif source.__contains__('008'):
        return "-329813386#0" if direction=='to' else "329813386#0"
    elif source.__contains__('009'):
        return "102235300#2" if direction=='to' else "-102235300#2"
    elif source.__contains__('00A'):
        return "-371126853#0" if direction=='to' else "1134538182"
    elif source.__contains__('010'):
        return "671510078#4" if direction=='to' else "-671510078#4"
    elif source.__contains__('014'):
        return "-160827513#0" if direction=='to' else "160827513#0"
    elif source.__contains__('015'):
        return "-160827520#1" if direction=='to' else "160827520#1"
    elif source.__contains__('018'):
        return "671510078#4" if direction=='to' else "-671510078#4"
    elif source.__contains__('019'):
        return "-714367012#1" if direction=='to' else "714367012#0"
    elif source.__contains__('022'):
        return "50827474" if direction=='to' else "-567101152"
    elif source.__contains__('025'):
        return "671510078#4" if direction=='to' else "-108541471"
    elif source.__contains__('026'):
        return "-1254342108" if direction=='to' else "101546296"
    elif source.__contains__('039'):
        return "124075783#1" if direction=='to' else "-124075783#1"
    elif source.__contains__('050'):
        return "102235300#2" if direction=='to' else "-102235300#2"

def calculate_counter(starttime,giventime,notuni=False):
    givsplit = giventime.split(':')
    h = int(givsplit[0])-starttime
    m = int(givsplit[1])
    val = h*60+m+(-5 if notuni else 0)
    return 0 if val<0 else val
