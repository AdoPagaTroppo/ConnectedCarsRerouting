import os

def parse_file_for_nodes(filename):
    dataread = []
    targets = []
    sources = []
    with open(filename,'r') as rf:
        dataread = rf.readlines()
    for el in dataread:
        el = el.replace('\n','')
        pieces = el.split(':')
        if pieces[0]=='target':
            targets.append((pieces[1],pieces[2],float(pieces[3])))
        elif pieces[0]=='start':
            sources.append(pieces[1])
    return targets,sources

def parse_file_for_work(filename):
    works = {}
    dataread = []
    with open(filename,'r') as rf:
        dataread = rf.readlines()
    for el in dataread:
        el = el.replace('\n','')
        works[el] = 0
    return works
    
# targets, sources = parse_file_for_nodes('config.txt')
# print("Targets:")
# for t in targets:
#     print(t)
# print("Sources: ")
# for s in sources:
#     print(s)