import math
import matplotlib.pyplot as plt

import numpy as np  # for array management
from behaviours_maker import index2alg

# UNISA NET ------------
behaviors_name = r"test_behaviours.npy"

# ###This is where the behaviors are loaded
behaviors = np.load(behaviors_name) 

class Crowdsourcing():
    def __init__(self, agent, numalgs):
        #the sources are a global variable
        # self.behaviors = np.load("test_behaviours.npy") 
        self.edgeNum = behaviors.shape[1]
        self.targetNum = behaviors.shape[0]
        self.targetIndex = agent.id_goal
        # print(self.edgeNum)
        # print(self.targetNum)
        # print(self.targetIndex)
        # self.PMF_target = self.behaviors[self.targetIndex if self.targetIndex<12 else 0]
        self.PMF_target = behaviors[self.targetIndex]
        self.numalgs = numalgs
        # print(self.PMF_target)

        
    # initialing an empty array for the returned behavior (N x N_nodes)

    def __DKL(self, l1, l2):
        ###DKL between two arrays, they have to be pdfs
        # behavior identical to scipy.stats.entropy, at least for pdfs in this example's format
        # return entropy(l1, l2)
        x = 0
        for i in range(len(l1)):
            if l1[i] != 0 and l2[i] != 0:
                x = x + l1[i] * math.log(l1[i] / l2[i])
            if l2[i] == 0 and l1[i] != 0:
                return math.inf
        return x
        
        
    def receding_horizon_DM(self, state, tHor, stateSpace, r, online = False, behaviours = None, edgelist=None):
        ### Decision-making loop
        # Arguments: state, time horizon, state space, reward array (actually a cost)
        xInd = np.where(stateSpace == state)[0][0] #Index of the state in the reduced state space
        if online and behaviours is not None:
            behaviors = behaviours
        # if online and behaviours is not None:
        #     self.edgeNum = behaviours.shape[1]
        #     self.targetNum = behaviours.shape[0]
        #     self.targetIndex = int(self.targetIndex/self.numalgs)
        #     self.PMF_target = behaviours[0]
        dim = len(stateSpace)
        rHat = np.array([0]*self.edgeNum) #rHat as in the algorithm 
        # weights = np.zeros((self.targetNum, dim)) #Decision-making weights
        weights = np.zeros((self.numalgs, dim)) #Decision-making weights
        # if len(r)<len(rHat):
        #     r = np.append(r,[0]*(len(rHat)-len(r)))
        # sources = np.zeros((self.targetNum, dim, self.edgeNum)) #The sources and target we use correspond to the reduced state space
        sources = np.zeros((self.numalgs, dim, self.edgeNum)) #The sources and target we use correspond to the reduced state space
        target = np.zeros((dim,self.edgeNum))
        self.PMF_target = behaviors[self.targetIndex]
        for i in range(dim):
            # print("index "+str(i))
            # if not online:
                target[i] = self.PMF_target[stateSpace[i]]
            # else:
            #     target[i] = self.PMF_target[i]
            # target[i] = self.PMF_target[np.where(stateSpace == stateSpace[i])]
        # for i in range(self.targetNum):
        for i in range(self.numalgs):
            for j in range(dim):
                # sources[i,j] = behaviors[i,stateSpace[j]]
                # sources[i,j] = behaviors[i+self.targetIndex,np.where(stateSpace == stateSpace[j])]
                # if not online:
                    sources[i,j] = behaviors[i+self.targetIndex,stateSpace[j]]
                # else:
                #     sources[i,j] = behaviours[i,j]
        if tHor == 0: #Safety as python doesn't like range(0)
            tHor = 1
    
        timeHor = list(range(tHor))
        timeHor.reverse()
        # rBar = r
        for t in timeHor:
            # print('time '+str(t))
            rBar = r + rHat #Adapt reward
            # print([x for x in rBar if x!=0])
            # for i in range(self.targetNum):
            for i in range(self.numalgs):
                for j in range(dim):
                    weights[i,j] = self.__DKL(sources[i,j], target[j]) - np.dot(sources[i,j], rBar) #calculate weights
                    # if t == 3:
                    #     plt.plot(target[j],color='green')
                    #     plt.plot(sources[i,j],color='red')
                    #     for k in range(len(sources[i,j])):
                    #         if sources[i,j,k]!=0:
                    #             print('edge '+str(edgelist[k]))
                    #     plt.title('divergence on '+str(edgelist[stateSpace[j]].getID())+' alg '+str(index2alg(i)))
                    #     plt.show()
            for i in range(dim):
                # if online:
                #     rHat[i] = -min(weights[:,i]) #Calculate rHat
                # else:
                    rHat[stateSpace[i]] = -min(weights[:,i]) #Calculate rHat
                    # if edgelist is not None:
                    #     print(edgelist[stateSpace[i]].getID())
                    # print(rHat[stateSpace[i]])
                    # print('weights')
                    # print([x for x in weights[:,i] if x!=0])
        # indMin = np.argmin(weights[self.targetIndex:self.targetIndex+self.numalgs,xInd])
        # print('vehicle to '+str(self.targetIndex))
        # print('rHat')
        # print([x for x in rHat if x!=0])
        indMin = np.argmin(weights[:,xInd])
        # print('index is '+str(indMin)+' of item '+str(rHat[indMin])+' obtained through alg '+str(index2alg(indMin)))
        pf = sources[indMin, xInd] #Pick pf
        # pf = sources[self.targetIndex, xInd] #Pick pf
        return(np.random.choice(range(self.edgeNum), p = pf)) #Sample pf and return resulting state