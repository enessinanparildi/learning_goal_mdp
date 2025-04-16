import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# This code can get all graphs with slight modifications
G = 4
state_space = np.arange(0, G+1)
P_final = np.arange(0, 1.01, 0.01)
P_upper = np.arange(0, 1.01, 0.01)
action_space = [0, 1]
pvalue_set = np.zeros(len(state_space))
optimalpvalue_set = np.zeros(len(state_space))
oldpvalue_set = np.zeros(len(state_space))
finaloverallset = np.zeros((len(P_upper), len(P_final)))
pvalue_set[G] = 1

for u in range(len(P_upper)):
    for f in range(len(P_final)):
        check = 1
        while check == 1:
            oldpvalue_set = pvalue_set.copy()
            for i in range(1, G):
                a1 = P_upper[u] * pvalue_set[i + 1] + (1 - P_upper[u]) * pvalue_set[i - 1]
                a2 = P_final[f] * pvalue_set[G] + (1 - P_final[f]) * pvalue_set[0]
                if a2 > a1:
                    pvalue_set[i] = a2
                else:
                    pvalue_set[i] = a1
                
                check = 0
                for b in range(1, G):
                    if abs(oldpvalue_set[i] - pvalue_set[i]) > 0.0001:
                        check = 1
                        break
        
        for i in range(1, G):
            a1 = P_upper[u] * pvalue_set[i + 1] + (1 - P_upper[u]) * pvalue_set[i - 1]
            a2 = P_final[f] * pvalue_set[G] + (1 - P_final[f]) * pvalue_set[0]
            if a2 > a1:
                optimalpvalue_set[i] = a2
                print('Final')
            else:
                optimalpvalue_set[i] = a1
                print('Cont')
        
        finaloverallset[u, f] = optimalpvalue_set[1]

# Create a meshgrid for 3D plotting
X, Y = np.meshgrid(P_upper, P_final)

# Create a 3D plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(X, Y, finaloverallset.T, cmap='viridis')

# Add labels and title
ax.set_title('Average prob value for state 2 versus transition variable fixed G=4')
ax.set_xlabel('P_upper')
ax.set_ylabel('P_final')
ax.set_zlabel('P*(s) value state 2')

# Add a color bar
fig.colorbar(surf)

plt.show()


# Python implementation of the State class
class State:
    def __init__(self, val):
        self.state_number = val  # State index in MDP
        self.goalprob = 0  # goal prob value
        self.actions = []  # set of actions initially empty
        self.action_size = 0  # number of actions
        self.optimalactionindex = 1  # Number of optimal action index
        self.Nsa = None  # Another counter which ticks according to action chosen
        self.delta = None
        
    def addaction(self, action):
        # adding actions can take multiple actions at once
        if isinstance(action, list):
            self.actions.extend(action)
            self.action_size += len(action)
        else:
            self.actions.append(action)
            self.action_size += 1
            
        self.Nsa = np.zeros(self.action_size)
        self.delta = np.zeros(self.action_size)


# Python implementation of the Action class
class Action:
    def __init__(self):
        self.nextstates = []  # Set of next states initially empty
        self.realtprob = []  # Set of real t probs initially empty
        self.tprobestimate = None  # For keeping estimated probs
        self.betterstates = None
        self.worsestates = None
        self.size = 0  # Num of next states
        self.Nsas = None  # This is counter for simulation. This is for counting nextstate transition
        
    def addnextstatetprob(self, state, val):
        # Adding nextstate and tprob to particular action
        if isinstance(state, list):
            self.nextstates.extend(state)
            self.realtprob.extend(val)
            self.size += len(state)
        else:
            self.nextstates.append(state)
            self.realtprob.append(val)
            self.size += 1
            
        self.Nsas = np.zeros(self.size)
        self.tprobestimate = np.zeros(self.size)
        
    def getprobnextstate(self, state):
        # From next state get tprob
        prob = -1
        for i in range(self.size):
            if state == self.nextstates[i]:
                prob = self.tprobestimate[i]
        return prob
        
    def setbetterstates(self, index):
        # For setting betterstates just like in algorithm in paper
        i = np.arange(self.size)
        self.betterstates = i[index]
        self.worsestates = np.setdiff1d(i, self.betterstates)
        
    def reachcounter(self, stateindex):
        # To get counter value from nextstate index
        c = 0
        for i in range(self.size):
            if stateindex == self.nextstates[i].state_number:
                c = self.Nsas[i]
        return c
        
    def setcounter(self, stateindex, val):
        # Set counter to new value
        for i in range(self.size):
            if stateindex == self.nextstates[i].state_number:
                self.Nsas[i] = val


# Code for setup with varying probability values (derived from the report)
def setup_simulation(u_prob, f_prob, d_prob=None):
    # Setup 1: pc = (1- u_prob)/2;
    pu = np.linspace(u_prob, 1, 4)
    pd = np.linspace(d_prob if d_prob is not None else (1-u_prob)/2, 0, 4)
    pc = 1 - pu - pd
    pf = np.linspace(f_prob, 1, 4)
    
    return pu, pd, pc, pf

# Alternative setup mentioned in the report
def setup_simulation_alt(u_prob, f_prob):
    pu = np.linspace(u_prob, 1, 4)
    pd = (1-pu)/2
    pc = (1-pu)/2
    pf = np.linspace(f_prob, 1, 4)
    
    return pu, pd, pc, pf
