import numpy as np 


class Phase:

    def __init__(self, NumberOfAgents, NumberOfInnerStates):
        self.NumberOfAgents = NumberOfAgents
        self.Coordinates = np.zeros((NumberOfAgents, 3), dtype=float)  # 位置信息
        self.Velocities = np.zeros((NumberOfAgents, 3), dtype=float)   # 速度信息
        self.InnerStates = np.zeros((NumberOfAgents, NumberOfInnerStates),
                            dtype=float)  # 个体情况信息
        self.RealIDs = np.zeros((NumberOfAgents, 1), dtype=int) 
        self.NumberOfInnerStates = NumberOfInnerStates 

        for i in range(NumberOfAgents):
            self.RealIDs[i] = i  
 