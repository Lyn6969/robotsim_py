import numpy as np
from math_utils import math_utils 
import random

math_fun = math_utils()

class Phase:

    def __init__(self, NumberOfAgents=0, NumberOfInnerStates=0):
        self.NumberOfAgents = NumberOfAgents
        self.Coordinates = np.zeros((NumberOfAgents, 3), dtype=float)  # 位置信息
        self.Velocities = np.zeros((NumberOfAgents, 3), dtype=float)   # 速度信息
        self.InnerStates = np.zeros((NumberOfAgents, NumberOfInnerStates),
                            dtype=float)  # 个体情况信息
        self.RealIDs = np.zeros(NumberOfAgents) 
        self.NumberOfInnerStates = NumberOfInnerStates 

        for i in range(NumberOfAgents):
            self.RealIDs[i] = i  
    
    def GetAgentsVelocity(self, Phase, WhitchAgent):

        Velocity = np.zeros(3)

        for i in range(3):
            Velocity[i] = Phase.Velocities[WhitchAgent][i] 
        
        return Velocity

    def GetAgentsCoordinates(self, Phase, WhitchAgent):
        Coords = np.zeros(3)
        for j in range(3):
            Coords[j] = Phase.Coordinates[WhitchAgent][j]
        
        return Coords

    def HowManyCollisions(self, ActualPhase, AgentsInDanger,
                          CountCollisions, RadiusOfCopter):
        Collisions = 0

        PreviousSituation = False

        if CountCollisions is True:
            for j in range(ActualPhase.NumberOfAgents):
                PreviousSituation = AgentsInDanger[j]
                AgentsInDanger[j] = False
                for i in range(j):
                    if i != j:
                        ithAgentsCoordinates = self.GetAgentsCoordinates(ActualPhase, i)
                        jthAgentsCoordinates = self.GetAgentsCoordinates(ActualPhase, j)
                        RelativeCoordinates = math_fun.VectDifference(ithAgentsCoordinates, jthAgentsCoordinates)

                        if math_fun.VectAbs(RelativeCoordinates) <= RadiusOfCopter:
                            AgentsInDanger[j] = True
                            AgentsInDanger[i] = True
                        
                Collisions += (PreviousSituation is False) and (AgentsInDanger[j] is True)
        
        return Collisions

    # 将位置输入Phase
    def InsertAgentsCoordinates(self, phase, Coords, WhitchAgent):

        for j in range(3):
            phase.Coordinates[WhitchAgent][j] = Coords[j]
    
    # 将速度输入Phase
    def InsertAgentsVelocity(self, phase, Velocity, WhitchAgent):

        for j in range(3):
            phase.Velocities[WhitchAgent][j] = Velocity[j]

    def insert_phase_to_dataline(self,phasedata, phase, whitchstep):
        for i in range(phase.NumberOfAgents):
            for j in range(3):
                phasedata[whitchstep].Coordinates[i][j] = phase.Coordinates[i][j]
                phasedata[whitchstep].Velocities[i][j] = phase.Velocities[i][j]

    # 交换状态
    def SwapAgents(self, Phase, i, j):

        Phase.Coordinates[i], Phase.Coordinates[j] = Phase.Coordinates[j], Phase.Coordinates[i]

        Phase.Velocities[i], Phase.Velocities[j] = Phase.Velocities[j], Phase.Velocities[i]

        Phase.InnerStates[i], Phase.InnerStates[j] = Phase.InnerStates[j], Phase.InnerStates[i]

        Phase.RealIDs[i], Phase.RealIDs[j] = Phase.RealIDs[j], Phase.RealIDs[i]

    # 将在附近的个体放到第一梯队中
    def SelectNearbyVisibleAgents(self, Phase, ReferencePosition, Range, PacketLossQuadraticCoeff):

        DistFromRef = np.zeros(3)
        Dist = 0
        NumberOfNearbyAgents = 1

        i = Phase.NumberOfAgents-1 

        while(i >= NumberOfNearbyAgents):
            DistFromRef = Phase.GetAgentsCoordinates(Phase, i)
            DistFromRef = math_fun.VectDifference(DistFromRef, ReferencePosition)

            Dist = math_fun.VectAbs(DistFromRef)

            packet_lost = random.uniform(0, 1) < Dist * Dist * PacketLossQuadraticCoeff

            if ((Dist!=0) and (Dist<=Range) and (not packet_lost)):
                Phase.SwapAgents(Phase, i, NumberOfNearbyAgents)
                NumberOfNearbyAgents = NumberOfNearbyAgents + 1 
                i = i + 1
            elif Dist == 0:
                Phase.SwapAgents(Phase,i,0)
                i = i + 1
        
            i = i - 1

        return NumberOfNearbyAgents















                        




        
