
import numpy as np 
from dynamics_utils import Phase
from math_utils import math_utils


'''
集群规则定义，尽量可扩展
'''

M = math_utils()


class interactions:

    def RepulsionLin(self, Phase, V_Rep_l, p_l, R_0_l, WhichAgent, Dim_l, normlize):
        '''
        斥力作用函数

        :Phase: 状态空间
        :
        '''

        n = 0
        OutputVelocity = np.zeros(3)
        AgentsCoordinates = Phase.Coordinates[WhichAgent]
        # print(AgentsCoordinates)


        for i in range(Phase.NumberOfAgents):
            if i == WhichAgent:
                continue
            
            NeighbourCoordinates = Phase.Coordinates[i]
            # print(NeighbourCoordinates)
            DifferenceVector = M.VectDifference(AgentsCoordinates, NeighbourCoordinates)

            if Dim_l == 2:
                DifferenceVector[2] = 0
           
            DistanceFromNeighbour = M.VectAbs(DifferenceVector)
            # print("DistancFromNeighbour=", DistanceFromNeighbour)

            if DistanceFromNeighbour >= R_0_l:
                continue

            n += 1
            DifferenceVector = M.UnitVect(DifferenceVector)
            # print(DifferenceVector)
            DifferenceVector = M.MultiplicateWithScalar(DifferenceVector, M.SigmoidLin(DistanceFromNeighbour, p_l, V_Rep_l, R_0_l), Dim_l)
            # print(DifferenceVector)

            OutputVelocity = M.VectSum(OutputVelocity, DifferenceVector)
            # print(OutputVelocity)

            if (normlize and n) > 1:
                length = M.VectAbs(OutputVelocity) / n
                OutputVelocity = M.UnitVect(OutputVelocity) 
                OutputVelocity = M.MultiplicateWithScalar(OutputVelocity, length, Dim_l)
            
        return OutputVelocity

    def FrictionLinSqrt(self, Phase, C_Frict_l, V_Frict_l, Acc_l, p_l, R_0_l, WhichAgent, Dim_l): # 吸引力
        OutputVelocity = np.zeros(3)

        AgentsCoordinates = Phase.Coordinates[WhichAgent]
        AgentVelocity = Phase.Velocities[WhichAgent]

        for i in range(Phase.NumberOfAgents):
            if i == WhichAgent:
                continue

            # 计算邻居距离
            NeighbourCoordinates = Phase.Coordinates[i]
            DifferenceVector = M.VectDifference(NeighbourCoordinates, AgentsCoordinates)
            DistanceFromNeighbour = M.VectAbs(DifferenceVector)

            #  计算和邻居的速度差
            NeiboursVelocity = Phase.Velocities[i]
            DifferenceVector = M.VectDifference(NeiboursVelocity, AgentVelocity)
            if 2 == Dim_l:
                DifferenceVector[2] = 0
            VelDiff = M.VectAbs(DifferenceVector)
            DifferenceVector = M.UnitVect(DifferenceVector)

            MaxVelDiff = max(V_Frict_l, M.VelDecayLinSqrt(DistanceFromNeighbour, p_l, Acc_l, VelDiff, R_0_l))

            if VelDiff > MaxVelDiff:
                DifferenceVector = M.MultiplicateWithScalar(DifferenceVector, C_Frict_l * (VelDiff-MaxVelDiff), Dim_l)
                # print(OutputVelocity)
                # print(DifferenceVector)
                OutputVelocity = M.VectSum(OutputVelocity, DifferenceVector)
                
        return OutputVelocity





        











