import numpy as np 
from math_utils import math_utils as M
from interactions import interactions as inter 
from arenas import arena 
from arenas import arenas_t

'''
3D 自推进集群模型
'''


class algo:

    def __init__(self, V_Flock, V_Max, V_Rep, R_0, Slope_Rep, C_Frict, V_Frict,
                 R_0_Offset_Frict, Slope_Frict, Acc_Frict, V_Shill, Acc_Shill,
                 Slope_Shill, R_0_SHill, Dim, ArenaRadius, ArenaShape,  ArenaCenterX, 
                 ArenaCenterY):
        
        # 场地结构
        self.Arenas = arenas_t()

        # 自驱动参数
        self.V_Flock = V_Flock
        self.V_Max = V_Max

        # 斥力参数
        self.V_Rep = V_Rep
        self.R_0 = R_0
        self.Slope_Rep = Slope_Rep

        # 引力参数
        self.C_Frict = C_Frict
        self.V_Frict = V_Frict
        self.R_0_Offset_Frict = R_0_Offset_Frict
        self.Slope_Frict = Slope_Frict
        self.Acc_Frict = Acc_Frict

        # 墙和虚拟智能体参数
        self.V_Shill = V_Shill
        self.Acc_Shill = Acc_Shill
        self.Slope_Shill = Slope_Shill
        self.R_0_Shill = R_0_SHill

        # 二维或者是三维
        self.Dim = Dim

        # 场地信息
        self.ArenaRadius = ArenaRadius
        self.ArenaShape = ArenaShape
        self.ArenaCenterX = ArenaCenterX
        self.ArenaCenterY = ArenaCenterY

    def CalulatePreferredVelocity(self, Phase, WhitchAgent, FlockingParams, VizParams, Delay, ActualTime):
        OutputVelocity = np.zeros(3)

        AgentCoordinates = Phase.Coordinates[WhitchAgent]
        AgentsVelocity = Phase.Velocities[WhitchAgent]

        # 自驱动项
        NormalizedAgentsVelocity = M.Fillvect(AgentsVelocity[0], AgentsVelocity[1], AgentsVelocity[2])
        NormalizedAgentsVelocity = M.UnitVect(NormalizedAgentsVelocity)
        NormalizedAgentsVelocity = M.MultiplicateWithScalar(NormalizedAgentsVelocity, self.V_Flock, self.Dim)

        # 斥力项
        PotentialVelocity = inter.RepulsionLin(Phase, self.V_Rep, self.Slope_Rep, self.R_0,
                                               WhitchAgent, self.Dim)

        # 引力项
        SlipVelocity = inter.FrictionLinSqrt(Phase, self.C_Frict, self.V_Frict, self.Acc_Frict,
                                             self.Slope_Frict, self.R_0+self.R_0_Offset_Frict,
                                             WhichAgent, self.Dim)

        # 与墙的交互
        # ArenaVelocity = arena.Shill_Wall_LinSqrt(Phase, self.ArenaCenterX, self.ArenaCenterY, self.ArenaRadius,
        #                                          self.Arenas.a[self.ArenaShape], self.V_Shill, self.R_0_Shill,
        #                                          self.Acc_Shill, self.R_0_Shill, self.Acc_Shill, self.Slope_Shill, WhitchAgent, self.Dim)

        # 与障碍物交互，暂时不添加


        # 最终结果
        OutputVelocity = M.VectSum(OutputVelocity, NormalizedAgentsVelocity)
        OutputVelocity = M.VectSum(OutputVelocity, PotentialVelocity)
        OutputVelocity = M.VectSum(OutputVelocity, SlipVelocity)
        OutputVelocity = M.VectSum(OutputVelocity, ArenaVelocity)

        # 输出速度大于最大速度则使用最大速度 存疑
        CutOffMode = False
        if CutOffMode is False:
            OutputVelocity = M.UnitVect(OutputVelocity)
            OutputVelocity = M.MultiplicateWithScalar(OutputVelocity, self.V_Flock, self.Dim)
        else:
            if M.VectAbs(OutputVelocity) > self.V_Max:
                OutputVelocity = M.UnitVect(OutputVelocity)
                OutputVelocity = M.MultiplicateWithScalar(OutputVelocity, self.V_Max, self.Dim)

        return OutputVelocity

            



        





        








                                
