from math_utils import math_utils as M 
import numpy as np
import math


ARENA_SQUARE = 1
ARENA_CIRCLE = 0

# 关于边界


class arena_t:

    def __init__(self):
        self.index = 0
        self.name = ''
        self.p = np.zeros((128, 3))
        self.p_count = 0


class arenas_t:
    def __init__(self):
        self.a = [arena_t()]*32
        self.a_count = 0



class arena:

    def Shill_Wall_LinSqrt(self, Phase, ArenaCenterX, ArenaCenterY, ArenaRadius, Arena, V_Shill, R0_Offset_Shill, Acc_Shill,
                           Slope_Shill, WhitchAgent, Dim_l):
        OutputVelocity = np.zeros(3)
        AgentsPosition = Phase.Coordinates[WhitchAgent]
        AgentsVelocity = Phase.Velocities[WhitchAgent]

        ArenaCenter = M.Fillvect(ArenaCenterX, ArenaCenterY, 0.0)

        ToCenter = M.VectDifference(ArenaCenter, AgentsPosition)
        if 2.0 == Dim_l:
            ToCenter[2] = 0.0

        # 方形区域
        if Arena.index == ARENA_SQUARE:
            for i in range(Dim_l):
                DistFromWall = ArenaRadius - math.fabs(ToCenter[i])
                ToArena = M.NullVect(3)
                ToArena[i] = ToCenter[i]
                ToArena = M.UnitVect(ToArena)
                ToArena = M.MultiplicateWithScalar(ToArena, V_Shill, 3)
                ToArena = M.VectDifference(ToArena, AgentsVelocity)
                if 2 == Dim_l:
                    ToArena[2] = 0.0
                VelDiff = M.VectAbs(ToArena)
                ToArena = M.UnitVect(ToArena)

                MaxVelDiff = M.VelDecayLinSqrt(DistFromWall, Slope_Shill, Acc_Shill, VelDiff. R0_Offset_Shill)  # 计算最大允许速度差
                if VelDiff > MaxVelDiff:
                    ToArena = M.MultiplicateWithScalar(ToArena, VelDiff - MaxVelDiff, Dim_l)
                    OutputVelocity = M.VectSum(OutputVelocity, ToArena)
        # 圆形区域
        elif Arena.index == ARENA_CIRCLE:
            DistFromWall = ArenaRadius - M.VectAbs(ToCenter)
            ToArena = M.UnitVect(ToCenter)
            ToArena = M.MultiplicateWithScalar(ToArena, V_Shill, 3)
            ToArena = M.VectDifference(ToArena, AgentsVelocity)
            if 2 == Dim_l:
                ToArena[2] = 0.0
            VelDiff = M.VectAbs(ToArena)
            ToArena = M.UnitVect(ToArena)

            MaxVelDiff = M.VelDecayLinSqrt(DistFromWall, Slope_Shill, Acc_Shill, VelDiff. R0_Offset_Shill)  # 计算最大允许速度差
            if VelDiff > MaxVelDiff:
                ToArena = M.MultiplicateWithScalar(ToArena, VelDiff - MaxVelDiff, Dim_l)
                OutputVelocity = M.VectSum(OutputVelocity, ToArena)
        else:
            OutputVelocity = M.NullVect(3)

        if 2 == Dim_l:
            OutputVelocity[2] = 0

        return OutputVelocity




                


            





