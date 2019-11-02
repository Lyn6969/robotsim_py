import numpy as np 
from math_utils import math_utils as M
from interactions import interactions as inter 
from arenas import arena 
from arenas import arenas_t
from dynamics_utils import Phase
from algo_spp_evol import algo 

'''
真实无人集群个体模型
'''
SteppedPhase = Phase()
NubmerOfNeighbours = 0

class robotmodel:


    # 欧拉法求解运动学
    def RealCoptForceLaw(self, phase, RealVelocity, UnitParams,
                         FlockingParams, VizParams, DeltaT, TimeStepReal,
                         TimeStepLooped, WhitchAgent, WindVelocityVector):

        PreviousVelocity = Phase.GetAgentsVelocity(phase, 0)
        OutputInnerStates = np.zeros(3)
        OutputVelocity = np.zeros(3)
        PreferredVelocities = np.zeros((phase.NumberOfAgents, 3))

        for i in range(phase.NumberOfInnerStates):
            OutputInnerStates[i] = phase.InnerStates[0][i]

        if TimeStepLooped % ((int)(UnitParams.t_GPS.Value) / DeltaT) == 0:

            TempTarget = M.NullVect(3)

            TempTarget = algo.CalulatePreferredVelocity(phase, 0, FlockingParams, VizParams, UnitParams.t_del.value, TimeStepReal * DelTaT)

            for i in range(3):
                PreferredVelocities[WhitchAgent][i] = TempTarget[i]  # 计算应该有的速度s

        for i in range(2):
            OutputVelocity[i] = RealVelocity[i] + (DeltaT / UnitParams.Tau_PID_XY_Value)*(PreferredVelocities[WhitchAgent][i]-PreviousVelocity[i])   # 计算一个时间步XY应该有的速度变化量

        OutputVelocity[2] = RealVelocity[2] + (DeltaT / UnitParams.Tau_PID_Z_Value) * (PreferredVelocities[WhitchAgent][i] - PreviousVelocity[i])   #  计算一个时间步Z应该有的速度变化量 

        return OutputVelocity, OutputInnerStates

    # 计算智能体观测到的智能体的相空间，参数可扩展
    def CreatPhase(self, phase, WhitchAgent, R_C, packet_loss_ratio, packet_loss_distance, OderBydistance):
        LocalActualPhaseToCreate = Phase()
        LocalActualPhaseToCreate.NumberOfAgentes = phase.NumberOfAgentes
        LocalActualPhaseToCreate.NumberOfInnerStates = phase.NumberOfInnerStates

        for i in range(phase.NumberOfAgents):

            LocalActualPhaseToCreate.RealIDs[i] = phase.RealIDs[i]

            for j in range(3):
                LocalActualPhaseToCreate.Coordinates[i][j] = phase.Coordinates[i][j]
                LocalActualPhaseToCreate.Velocity[i][j] = phase.Velocities[i][j]
            
            for j in range(phase.NumberOfInnerStates):
                LocalActualPhaseToCreate.InnerStates[i][j] = phase.InnerStates[i][j] 

        ActualAgentsPosition = np.zeros(3)
        ActualAgentsPosition = Phase.SelectNearbyVisibleAgents(phase, WhitchAgent)

        if OderBydistance is True:

            NubmerOfNeighbours = Phase.SelectNearbyVisibleAgents(LocalActualPhaseToCreate, ActualAgentsPosition, R_C,
                                                        (packet_loss_ratio / packet_loss_distance / packet_loss_distance) if(packet_loss_distance > 0) else 0)
        else:
            Phase.SwapAgents(LocalActualPhaseToCreate, WhitchAgent, 0)
            NubmerOfNeighbours = 1

        # 添加延迟和GPS不精确，造成的位置和速度差异(可用于其他接口，暂时略过)

        LocalActualPhaseToCreate.NumberOfAgents = NubmerOfNeighbours

        return LocalActualPhaseToCreate



    #  更新速度和位置
    def Step(self, OutputPhase, PhaseData, UnitParams,
             FlockingParams, SitParams, VizParams,
             TimeStepLooped, TimeStepReal, CountCollisions,
             ConditionReset, Collosions, AgentsInDanger, Accelerations):

        CheckVelocityCache = np.zeros(3)
        CheckAccelerationCache = np.zeros(3)
        CheckDiffrenceCache = np.zeros(3)
        UnitVectDifference = np.zeros(3)

        LocalActualPhase = PhaseData[TimeStepLooped]

        PreviousColl = 0
        if CountCollisions is True:
            Collisions += Phase.HowManyCollisions(LocalActualPhase, AgentsInDanger, CountCollisions, SitParams.Radius) # AgentsInDanger是布尔型的数组

        # 更新位置

        Velocity = np.zeros(3)
        CoordinatesToStep = np.zeros(3)

        for j in range(SitParams.NumberOfAgentes):
            Velocity = Phase.GetAgentsVelocity(LocalActualPhase, j)
            CoordinatesToStep = Phase.GetAgentsCoordinates(LocalActualPhase, j)

            for i in range(3):
                CoordinatesToStep[i] += Velocity[i] * SitParams.DeltaT

            Phase.InsertAgentsCoordinates(SteppedPhase, CoordinatesToStep, j)


        # GPS 跳过

        # 风速跳过

        # 真实力学规律

        RealCoptForceVector = np.zeros(3)
        ActualRealVelocity = np.zeros(3)

        for j in range(SitParams.NumberOfAgents):

            # Debug部分暂时省略

            # 创建所有邻居的状态空间
            TempPhase = self.CreatPhase(LocalActualPhase, j, UnitParams.R_C.Value,UnitParams.packet_loss_ratio.Value, 
                                        UnitParams.packet_loss_distance.Value, (TimeStepLooped % (int)(UnitParams.t_GPS.Value / SitParams.DeltaT) == 0) )

            ActuralRealVelocity = Phase.GetAgentsVelocity(LocalActualPhase, j)

            # 使用欧拉谷山法求解牛顿运动学问题
            RealCoptForceVector, ChangedInnerStateOfActualAgent = self.RealCoptForceLaw(TempPhase, ActualRealVelocity, UnitParams, FlockingParams,
                                                                                        VizParams, SitParams.DeltaT, TimeStepReal, TimeStepLooped, j, WindVelocityVector)
            CheckVelocityCache = M.VectSum(CheckVelocityCache, RealCoptForceVector)

            # 更新InnerStates

            Phase.InsertAgentsVelocity(SteppedPhase, CheckVelocityCache, j)
            for k in range(PhaseData[0].NumberOfInnerStates):
                SteppedPhase.InnerStates[j][k] = ChangedInnerStateOfActualAgent[k]

        # 在添加噪声之前保存加速度的最大值
        OnePerDeltaT = 1.0 / SitParams.DeltaT
        
        for i in range(SitParams.NumberOfAgents):
            CheckAccelerationCache = Phase.GetAgentsVelocity(LocalActualPhase, i)
            CheckVelocityCache = Phase.GetAgentsVelocity(SteppedPhase, i)
            CheckDiffrenceCache = M.VectDifference(CheckVelocityCache, CheckAccelerationCache)
            UnitVectDifference = M.UnitVect(CheckVeAccelerationCache)

            Accelerations[i] = M.VectAbs(CheckDiffrenceCache) * OnePerDeltaT  # 一秒钟的加速度

            if Accelerations[i] > UnitParams.a_max.Value:
                for k in range(3):
                    CheckAccelerationCache[k] = CheckAccelerationCache[k] + UnitParams.a_max.Value * SitParams.DeltaT * UnitVectDifference[k] 

            
            





        

        





            
        





        



        







