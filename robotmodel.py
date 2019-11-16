import numpy as np
from math_utils import math_utils
from interactions import interactions as inter 
from arenas import arena 
from arenas import arenas_t
from dynamics_utils import Phase
from algo_spp_evol import algo


'''
真实无人集群个体模型
'''
phase_fun = Phase()
math_fun = math_utils()
algo_fun = algo()


class robotmodel:

    def __init__(self):

        self.PreferredVelocities = 0
        self.SteppedPhase = 0
        self.NubmerOfNeighbours = 0
        self.TempPhase = 0
        





    def InitializePreferredVelocities(self, phase, FlockingParams, SitParams, UnitParams):

        self.PreferredVelocities = np.zeros((SitParams.NumberOfAgents, 3), dtype =float)   # 期望速度速度信息

        for i in range(SitParams.NumberOfAgents):
            for j in range(3):
                self.PreferredVelocities[i][j] = 0.0
        
        self.SteppedPhase = Phase(SitParams.NumberOfAgents, phase.NumberOfInnerStates)
        self.TempPhase = Phase(SitParams.NumberOfAgents, phase.NumberOfInnerStates)
        

        

        


    # 欧拉法求解运动学
    def RealCoptForceLaw(self, phase, RealVelocity, UnitParams,
                         FlockingParams, DeltaT, TimeStepReal,
                         TimeStepLooped, WhitchAgent):

        PreviousVelocity = phase_fun.GetAgentsVelocity(phase, 0)
        OutputInnerStates = np.zeros(3)
        OutputVelocity = np.zeros(3)
        PreferredVelocities = np.zeros((phase.NumberOfAgents, 3))

        for i in range(phase.NumberOfInnerStates):
            OutputInnerStates[i] = phase.InnerStates[0][i]

        if TimeStepLooped % ((int)(UnitParams.t_GPS) / DeltaT) == 0:

            TempTarget = math_fun.NullVect(3)

            TempTarget = algo_fun. CalulatePreferredVelocity(phase, 0, FlockingParams,UnitParams.t_del, TimeStepReal * DeltaT)

            for i in range(3):
                PreferredVelocities[WhitchAgent][i] = TempTarget[i]  # 计算应该有的速度s

        for i in range(2):
            OutputVelocity[i] = RealVelocity[i] + (DeltaT / UnitParams.Tau_PID_XY)*(PreferredVelocities[WhitchAgent][i]-PreviousVelocity[i])   # 计算一个时间步XY应该有的速度变化量

        OutputVelocity[2] = RealVelocity[2] + (DeltaT / UnitParams.Tau_PID_Z) * (PreferredVelocities[WhitchAgent][i] - PreviousVelocity[i])   #  计算一个时间步Z应该有的速度变化量 

        return OutputVelocity, OutputInnerStates

    # 计算智能体观测到的智能体的相空间，参数可扩展
    def CreatPhase(self, phase, WhitchAgent, R_C, packet_loss_ratio, packet_loss_distance, OderBydistance):
        LocalActualPhaseToCreate = Phase(phase.NumberOfAgents)
        LocalActualPhaseToCreate.NumberOfAgents = phase.NumberOfAgents
        LocalActualPhaseToCreate.NumberOfInnerStates = phase.NumberOfInnerStates

        for i in range(phase.NumberOfAgents):

            LocalActualPhaseToCreate.RealIDs[i] = phase.RealIDs[i]

            for j in range(3):
                LocalActualPhaseToCreate.Coordinates[i][j] = phase.Coordinates[i][j]
                LocalActualPhaseToCreate.Velocities[i][j] = phase.Velocities[i][j]
            
            for j in range(phase.NumberOfInnerStates):
                LocalActualPhaseToCreate.InnerStates[i][j] = phase.InnerStates[i][j] 

        ActualAgentsPosition = np.zeros(3)
        ActualAgentsPosition = phase_fun.GetAgentsCoordinates(phase, WhitchAgent)

        if OderBydistance is True:

            NubmerOfNeighbours = phase_fun.SelectNearbyVisibleAgents(LocalActualPhaseToCreate, ActualAgentsPosition, R_C,
                                                        (packet_loss_ratio / packet_loss_distance / packet_loss_distance) if(packet_loss_distance > 0) else 0)
        else:
            phase_fun.SwapAgents(LocalActualPhaseToCreate, WhitchAgent, 0)
            NubmerOfNeighbours = 1

        # 添加延迟和GPS不精确，造成的位置和速度差异(可用于其他接口，暂时略过)

        LocalActualPhaseToCreate.NumberOfAgents = NubmerOfNeighbours

        return LocalActualPhaseToCreate



    #  更新速度和位置
    def Step(self, OutputPhase, PhaseData, UnitParams,
             FlockingParams, SitParams,
             TimeStepLooped, TimeStepReal, CountCollisions,
             ConditionReset, AgentsInDanger, Accelerations):

        CheckVelocityCache = np.zeros(3)
        CheckAccelerationCache = np.zeros(3)
        CheckDiffrenceCache = np.zeros(3)
        UnitVectDifference = np.zeros(3)
        Collisions = 0

        LocalActualPhase = PhaseData[TimeStepLooped]

        PreviousColl = 0
        if CountCollisions is True:
            Collisions += phase_fun.HowManyCollisions(LocalActualPhase, AgentsInDanger, CountCollisions, SitParams.Radius) # AgentsInDanger是布尔型的数组

        # 更新位置

        Velocity = np.zeros(3)
        CoordinatesToStep = np.zeros(3)

        for j in range(SitParams.NumberOfAgents):
            Velocity = phase_fun.GetAgentsVelocity(LocalActualPhase, j)
            CoordinatesToStep = phase_fun.GetAgentsCoordinates(LocalActualPhase, j)

            for i in range(3):
                CoordinatesToStep[i] += Velocity[i] * SitParams.DeltaT

            phase_fun.InsertAgentsCoordinates(self.SteppedPhase, CoordinatesToStep, j)


        # GPS 跳过

        # 风速跳过

        # 真实力学规律

        RealCoptForceVector = np.zeros(3)
        ActualRealVelocity = np.zeros(3)

        for j in range(SitParams.NumberOfAgents):

            # Debug部分暂时省略

            # 创建所有邻居的状态空间
            TempPhase = self.CreatPhase(LocalActualPhase, j, UnitParams.R_C,UnitParams.packet_loss_ratio, 
                                        UnitParams.packet_loss_distance, (TimeStepLooped % (int)(UnitParams.t_GPS / SitParams.DeltaT) == 0) )

            ActuralRealVelocity = phase_fun.GetAgentsVelocity(LocalActualPhase, j)

            # 使用欧拉谷山法求解牛顿运动学问题
            RealCoptForceVector, ChangedInnerStateOfActualAgent = self.RealCoptForceLaw(TempPhase, ActualRealVelocity, UnitParams, FlockingParams,
                                                                                        SitParams.DeltaT, TimeStepReal, TimeStepLooped, j)
            CheckVelocityCache = math_fun.VectSum(CheckVelocityCache, RealCoptForceVector)

            # 更新InnerStates

            phase_fun.InsertAgentsVelocity(SteppedPhase, CheckVelocityCache, j)
            for k in range(PhaseData[0].NumberOfInnerStates):
                SteppedPhase.InnerStates[j][k] = ChangedInnerStateOfActualAgent[k]

        # 在添加噪声之前保存加速度的最大值
        OnePerDeltaT = 1.0 / SitParams.DeltaT
        
        for i in range(SitParams.NumberOfAgents):
            CheckAccelerationCache = phase_fun.GetAgentsVelocity(LocalActualPhase, i)
            CheckVelocityCache = phase_fun.GetAgentsVelocity(SteppedPhase, i)
            CheckDiffrenceCache = math_fun.VectDifference(CheckVelocityCache, CheckAccelerationCache)
            UnitVectDifference = math_fun.UnitVect(CheckVeAccelerationCache)

            Accelerations[i] = math_fun.VectAbs(CheckDiffrenceCache) * OnePerDeltaT  # 一秒钟的加速度

            if Accelerations[i] > UnitParams.a_max:
                for k in range(3):
                    CheckAccelerationCache[k] = CheckAccelerationCache[k] + UnitParams.a_max * SitParams.DeltaT * UnitVectDifference[k] 
                
                phase_fun.InsertAgentsVelocity(SteppedPhase, CheckVelocityCache, i)
                Accelerations[i] = UnitParams.a_max

        # 外部噪声

        # 重置智能体位置


        # 将相写入相空间
        for j in range(SitParams.NumberOfAgents):
            for i in range(3):
                OutputPhase.Coordinates[j][i] = SteppedPhase.Coordinates[j][i]
                OutputPhase.Velocities[j][i] = SteppedPhase.Velocities[j][i]
            for i in range(SteppedPhase.NumberOfInnerStates):
                OutputPhase.InnerStates[j][i] = SteppedPhase.InnerStates[j][i] 
        
        return Collisions
            






        




            
            
            





        

        





            
        





        



        







