import numpy as np
from dynamics_utils import Phase
from interactions import interactions
from math_utils import math_utils
from param_utils import sit_parameters_t as sit
from param_utils import flocking_model_params_t as fl
from param_utils import flocking_params_test as fl_test
from param_utils import unit_model_params_t as unit
from robotmodel import robotmodel 
import random 


model = robotmodel()
M = math_utils()
P = Phase()  # 函数调用
# 定义系统参数
ActualSitParams = sit()

# 定义集群参数 并且初始化（以后考虑将集群参数初始化模块化）

ActualFlockingParams = fl_test()

# 定义个体参数

ActualUnitParams = unit()

Now = 0 
TimeStep = 0



# PhaseData = [Phase(3, 3)]*10
inter = interactions()


# C_Frict_l = 0.05
# V_Frict_l = 0.63
# Acc_l = 4.16
# p_l = 3.2
# R_0_l = 85.3
# Dim_l = 3

# for i in range(3):
#      PhaseData[0].Coordinates[0][i] = i+1
#      PhaseData[0].Coordinates[1][i] = i*2
#      PhaseData[0].Coordinates[2][i] = i*3

#      PhaseData[0].Velocities[i] = 3



# B = inter.RepulsionLin(PhaseData[0], 7, 0.4, 4, 0, 3, 0)

# a = np.zeros([1,3])


# for i in range(0, 3):
#     a[0][i] = i+1
# print(a)
# math_utils.UnitVect(a[0], 3)
# print(a)

#------------------------------------------------------------------------------

if __name__ == '__main__':

    # 定义系统参数
    ActualSitParams.Length = 600
    ActualSitParams.InitialX = 3000
    ActualSitParams.InitialY = 3000
    ActualSitParams.InitialZ = 0.0
    ActualSitParams.DeltaT = 0.01
    ActualSitParams.Radius = 300
    ActualSitParams.LengthToStore = 15
    ActualSitParams.VizSpeedUp = 10
    ActualSitParams.StartOfSteadyState = 0
    ActualSitParams.NumberOfAgents = 20
    
    # 定义集群参数 上面已经定义了

    # 定义个体参数
    ActualUnitParams.Tau_PID_XY = 1
    ActualUnitParams.Tau_PID_Z = 1
    ActualUnitParams.a_max = 600
    ActualUnitParams.R_C = 11000.0
    ActualUnitParams.t_del = 1.0
    ActualUnitParams.t_GPS = 1.0
    ActualUnitParams.Sigma_GPS_XY = 50.0
    ActualUnitParams.Sigma_Outer_XY = 2000.0
    ActualUnitParams.packet_loss_distance = 8000.0

    # 定义相空间
    ActualPhase = Phase(ActualSitParams.NumberOfAgents, ActualFlockingParams.NumberOfInnerStates)
    
    TimeStepsToStore = (int)((20 + ActualSitParams.LengthToStore) / ActualSitParams.DeltaT - 1.0)

    PhaseData = [Phase(ActualSitParams.NumberOfAgents, ActualFlockingParams.NumberOfInnerStates)] * (1 + TimeStepsToStore)



    AgentsInDanger = np.zeros(ActualSitParams.NumberOfAgents)

    for i in AgentsInDanger:
        i = False
    
    model.InitializePreferredVelocities(ActualPhase, ActualFlockingParams, ActualSitParams, ActualUnitParams)

    # 初始化位置

    MaxStep = 100 * PhaseData[0].NumberOfAgents


    RandomPlaceVector = M.Fillvect(27000,27000,27000)
    ActualAgentsVelocity = M.Fillvect(0, 0, 0)

    isArrangementCorrect = False

    for i in range(PhaseData[0].NumberOfAgents):
        PhaseData[0].InsertAgentsCoordinates(PhaseData[0], RandomPlaceVector, i)
        PhaseData[0].InsertAgentsVelocity(PhaseData[0],ActualAgentsVelocity, i)

    for i in range(PhaseData[0].NumberOfAgents):
        while isArrangementCorrect is False:
            isArrangementCorrect = True

            RandomPlaceVector[0] = random.uniform(-ActualSitParams.InitialX / 2.0, ActualSitParams.InitialX /2.0)
            RandomPlaceVector[1] = random.uniform(-ActualSitParams.InitialY / 2.0, ActualSitParams.InitialY /2.0)
            RandomPlaceVector[2] = random.uniform(-ActualSitParams.InitialZ / 2.0, ActualSitParams.InitialZ /2.0)

            for j in range(PhaseData[0].NumberOfAgents):
                if i == j:
                    j = PhaseData[0].NumberOfAgents - 1
                    continue
                AgentjsCoords = PhaseData[0].GetAgentsCoordinates(PhaseData[0], j)
                DiffCoords = M.VectDifference(AgentjsCoords, RandomPlaceVector)
                if (M.VectAbs(DiffCoords) <= 4 * ActualSitParams.Radius):
                    isArrangementCorrect = False
                    break

        PhaseData[0].InsertAgentsCoordinates(PhaseData[0], RandomPlaceVector, i)
        PhaseData[0].InsertAgentsVelocity(PhaseData[0], ActualAgentsVelocity, i)
        isArrangementCorrect = False

    # 赋值给当前Phase
    for i in range(ActualSitParams.NumberOfAgents):
        for j in range(3):
            ActualPhase.Velocities[i][j] = PhaseData[0].Velocities[i][j]
            ActualPhase.Coordinates[i][j] = PhaseData[0].Coordinates[i][j]

    # 初始化场地，根据场地初始化位置

    # InitializePhase()

    # 填充时间线waiting...
    TimeToWait = 5.0 + ActualSitParams.DeltaT
    # print((int)(TimeToWait / ActualSitParams.DeltaT))
    for i in range(1, (int)((1+TimeToWait) / ActualSitParams.DeltaT)):
        for j in range(PhaseData[0].NumberOfAgents):
            for k in range(3):
                PhaseData[i].Coordinates[j][k] = PhaseData[i-1].Coordinates[j][k]
                PhaseData[i].Velocities[j][k] = 0.0
            for k in range(PhaseData[0].NumberOfInnerStates):
                PhaseData[i].InnerStates[j][k] = PhaseData[i-1].InnerStates[j][k]
        #print("第%d个的位置为：\r\n" % i, PhaseData[i].Coordinates)

    # 初始化Now
    Now = Now + round((5.0 + ActualUnitParams.t_del)/ActualSitParams.DeltaT)
    TimeBeforeFlock = 10.0 + ActualUnitParams.t_del

    ConditionsReset = [True, True]


    # 设置观察参数

    # 开始主循环
    ElapsedTime = (Now * ActualSitParams.DeltaT) - 5.0 - ActualUnitParams.t_del


    Collisions = 0
    print(Collisions)

    Accelerations = np.zeros(ActualSitParams.NumberOfAgents)

    while(ElapsedTime < 100):
        if Now < TimeStepsToStore:
            
            Collisions = model.Step(ActualPhase, PhaseData, ActualUnitParams, ActualFlockingParams, ActualSitParams,
                       Now, (int)(ElapsedTime/ActualSitParams.DeltaT), True, ConditionsReset, AgentsInDanger, Accelerations)           
            P.insert_phase_to_dataline(PhaseData, ActualPhase, Now + 1 )
            print(PhaseData[Now+1].Coordinates[0])
        else:
            pass

        ElapsedTime += ActualSitParams.DeltaT 
        Now += 1



    # print(PhaseData[0].Coordinates)
    # print(ActualPhase.Coordinates)









    














     






    





