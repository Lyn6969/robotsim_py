import numpy as np
from dynamics_utils import Phase
from interactions import interactions
from math_utils import math_utils
from param_utils import sit_parameters_t as sit
from param_utils import flocking_model_params_t as fl
from param_utils import flocking_params_test as fl_test
from param_utils import unit_model_params_t as unit
from robotmodel import robotmodel 





model = robotmodel()
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
    ActualSitParams.NumberOfAgents = 50
    ActualSitParams.Length = 600
    ActualSitParams.InitialX = 30000
    ActualSitParams.InitialY = 30000
    ActualSitParams.InitialZ = 30000
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



     






    





