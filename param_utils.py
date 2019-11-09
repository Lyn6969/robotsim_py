import numpy as np 

# 个体参数
class unit_param_double_t:

    def __init__(self):
        self.Name = ''
        self.UnitOfMeas = ''
        self.Value = 0


class unit_model_params_t:

    def __init__(self):

        # PID 控制器松弛时间
        self.Tau_PID_XY = 0
        self.Tau_PID_Z =0

        # 最大加速度
        self.a_max = 0

        # GPS设备的刷新率
        self.t_GPS = 0

        # 通信延迟
        self.t_del = 0

        # 通信范围
        self.R_C = 0

        # 丢包率相关
        self.packet_loss_ratio = 0
        self.packet_loss_distance = 0

        # 噪声参数
        self.Sigma_GPS_XY = 0 # inner noise
        self.Sigma_Outer_XY = 0 # outer noise
        self.Sigma_GPS_Z = 0  # inner noise
        self.Sigma_Outer_Z = 0 # outer noise

        # 风的影响（暂时加入，实际上并不需要）
        # self.Wind_Magn_Avg = unit_param_double_t()
        # self.Wind_StDev = unit_param_double_t()
        # self.Wind_Angle = unit_param_double_t()
        # self.ViscosityCoeff = unit_param_double_t()

# 系统（环境）参数
class sit_parameters_t:

    def __init__(self):
        
        # 智能体数量
        self.NumberOfAgents = 0

        # 初始化x,y,z的位置
        self.InitialX = 0.0
        self.InitialY = 0.0
        self.InitialZ = 0.0

        # 测试时间
        self.Length = 0

        # 欧拉方法的步长
        self.DeltaT = 0

        # 无人机的避险半径
        self.Radius = 0

        # 存储时间长度
        self.LengthToStore = 0

        # 默认的观察速度
        self.VizSpeedUp = 0

        # 
        self.StartOfSteadyState = 0


# 单个集群参数类
class fl_param_double_t:

    def __init__(self):
        # 名称
        self.Name = ''
        # 单位
        self.UnitOfMeas = ''
        # 值
        self.Value = 0

        # 显示参数暂时不需要
        self.Digits = 2
        self.SizeOfStep = 0
        self.Mult = 1 # 线性算子

        # 最大最小值
        self.Min = 0
        self.Max = 0

        # 是否可以修改
        self.Constant = False
        self.StoredValus = 0


# 所有集群参数类
class flocking_model_params_t:
    def __init__(self):
        self.Name = ''
        self.NumberOfParameters = 0
        self.Params = [fl_param_double_t()]*19 
        self.NumberOfInpus = 0
        self.NumberOfInnerStats = 0


class flocking_params_test:
    def __init__(self):
        self.NumberOfInnerStates = 0
        self.NumberOfParameters = 19
        # Preferred SPP velocity (Multiply it with 0.010000 to get its Value in m/s)
        self.V_Flock = 400.000000
        # Maximum repulsion velocity (Multiply it with 0.010000 to get its Value in m/s)
        self.V_Rep = 619.664000
        # Friction velocity slack (Multiply it with 0.010000 to get its Value in m/s)
        self.V_Frict = 50.000000
        # Maximum velocity (Multiply it with 0.010000 to get its Value in m/s)
        self.V_Max = 800.000000
        # Equilibrium distance (Multiply it with 0.010000 to get its Value in m)
        self.R_0 = 2566.680000
        # Friction range (Multiply it with 0.010000 to get its Value in m)
        self.R_0_Offset_Frict = 10000.000000
        # Shill dist offset (Multiply it with 0.010000 to get its Value in m)
        self.R_0_Shill = 0.000000
        # Slope of repulsion (1/s)
        self.Slope_Rep = 0.400000
        # Slope of friction (1/s)
        self.Slope_Frict = 0.400000
        # Acc limit of Friction (Multiply it with 0.010000 to get its Value in m/s^2)
        self.Acc_Frict = 250.000000
        # Slope of wall (1/s)
        self.Slope_Shill = 0.400000
        # Velocity of shill agents (Multiply it with 0.01000s20 to get its Value in m/s)
        self.V_Shill = 652.174000
        # Acc limit of shill (Multiply it with 0.010000 to get its Value in m/s^2)
        self.Acc_Shill = 250.000000
        # Arena Radius (Multiply it with 0.010000 to get its Value in m)
        self.ArenaRadiu = 40000.000000
        # Friction coefficient (-)
        self.C_Frict = 0.023802
        # Arena  Center X (Multiply it with 0.010000 to get its Value in m)
        self.ArenaCenterX = -5717.009528
        # Arena Center Y (Multiply it with 0.010000 to get its Value in m)
        self.ArenaCenterY = -18249.859933
        # Shape of the arena (Multiply it with 0.000000 to get its real Value)
        self.ArenaShape = 1.000000
        # Number of dimensiodns in the simulation (2 or 3)
        self.Dim = 2.000000






        
