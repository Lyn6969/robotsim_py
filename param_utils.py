import numpy as np 


class unit_param_double_t:

    def __init__(self):
        self.Name = ''
        self.UnitOfMeas = ''
        self.Value = 0


class unit_model_params_t:

    def __init__(self):

        # PID 控制器松弛时间
        self.Tau_PID_XY = unit_param_double_t()
        self.Tau_PID_Z = unit_param_double_t()

        # 最大加速度
        self.a_max = unit_param_double_t()

        # GPS设备的刷新率
        self.t_GPS = unit_param_double_t()

        # 通信延迟
        self.t_del = unit_param_double_t()

        # 通信范围
        self.R_C = unit_param_double_t()

        # 丢包率相关
        self.packet_loss_ratio = unit_param_double_t()
        self.packet_loss_distance = unit_param_double_t()

        # 噪声参数
        self.Sigma_GPS_XY = unit_param_double_t()  # inner noise
        self.Sigma_Outer_XY = unit_param_double_t()  # outer noise
        self.Sigma_GPS_Z = unit_param_double_t()   # inner noise
        self.Sigma_Outer_Z = unit_param_double_t() # outer noise

        # 风的影响（暂时加入，实际上并不需要）
        self.Wind_Magn_Avg = unit_param_double_t()
        self.Wind_StDev = unit_param_double_t()
        self.Wind_Angle = unit_param_double_t()
        self.ViscosityCoeff = unit_param_double_t()



        




