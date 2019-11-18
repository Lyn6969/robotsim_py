import math  
import numpy as np 

'''
一些用到的数学函数
'''


class math_utils:

    def __init__(self):
        pass


    '''
    填充向量
    '''
    def Fillvect(self, x, y, z):

        VectToFill = np.zeros(3)
        VectToFill[0] = x
        VectToFill[1] = y
        VectToFill[2] = z
        return VectToFill


    '''
    平方和开根号
    '''
    def VectAbs(self, InputVector):
        return math.sqrt(InputVector[0]**2+InputVector[1]**2+InputVector[2]**2)

    '''
    两个向量的差
    '''
    def VectDifference(self, VectA, VectB):
        ABDiff = np.zeros(3)
        ABDiff = self.Fillvect(VectA[0]-VectB[0], VectA[1]-VectB[1], VectA[2]-VectB[2])
        return ABDiff

    '''
    两个向量的和
    '''
    def VectSum(self, VectA, VectB):
        ABSum = np.zeros(3)
        ABSum = self.Fillvect(VectA[0]+VectB[0], VectA[1]+VectB[1], VectA[2]+VectB[2])
        return ABSum



    '''
    单位化某个向量
    '''
    def UnitVect(self, InputVector):
        OutputVector = np.zeros(3)
        Abs = self.VectAbs(InputVector)
        if Abs < 0.000000000001:
            OutputVector = self.NullVect(3)
        else:
            for i in range(3):
                OutputVector[i] = InputVector[i] / float(Abs)
        return OutputVector



    '''
    将某个向量所有值置零，具体实现方法可以再商榷
    '''
    def NullVect(self, Dim):
        VectorToNull = np.zeros(Dim)
        for i in range(Dim):
            VectorToNull[i] = 0
        return VectorToNull

    '''
    向量乘一个数
    '''
    def MultiplicateWithScalar(self, VectorToMultiplicate, Scalar, Dim):
        OutputVector = np.zeros(3)
        for i in range(Dim):
            OutputVector[i] = Scalar * VectorToMultiplicate[i]
        return OutputVector

    '''
    对应文章中的斥力部分的计算公式
    v_rep = p_rep*(r_0_rep-r_ij)
     '''
    def SigmoidLin(self, x, p, v_max, r0):
        vel = (r0-x) * p
        if (p <= 0 or vel <= 0):
            return 0
        if (vel >= v_max):
            return v_max
        return vel
    
    '''
    引力刹车曲线
    '''
    def VelDecayLinSqrt(self,x, p, acc, v_max, r0):
        vel = (x - r0) * p
        if (acc <= 0) or (p <= 0) or (vel <= 0):
            return 0
        if vel < acc/p:
            if vel >= v_max:
                return v_max
            return vel
        
        vel = math.sqrt(2 * acc * (x-r0) - acc * acc / p / p)
        if vel >= v_max:
            return v_max
        return vel












