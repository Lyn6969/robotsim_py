import numpy as np 

from dynamics_utils import Phase
from math_utils import math_utils 
# PhaseData = [Phase(5, 5)]*10
# for i in range(3):
#     PhaseData[0].Coordinates[0][i] = i+1
#     PhaseData[0].Coordinates[1][i] = i+2

# print(PhaseData[0].Coordinates[0])

a = np.zeros((1,3))
b = np.array([4,5,6])
c = np.array([1,2,3])
for i in range(3):
    a[0][i] = i+1
print(a,b)

A = math_utils()
# A.UnitVect(a[0], a[0])

#print(A.VectAbs(b))

print(A.VectDifference(b,c))

print(A.MultiplicateWithScalar(b, 2, 3))

#print(A.UnitVect(b))


