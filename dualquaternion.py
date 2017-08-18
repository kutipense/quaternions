from quaternion import Quaternion

import numpy as np

class DualQuaternion():

    def __init__(self,quaternionR,quaternionD):
        self.real = quaternionR
        self.dual = quaternionD


    def __repr__(self):
        return "%s object: \n  Real: %s\n  Dual: %s" %(self.__class__.__name__, str(self.real), str(self.dual))

    def __str__(self):
        return "%s, %s" %(str(self.real), str(self.dual))

    def __mul__(self,other):
        real = dual = None
        if isinstance(other,DualQuaternion):
            real = self.real * other.real
            dual = self.real * other.dual + self.dual * other.real
        elif isinstance(other,(int,float,np.float32)):
            real = self.real * other
            dual = self.dual * other

        return DualQuaternion(real,dual)

    def __invert__(self):
        return self.conjugate

    def __add__(self,other):
        real = self.real + other.real
        dual = self.dual + other.dual

        return DualQuaternion(real,dual)

    def rotateit(self,other):
        return self*other*self.conjugate

    @property
    def conjugate(self):
        return DualQuaternion(self.real.conjugate, self.dual.conjugate * -1)

    @property
    def inverse(self):
        real = self.real.conjugate
        dual = self.dual*self.real.conjugate * -1
        return DualQuaternion(real,dual)

    @property
    def magnitude(self):
        return self*self.conjugate

    @classmethod
    def from_quaternions(cls,quaternionR,quaternionD):
        real, dual = quaternionR.normalized, quaternionD

        return DualQuaternion(real,dual)

    @classmethod
    def from_quaternion_vector(cls,quaternionR,vector3d):
        real = quaternionR.normalized

        vector3d = np.asarray(vector3d,dtype=np.float32)
        vector4d = np.append(vector3d,0)

        dual = real * Quaternion(vector4d) * 0.5

        return DualQuaternion(real,dual)

    @classmethod
    def from_translation(cls,vector3d):
        return DualQuaternion.from_quaternion_vector(Quaternion([0,0,0,1]),vector3d)

    @classmethod
    def from_rotation(cls,quaternionR):
        return DualQuaternion.from_quaternion_vector(quaternionR,(0,0,0))

    @classmethod
    def to_quaternion(cls,q1,q2):
        (q1.r+q2d) * ()
        return DualQuaternion.from_quaternion_vector(quaternionR,(0,0,0))

    @classmethod
    def I(cls):
        return DualQuaternion.from_translation([0,0,0])