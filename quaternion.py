import numpy as np

class Quaternion():

    def __init__(self,vector4d):
        """
            :params:
                self.vector: [xi + yj + zk + w] formatted 4D Quaternion vector
        """
        self.vector = np.asarray(vector4d, dtype=np.float32)

    def __repr__(self):
        return "%s object: " %self.__class__.__name__ + str(self.vector.round(4))

    def __str__(self):
        return str(self.vector.round(4))

    def __mul__(self,other):
        """
            :params:
                Pv: 3D Vector part of P quaternion
                Pv: Scalar part of P quaternion
                Qv: 3D Vector part of Q quaternion
                Qv: Scalar part of Q quaternion

            multiplication: https://en.wikipedia.org/wiki/Quaternion#Quaternions_and_the_geometry_of_R3

            Quaternion pattern: [Ux, Uy, Uz, s] -> first 3 values are presentation \
                                                   of a vector, last value is a scalar.
        """
        PQ = None
        if isinstance(other, Quaternion):
            Pv = self.vector[:3]
            Ps = self.vector[-1]
            Qv = other.vector[:3]
            Qs = other.vector[-1]

            PQv = (Ps*Qv + Pv*Qs + np.cross(Pv,Qv))
            PQs = (Ps*Qs - np.dot(Pv,Qv))
            PQ = np.append(PQv,PQs)
        elif isinstance(other,(int,float,np.float32)):
            PQ = self.vector * other
        else:
            assert TypeError

        return Quaternion(PQ)

    def __add__(self,other):
        ret = None
        if isinstance(other, Quaternion):
            ret = Quaternion(np.add(self.vector,other.vector))
        else:
            q = np.asarray(other,dtype=np.float32)
            if not (4>=len(q)>=3):
                assert ValueError
            elif len(q)==3:
                q = np.append(q,0)

            ret = Quaternion(np.add(self.vector,q))

        return ret

    def __invert__(self):
        P = self.conjugate
        return P.normalized

    @property
    def normalized(self):
        vector = self.vector / self.magnitude
        return Quaternion(vector)

    def rotateit(self,vector3d):
        vector3d = np.asarray(vector3d,dtype=np.float32)

        if not (4>=len(vector3d)>=3):
            assert ValueError
        elif len(vector3d)==3:
            vector3d = np.append(vector3d,0)

        q = Quaternion(vector3d)
        p = self.normalized
        return (p*q*~p).vector[:3]
    
    @property
    def conjugate(self):
        return Quaternion(self.vector * np.array([-1,-1,-1,1]))

    @property
    def inverse(self):
        return ~self

    @property
    def magnitude(self):
        return (self*self.conjugate).vector[-1]

    @classmethod
    def from_axis_angle(cls,vector3d,angle):
        """
            :params:
                vector3d: 3d point [x,y,z]
                angle: radian
        """
        vector3d = np.asarray(vector3d,dtype=np.float32) * np.sin(angle/2.)
        vector4d = np.append(vector3d, np.cos(angle/2.))

        return Quaternion(vector4d)

    @classmethod
    def from_euler_angle(cls,euler_angles):
        """
            WORK IN PROGRESS!!!
            :params:
                euler_angles: [roll, pitch, yaw] XYZ
        """
        euler_angles = np.asarray(euler_angles,dtype=np.float32)
        r_s,p_s,y_s = np.sin(euler_angles)
        r_c,p_c,y_c = np.cos(euler_angles)

        w = r_c*p_c*y_c - r_s*p_s*y_s
        x = r_s*p_c*y_c + r_c*p_s*y_s
        y = r_c*p_s*y_c - r_s*p_c*y_s
        z = r_c*p_c*y_s + r_s*p_s*y_c

        vector4d = [x,y,z,w]

        return Quaternion(vector4d)
