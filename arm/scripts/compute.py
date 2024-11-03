#!/usr/bin/env python3
import numpy as np
import os
import rospy

class IK_4dof:
    def __init__(self, a1, a2, a3, a4):
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4

        self.t1 = (0 / 180) * np.pi
        self.t2 = (0 / 180) * np.pi
        self.t3 = (0 / 180) * np.pi
        self.t4 = (0 / 180) * np.pi

        self.r90 = (90 / 180) * np.pi
        self.r180 = (180 / 180) * np.pi

    def transform(self, c, d):
        end_eff = np.array([[c[0]], [c[1]], [c[2]]]) # (x,y,z) of end-effector where each value is a row
        r0_6 = np.array([[1,0,0],
                         [0,-1,0],
                         [0,0,-1]])
        d = np.array([[0], [0], [d]])
        wcp = end_eff - np.dot(r0_6, d)
        coords = [wcp[0][0], wcp[1][0], wcp[2][0]]
        return coords

    def cosine(self, r, a2, a3):
        cos_phi = (((r**2) + (a2**2) - (a3**2)) / (2*a2*r))
        sin_phi = np.sqrt(1 - np.square(cos_phi))
        phi = np.arctan2(sin_phi, cos_phi)
        return phi

    def convert(self, angles, to):
        result = []
        if to=="to_def":
            for a_in_rad in angles:
                a_in_deg = (a_in_rad * 180) / np.pi
                result.append(a_in_deg)
        else:
            for a_in_deg in angles:
                a_in_rad = (a_in_deg / 180) * np.pi
                result.append(a_in_rad)
        return result
    
    # def rth(self):
    #     x = Floats()
    #     x.data.append(90.0)
    #     x.data.append(90.0)
    #     x.data.append(90.0)
    #     x.data.append(90.0)
    #     x.data.append(0.0)
    #     return x

    def __call__(self, location):
        d = self.a4
        (x,y,z)  = location
        coordinates = (x,y,z)
        xt, yt, zt = self.transform(coordinates, d)
        # print(xt, yt, zt)
        self.t1 = np.arctan2(yt, xt)
        self.t1 = np.abs(self.t1)
        theta1 = (180/np.pi) * self.t1
        # print((180/np.pi) * self.t1)
        r1_sq = np.square(xt) + np.square(yt)
        r1 = np.sqrt(r1_sq)
        r2 = zt-self.a1
        r2_sq = np.square(r2)
        r_sq = r1_sq + r2_sq
        r = np.sqrt(r_sq)
        phi1 = np.arctan2(r2, r1)
        phi2 = self.cosine(r, self.a2, self.a3)
        self.t2 = np.abs(phi1 + phi2)
        # print((180/np.pi) * self.t2)
        theta2 = (180/np.pi) * self.t2
        phi3 = self.cosine(self.a3, self.a2, r)
        # self.t3 = np.abs(phi3 - self.r180)
        self.t3 = np.pi - phi3
        # print(((180/np.pi) * self.t3) + 6)
        theta3 = ((180/np.pi) * self.t3) + 6
        self.t4 = self.r90 + self.t3 - self.t2
        theta4 = (180/np.pi) * self.t4
        # print((180/np.pi) * self.t4)
        # angles_in_deg = self.convert((self.t1, self.t2, self.t3, self.t4), "to_rad")
        # print(angles_in_deg)
        # x = Floats()
        # x.data.append(angles_in_deg[0])
        # x.data.append(angles_in_deg[1])
        # x.data.append(angles_in_deg[2])
        # x.data.append(angles_in_deg[3])
        # if open:
        #     x.data.append(1.0)
        # else:
        #     x.data.append(0.0)
        # return x
        return (theta1, theta2, theta3, theta4)