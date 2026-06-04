import scipy
import mpu6050
import math
import numpy as np
import time


class KALMANFILTER:
    def __init__(self,gyrox,gyroy,gyroz,accx, accy,accz):

        self.xk = np.array([[1],[0],[0],[0]])                                                            #Zustandsvektor
        #self.u = np.array([math.radians(gyrox), math.radians(gyroy), math.radians(gyroz)])      #Eingangsvektor
        self.Pk = np.eye(4)                                                                      #Startwert KovarianzMatrix einheitsmatrix  4x4
        sensor = mpu6050.MPU6050(adress=0x68)
        self.periode = sensor.period


    def prediction(self,gyrox,gyroy,gyroz,dt):
        # x werte von korrektur fpr prädiktion
        a = self.xk[0,0] # zeile 1 spalte 1
        b = self.xk[1,0]
        c = self.xk[2,0]
        d = self.xk[3,0]

        u = np.array(   [[0,-gyrox, -gyroy, -gyroz],
                            [gyrox,0, gyroz, -gyroy],
                            [gyroy, -gyroz, 0, gyrox,],
                            [gyroz, gyroy, -gyrox, 0]])


        Aj = np.array([
            [1.0,           -0.5 * dt * gyrox, -0.5 * dt * gyroy, -0.5 * dt * gyroz],
            [0.5 * dt * gyrox, 1.0,             0.5 * dt * gyroz, -0.5 * dt * gyroy],
            [0.5 * dt * gyroy, -0.5 * dt * gyroz, 1.0,            0.5 * dt * gyrox],
            [0.5 * dt * gyroz, 0.5 * dt * gyroy, -0.5 * dt * gyrox, 1.0]
            ])


        nsd = 0.05                          #°/s-rms
        sigma_sqr = (nsd/360 * 2 * np.pi )**2

        Q = np.eye(3) * sigma_sqr * 100


        G = np.array([                #herleitung siehe TAblet
            [-b,  -c, -d],
            [ a,  -d, c],
            [ d, a, -b],
            [ -c, b, a]
            ]) * 0.5 * dt



        self.x = self.xk + 0.5 * (u @ self.xk) * dt                         #Bestimmung des neuen quaternions
        self.x = self.x / np.sqrt(self.x[0,0]**2 + self.x[1,0]**2 + self.x[2,0]**2 + self.x[3,0]**2)
        self.P = Aj @ self.Pk @ np.transpose(Aj) + G @ Q @ np.transpose(G)

        return self.x, self.P

    def correction(self,accx, accy,accz):
        # x werte von prädikation für korrektur
        a = self.x[0,0]
        b = self.x[1,0]
        c = self.x[2,0]
        d = self.x[3,0]
        

        h = np.array([
            [2 * (b*d - a*c)],
            [2 * (c*d + a*b)],
            [a**2 - b**2 - c**2 + d**2]
        ])

        Cj = np.array([
            [-2*c,  2*d, -2*a,  2*b],
            [ 2*b,  2*a,  2*d,  2*c],
            [ 2*a, -2*b, -2*c,  2*d]
            ])

        y = np.array([[accx], [accy],[accz]])     #messwerte
        y = y / np.sqrt(y[0,0]**2 + y[1,0]**2 + y[2,0]**2)  

        sigma_sqr = (400 * 10**(-6))**2 * (1 / self.periode)
        R = np.eye(3) * sigma_sqr

        K = self.P @ np.transpose(Cj) @ scipy.linalg.inv(Cj @ self.P @ np.transpose(Cj) + R)
        self.xk = self.x + K @ (y - h)
        self.xk = self.xk / np.sqrt(self.xk[0,0]**2 + self.xk[1,0]**2 + self.xk[2,0]**2 + self.xk[3,0]**2)
        self.Pk = (np.eye(4) - K @ Cj) @ self.P


        return self.xk, self.Pk

    def geteuler(self):
        a = self.xk[0,0] # zeile 1 spalte 1
        b = self.xk[1,0]
        c = self.xk[2,0]
        d = self.xk[3,0]

        phi = math.degrees(math.atan2(2*a*b + 2*c*d, a**2 - b**2 - c**2 + d**2))

        theta_value = 2*a*c - 2*b*d
        theta_value = max(-1.0, min(1, theta_value))
        theta = math.degrees(math.asin(theta_value))


        psi = math.degrees(math.atan2(-2*a*d + 2*b*c, a**2 + b**2 - c**2 - d**2))

        self.euler = np.array([[phi],[theta],[psi]])

        return self.euler

    def getoffset(self, value):
        sum = 0
        counts = 1000
        for i in range(counts):
            sum += value
            time.sleep(self.periode/200)
        offset = 1/counts * sum

        return offset






