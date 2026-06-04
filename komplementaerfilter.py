import mpu6050
import math
import time


class KOMPFILTER:
    def __init__(self):
        self.alpha = 0.98
        self.roll = 0
        self.pitch = 0
        self.yaw = 0



    def kompfilter(self, gyrox, gyroy, gyroz, accx, accy, accz, dt):
        #Winkel aus acc
        roll_acc = math.degrees(math.atan2(accy, accz))
        pitch_acc = math.degrees(math.atan2(-accx, math.sqrt(accy**2 + accz**2)))

        #Umrechnung der Winkel vom Komplementärfilter in radiant
        roll_rad = math.radians(self.roll)
        pitch_rad = math.radians(self.pitch)

        #Eulerwinkel änderungsrate
        rolldt = (gyroy * math.sin(roll_rad) + gyroz * math.cos(roll_rad)) * math.tan(pitch_rad) + gyrox
        pitchdt = gyroy * math.cos(roll_rad) - gyroz * math.sin(roll_rad)
        yawdt = (gyroy * math.sin(roll_rad) + gyroz * math.cos(roll_rad))/math.cos(pitch_rad)

        #komplementärfilter
        self.roll = self.alpha * (self.roll + rolldt * dt) + (1- self.alpha) * roll_acc
        self.pitch = self.alpha * (self.pitch + pitchdt * dt) + (1- self.alpha) * pitch_acc
        self.yaw = (self.yaw + yawdt * dt) 


        return self.roll, self.pitch, self.yaw


if __name__ == '__main__':
    main()
