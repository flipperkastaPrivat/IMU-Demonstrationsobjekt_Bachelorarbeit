import smbus2
import time

class BNO055:
    
    def __init__(self, adress):                          # mit i2cdetect -y 1 überprüft
        self.adress = adress
        self.bus = smbus2.SMBus(1)                              #Bus 1 (Pin3 & 5)
        self.bus.write_byte_data(self.adress, 0x3F,0x00)        #register 0x3F alle bits spez bit 7 auf 0 sezten für interne osxillator als clk_selected
        self.bus.write_byte_data(self.adress, 0x3D,0x08)        #betreiben des sensor im IMU modus , ohne magnetometer
        self.sleep_time = 0.1


    def geteuler(self, register_h, register_l):
        high = self.bus.read_byte_data(self.adress, register_h)
        low = self.bus.read_byte_data(self.adress, register_l)
        raw = (high<< 8) | low                                   # kombinieren der 8 bit zu 16 bit resp 2 byte

        if raw > 32767:                                    # + 32767 und -32768, + 1 zeichen für vorzeichen
            raw -= 65536                                   # 2**16 = 65536

        euler = raw / 16                              # nach 3.6.5.4 kapitel, 16 lsb 
        return euler


    def getquaternion(self, register_h, register_l):
        high = self.bus.read_byte_data(self.adress, register_h)
        low = self.bus.read_byte_data(self.adress, register_l)
        raw = (high<< 8) | low                                   # kombinieren der 8 bit zu 16 bit resp 2 byte

        if raw > 32767:                                    # + 32767 und -32768, + 1 zeichen für vorzeichen
            raw -= 65536                                   # 2**16 = 65536

        quaternion = raw / (2**14)                              # nach 3.6.5.4 kapitel, 16 lsb 
        return quaternion

