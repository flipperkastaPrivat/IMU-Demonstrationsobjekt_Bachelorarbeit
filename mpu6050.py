import smbus2
import time

class MPU6050:

    

    def __init__(self, adress = 0x68):
        self.adress = adress
        self.bus = smbus2.SMBus(1)                              #Bus 1 (Pin3 & 5)
        self.bus.write_byte_data(self.adress, 0x6b,0x01)        #register 0x6b wird geschrieben mit werten 0x01 für PPL mit X_Gyroskop, ist taktgeber für analog zu digital umwadel
        self.bus.write_byte_data(self.adress, 0x1b,0x00)        # setzt full sclale Range auf +-250°
        self.bus.write_byte_data(self.adress, 0x1c,0x00)        # setzt full sclale Range auf +-2g
        self.period = 0.01

    def get_gyrox(self):
        high= self.bus.read_byte_data(self.adress, 0x43)   # lesen der ertsen 8 bit
        low= self.bus.read_byte_data(self.adress, 0x44)    # lesen der zweiten 8 bit
        raw = (high<< 8) | low                   # kombinieren der 8 bit zu 16 bit resp 2 byte

        if raw > 32767:                                    # + 32767 und -32768, + 1 zeichen für vorzeichen
            raw -= 65536                                   # 2**16 = 65536

        gyro = raw / 131                              # 16bit = +- 32767 durch range 250° = 131 data points pro grad

        return gyro

    def get_gyroy(self):
        high= self.bus.read_byte_data(self.adress, 0x45)   # lesen der ertsen 8 bit
        low= self.bus.read_byte_data(self.adress, 0x46)    # lesen der zweiten 8 bit
        raw = (high<< 8) | low                   # kombinieren der 8 bit zu 16 bit resp 2 byte

        if raw > 32767:                                    # + 32767 und -32768, + 1 zeichen für vorzeichen
            raw -= 65536                                   # 2**16 = 65536

        gyro = raw / 131                              # 16bit = +- 32767 durch range 250° = 131 data points pro grad


        return gyro

    def get_gyroz(self):
        high= self.bus.read_byte_data(self.adress, 0x47)   # lesen der ertsen 8 bit
        low= self.bus.read_byte_data(self.adress, 0x48)    # lesen der zweiten 8 bit
        raw = (high<< 8) | low                   # kombinieren der 8 bit zu 16 bit resp 2 byte

        if raw > 32767:                                    # + 32767 und -32768, + 1 zeichen für vorzeichen
            raw -= 65536                                   # 2**16 = 65536

        gyro = raw / 131                              # 16bit = +- 32767 durch range 250° = 131 data points pro grad

        return gyro


    def get_accx(self):
        high= self.bus.read_byte_data(self.adress, 0x3b)   # lesen der ertsen 8 bit
        low= self.bus.read_byte_data(self.adress, 0x3c)    # lesen der zweiten 8 bit
        raw = (high<< 8) | low                   # kombinieren der 8 bit zu 16 bit resp 2 byte

        if raw > 32767:                                    # + 32767 und -32768, + 1 zeichen für vorzeichen
            raw -= 65536                                   # 2**16 = 65536

        acc = raw / 16384                              # 16bit = +- 32767 durch range 250° = 131 data points pro grad


        return acc

    def get_accy(self):
        high= self.bus.read_byte_data(self.adress, 0x3d)   # lesen der ertsen 8 bit
        low= self.bus.read_byte_data(self.adress, 0x3e)    # lesen der zweiten 8 bit
        raw = (high<< 8) | low                   # kombinieren der 8 bit zu 16 bit resp 2 byte

        if raw > 32767:                                    # + 32767 und -32768, + 1 zeichen für vorzeichen
            raw -= 65536                                   # 2**16 = 65536

        acc = raw / 16384                              # 16bit = +- 32767 durch range 250° = 131 data points pro grad


        return acc

    def get_accz(self):
        high= self.bus.read_byte_data(self.adress, 0x3f)   # lesen der ertsen 8 bit
        low= self.bus.read_byte_data(self.adress, 0x40)    # lesen der zweiten 8 bit
        raw = (high<< 8) | low                   # kombinieren der 8 bit zu 16 bit resp 2 byte

        if raw > 32767:                                    # + 32767 und -32768, + 1 zeichen für vorzeichen
            raw -= 65536                                   # 2**16 = 65536

        acc = raw / 16384                              # 16bit = +- 32767 durch range 250° = 131 data points pro grad


        return acc

    def offset(self, argument):
        sum = 0
        counts = 1000
        for i in range(counts):
            sum += argument()
            time.sleep(self.period/2000)
        offset = 1/counts * sum

        return offset
        

if __name__ == '__main__':
    main()