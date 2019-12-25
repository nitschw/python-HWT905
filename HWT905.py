import serial
from serial import Serial

# /dev/tty.usbserial-14520

class HWT905():
    def __init__(self, port, baud):
        self.baud_rate = baud
        self.return_rate = 10
        self.x_accel_bias = 0
        self.y_accel_bias = 0
        self.z_accel_bias = 0
        self.x_angular_vel_bias = 0
        self.y_angular_vel_bias = 0
        self.z_angular_vel_bias = 0
        self.x_magnetic_bias = 0
        self.y_magnetic_bias = 0
        self.z_magnetic_bias = 0

        self.ser = Serial(port, baudrate=baud)

    def closeSerial(self):
        self.ser.close()

    def hexify(self, data):
        data_int = []
        print(data)
        for i in range(0, len(data), 2):
            data_int.append(int('0x'+data[i:i+2],16))
        print(data_int)
        return data_int

    def getReading(self):
        data = b'0'
        for i in range(4):
            while data.hex() != '55':
                data = self.ser.read(1) 
            data = self.ser.read(1)
            if data.hex() == '51':  # Acceleration output
                print('Got acceleration data')
                data = self.ser.read(9)
                data_int = self.hexify(data.hex())
                self.ax = (data_int[1] << 8 + data_int[0])/32768*16*9.81
                self.ay = (data_int[3] << 8 + data_int[2])/32768*16*9.81
                self.az = (data_int[5] << 8 + data_int[4])/32768*16*9.81
                self.acc_temperature = (data_int[7] << 8 + data_int[6])/100
            elif data.hex() == '52':  # Angular velocity output
                print('Got angular velocity data')
                data = self.ser.read(9)
                data_int = self.hexify(data.hex())
                self.wx = (data_int[1] << 8 + data_int[0])/32768*2000
                self.wy = (data_int[3] << 8 + data_int[2])/32768*2000
                self.wz = (data_int[5] << 8 + data_int[4])/32768*2000
                self.w_temperature = (data_int[7] << 8 + data_int[6])/100
            elif data.hex() == '53':  # Angle output
                print('Got angular data output')
                data = self.ser.read(9)
                data_int = self.hexify(data.hex())
                self.roll = (data_int[1] << 8 + data_int[0])/32768*180
                self.pitch = (data_int[3] << 8 + data_int[2])/32768*180
                self.yaw = (data_int[5] << 8 + data_int[6])/32768*180
                self.rpy_temperature = (data_int[7] << 8 + data_int[8])/100
            elif data.hex() == '54':  # Magnetometer output
                print('Got magnetometer data output')
                data = self.ser.read(9)
                data_int = self.hexify(data.hex())
                self.hx = data_int[1] << 8 + data_int[0]
                self.hy = data_int[3] << 8 + data_int[2]
                self.hz = data_int[5] << 8 + data_int[4]

    def setBaud(self, baud):
        if baud == 2400:
            baud_byte = 0x00
        elif baud == 4800:
            baud_byte = 0x01
        elif baud == 9600:
            baud_byte = 0x02
        elif baud == 19200:
            baud_byte = 0x03
        elif baud == 38400:
            baud_byte = 0x04
        elif baud == 57600:
            baud_byte = 0x05
        elif baud == 115200:
            baud_byte = 0x06
        elif baud == 230400:
            baud_byte = 0x07
        elif baud == 460800:
            baud_byte = 0x08
        elif baud == 921600:
            baud_byte = 0x09
        else:
            print('Selected baud rate is not suppported!',
                  ' No data written to device.\n',
                  'Please select from the following:\n',
                  '[2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]')
            return
        self.baud_rate = baud
        bytes = [0xFF, 0xAA, 0x04, baud_byte, 0x00]
        self.writeBytes(bytes)

    def setReturnRate(self, rate):
        if rate == 0.1:
            rate_byte = 0x01
        elif rate == 0.5:
            rate_byte = 0x02
        elif rate == 1:
            rate_byte = 0x03
        elif rate == 2:
            rate_byte = 0x04
        elif rate == 5:
            rate_byte = 0x05
        elif rate == 10:
            rate_byte = 0x06
        elif rate == 20:
            rate_byte = 0x07
        elif rate == 50:
            rate_byte = 0x08
        elif rate == 100:
            rate_byte = 0x09
        elif rate == 125:
            rate_byte = 0x0A
        elif rate == 200:
            rate_byte = 0x0B
        elif rate == -1:  #  'single' reading output (as per doc)
            rate_byte = 0x0C
        elif rate == 0:  #  'no output' (as per doc)
            rate_byte = 0x0D
        else:
            print('Return rate selected is not supported!',
                  ' No data written to device.\n',
                  'Please select from the following:\n',
                  '[0.1, 0.5, 1, 2, 5, 10, 20, 50, 100, 125, 200, single (use -1 as in input), 0]')
            return
        self.return_rate = rate
        bytes = [0xFF, 0xAA, 0x03, rate_byte, 0x00]
        self.writeBytes(bytes)

    def setInstallationDirection(self, direction):
        if direction.lower() == 'horizontal':
            dir_byte = 0x00
        elif direction.lower() == 'vertical':
            dir_byte = 0x01
        else:
            print('Provided direction not supported. Select ''horizontal'' or ''vertical''')
            return
        self.install_direction = direction
        bytes = [0xFF, 0xAA, 0x23, dir_byte, 0x00]
        self.writeBytes(bytes)

    def selectAlgorithm(self, alg):
        if alg == 6:
            alg_byte = 0x00
        elif alg == 9:
            alg_byte = 0x01
        else:
            print('Option provided not supported. Select 6-axis (input: 6) or',
                  ' 9-axis (input: 9) mode')
            return
        self.algorithm = alg
        bytes = [0xFF, 0xAA, 0x24, alg_byte, 0x00]        

    def resetDefaultConfig(self):
        self.writeBytes([0xFF, 0xAA, 0x00, 0x01, 0x00])

    def toggleSleep(self):
        self.writeBytes([0xFF, 0xAA, 0x22, 0x01, 0x00])

    

    #def calibrate():

    def writeBytes(self, bytes):
        self.ser.write(serial.to_bytes(bytes))
