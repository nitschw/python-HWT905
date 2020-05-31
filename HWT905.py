import serial as s

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
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.wx = 0
        self.wy = 0
        self.wz = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.hx = 0
        self.hy = 0
        self.hz = 0
        self.time_year = 0
        self.time_month = 0
        self.time_day = 0
        self.time_hour = 0
        self.time_second = 0
        self.time_milli = 0
        self.acc_temperature = 0
        self.ser = s.Serial(port, baudrate=baud)

    def closeSerial(self):
        self.ser.close()

    def getReading(self):
        self.ser.flush()
        data = b'0'
        for i in range(6):
            while data.hex() != '55':
                data = self.ser.read(1) 
            data = self.ser.read(1)
            if data.hex() == '50':  # Time output
                print('Got time data')
                data = self.ser.read(9)
                self.time_year = 2000 + data[0]
                self.time_month = data[1]
                self.time_day = data[2]
                self.time_hour = data[3]
                self.time_minute = data[4]
                self.time_milli = ((data[6] << 8) | data[5])
            elif data.hex() == '51':  # Acceleration output
                print('Got acceleration data')
                data = self.ser.read(9)
                self.ax = int.from_bytes(((data[1] << 8) | data[0]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*16*9.81
                self.ay = int.from_bytes(((data[3] << 8) | data[2]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*16*9.81
                self.az = int.from_bytes(((data[5] << 8) | data[4]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*16*9.81
                self.acc_temperature = ((data[7] << 8) + data[6])/100
            elif data.hex() == '52':  # Angular velocity output
                print('Got angular velocity data')
                data = self.ser.read(9)
                self.wx = int.from_bytes(((data[1] << 8) | data[0]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*2000
                self.wy = int.from_bytes(((data[3] << 8) | data[2]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*2000
                self.wz = int.from_bytes(((data[5] << 8) | data[4]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*2000
                self.w_temperature = ((data[7] << 8) | data[6])/100
            elif data.hex() == '53':  # Angle output
                print('Got angular data output')
                data = self.ser.read(9)
                data_int = self.unhexify(data.hex())
                self.roll = int.from_bytes(((data[1] << 8) | data[0]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*180
                self.pitch = int.from_bytes(((data[3] << 8) | data[2]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*180
                self.yaw = int.from_bytes(((data[6] << 8) | data[5]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768*180
                self.rpy_temperature = ((data[7] << 8) | data[8])/100
            elif data.hex() == '54':  # Magnetometer output
                print('Got magnetometer data output')
                data = self.ser.read(9)
                self.hx = int.from_bytes(((data[1] << 8) | data[0]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768
                self.hy = int.from_bytes(((data[3] << 8) | data[2]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768
                self.hz = int.from_bytes(((data[5] << 8) | data[4]).to_bytes(2, byteorder='big'), byteorder='big', signed=True)/32768
        print('time_year: ', self.time_year)
        print('ax: ', self.ax)
        print('ay: ', self.ay)
        print('az: ', self.az)
        print('accel_temp: ', self.acc_temperature)
        print('wx: ', self.wx)
        print('wy: ', self.wy)
        print('wz: ', self.wz)
        print('roll: ', self.roll)
        print('pitch: ', self.pitch)
        print('yaw: ', self.yaw)
        print('hx: ', self.hx)
        print('hy: ', self.hy)
        print('hz: ', self.hz)

    def unhexify(self, data):
        data_int = []
        for i in range(0, len(data), 2):
            data_int.append(int('0x'+data[i:i+2],16))
        return data_int

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
        """
        make a proper docstring
        """
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

    def change_char(self, s, p, r):
        return s[:p]+r+s[p+1:]

    def setReturnContent(self, content_dict=None):
        """
        The setReturnContent function takes a dict "content_dict" and uses it to
        set the values the HWT905 device will be spamming in it's serial output.
        Set the fields in the dict to true or false and they will or will not be
        included in your data.
        
        content_dict: this variable should be a list with a boolean 
                      entry for each possible content type that can be
                      contained in the output message
                      
                      From the manual, here are the content types:
                      0x50 pack: time    (doesn't seem to work)
                      0x51 pack: acceleration
                      0x52 pack: angular velocity
                      0x53 pack: angle
                      0x54 pack: magnetometer
                      0x55 pack: port status    (doesn't seem to work)
                      0x56 pack: atmospheric pressure and height (doesn't seem to work)
                      0x57 pack: lat/lon output (I doubt this is supported)
                      0x58 pack: GPS speed (or this? how does it use GPS wtf?)
                      0x59 pack: quaternion
                      0x5A pack: satellite position accuracy (is this real? what satellites is this thing using)
                      
                      "content_dict" should be a dict with the fields above, as shown below:
                      
                      content_dict = {'time', True,
                                      'acceleration', True,
                                      'angular_vel, True,
                                      'angle', True,
                                      'magnetometer', True,
                                      'port_status', True,
                                      'atmo', True,
                                      'lat_lon', True,
                                      'gps_speed', True,
                                      'quaternion', True,
                                      'sat_pos_acc', True}
        """
        if content_dict is None:
            content_dict = {'time': True,
                            'acceleration': True,
                            'angular_vel': True,
                            'angle': True,
                            'magnetometer': True,
                            'port_status': True,
                            'atmo': True,
                            'lat_lon': True,
                            'gps_speed': True,
                            'quaternion': True,
                            'sat_pos_acc': True}

        class content:
            def __init__(self, var, idx, val):
                self.variable = var
                self.idx = idx
                self.val = val

        # this makes sense if you refer to the manual's "set return content" section 1.2.8
        time = content('RSWL', 7, content_dict['time'])
        acceleration = content('RSWL', 6, content_dict['acceleration'])
        angular_vel = content('RSWL', 5, content_dict['angular_vel'])
        angle = content('RSWL', 4, content_dict['angle'])
        magnetometer = content('RSWL', 3, content_dict['magnetometer'])
        port_status = content('RSWL', 2, content_dict['port_status'])
        atmo = content('RSWL', 1, content_dict['atmo'])
        lat_lon = content('RSWL', 0, content_dict['lat_lon'])
        gps_speed = content('RSWH', 7, content_dict['gps_speed'])
        quaternion = content('RSWH', 6, content_dict['quaternion'])
        sat_pos_acc = content('RSWH', 5, content_dict['sat_pos_acc'])
                
        RSWL = list('00000000')
        RSWH = list('00000000')
        for stat in content_dict.keys():
            val = str(int(vars()[stat].val))
            idx = int(vars()[stat].idx)
            vars()[vars()[stat].variable][idx] = val
        RSWL = int("".join(RSWL), 2)
        RSWH = int("".join(RSWH), 2)

        bytes = [0xFF, 0xAA, 0x02, RSWL, RSWH]
        self.writeBytes(bytes)

    def resetDefaultConfig(self):
        self.writeBytes([0xFF, 0xAA, 0x00, 0x01, 0x00])

    def toggleSleep(self):
        self.writeBytes([0xFF, 0xAA, 0x22, 0x01, 0x00])

    #def calibrate():

    def writeBytes(self, bytes):
        self.ser.write(s.to_bytes(bytes))