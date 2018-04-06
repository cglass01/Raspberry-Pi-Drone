import smbus
import time
import colorama
from colorama import Fore, Back, Style
import math
import datetime

class mpu6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1):
        # DLPF REGISTER
        DLPF_SET_ADDR = 0x1A
        DLPF = 0x06
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.5)
        self.bus.write_byte_data(self.address, DLPF_SET_ADDR, DLPF)


    # I2C communication methods

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.
        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    # MPU-6050 Methods

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.
        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.
        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.
        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False):
        """Gets and returns the X, Y and Z values from the accelerometer.
        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        #x = x / accel_scale_modifier
        #y = y / accel_scale_modifier
        #z = z / accel_scale_modifier
        #Euler formula
        # accelangleX = math.atan(y/ math.sqrt(math.pow(x, 2) + math.pow(z, 2))) * 180/3.141592654
        # accelangleY = math.atan(-1*(x/ math.sqrt(math.pow(y, 2) + math.pow(z, 2)))) * 180/3.141592654

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.
        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to.
        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.
        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        #angle_pitch += x * 0.0000611
        #angle roll += y * 0.0000611
        #
        #Apparently this degrees per second

        #x = x / gyro_scale_modifier
        #y = y / gyro_scale_modifier
        #z = z / gyro_scale_modifier


        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel_data = self.get_accel_data()
        gyro_data = self.get_gyro_data()

        return [accel_data, gyro_data, temp]

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c


def read_byte(reg):
    return bus.read_byte_data(address, reg)


def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg + 1)
    value = (h << 8) + l
    return value


def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val


def dist(a, b):
    return math.sqrt((a * a) + (b * b))


def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)


def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)


bus = smbus.SMBus(1)  # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68  # via i2cdetect

# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)

if __name__ == "__main__":
    RAD_TO_DEG = 57.29578
    M_PI = 3.14159265358979323846
    G_GAIN = 0.070
    gyroXangle = 0
    mpu = mpu6050(0x68)
    mpu.set_accel_range(mpu.ACCEL_RANGE_8G)
    mpu.set_gyro_range(mpu.GYRO_RANGE_500DEG)
    print(mpu.get_temp())
    print("The address of the mpu is :", mpu.address)
    count = 0
    colorama.init()
    text = "start"
    a = datetime.datetime.now()
    totalAngleX= 0
    totalAngleY = 0
    gyrox = 0
    gyroy = 0
    gyroz = 0
    gyro_roll_input= 0
    gyro_pitch_input= 0
    gyro_yaw_input= 0
    angle_pitch= 0
    angle_roll= 0
    gyro_pitch= 0
    gyro_roll= 0
    gyro_yaw= 0
    acc_total_vector= 0
    acc_x= 0
    acc_y= 0
    acc_z= 0
    angle_roll_acc = 0
    angle_pitch_acc = 0
    pid_roll_setpoint = 0
    pid_pitch_setpoint = 0
    pid_yaw_setpoint = 0
    pid_i_gain_roll = 0
    pid_max_roll = 400
    pid_p_gain_roll = 1.3
    pid_i_gain_roll = 0.04
    pid_d_gain_roll = 18.0
    pid_max_roll = 400
    pid_p_gain_pitch = 1.3
    pid_i_gain_pitch = 0.04
    pid_d_gain_pitch = 18.0
    pid_max_pitch = 400
    pid_p_gain_yaw = 4.0
    pid_i_gain_yaw = 0.02
    pid_d_gain_yaw = 0.0
    pid_max_yaw = 400
    gyrox_cal = 0
    gyroy_cal = 0
    gyroz_cal = 0
    count = 0
    gryox = 0
    pitch_angle = 0
    roll_angle = 0
    print("Calibration....")
    while count < 2000:
        gyroy_cal += read_word_2c(0x43)
        gyrox_cal += read_word_2c(0x45)
        gyroz_cal += read_word_2c(0x47)
        time.sleep(0.004)
        count += 1

    gyrox_cal /= 2000
    gyroy_cal /= 2000
    gyroz_cal /= 2000
    print(gyrox_cal)
    print(gyroy_cal)
    print(gyroz_cal)
    print("Calibrations complete")
    time.sleep(5)

    while True:
        gyroy_raw = read_word_2c(0x43)
        gyroy_raw -= - gyroy_cal - 350 #to ensure the reading is accurate after testing I found that value
        gyrox_raw = read_word_2c(0x45)
        gyrox_raw -= gyrox_cal
        gyroz_raw = read_word_2c(0x47)
        gyroz_raw -= gyroz_cal
        roll_angle += gyrox_raw * 0.000611
        pitch_angle += gyroy_raw * 0.000611
        print(Fore.GREEN)
        print"gryo X: ", ("%5d" % gyrox_raw), " degrees: ", (gyrox_raw / 131)
        print"gryo Y: ", ("%5d" % gyroy_raw), " degrees: ", (gyroy_raw / 131)
        print"gyro Z: ", ("%5d" % gyroz_raw), " degrees: ", (gyroz_raw / 131)
       # print"gryo X: ", gryox
        gyro_roll_input = (gyro_roll_input * 0.7) + ((gyrox_raw/ 131) * 0.3)
        gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyroy_raw / 131) * 0.3)
        #print(gyro_roll_input)
        #print(float(gyro_pitch_input))

        accely_raw = read_word_2c(0x3b)
        accelx_raw = read_word_2c(0x3d)
        accelz_raw = read_word_2c(0x3f)

        accelx = accelx_raw / 16384.0
        accely = accely_raw / 16384.0
        accelz = accelz_raw / 16384.0
        accel_pitch = math.atan((-accelx)/ math.sqrt((math.pow(accely, 2) + math.pow(accelz, 2)))) * (180/3.141592654)
        accel_roll = math.atan(accely/ math.sqrt((math.pow(accelx, 2) + math.pow(accelz, 2)))) * (180/3.141592654)
        accel_pitch -= 2.1
        accel_roll -= 1.3
        print(Fore.CYAN)

        c_pitch = math.cos(accel_pitch)
        t_pitch = math.tan(accel_pitch)
        c_roll = math.cos(accel_roll)
        s_roll = math.sin(accel_roll)

        euroll = gyrox_raw + gyroy_raw * s_roll * t_pitch + gyroz_raw * c_roll * t_pitch



        #print"accely_raw: ", ("%6d" % accely_raw), " degrees: ", accel_pitch
        #print"accelx_raw: ", ("%6d" % accelx_raw), " degrees: ", accel_roll

        pitch = pitch_angle * 0.98 + accel_pitch * 0.02
        roll = roll_angle * 0.98 + accel_pitch * 0.02
       # print("Pitch: ", int(pitch))
        #print("Roll: ", int(euroll / 4))




        


    print("Calibrating....")


    while count < 2000:
        gyro = mpu.get_gyro_data()
        acc = mpu.get_accel_data()
        gyrox += gyro['x']
        gyroy += gyro['y']
        gyroz += gyro['z']
        acc_y += acc['y']
        acc_z += acc['z']
        acc_x += acc['x']
        time.sleep(0.004)
        count += 1


    acc_x_cal = acc_x / 2000
    acc_y_cal = acc_y / 2000
    acc_z_cal = acc_z / 2000
    gyrox_cal = gyrox / 2000
    gyroy_cal = gyroy / 2000
    gyroz_cal = gyroz / 2000
    print("Calibrations complete")
    print(gyrox_cal)
    print(gyroy_cal)
    print(gyroz_cal)
    talkcount = 0

    while True:
        gyro = mpu.get_gyro_data()
        acc = mpu.get_accel_data()
        acc_y = acc['y']
        acc_z = acc['z']
        acc_x = acc['x']
        gyrox = gyro['x']
        gyroy = gyro['y']
        gyroz = gyro['z']

        gyrox = gyrox - gyrox_cal
        gyroy = gyroy - gyroy_cal
        gyroz = gyroz - gyroz_cal

        #degrees in seconds angles_in_degrees = rawX /65.5

        gyro_pitch += gyrox * 0.0000611
        gyro_roll += gyroy * 0.0000611
        #gyro_pitch -= gyro_roll * math.sin(gyroz * 0.000001066)
        #gyro_roll += gyro_pitch * math.sin(gyroz * 0.000001066)
        acc_total_vector = math.sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z))
        angle_pitch_acc = math.asin(float(acc_y / acc_total_vector)) * 57.296
        angle_roll_acc = math.asin(float(acc_x / acc_total_vector)) * -57.296
        angle_pitch = gyro_pitch * 0.9996 + angle_pitch_acc * 0.0004
        angle_roll = gyro_roll * 0.9996 + angle_roll_acc * 0.0004

        talkcount += 1

        if (talkcount % 200) == 0:
            print("GX: ", gyro_roll)
            print("GY: ", gyro_pitch)
            print("AX: ", angle_roll_acc)
            print("AY: ", angle_pitch_acc)
            print("Angle pitch: ", angle_pitch)
            print("Angle roll: ", angle_roll)










































