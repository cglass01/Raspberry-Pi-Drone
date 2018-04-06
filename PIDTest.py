import smbus
import time
import colorama
from colorama import Fore, Back, Style
import math
import os
import datetime
from multiprocessing import Process, Value
os.system("sudo pigpiod")
time.sleep(2)
import pigpio
import RPi.GPIO as GPIO


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

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier
        #Euler formula
        accelangleX = math.atan(y/ math.sqrt(math.pow(x, 2) + math.pow(z, 2))) * 180/3.141592654
        accelangleY = math.atan(-1*(x/ math.sqrt(math.pow(y, 2) + math.pow(z, 2)))) * 180/3.141592654

        if g is True:
            return {'x': accelangleX, 'y': accelangleY, 'z': z}
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

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier


        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return [accel, gyro, temp]

pi = pigpio.pi()
totalAngleX =  Value('f', 0)
totalAngleY =  Value('f', 0)
accelAngleX =  Value('f', 0)
accelAngleY =  Value('f', 0)
elapsedTime =  Value('f', 0)
sysExit =  Value('f', 1)
desiredAngle = 0
kp = 6.55 #3.55
ki = 3.006 #0.006
kd = 2.05
pid_p = 0
pid_i = 0
pid_d = 0
error = 0
previousError = 0
pid = 0
throttle = 1100
G_GAIN = 0.070
gyroXangle = 0

#Change whichever we are testing to leftProp and rightProp
leftProp=19
#leftProp=27
rightProp=20
#rightProp=7

#def getAngles():
#    mpu = mpu6050(0x68)
#    i = 0
#    while sysExit.value:
#        if i == 0:
#            a = datetime.datetime.now()
#            i+=1
#        global totalAngleX
#        gyro = mpu.get_gyro_data()
#        accel = mpu.get_accel_data()
#        b = datetime.datetime.now() - a
#        a = datetime.datetime.now()
#        elapsedTime.value = b.microseconds / (1000000 * 1.0)
#        accelAngleX.value = accel['x']
#        accelAngleY.value = accel['y']
#        totalAngleX.value = 0.98 * (totalAngleX.value + gyro['x'] * elapsedTime.value) + 0.02 * accel['x']
#        totalAngleY.value = 0.98 * (totalAngleY.value + gyro['x'] * elapsedTime.value) + 0.02 * accel['y']


if __name__ == "__main__":

#    a = Process(target=getAngles, args=())
#    a.start()
    pi.set_servo_pulsewidth(leftProp, 1000)
    pi.set_servo_pulsewidth(rightProp, 1000)
    time.sleep(1)
    pi.set_servo_pulsewidth(leftProp, 1065)
    pi.set_servo_pulsewidth(rightProp, 1065)
    time.sleep(1)


try:
    mpu = mpu6050(0x68)
    i = 0
    while sysExit.value:
        if i == 0:
            c = datetime.datetime.now()
            a = time.time()
            i+=1
        gyro = mpu.get_gyro_data()
        accel = mpu.get_accel_data()
        b = time.time() - a
        a = time.time()
        elapsedTime.value = b
        d = datetime.datetime.now() - c
        d = datetime.datetime.now()
        LP = b.microseconds / (1000000 * 1.0)
        accelAngleX.value = accel['x']
        accelAngleY.value = accel['y']
        gyro_rate_x = gyro['x'] * G_GAIN
        gyroXangle += gyro_rate_x * d
        totalAngleX.value = 0.98 * (totalAngleX.value + gyro['x'] * elapsedTime.value) + 0.02 * accel['x']
        totalAngleY.value = 0.98 * (totalAngleY.value + gyro['x'] * elapsedTime.value) + 0.02 * accel['y']
        print(totalAngleY.value)
        error = int(accelAngleY.value) - desiredAngle
        print(error)
        pid_p = kp * error

        if -3 < error  < 3:
            pid_i = pid_i + (ki * error)

        pid_d = kd*((error - previousError)/ elapsedTime.value)

        pid = pid_p + pid_i + pid_d

        if pid < -1000:
            pid = -1000
        if pid > 1000:
            pid = 1000

        #Pistuffing decreases throttle in the front motor while increasing in the back.
        pwmleft = throttle - pid
        pwmRight = throttle + pid

        if pwmRight < 1100: #1000
            pwmRight = 1100
        if pwmRight > 1300: #2000
            pwmRight = 1300
        if pwmleft < 1100:
            pwmleft = 1100
        if pwmleft > 1300:
            pwmleft = 1300

        pi.set_servo_pulsewidth(leftProp, pwmleft)
        pi.set_servo_pulsewidth(rightProp, pwmRight)

        previousError = error



except KeyboardInterrupt:
    pass
finally:
    sysExit.value = 0
    os.system("sudo killall pigpiod")
    time.sleep(2)
    print(pwmleft)
    print(pwmRight)
    print("Done")
#    a.join()
#    if a.is_alive():
#        a.terminate()


