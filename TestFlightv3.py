# !/usr/bin/python
# This program tests getting the gyroscope, accel, compass, pressure, and temperature all in one file.
import smbus
import time
import math
import datetime
import os
from multiprocessing import Process, Value
import pygame
import sys
import pygame.display
os.system ("sudo pigpiod")
os.environ['SDL_VIDEODRIVER'] = 'dummy'
time.sleep(1)
import pigpio
import RPi.GPIO as GPIO
pygame.init()
pygame.display.set_mode((1,1))
j = pygame.joystick.Joystick(0)
j.init()
pi = pigpio.pi()


ESC1=19
ESC2=27
ESC3=20
ESC4=7
# Max and Min speed of the motors. The values of the motors can not go under 1000 or above 1999
max_value = 1999
min_value = 1000
#speed value for the motors. these are used to change the speed of the motors. These interact with the sensor reading
speed1 = Value('f', 1000)
speed2 = Value('f', 1000)
speed3 = Value('f', 1000)
speed4 = Value('f', 1000)
#original speed value of the motors. These values are unaffected by the sensor calibration.
orispeed1 = Value('f', 1000)
orispeed2 = Value('f', 1000)
orispeed3 = Value('f', 1000)
orispeed4 = Value('f', 1000)
#values for the controller.
holdup = Value('f', 0.0) #value for the D-pad up button
holddown = Value('f', 0.0) #value for the D-pad down button
holdleft = Value('f', 0.0) #value for the D-pad left button
holdright = Value('f', 0.0) #value for the D-pad right button
holdstart = Value('f', 0.0) #value for the start button
holdx = Value('f', 0.0) #value for the x button
holdtriangle = Value('f', 0.0) #value for the triangle button
holdselect = Value('f', 0.0) #value for the select button
accelx = Value('f', 0.0) #value for the triangle button
accely = Value('f', 0.0) #value for the triangle button
accelz = Value('f', 0.0) #value for the triangle button
gyrox = Value('f', 0.0) #value for the triangle button
gyroy = Value('f', 0.0) #value for the triangle button
gyroz = Value('f', 0.0) #value for the triangle button
#values for the sensor loop
totalAngleX =  Value('f', 0)
totalAngleY =  Value('f', 0)
accelAngleX =  Value('f', 0)
accelAngleY =  Value('f', 0)
elapsedTime =  Value('f', 0)
calibrationMode =  Value('f', 1)
motorsRunning =  Value('f', 1)

sysExit = Value('f', 1.0) #value for the loops true == 1 and false == 0

###############################################################################################
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
######            https://github.com/Tijndagamer/mpu6050                                 ######
######               This class is from this user.                                       ######
######                All credit for this goes Tijndagamer                               ######
######                Added low pass filter part only                                    ######
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
###############################################################################################

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
        # This sets the low pass filter. With this high frequency results are ignored (filtered out)
        # This leaves only the low filter results, which consists of more accurate readings.
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
        # Euler formula
        accelangleX = math.atan(y / math.sqrt(math.pow(x, 2) + math.pow(z, 2))) * 180 / 3.141592654
        accelangleY = math.atan(-1 * (x / math.sqrt(math.pow(y, 2) + math.pow(z, 2)))) * 180 / 3.141592654

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


###############################################################################################
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
######    Function to restart the motors should one motor not start in the beginning     ######
######       This function should only be called when a certain button is pressed        ######
######                   Should not be called while in flight                            ######
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
###############################################################################################
def restartmotors():
    speed1.value = 1000
    speed2.value = 1000
    speed3.value = 1000
    speed4.value = 1000
    pi.set_servo_pulsewidth(ESC1, speed1.value)
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    pi.set_servo_pulsewidth(ESC3, speed3.value)
    pi.set_servo_pulsewidth(ESC4, speed4.value)
    time.sleep(1)
    time.sleep(0.5)
    print("are they all running?")
    speed2.value = 1030
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    time.sleep(0.5)
    speed2.value = 1028
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    time.sleep(0.5)
    speed2.value = 1031
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    time.sleep(0.5)
    speed1.value = 1065
    speed2.value = 1030
    speed3.value = 1065
    speed4.value = 1065
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    time.sleep(0.2)
    speed1.value = 1181
    speed2.value = 1200
    speed3.value = 1195
    speed4.value = 1165
    orispeed1.value = 1081
    orispeed2.value = 1099
    orispeed3.value = 1095
    orispeed4.value = 1065
    pi.set_servo_pulsewidth(ESC1, speed1.value)
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    pi.set_servo_pulsewidth(ESC3, speed3.value)
    pi.set_servo_pulsewidth(ESC4, speed4.value)

###############################################################################################
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
######        Function to interact with the motors depending on the user's input         ######
######        This function is a separate process that is started to run with others     ######
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
###############################################################################################
def motorControl(): #This function only tells the program what throttle speed the motors should have and does not interact with the controller at all.
    while calibrationMode.value == 1 and sysExit.value == 1:
        time.sleep(1)
    time.sleep(5)
    print("Motors are ready to go!")
    global max_value
    maxSpeed = max_value #This ensures the motors do not speed a at a speed that they are not suppoed to go over. the range is 1000-2000
    global min_value
    minSpeed = min_value #this ensures the motors do not speed at a speed that is lower than they are supposed to go. The range is 1000-2000
    count = 0
    #time.sleep(10)
    speed1.value = 1000
    speed2.value = 1000
    speed3.value = 1000
    speed4.value = 1000
    pi.set_servo_pulsewidth(ESC1, speed1.value)
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    pi.set_servo_pulsewidth(ESC3, speed3.value)
    pi.set_servo_pulsewidth(ESC4, speed4.value)
    time.sleep(1)
    time.sleep(0.5)
    print("are they all running?")
    speed2.value = 1030
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    time.sleep(0.5)
    speed2.value = 1028
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    time.sleep(0.5)
    speed2.value = 1031
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    time.sleep(0.5)
    speed2.value = 1030
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    time.sleep(0.2)
    speed1.value = 1181
    speed2.value = 1200
    speed3.value = 1195
    speed4.value = 1165
    orispeed1.value = 1181
    orispeed2.value = 1200
    orispeed3.value = 1195
    orispeed4.value = 1165
    pi.set_servo_pulsewidth(ESC1, speed1.value)
    pi.set_servo_pulsewidth(ESC2, speed2.value)
    pi.set_servo_pulsewidth(ESC3, speed3.value)
    pi.set_servo_pulsewidth(ESC4, speed4.value)


    time.sleep(2)
    print("They all should be running now")
    motorsRunning.value = 0
    while sysExit.value == 1:
        time.sleep(0.05)
        count += 1
        if (count % 1000) == 0:
           print ("Current speeds \nESC1 = %d \n ESC2 = %d \n ESC3 = %d \n ESC4 = %d \n" %(speed1.value, speed2.value, speed3.value, speed4.value))


        if holdstart.value: #Start button on the controller
           print ("Exiting...")
        elif holdselect.value:
            restartmotors()
        elif holdtriangle.value: #Triangle on the controller
            #Maybe when you press this button the drone will change to takeoff speed?
            print("You pressed triangle")
        elif holdx.value: #X on the controller
            #Maybe this will set the drone to hover speed?
            print("Speed very up")
        elif holdup.value: #Up on the D-pad
            #This should increase the speed of the motors by 1 for as long as the button is held down. NEEDS TESTING
            if speed1.value >= maxSpeed or speed2.value >= maxSpeed or speed3.value >= maxSpeed or speed4.value >= maxSpeed:
                print ("can't go any higher. This is only a test program.")
            else:
               # print("speed up")
               # have not tested since we obtained takeoff.
                speed1.value += 1
                orispeed1.value += 1
                speed2.value += 1
                orispeed2.value += 1
                speed3.value += 1
                orispeed3.value += 1
                speed4.value += 1
                orispeed4.value += 1
        elif holddown.value: #Down on the D-pad
            #This should decrease the speed of the motors by 1 for as long as the button is held down. NEEDS TESTING
            if speed1.value <= minSpeed or speed2.value <= minSpeed or speed3.value <= minSpeed or speed4.value <= minSpeed:
                print ("Can't go any lower. The range of the motors is 1000-2000")
            else:
                #this would make the drone go forward and to the left after takeoff. need to work on it.
                print("speed down")
                speed1.value -= 1
                orispeed1.value -= 1
                speed2.value -= 1
                orispeed2.value -= 1
                speed3.value -= 1
                orispeed3.value -= 1
                speed4.value -= 1
                orispeed4.value -= 1



    #Should sysExit become the value 0 the process will reach here and kill the power to the motors. The motors will not speed at a speed of 1000.
    #this also kills the pigpio library and everything should end nicely with this process.
    print("Speeds of finished program, \n ESC1 = %d \n ESC2 = %d \n ESC3 = %d \n ESC4 = %d \n" %(speed1.value, speed2.value, speed3.value, speed4.value))
    time.sleep(0.005)
    pi.set_servo_pulsewidth(ESC1, 1000)
    pi.set_servo_pulsewidth(ESC2, 1000)
    pi.set_servo_pulsewidth(ESC3, 1000)
    pi.set_servo_pulsewidth(ESC4, 1000)
    print("Motors are ready to join comrades at the finish")

###############################################################################################
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
######    Function to interact with the motors depending on the sensor's reading         ######
######    This will only interact with the motors should the drone tilt                  ######
######    Once it is no longer tilting it will return the values to what they            ######
######          Were before the sensor changed them.                                     ######
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
###############################################################################################
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

# Wake up the sensor from sleep mode.
bus.write_byte_data(address, power_mgmt_1, 0)

def sensorAction():
    desiredAngle = 0
    #pid values for vertical velocity
    kvp = 50.0  # 3.55
    kvi = 0.00006  # 0.006 we use 3.006
    kvd = 2.0
    #as we are able to get the drone to be stable during vertical takeoff I will add pid values for pitch and roll.
    pid_pX = 0
    pid_iX = 0
    pid_dX = 0
    errorX = 0
    previousErrorX = 0
    pidX = 0
    pid_pY = 0
    pid_iY = 0
    pid_dY = 0
    errorY = 0
    previousErrorY = 0
    pidY = 0
    throttleX = 1100
    throttleY = 1100
    accspeed1 = 0
    accspeed2 = 0
    accspeed3 = 0
    accspeed4 = 0
    time.sleep(5) #Give the motor process time to run
    mpu = mpu6050(0x68)
    i = 0
    count = 0
    gyrox = 0
    gyroy = 0
    gyroz = 0
    gyro_roll_input = 0
    gyro_pitch_input = 0
    gyro_yaw_input = 0
    angle_pitch = 0
    angle_roll = 0
    gyro_pitch = 0
    gyro_roll = 0
    gyro_yaw = 0
    acc_total_vector = 0
    acc_x = 0
    acc_y = 0
    acc_z = 0
    angle_roll_acc = 0
    angle_pitch_acc = 0
    pid_roll_setpoint = 0
    pid_pitch_setpoint = 0
    pid_yaw_setpoint = 0
    pid_i_gain_roll = 0
    pid_p_gain_roll = 4.3
    pid_i_gain_roll = 0.04
    pid_d_gain_roll = 18.0
    pid_max_roll = 200
    pid_p_gain_pitch = 4.3
    pid_i_gain_pitch = 0.04
    pid_d_gain_pitch = 18.0
    pid_max_pitch = 200
    pid_p_gain_yaw = 4.0
    pid_i_gain_yaw = 0.02
    pid_d_gain_yaw = 0.0
    pid_max_yaw = 200
    gyrox_cal = 0
    gyroy_cal = 0
    gyroz_cal = 0
    count = 0
    gryox = 0
    pitch_angle = 0
    roll_angle = 0
    pid_i_mem_roll = 0
    pid_last_roll_d_error = 0
    pid_i_mem_pitch =0
    pid_last_pitch_d_error = 0
    pid_i_mem_yaw = 0
    pid_last_yaw_d_error =0
    #Get the offset of the gyroscope
    while count < 2000 and sysExit.value:
        calibrationMode.value = 1
        gyroy_cal += read_word_2c(0x43)
        gyrox_cal += read_word_2c(0x45)
        gyroz_cal += read_word_2c(0x47)
        time.sleep(0.004)
        count += 1

    gyrox_cal /= 2000
    gyroy_cal /= 2000
    gyroz_cal /= 2000
    print("Calibrations complete")
    calibrationMode.value = 0
    while motorsRunning.value == 1 and sysExit.value == 1: #to ensure that the motors are running and good to go we check here or we exit if the user presses the start button.
        time.sleep(1) #we wait 1 second then check again.
    time.sleep(5)
    speed1.value = 1229
    speed2.value = 1248
    speed3.value = 1243
    speed4.value = 1213
    #Hover speed values? This raises the drone.
    #speed1.value = 1307
    #speed2.value = 1326
    #speed3.value = 1321
    #speed4.value = 1291
    while sysExit.value: #This is where the PID magic happens. This loop does not happen if the user presses start beforehand.
        gyroy_raw = read_word_2c(0x43)
        gyroy_raw -= - gyroy_cal - 350  # to ensure the reading is accurate after testing I found that value
        gyrox_raw = read_word_2c(0x45)
        gyrox_raw -= gyrox_cal
        gyroz_raw = read_word_2c(0x47)
        gyroz_raw -= gyroz_cal
        roll_angle = gyrox_raw * 0.000611
        pitch_angle = gyroy_raw * 0.000611

        gyro_roll_input = (gyro_roll_input * 0.7) + ((gyrox_raw / 131) * 0.3)
        gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyroy_raw/ 131) * 0.3)
        gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyroz_raw/ 131) * 0.3)

        pid_error_temp_roll = (gyrox_raw / 131) - pid_roll_setpoint
        print("Roll (not the character) : ", pid_error_temp_roll)
        pid_i_mem_roll += pid_i_gain_roll * pid_error_temp_roll
        if pid_i_mem_roll > pid_max_roll:
            pid_i_mem_roll = pid_max_roll
        elif pid_i_mem_roll < (pid_max_roll * -1):
            pid_i_mem_roll = pid_max_roll * -1

        pid_output_roll = pid_p_gain_roll * pid_error_temp_roll + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp_roll - pid_last_roll_d_error)
        if pid_output_roll > pid_max_roll:
            pid_output_roll = pid_max_roll
        elif pid_output_roll < (pid_max_roll * -1):
            pid_output_roll = pid_max_roll * -1

        pid_last_roll_d_error = pid_error_temp_roll

        # Pitch calculations
        pid_error_temp_pitch = ((gyroy_raw/ 131) + 0) - pid_pitch_setpoint
        print("Pitch : ", pid_error_temp_pitch)
        pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp_pitch
        if pid_i_mem_pitch > pid_max_pitch:
            pid_i_mem_pitch = pid_max_pitch
        elif pid_i_mem_pitch < (pid_max_pitch * -1):
            pid_i_mem_pitch = pid_max_pitch * -1

        pid_output_pitch = pid_p_gain_pitch * pid_error_temp_pitch + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp_pitch - pid_last_pitch_d_error)
        if pid_output_pitch > pid_max_pitch:
            pid_output_pitch = pid_max_pitch
        elif pid_output_pitch < (pid_max_pitch * -1):
            pid_output_pitch = pid_max_pitch * -1

        pid_last_pitch_d_error = pid_error_temp_pitch

        # Yaw calculations
        pid_error_temp_yaw = (gyroz_raw/ 131) - pid_yaw_setpoint
        print("Yaw : ", pid_error_temp_yaw)
        pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp_yaw
        if pid_i_mem_yaw > pid_max_yaw:
            pid_i_mem_yaw = pid_max_yaw
        elif pid_i_mem_yaw < (pid_max_yaw * -1):
            pid_i_mem_yaw = pid_max_yaw * -1

        pid_output_yaw = pid_p_gain_yaw * pid_error_temp_yaw + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp_yaw - pid_last_yaw_d_error)
        if pid_output_yaw > pid_max_yaw:
            pid_output_yaw = pid_max_yaw
        elif pid_output_yaw < (pid_max_yaw * -1):
            pid_output_yaw = pid_max_yaw * -1

        pid_last_yaw_d_error = pid_error_temp_yaw

        accspeed1 = speed1.value - pid_output_roll
        accspeed2 = speed2.value - pid_output_roll
        accspeed3 = speed3.value + pid_output_roll #+ 180
        accspeed4 = speed4.value + pid_output_roll #+ 180

        accspeed1 = accspeed1 - pid_output_pitch
        accspeed4 = accspeed4 - pid_output_pitch
        accspeed2 = accspeed2 + pid_output_pitch
        accspeed3 = accspeed3 + pid_output_pitch

       # accspeed1 = accspeed1 + pid_output_yaw
       # accspeed4 = accspeed4 - pid_output_yaw
       # accspeed2 = accspeed2 - pid_output_yaw + 80
       # accspeed3 = accspeed3 + pid_output_yaw

        if accspeed1 < speed1.value - 200:
            accspeed1 = speed1.value - 200
        if accspeed2 < speed2.value - 120:
            accspeed2 = speed2.value - 120
        if accspeed3 < speed3.value - 200:
            accspeed3 = speed3.value - 200
        if accspeed4 < speed4.value - 200:
            accspeed4 = speed4.value - 200

        if accspeed1 < 1200:
            accspeed1 = 1200
        if accspeed2 < 1280:
            accspeed2 = 1280
        if accspeed3 < 1200:
            accspeed3 = 1200
        if accspeed4 < 1200:
            accspeed4 = 1200

        if accspeed1 > speed1.value + 200:
            accspeed1 = speed1.value + 200
        if accspeed2 > speed2.value + 400: #What if we unrestrict the max speed of the motor? Will it have enough power to stabilize the drone?
            accspeed2 = speed2.value + 400
        if accspeed3 > speed3.value + 280:
            accspeed3 = speed3.value + 280
        if accspeed4 > speed4.value + 280:
            accspeed4 = speed4.value + 280
            
        if accspeed1 > 1900:
            accspeed1 = 1900
        if accspeed2 > 1900:
            accspeed2 = 1900
        if accspeed3 > 1900:
            accspeed3 = 1900
        if accspeed4 > 1900:
            accspeed4 = 1900

        print(accspeed1)
        print(accspeed2)
        print(accspeed3)
        print(accspeed4)

        pi.set_servo_pulsewidth(ESC1, int(round(accspeed1)))
        pi.set_servo_pulsewidth(ESC3, int(round(accspeed3)))
        pi.set_servo_pulsewidth(ESC2, int(round(accspeed2)))
        pi.set_servo_pulsewidth(ESC4, int(round(accspeed4)))

    print("accspeeds: speed1= %d \nspeed2= %d \nspeed3 = %d \nspeed4 = %d" %(accspeed1, accspeed2, accspeed3, accspeed4))
    print("Senor Action is complete!")

###############################################################################################
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
######    Function to interact with the controller connected to the pi                   ######
######    The controller is connected via bluetooth and is only reading certain buttons  ######
######    The user can increase or decrease speeds, restart the motors,                  ######
######        and end the program                                                        ######
######                                                                                   ######
######                                                                                   ######
######                                                                                   ######
###############################################################################################
def f(name):
    #print('hello', name)
    time.sleep(1)
    print("Lets get started on the controller wooo!")
    while sysExit.value == 1:

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 3:  # Start button on the controller
                    sysExit.value = 0  # Exit the infinite loop
                elif event.button == 0:
                    holdselect.value = 1
                elif event.button == 12:  # Triangle on the controller
                    holdtriangle.value = 1
                elif event.button == 14:  # X on the controller
                    holdx.value = 1
                elif event.button == 4:  # Up on the D-pad
                    holdup.value = 1
                elif event.button == 6:  # Down on the D-pad
                    holddown.value = 1
            if event.type == pygame.JOYBUTTONUP:
                if event.button == 3:  # Start button on the controller
                    sysExit.value = 0  # Exit the infinite loop
                elif event.button == 0:
                    holdselect.value = 0
                elif event.button == 12:  # Triangle on the controller
                    holdtriangle.value = 0
                elif event.button == 14:  # X on the controller
                    holdx.value = 0
                elif event.button == 4:  # Up on the D-pad
                    holdup.value = 0
                elif event.button == 6:  # Down on the D-pad
                    holddown.value = 0

    print("Let me join my comrade in arms at the finish line.")

#This is where the program starts. It creates processes to run concurrently with the main process
if __name__ == '__main__':
   # time.sleep(30)
    p = Process(target=f, args=('bob',)) #create processes to run corruently with main process
    #s = Process(target=sensors, args=('bob',))
    a = Process(target=sensorAction, args=())
    #s.start() #start the processes. they will only interact with the function that was defined for them
    a.start()
    p.start()
    print("Process started")
    time.sleep(1)
    motorControl()
    print("No longer looking for controller input")
    p.join() #block main process until this p process
    #s.join()
    a.join()
    #if s.is_alive(): #check if the process is still alive
    #    s.terminate() #if it is then terminate the process.
    if p.is_alive():
        p.terminate()
    if a.is_alive():
        a.terminate()
    pi.stop()
    time.sleep(1)
    os.system("sudo killall pigpiod")
    print("And the other guy is closed, so we are done")