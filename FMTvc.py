# This program will let you test your ESC and brushless motor.
# Make sure your battery is not connected if you are going to calibrate it at first.
# Since you are testing your motor, I hope you don't have your propeller attached to it otherwise you are in trouble my friend...?
# This program is made by AGT @instructable.com. DO NOT REPUBLISH THIS PROGRAM... actually the program itself is harmful                                             pssst Its not, its safe.

import os     #importing os library so as to communicate with the system
import time   #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod") #Launching GPIO library
time.sleep(1) # As i said it is too impatient and so if this delay is removed you will get an error
import pigpio #importing GPIO library

ESC1=19  #Connect the ESC in this GPIO pin 
ESC2=27
ESC3=20
ESC4=7

pi = pigpio.pi();
pi.set_servo_pulsewidth(ESC1, 0) 
pi.set_servo_pulsewidth(ESC2, 0) 
pi.set_servo_pulsewidth(ESC3, 0) 
pi.set_servo_pulsewidth(ESC4, 0) 

max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 1000  #change this if your ESC's min value is different or leave it be
print "For first time launch, select calibrate"
print "Type the exact word for the function you want"
print "c OR cc OR stop"

                
def calibrate():   #This is the auto calibration procedure of a normal ESC
    pi.set_servo_pulsewidth(ESC1, 0)
    pi.set_servo_pulsewidth(ESC2, 0)
    pi.set_servo_pulsewidth(ESC3, 0)
    pi.set_servo_pulsewidth(ESC4, 0)

    print("Disconnect the battery and press Enter")
    inp = raw_input()
    if inp == '':
        pi.set_servo_pulsewidth(ESC1, max_value)
        pi.set_servo_pulsewidth(ESC2, max_value)
        pi.set_servo_pulsewidth(ESC3, max_value)
        pi.set_servo_pulsewidth(ESC4, max_value)

        print("Connect the battery NOW.. you will hear two beeps, then wait for a gradual falling tone then press Enter")
        inp = raw_input()
        if inp == '':            
            pi.set_servo_pulsewidth(ESC1, min_value)
            pi.set_servo_pulsewidth(ESC2, min_value)
            pi.set_servo_pulsewidth(ESC3, min_value)
            pi.set_servo_pulsewidth(ESC4, min_value)
            print "Wierd eh! Special tone"
            time.sleep(7)
            print "Wait for it ...."
            time.sleep (5)
            print "Im working on it, DO NOT WORRY JUST WAIT....."
            pi.set_servo_pulsewidth(ESC1, 0)
            pi.set_servo_pulsewidth(ESC2, 0)
            pi.set_servo_pulsewidth(ESC3, 0)
            pi.set_servo_pulsewidth(ESC4, 0)
            time.sleep(2)
            print "Arming ESC now..."
            pi.set_servo_pulsewidth(ESC1, min_value)
            pi.set_servo_pulsewidth(ESC2, min_value)
            pi.set_servo_pulsewidth(ESC3, min_value)
            pi.set_servo_pulsewidth(ESC4, min_value)
            time.sleep(1)
            print "See.... uhhhhh"
            control() # You can change this to any other function you want
            
def control(): 
    time.sleep(1)
    speed = 1000    # change your speed if you want to.... it should be between 700 - 2000
    print "Controls: \na to decrease speed\nd to increase speed\ns to decrease a lot of speed\nw to increase a lot of speed\nx to stop the motors and end the program"
    while True:
        pi.set_servo_pulsewidth(ESC1, (speed))
        pi.set_servo_pulsewidth(ESC2, (speed))
        pi.set_servo_pulsewidth(ESC3, (speed)) #-30
        pi.set_servo_pulsewidth(ESC4, (speed)) #+45
        inp = raw_input()
        
        if inp == "s":
            speed -= 1    # decrementing the speed like hell
            print "Decrease --"
            print "speed = %d" % speed
        elif inp == "w":    
            speed += 1    # incrementing the speed like hell
            print "Increase ++"
            print "speed = %d" % speed
        elif inp == "d":
            speed += 10     # incrementing the speed
            print "Increase +"
            print "speed = %d" % speed
        elif inp == "a":
            speed -= 10     # decrementing the speed
            print "Decrease - "
            print "speed = %d" % speed
        elif inp == "e":
            speed -= 50 
            pi.set_servo_pulsewidth(ESC3, (speed - 27)) 
            pi.set_servo_pulsewidth(ESC4, (speed + 38))
	    speed += 50
	elif inp == "r":
	    speed += 10
            pi.set_servo_pulsewidth(ESC3, (speed - 27)) 
            pi.set_servo_pulsewidth(ESC4, (speed + 38)) 
            speed -= 10 
        elif inp == "x":
            print "now stopping..."
            stop()          #going for the stop function
            break	
        else:
            print "Incorrect input, please enter w,s,a,d for speed"

    print("Speeds of finished program, \n ESC1 = %d " % (
    speed))
            
def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC1, 0)
    pi.set_servo_pulsewidth(ESC2, 0)
    pi.set_servo_pulsewidth(ESC3, 0)
    pi.set_servo_pulsewidth(ESC4, 0)
    pi.stop()
    os.system ("sudo killall pigpiod") #Kill GPIO library demon
    print "Have a nice day"


#This is the start of the program actually, to start the function it needs to be initialized before calling... stupid python.    
inp = raw_input()
if inp == "c":
    calibrate()
elif inp == "cc":
    control()
elif inp == "stop":
    stop()
else :
    print "I am only designed to test 4 motors running at once. Please use another program if this task is not what you desire."
