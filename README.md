# Raspberry-Pi-Drone
A project that uses a raspberry pi as a flight controller for a drone.
The goal of this project is to create a Drone that can take off without using a flight controller.
This project uses the following material to achieve this goal all of which can be bought through amazon with 2 day delivery (Amazon Prime).


[Raspberry Pi 3](https://www.amazon.com/Raspberry-Pi-RASPBERRYPI3-MODB-1GB-Model-Motherboard/dp/B01CD5VC92)
    
[MPU6050](https://www.amazon.com/MPU-6050-MPU6050-Accelerometer-Gyroscope-Converter/dp/B008BOPN40)
    
[Female to male T-Connector for RC Lipo battery](https://www.amazon.com/gp/product/B01MY4QSO4/)

[Gens ace LiPo Battery Pack 2200mAh 25C 3S 11.1V with Deans Plug for RC Car Boat Truck Heli Airplane](https://www.amazon.com/gp/product/B00WJN4LG0/)

[HOBBYMATE Imax B6 Clone Lipo Battery Balance Charger, Rc Hobby Battery Balance Charger LED W/ AC Power Adapter](https://www.amazon.com/gp/product/B01NB9A36R)

[LHI 250mm Pro Pure Carbon Fiber Quadcopter Frame Kit + CC3D Flight Controller + MT2204 2300KV Motor + Simonk 12A ESC + 6030 CF Propeller Prop](https://www.amazon.com/gp/product/B00YACIDNU)

[4pcs 2300KV 2204 Brushless Motor CW/CCW & 4pcs SimonK 15A ESC For Quadcopter FPV Drones](https://www.amazon.com/gp/product/B01K9ZZPYQ/)
	
Programs:
1) FMTvc.py (Four Motor Test)
	
	-This program is meant to calibrate the ESCs for all four motors and test to see if they are all spinning and are spinning in the right direction. You can comment out some lines to have less than 4 motors running.
2) gyrocalibration.py (calibrating the gyroscope)
	
	-This program is meant to calibrate the gyroscope for more accurate readings. Use this to not only calibrate the MPU6050 but to see what readings you will get when you tilt the drone in a certain direction.
		
		*Example: downard forward tilt could yeild a -Y axis result or a +Y axis result (-23 or +23)
3) PIDTest.py (PID testing)
	
	-This program is meant to ensure that when you do use PID the motors react in a way that is desired.
		
		*Example: downard forward tilt will cause the front motors to spin faster and the back motors to spin slower.
4) TestFlightv3.py (Flight controller program)
	
	-This program is meant to control the drone using a Playstation 3 controller for user input. This utilizes the PID and calibration of the gyroscope to ensure the drone is stable during flight. The numbers for the ESCs will vary depending on the ESCs you use. For this project we are mix matching ESCs so one value is different compared to the others.
	
	- Playstation 3 controller is connected to the Raspberry Pi 3 via bluetooth and uses the Pygame library.
