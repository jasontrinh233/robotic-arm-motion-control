Leap Motion SDK only supports python 2.7 (my current python version is 2.7.15)

Install pyserial (python library to talk with arduino through USB port) using $pip
	numpy

Execute 4_servo_main.py to run the program.

Note:
	pyduino.py = python to arduino library. Main purpose is to read and write values
		     on a specific PIN
	
	4_servo_main.py = Main program that control all 4 servo motors. It using 
			  pyduino.py library to send message to Arduino device

	4_servo/4_servo.ino = Recieve messages from 4_servo_main.py, but Arduino can only
	                      execute in Arduino language. Thus, 4_servo.in helps communicate
			      with physical components.

