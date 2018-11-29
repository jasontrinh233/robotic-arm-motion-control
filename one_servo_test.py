# Simple program to test each servo without Leap device
from pyduino import *
import time

if __name__ == '__main__':
    
    # declare: a = Arduino(serial_port='port_name')
    a = Arduino(serial_port='COM7')    

    # sleep to have enough time for computer to make serial connection 
    time.sleep(3)    

    # servo pin
    PIN = 2         

    try:
        for i in range(0,1000):
            if i%2 == 0:
                print('180')
                # move servo on pin to an angle of 170 deg
                a.servo_write(PIN, 170) 
            else:
                print('10')
                # move servo on pin to an angle of 10 deg
                a.servo_write(PIN, 10) 

            time.sleep(1)

    except KeyboardInterrupt:
        # reset position of servo to 90 degree and close connection
        a.servo_write(PIN, 90)
        a.close()