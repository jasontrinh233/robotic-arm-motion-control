"""
This program with control 4 servo motors using Leap inputs
"""
import sys
sys.path.insert(0, "LeapLib/")  # path to Leap library for Python

import Leap, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
from pyduino import *
import numpy as np

class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']
    oldtime = time.time()
    newtime = time.time()

    def on_init(self, controller):

        # define a connection to USB serial port
        self.a = Arduino(serial_port='COM8')
        
        # allow time for computer to make serial connection to arduino
        time.sleep(3)
        
        # define PIN for each servo motor
        self.PIN2 = 2 # Azimuthal/Base
        self.PIN3 = 3 # Altitude 
        self.PIN4 = 4 # Wrist   
        self.PIN5 = 5 # Claw
        self.previous_angles = [0,0,0,0] # cordination matrix

        # allow time to make connection
        time.sleep(1)

        print("Initialized")

    def on_connect(self, controller):
        print("Connected")

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        print("Disconnected")

    def on_exit(self, controller):
        time.sleep(1)
        # Reset servo to default position when stop program
        self.a.servo_write(self.PIN2, 90)   # Az/base
        self.a.servo_write(self.PIN3, 0)    # Altitude
        self.a.servo_write(self.PIN4, 100)  # Wrist
        self.a.servo_write(self.PIN5, 70)   # Claw
        self.a.close()

        print("Exited")

    # All movement will be in this function
    def on_frame(self, controller):

        # determine time range for each between each frame obj
        self.newtime = time.time()
        if self.newtime-self.oldtime > 0.1: # every 10ms get a frame from Leap device

        # Get the most recent frame and report some basic information
            frame = controller.frame()
            interaction_box = frame.interaction_box       

            # basic frame information
            print("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
                  frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures())))

            dist_norm = 0 # global variable for claw status (open/close)

            # Get hands
            for hand in frame.hands:

                handType = "Left hand" if hand.is_left else "Right hand"
                normalized_point = interaction_box.normalize_point(hand.palm_position, True)

                self.XPOS = normalized_point.x
                self.YPOS = normalized_point.y
                self.ZPOS = normalized_point.z
                
                print("  %s, id %d, x-position: %s" % (handType, hand.id, int(self.XPOS*180) ))
                print("  %s, id %d, y-position: %s" % (handType, hand.id, int(self.YPOS*85) ))
                print("  %s, id %d, z-position: %s" % (handType, hand.id, int(self.ZPOS*180) ))

                # show how many fingers are being recorded
                print("my fingers = %d" % (len(hand.fingers)) )

                if len(hand.fingers) >= 2:
                    x1 = hand.fingers[0].tip_position[0]
                    y1 = hand.fingers[0].tip_position[1]
                    z1 = hand.fingers[0].tip_position[2]
                    # calc distance between two fingers
                    x2 = hand.fingers[1].tip_position[0]
                    y2 = hand.fingers[1].tip_position[1]
                    z2 = hand.fingers[1].tip_position[2]
            
                    # calc 2norm for difference between vector to thumb and pointer finger 
                    # basic vectors distance calculation formula
                    r = ( (x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2 )**0.5

                    # perform normalization
                    dist_norm = r/100

                    # either 0 (close) or 1 (open)
                    print("Finger Tip Distance = %d" % (dist_norm) ) 
            
            if dist_norm >= 1:
                dist_norm=1 # only need 2 values: 0,1 to indicate close,open relationship
    
            # determine range/angle for each motor
            # FIXME: MAKE MODIFICATION HERE IF NEEDED
            XPOS_servo = abs(145 - self.XPOS * 145) # Base
            YPOS_servo = abs(85 - self.YPOS * 85)   # Altitude
            ZPOS_servo = 35 + 135 * self.ZPOS       # Wrist angle
     
            # write calculated angle values onto arduino
            self.a.servo_write(self.PIN2, int(XPOS_servo)) # Azimuth
            self.a.servo_write(self.PIN3, int(YPOS_servo)) # Altitude
            self.a.servo_write(self.PIN4, int(ZPOS_servo)) # Wrist

            # claw range (from 10 to 90 degree)
            # FIXME: MAKE MODIFICATION HERE IF NEEDED
            CLAW_servo = abs(90 - dist_norm * 70) # change this formula if want different range of claw
            print("Claw Angle = %d" % (CLAW_servo) )
            print("---------------------------------------------------------") # line brake for each frame
            
            self.a.servo_write(self.PIN5, int(CLAW_servo)) # Claw

            # update oldtime
            self.oldtime = self.newtime

            # update previous cordinate values
            self.previous_angles[0] = XPOS_servo
            self.previous_angles[1] = YPOS_servo
            self.previous_angles[2] = ZPOS_servo
            self.previous_angles[3] = CLAW_servo
        else:
            pass # keep advancing in time until 1 second is reached


def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print("Press Enter to quit...")
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

if __name__ == "__main__":
    main()