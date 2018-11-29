# Leap program to track hand position using LeapMotionSDK
# import the libraries where the LeapMotionSDK is
import sys
sys.path.insert(0, "LeapLib/")  #path to leap library

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
        # declare: a = Arduino(serial_port='port_name')
        self.a = Arduino(serial_port='COM7')
        
        # sleep to ensure ample time for computer to make serial connection 
        time.sleep(3)
        
        # Define Pins for Arduino Servos
        self.PIN2 = 2 # Azimuthal / Base
        self.PIN3 = 3 # Altitude 
        self.PIN4 = 4 # Wrist   
        self.PIN5 = 5 # Claw
        self.previous_angles = [0,0,0,0]

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
        # Reset arduino to default when stop program
        self.a.servo_write(self.PIN2, 90)   # Az / Base
        self.a.servo_write(self.PIN3, 0)    # Altitude
        self.a.servo_write(self.PIN4, 100)  # Wrist
        self.a.servo_write(self.PIN5, 70)   # Claw
        self.a.close()

        print("Exited")

    def on_frame(self, controller):

        # we only want to get the position of the hand every so often
        self.newtime = time.time()
        if self.newtime-self.oldtime > 0.1: # every 10 ms get a frame

        # Get the most recent frame and report some basic information
            frame = controller.frame()
            interaction_box = frame.interaction_box       

            print("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
                  frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures())))

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

                print('my fingers =', len(hand.fingers))
                if len(hand.fingers) >= 2:
                    x1 = hand.fingers[0].tip_position[0]
                    y1 = hand.fingers[0].tip_position[1]
                    z1 = hand.fingers[0].tip_position[2]
                    # calc distance between two fingers
                    x2 = hand.fingers[1].tip_position[0]
                    y2 = hand.fingers[1].tip_position[1]
                    z2 = hand.fingers[1].tip_position[2]
            
                    # calc 2norm for difference between vector to thumb and pointer finger 
                    r = ( (x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2 )**0.5

                    # perform a crude normalization
                    dist_norm = r/100.
                    # may need to change the 100 to something else

                    print('Finger Tip Distance = ', dist_norm)
            
            if dist_norm >= 1:
                dist_norm=1
                #for finger in hand.fingers:
                    #print finger,' - ',finger.tip_position

    
            # determine motors - adjust angle ranges here too
            XPOS_servo = abs(145 - self.XPOS * 145) # Base
            YPOS_servo = abs(85 - self.YPOS * 85)   # Altitude
            ZPOS_servo = 35 + 135 * self.ZPOS       # Wrist angle
     
            # write the value to servo on arduino
            self.a.servo_write(self.PIN2, int(XPOS_servo)) # Azimuth
            self.a.servo_write(self.PIN3, int(YPOS_servo)) # Altitude
            self.a.servo_write(self.PIN4, int(ZPOS_servo)) # Wrist

            # claw range
            CLAW_SERVO = abs(90 - dist_norm * 70)
            print('Claw Angle =', CLAW_SERVO)
            
            self.a.servo_write(self.PIN5, int(CLAW_SERVO))

            # update the old time
            self.oldtime = self.newtime

            # update previous values
            self.previous_angles[0] = XPOS_servo
            self.previous_angles[1] = YPOS_servo
            self.previous_angles[2] = ZPOS_servo
            self.previous_angles[3] = CLAW_SERVO
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