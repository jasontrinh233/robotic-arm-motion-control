"""
Test program to control 1 servo by Leap device
"""
# Path to import Leap library
import sys
sys.path.insert(0, "LeapLib/")

import Leap, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
from pyduino import *

class SampleListener(Leap.Listener):
    
    oldtime = time.time()
    newtime = time.time()

    SERVO_PIN = 2 # Azimuthal Servo servo pin
    AZIMUTHAL_LIMIT = 180

    def on_init(self, controller):

        # define a connection to USB port
        # declare: a = Arduino(serial_port='port_name')
        self.a = Arduino(serial_port='COM7')
        
        # sleep to ensure ample time for computer to make serial connection 
        time.sleep(3)
        
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
        
        # Reset servo position when stop program
        self.a.servo_write(self.SERVO_PIN,90) 
        self.a.close()

        print("Exited")

    def on_frame(self, controller):

        # record frame information after certain time
        self.newtime = time.time()
        if self.newtime-self.oldtime > 0.1: # speed of a frame, eg: 0.03 = 3ms

            # Get the most recent frame and report some basic information
            frame = controller.frame()
            interaction_box = frame.interaction_box
           
            print ("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
                  frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures())))

            # Get hands
            for hand in frame.hands:

                handType = "Left hand" if hand.is_left else "Right hand"
                normalized_point = interaction_box.normalize_point(hand.palm_position,True)
                
                print("  %s, id %d, x-position: %s" % (handType, hand.id, normalized_point.x ))
                print("  %s, id %d, y-position: %s" % (handType, hand.id, normalized_point.y ))
                print("  %s, id %d, z-position: %s" % (handType, hand.id, normalized_point.z ))

            # FIXME depending on orientation of servo motor
            # if motor is upright, Leap Device will register a 0 degree angle if hand is all the way to the left
            # limit servo angle is 0 - 180
            XPOS_servo = abs(180 - normalized_point.x * 180) 
            print(" Servo Angle = %d " %(int(XPOS_servo)))
            
            # write the value to servo on arduino
            self.a.servo_write(self.SERVO_PIN,int(XPOS_servo)) # turn LED on

            # update the old time
            self.oldtime = self.newtime
        else:
            pass # keep advancing in time until 10 millisecond is reached

def main():

    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print('Press Enter to quit...')
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

if __name__ == "__main__":
    main()

