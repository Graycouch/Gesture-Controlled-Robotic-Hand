import sys

sys.path.insert(0, "../lib")
sys.path.insert(1, "../lib/x64")

from Pyduino import *
import numpy as np
import math

import Leap, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture


class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    oldtime = time.time()
    newtime = time.time()

    def on_init(self, controller):
        self.a = Arduino()

        # sleep to ensure enough time for the computer to make serial connection
        time.sleep(3)

        # Define Pins for Arduino Servos
        self.PIN3 = 3  # Pinky
        self.PIN5 = 5  # Ring
        self.PIN6 = 6  # Middle
        self.PIN9 = 9  # Index
        self.PIN10 = 10  # Thumb

        # make sure there is enough time to make a connection
        time.sleep(1)

        print "Initialized"

    @staticmethod
    def on_connect(controller):
        print "Connected"

        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

    @staticmethod
    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        time.sleep(1)

        # Reset arduino to initial position when you stop program
        self.a.servo_write(self.PIN3, 0)
        self.a.servo_write(self.PIN5, 0)
        self.a.servo_write(self.PIN6, 0)
        self.a.servo_write(self.PIN9, 0)
        self.a.servo_write(self.PIN10, 180)
        self.a.close()

        print "Exited"

    def on_frame(self, controller):
        textFile = open("log.txt", "a")

        # we only want to get the position of the hand every few milliseconds
        self.newtime = time.time()

        if self.newtime - self.oldtime > 0.1:  # every 10 ms we get a frame
            # Get the most recent frame and report some basic information
            frame = controller.frame()
            interaction_box = frame.interaction_box

            fingerAngle = [0, 0, 0, 0, 0]
            i = 0

            print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d\n" % (
                frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools),
                len(frame.gestures()))

            textFile.write("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d\n" % (
                frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools),
                len(frame.gestures())))

            # Get hands
            for hand in frame.hands:

                handType = "Left hand" if hand.is_left else "Right hand"

                print "%s, id %d, position: %s\n" % (
                    handType,
                    hand.id,
                    hand.palm_position)

                textFile.write("%s, id %d, position: %s\n" % (
                    handType,
                    hand.id,
                    hand.palm_position))

                # Get the hand's normal vector and direction
                normal = hand.palm_normal
                direction = hand.direction

                # Calculate the hand's pitch, roll, and yaw angles
                print "pitch: %f degrees, roll: %f degrees, yaw: %f degrees\n" % (
                    direction.pitch * Leap.RAD_TO_DEG,
                    normal.roll * Leap.RAD_TO_DEG,
                    direction.yaw * Leap.RAD_TO_DEG)

                textFile.write("pitch: %f degrees, roll: %f degrees, yaw: %f degrees\n" % (
                    direction.pitch * Leap.RAD_TO_DEG,
                    normal.roll * Leap.RAD_TO_DEG,
                    direction.yaw * Leap.RAD_TO_DEG))

                # Get arm bone
                arm = hand.arm
                print "Arm direction: %s, wrist position: %s, elbow position: %s\n" % (
                    arm.direction,
                    arm.wrist_position,
                    arm.elbow_position)

                textFile.write("Arm direction: %s, wrist position: %s, elbow position: %s\n" % (
                    arm.direction,
                    arm.wrist_position,
                    arm.elbow_position))

                # Get fingers
                for finger in hand.fingers:
                    print "%s finger, id: %d, length: %fmm, width: %fmm, \n" % (
                        self.finger_names[finger.type],
                        finger.id,
                        finger.length,
                        finger.width)
                    print("direction: ", finger.direction)

                    textFile.write("%s finger, id: %d, length: %fmm, width: %fmm, \n" % (
                        self.finger_names[finger.type],
                        finger.id,
                        finger.length,
                        finger.width))
                    # Get the finger angle
                    fingerAngle[i] = math.trunc(hand.palm_normal.angle_to(finger.direction) * Leap.RAD_TO_DEG)
                    i = i + 1

                    # Get bones
                    for b in range(0, 4):
                        bone = finger.bone(b)
                        print "Bone: %s, start: %s, end: %s, direction: %s\n" % (
                            self.bone_names[bone.type],
                            bone.prev_joint,
                            bone.next_joint,
                            bone.direction)

                        textFile.write("Bone: %s, start: %s, end: %s, direction: %s\n" % (
                            self.bone_names[bone.type],
                            bone.prev_joint,
                            bone.next_joint,
                            bone.direction))

            print("Pinky Finger Angle: %d\nRing Finger Angle: %d\nMiddle Finger Angle: %d\n"
                  "Index Finger Angle: %d\nThumb Finger Angle: %d\n" % (
                      fingerAngle[4],
                      fingerAngle[3],
                      fingerAngle[2],
                      fingerAngle[1],
                      180 - fingerAngle[0]))

            textFile.write("Pinky Finger Angle: %d\nRing Finger Angle: %d\nMiddle Finger Angle: %d\n"
                           "Index Finger Angle: %d\nThumb Finger Angle: %d\n" % (
                               fingerAngle[4],
                               fingerAngle[3],
                               fingerAngle[2],
                               fingerAngle[1],
                               180 - fingerAngle[0]))

            # Write the finger angle to servos
            self.a.servo_write(self.PIN3, fingerAngle[4])
            self.a.servo_write(self.PIN5, fingerAngle[3])
            self.a.servo_write(self.PIN6, fingerAngle[2])
            self.a.servo_write(self.PIN9, fingerAngle[1])
            self.a.servo_write(self.PIN10, 180 - fingerAngle[0])

            print("\n"
                  "-----------------------------------------------------------------------------------------------------"
                  "\n")

            textFile.write("\n"
                           "-----------------------------------------------------------------------------------------------------"
                           "\n")

            # update the old time
            self.oldtime = self.newtime
        else:
            pass  # keep advancing in time until 1 second is reached


def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
