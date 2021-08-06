################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

#uncomment oscmessage.append functions in order to send the chosen data
import thread, time

import sys
#sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), './lib/'))
sys.path.insert(0, "./lib")
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import OSC

#start OSC
c = OSC.OSCClient()
c.connect(('127.0.0.1', 6448))   # wekinator, port 6448

c2 = OSC.OSCClient()
c2.connect(('127.0.0.1', 57130))   # supercollider, port 57130
bn = []

def setInitNames():
    fingers = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    coordinates = ['x', 'y', 'z']

    names_array = ['Hand_Number']

    for finger in fingers:
        for coordinate in coordinates:
            names_array.append("%sExtension_%s" % (finger, coordinate))
    

    osc_init_names = OSC.OSCMessage()
    osc_init_names.setAddress("/wekinator/control/setInputNames")
    osc_init_names.append(names_array)
    c.send(osc_init_names)


def vectorCoordinates(vector):
    return [vector.x, vector.y, vector.z]

def tupleExtractor(elements):
    result = []
    for element in elements:
        if isinstance(element, str):
            result.append(element)
        elif isinstance(element, Leap.Vector): 
            result.extend(vectorCoordinates(element))
        elif isinstance(element, int):
            result.append(int(element))#has to be converted for some reason
        else:
            result.append(float(element))
    return result




class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        basics = ( frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        oscmsg = OSC.OSCMessage()
        oscmsg.setAddress("/wek/inputs")
        #oscmsg.append(tupleExtractor(basics))


        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % basics      # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            hand_pos = (handType, hand.id, hand.palm_position)

            print "  %s, id %d, position: %s" % hand_pos

            #oscmsg.append(tupleExtractor(hand_pos))

            #send 0 for left hand and 1 for right hand
            handNumber = 0 if hand.is_left else 1
            oscmsg.append(handNumber)

            #extracts coordinates of palm in order to calculate finger extensions
            handCoordinate = (vectorCoordinates(hand_pos[2]))


            # Get arm bone
            arm = hand.arm
            arm_pos = (
                arm.direction,
                arm.wrist_position,
                arm.elbow_position)
            #print "  Arm direction: %s, wrist position: %s, elbow position: %s" % arm_pos


            #oscmsg.append(tupleExtractor(arm_pos))

            #empty array
            fingerExtensions = []

            # Get fingers
            for finger in hand.fingers:

                finger_info = (
                    self.finger_names[finger.type],
                    finger.id,
                    finger.length,
                    finger.width,
                    finger.is_extended)
                #print "    %s finger, id: %d, length: %fmm, width: %fmm, Extended: %d" % finger_info

                #oscmsg.append(tupleExtractor(finger_info))

                # Get bones
                for b in range(0, 4):
                    bone = finger.bone(b)
                    bone_info = (
                        self.bone_names[bone.type],
                        bone.prev_joint,
                        bone.next_joint,
                        bone.direction)
                    #print "      Bone: %s, start: %s, end: %s, direction: %s" % bone_info
                    #oscmsg.append(tupleExtractor(bone_info))

                    extensionCoordinate = []

                    if b == 3:
                        #send only info about the end of the finger
                        #oscmsg.append(vectorCoordinates(bone.next_joint))
                        boneCoordinate = (vectorCoordinates(bone.next_joint))


                        for i in range(3):
                            extensionCoordinate.append(handCoordinate[i] - boneCoordinate[i])

                        print "       Extensions %s" % extensionCoordinate

                    fingerExtensions.append(extensionCoordinate)

            oscmsg.append(fingerExtensions)



        # Get gestures
        for gesture in frame.gestures():

            oscmsg_gest = OSC.OSCMessage()
            oscmsg_gest.setAddress("/leap/gestures")

            if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                circle = CircleGesture(gesture)

                # Determine clock direction using the angle between the pointable and the circle normal
                if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/2:
                    clockwiseness = "clockwise"
                else:
                    clockwiseness = "counterclockwise"

                # Calculate the angle swept since the last frame
                swept_angle = 0
                if circle.state != Leap.Gesture.STATE_START:
                    previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                    swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

                circle_result = (gesture.id, self.state_names[gesture.state], circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness)

                print "  Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % circle_result
                oscmsg_gest.append(circle_result)

            if gesture.type == Leap.Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)
                swipe_result =  (gesture.id, self.state_names[gesture.state], swipe.position, swipe.direction, swipe.speed)

                print "  Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % swipe_result
                oscmsg_gest.append(tupleExtractor(swipe_result))
                #oscmsg_gest.append(swipe_result[4])

            if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
                keytap = KeyTapGesture(gesture)
                keytap_result = (gesture.id, self.state_names[gesture.state], keytap.position, keytap.direction)

                print "  Key Tap id: %d, %s, position: %s, direction: %s" % keytap_result
                oscmsg_gest.append(keytap_result)

            if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
                screentap = ScreenTapGesture(gesture)
                screentap_result = (gesture.id, self.state_names[gesture.state], screentap.position, screentap.direction)
                print "  Screen Tap id: %d, %s, position: %s, direction: %s" %  screentap_result
                oscmsg_gest.append(screentap_result)
                
            #send osc message with information about gestures
            c2.send(oscmsg_gest)

        if not (frame.hands.is_empty and frame.gestures().is_empty):
            print ""

        c.send(oscmsg)
        #time.sleep(0.2)


    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

def main():


    #send names of data throug OSC
    setInitNames()

    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    #keep on running in background
    controller.set_policy(Leap.Controller.POLICY_BACKGROUND_FRAMES)

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
