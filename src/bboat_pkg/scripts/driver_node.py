#!/usr/bin/env python3

# Import required Python code.
import rospy
import sys
import numpy as np

from std_msgs.msg import Bool

from sensor_msgs.msg import Joy, BatteryState

from mavros_msgs.msg import State, OverrideRCIn

from bboat_pkg.srv import reset_lamb_serv, mode_serv, mode_servResponse, lambert_ref_serv
from bboat_pkg.msg import cmd_msg
from lib.bboat_lib import *

from subprocess import call
import os



Missions = ["VSB", "CAP", "PTN"]
max_fwrd = 1700
min_fwrd = 1300
max_turn = 1700
min_turn = 1300

class DriverNode():
    '''
        Blueboat driver node - Communicates with mavros
        Subscribers
            - Mavros state - Armed Disarmed
            - Marvros Battery - Battery level
            - Joystick node
            - Command topic from controller node
        Publishers
            - Mavros override - Sens forward and turning commands
        Service Clients
            - Reset Lambert Ref to current position client
            - Request current Lambert ref
        Service Providers
            - Mode - AUTO or MANUAL
    '''
    def __init__(self):

        self.rate =rospy.Rate(10) #10Hz

        self.joy_fwrd = 0
        self.joy_turn = 0

        self.mode = "MANUAL" #MANUAL - AUTO
        self.mission_index = 0
        self.auto_mission = Missions[self.mission_index]
        self.count_print = 0
        
        self.auto_cmd = np.zeros((2,1))
        self.max_speed_fwrd = 3 #m/s Bluerobotics website 
        self.max_speed_turn = 1 #rad/s -> ~30cm between thruster and center

        self.battery = 0.0

        self.armed_flag = False
        self.should_be_armed_flag = False

        self.buttons_prev = []

        self.prev_fwrd = 1500
        self.prev_turn = 1500

        # --- Subs
        rospy.Subscriber("/joy", Joy, self.Joy_callback)

        rospy.Subscriber("/mavros/state", State, self.State_callback)
        rospy.wait_for_message("/mavros/state", State, timeout=None)
        rospy.Subscriber("/mavros/battery", BatteryState, self.Battery_callback)



        self.sub_cmd = rospy.Subscriber('/command', cmd_msg, self.Command_callback)

        # --- Pubs
        self.pub_rc_override = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

        # --- Services
        rospy.wait_for_service('/reset_lamb_ref')
        connected = False
        while not connected:
            try:
                self.client_reset_lamb = rospy.ServiceProxy('/reset_lamb_ref', reset_lamb_serv)
                connected = True
            except rospy.ServiceException as exc:
                rospy.logwarn(f'[DRIVER] Reset Lambert ref service cannot be reached - {str(exc)}')
                connected = False


        rospy.wait_for_service('/lambert_ref')
        connected = False
        while not connected:
            try:
                self.client_ref_lambert = rospy.ServiceProxy('/lambert_ref', lambert_ref_serv)
                connected = True
            except resoyServiceException as exc:
                rospy.logwarn(f'[DRIVER] Lambert ref service cannot be reached - {str(exc)}')
                connected = False


        rospy.Service('/mode', mode_serv, self.Mode_Service_callback)

        # --- Init done
        rospy.loginfo('[DRIVER] Driver Node Start')


    def loop(self): 
        # Main while loop.
        while not rospy.is_shutdown():
            #rospy.loginfo('[DRIVER] Heartbeat')

            # Build and publish override message depending on mode
            rc_msg = OverrideRCIn()
            if self.mode == "MANUAL":
                fwrd = (1500+500*self.joy_fwrd) #int
                turn = (1500-500*self.joy_turn) #int

                fwrd = int(0.15*fwrd + 0.85*self.prev_fwrd)
                turn = int(0.15*turn + 0.85*self.prev_turn)
                self.prev_fwrd = fwrd
                self.prev_turn = turn
            elif self.mode == "AUTO":
                fwrd = self.auto_cmd[0,0]
                turn = self.auto_cmd[1,0]

            if fwrd > max_fwrd : 
                fwrd = max_fwrd
            elif fwrd < min_fwrd :
                fwrd = min_fwrd
            if turn > max_turn :
                turn = max_turn 
            elif turn < min_turn :
                turn = min_turn

                

            # All unused channels set to neutral value seems safe
            rc_msg.channels = [turn,1500,fwrd,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500]
            self.pub_rc_override.publish(rc_msg)

            if self.count_print == 50: 
                if self.armed_flag:
                    rospy.loginfo(f'[DRIVER] Status : Armed | Mode : {self.mode} | Mission : {self.auto_mission} | Battery : {round(self.battery, 2)}')
                
                else:
                    rospy.loginfo(f'[DRIVER] Status : Disarmed | Mode : {self.mode} | Mission : {self.auto_mission} | Battery : {round(self.battery, 2)}')

                if self.battery < 13.5:
                    rospy.logwarn('[DRIVER] LOW BATTERY LEVEL')
                self.count_print = 0
            else:
                self.count_print+=1

            self.rate.sleep()


    def Joy_callback(self, msg): 
        '''
            Parse Jaystick message
        '''
        #rospy.loginfo('[DRIVER] Joystick')

        buttons = msg.buttons
        axes = msg.axes
        if not (buttons == self.buttons_prev):
            # Arm - Pause
            if buttons[7]: 
                rospy.loginfo('[DRIVER] Arming Thrusters')
                #call(["rosrun", "mavros", "mavsafety", "arm"])
                os.system("rosrun mavros mavsafety arm")
                self.should_be_armed_flag = True
            # Disarm - Reset   
            if buttons[6]: 
                rospy.loginfo('[DRIVER] Disarming Thrusters')
                # call(["rosrun", "mavros", "mavsafety", "disarm"])
                os.system("rosrun mavros mavsafety disarm")
                self.should_be_armed_flag = False

            # Reset Lambert reference - A
            if buttons[0] : 
                rospy.loginfo('[DRIVER] Reset Lambert')
                resp = self.client_reset_lamb(True)

            # Switch mode - B
            if buttons[1]: 
                rospy.loginfo('[DRIVER] Switching mode to: ')    
                if self.mode == "MANUAL":
                    self.mode = "AUTO"
                elif self.mode == "AUTO":
                    self.mode = "MANUAL"
                rospy.loginfo(f'[DRIVER] {self.mode}')

            if axes[6] == -1:
                if self.mission_index < len(Missions)-1:
                    self.mission_index += 1
                else:
                    self.mission_index = 0
                self.auto_mission = Missions[self.mission_index]
                rospy.loginfo(f'[DRIVER] Switching Mission to {self.auto_mission}')
            elif axes[6] == 1:
                if self.mission_index > 0:
                    self.mission_index -=1
                else:
                    self.mission_index = len(Missions)-1
                self.auto_mission = Missions[self.mission_index]            
                rospy.loginfo(f'[DRIVER] Switching Mission to {self.auto_mission}')


        # Joystick values between -1 and 1
        self.joy_fwrd = axes[1]
        self.joy_turn = axes[0]

    def State_callback(self, msg):
        '''
            Parse robot state msg - armed disarmed - Raise Warnin if state doesn't match with required state
        '''
        #rospy.loginfo('[DRIVER] State callback')

        self.armed_flag = msg.armed
        if self.should_be_armed_flag and (self.should_be_armed_flag != self.armed_flag):
            rospy.logwarn('[DRIVER] BBoat should be armed but is NOT')
        elif not self.should_be_armed_flag and (self.should_be_armed_flag != self.armed_flag): 
            rospy.logwarn('[DRIVER] BBoat should NOT be armed but is armed')

    def Mode_Service_callback(self, req):
        '''
            Sends mode on request
        '''
        resp = mode_servResponse(self.mode, self.auto_mission)
        return resp

    def Command_callback(self, msg): 
        '''
            Parse command msg
            Turn forward and turning speed to 1100 - 2000 values to override
        '''
        u1, u2 = msg.u1.data, msg.u2.data

        if abs(u1) > self.max_speed_fwrd:
            u1 = np.sign(u1)*self.max_speed_fwrd
        if abs(u2) > self.max_speed_turn:
            u2 = np.sign(u2)*self.max_speed_turn

        fwrd = int(1500+(u1/self.max_speed_fwrd)*500)
        turn = int(1500+(u2/self.max_speed_turn)*500)

        self.auto_cmd = np.array([[fwrd], [turn]])

    def Battery_callback(self, msg):
        '''
            Parse battery msg
        '''
        self.battery = msg.voltage

# Main function.
if __name__ == '__main__':
    rospy.init_node('driver')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        driver = DriverNode()
        driver.loop()
        os.system("rosrun mavros mavsafety disarm")
    except rospy.ROSInterruptException: pass
