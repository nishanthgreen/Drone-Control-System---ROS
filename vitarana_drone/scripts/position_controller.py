#!/usr/bin/env python


# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import time



class Hover() :
    """docstring for Hover"""   
    def __init__(self):
         rospy.init_node('position_controller')  # initializing ros node with name drone_control
         self.i = 0
         self.j = 0
         self.stop = 0
         self.count=0
         self.settle=0
         self.set_point = [0.0,0.0,0.0]
         self.Gps  = [0.0, 0.0, 0.0]   # latitude, logitude, altitude
         self.Kp = [10000000, 0.0, 140.0]   # kp, ki, kd
         self.Ki = [0.0, 0.0, 0.008]
         self.Kd = [250000000,0.0, 1800.0]
         self.err_i  = [0.0, 0.0, 0.0]
         self.prev   = [0.0, 0.0, 0.0]
         self.err    = [0.0, 0.0, 0.0]
         self.err_d   =[0.0, 0.0, 0.0]
         self.lat =[19.0,19.0000451704]
         self.alt =[3.0 , 0.31]
         # drone command values 1000 - 2000
         self.set = edrone_cmd()
         self.set.rcRoll     = 1500
         self.set.rcPitch    = 1500
         self.set.rcYaw      = 1500
         self.set.rcThrottle = 1500
         # throttle error
         self.err_datat = Float32()
         self.err_datar = Float32()
         self.err_datap = Float32()
         # publishers
         self.set_pub            = rospy.Publisher('/drone_command',edrone_cmd,queue_size=1)
         self.throttle_error_pub = rospy.Publisher('/throttle_error', Float32, queue_size=1)
         self.roll_error_pub     = rospy.Publisher('/roll_err', Float32, queue_size=1)
         self.pitch_error_pub    = rospy.Publisher('/pitch_err', Float32, queue_size=1)
         # subscribers
         rospy.Subscriber('/edrone/gps',NavSatFix, self.gps)
         rospy.Subscriber('/pid_tuning_altitude',PidTune,self.throttle_set_pid)
         rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
         rospy.Subscriber('/pid_tuning_pitch',PidTune, self.pitch_set_pid)


    def gps(self,msg):
         
         self.Gps[0] = msg.latitude
         self.Gps[1] = msg.longitude
         self.Gps[2] = msg.altitude
    
    def throttle_set_pid(self, throttle):
        self.Kp[2] = throttle.Kp * 0.1
        self.Ki[2] = throttle.Ki * 0.008
        self.Kd[2] = throttle.Kd * 0.6    

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 20000 # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.01
        self.Kd[0] = roll.Kd * 50000

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.02
        self.Ki[1] = pitch.Ki * 0.0002
        self.Kd[1] = pitch.Kd * 0.0002    

    def pid(self):
        
        # error
        self.err[0] = self.lat[self.i]          - self.Gps[0]
        self.err[1] = 72.0                      - self.Gps[1]
        self.err[2] = self.alt[self.j]          - self.Gps[2]
        
        # error sum
        self.err_i[0] += self.err[0]
        self.err_i[1] += self.err[1]
        self.err_i[2] += self.err[2]
        # error diff
        self.err_d[0] = self.err[0] - self.prev[0]
        self.err_d[1] = self.err[1] - self.prev[1]
        self.err_d[2] = self.err[2] - self.prev[2]
        # prev error
        self.prev[0] = self.err[0]
        self.prev[1] = self.err[1]
        self.prev[2] = self.err[2]
        # output
        self.out_throttle = self.Kp[2] * self.err[2] + self.err_i[2] * self.Ki[2] + self.err_d[2] * self.Kd[2]
        self.out_roll     =( self.err[0] * self.Kp[0]  + self.err_i[0] * self.Ki[0] + self.err_d[0] * self.Kd[0]) * self.settle
        self.out_pitch    = self.err[1] * self.Kp[1]  + self.err_i[1] * self.Ki[1] + self.err_d[1] * self.Kd[1]
        # hovering control
        self.set.rcThrottle = 1500 + self.out_throttle
        self.set.rcRoll     = 1500 + self.out_roll
        self.set.rcPitch    = 1500 + self.out_pitch
        
      #  self.set.rcRoll     = 1500 + self.out_roll
       # self.set.rcPitch    = 1500 + self.out_pitch
        
         #       self.stop = 1
        if self.Gps[2] >= 2.80 :
           self.i = 1 
           self.settle=1
           
        if self.Gps[0] >= 19.0000450000 :
               
              rospy.loginfo("down")
              self.count += 1

        if self.count >= 10:
         self.j = 1
         if self.Gps[0] >= 19.000044:
             rospy.loginfo("mangatha da")
             if self.Gps[2] <= 0.34:
                rospy.loginfo("stoped")
                self.set.rcThrottle = 1000
                self.stop = 1          
        #limit values
        if self.set.rcThrottle > 2000 :
               self.set.rcThrottle = 2000
        if self.set.rcRoll > 2000:
               self.set.rcRoll = 2000
        if self.set.rcPitch > 2000:
               self.set.rcPitch = 2000
        #publish
        self.err_datat.data = self.out_throttle
        self.throttle_error_pub.publish(self.err_datat)
        self.err_datar.data = self.out_roll
        self.roll_error_pub.publish(self.err_datar)
        self.err_datap.data = self.out_pitch
        self.pitch_error_pub.publish(self.err_datap)
        self.set_pub.publish(self.set)


if __name__ == '__main__':
   try:
    pos = Hover()
    r = rospy.Rate(11.1111111)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        pos.pid()
        r.sleep()
        if pos.stop == 1:
            break
   except rospy.ROSInterruptException:
           pass

