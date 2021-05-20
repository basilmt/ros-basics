#!/usr/bin/env python

'''
This python file runs a ROS-node of name position_controller which controls the lat, lon, ht of the eDrone.
This node publishes and subsribes the following topics:
    PUBLICATIONS				SUBSCRIPTIONS
    /edrone/drone_command		/edrone/gps
    /z_error                    /pid_tuning_altitude
'''


from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32,String
from sensor_msgs.msg import NavSatFix
import rospy


class Position():
    """
    docstring for Position
    """
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_controller

        self.drone_pos_gps = [0.0, 0.0, 0.0] #lat,lon,ht
        # self.setpoint_cmd = [[19.0, 72.0, 3.0], [19.0000451704 , 72.0, 3.0], [19.0000451704 , 72.0, 0.31]]
        # self.setpoint_cmd = [[19.0, 72.0, 3.0], [19.0000451704 , 72.0, 3.0]]
        # self.setpoint_gps = [19.0, 72.0, 0.31]
        # self.setpoint_gps = [19.0, 72.0, 3.0]
        # self.setpoint_cmd = [[19.0007046575, 71.9998955286, 22.1599967919]]
        self.setpoint_cmd = []
        self.setpoint_gps = [19.0007046575, 71.9998955286, 22.1599967919]
         

        self.winner = False # just for log purposes, No other uses, yet. 
        self.barcode_data_read = False
        self.setpoint_change_for_qr_read = False
        self.grabbed_package = False


        self.min_value = [-90.0, -180.0, 0 ]
        self.max_value = [90.0, 180.0, 1000] # Let's just say max ht is 1000m. That's pretty high,I guess.

        self.prev_error = [0,0,0] # lat, lon ht
        self.Iterm = [0,0,0] # lat, lon, ht

        # stage 1
        # self.Kp = [5500*1000,5000*1000*2,175*0.1]
        # self.Ki = [0.9, 0.0, 96*0.01]
        # self.Kd = [1350*1000000,1419*1000000*2,5000*1.2]

        self.Kp = [5500*1000,5000*1000*2,175*0.1]
        self.Ki = [0.9, 0.9, 96*0.01]
        self.Kd = [1350*1000000,1350*1000000*2,5000*1.2]

        self.sample_time = 0.025

        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcRoll = 0.0
        self.drone_cmd.rcPitch = 0.0
        self.drone_cmd.rcYaw = 0.0
        self.drone_cmd.rcThrottle = 0.0

        self.drone_cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        self.lat_error = Float32()
        self.lat_error.data = 0.0
        self.lat_error_pub = rospy.Publisher('/lat_error',Float32,queue_size=1)

        self.lon_error = Float32()
        self.lon_error.data = 0.0
        self.lon_error_pub = rospy.Publisher('/lon_error',Float32,queue_size=1)

        self.z_error = Float32()
        self.z_error.data = 0.0
        self.z_error_pub = rospy.Publisher('/z_error',Float32,queue_size=1)

        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.lat_set_pid)
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.lon_set_pid)
        # rospy.Subscriber('/pid_tuning_altitude', PidTune, self.z_set_pid)


        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_pos_callback)
        rospy.Subscriber('/barcode', String, self.barcode_callback)



    def lat_set_pid(self,lat):
        self.Kp[0] = lat.Kp*1000
        self.Ki[0] = lat.Ki*0.01
        self.Kd[0] = lat.Kd*1000000

    def lon_set_pid(self,lon):
        self.Kp[1] = lon.Kp*0.01
        self.Ki[1] = lon.Ki*0.01
        self.Kd[1] = lon.Kd*0.01

    def z_set_pid(self,z_msg):
        self.Kp[2] = z_msg.Kp*0.1
        self.Ki[2] = z_msg.Ki*0.01
        self.Kd[2] = z_msg.Kd*1.1

    def gps_pos_callback(self,coordinate):
        self.drone_pos_gps[0] = coordinate.latitude
        self.drone_pos_gps[1] = coordinate.longitude
        self.drone_pos_gps[2] = coordinate.altitude

    def barcode_callback(self, data):
        if not self.barcode_data_read:
            rospy.loginfo(data)
            self.barcode_data_read = True
            # decode string to float[]
            self.setpoint_cmd.append(map(float,barcode.data.split(","))) 


    def posPid(self):

        if not self.setpoint_change_for_qr_read:
            self.setpoint_change_for_qr_read = True
            tempCord = self.setpoint_gps
            self.setpoint_cmd.insert(0,list(self.setpoint_gps))
            self.setpoint_gps[2] = self.setpoint_gps[2] + 1

        # clean the set point data
        if self.setpoint_gps[0] < self.min_value[0]:
            self.setpoint_gps[0] = self.min_value[0]
        elif self.setpoint_gps[0] > self.max_value[0]:
            self.setpoint_gps[0] = self.max_value[0]

        if self.setpoint_gps[1] < self.min_value[1]:
            self.setpoint_gps[1] = self.min_value[1]
        elif self.setpoint_gps[1] > self.max_value[1]:
            self.setpoint_gps[1] = self.max_value[1]

        if self.setpoint_gps[2] < self.min_value[2]:
            self.setpoint_gps[2] = self.min_value[2]
        elif self.setpoint_gps[2] > self.max_value[2]:
            self.setpoint_gps[2] = self.max_value[2]


        error = [0.0, 0.0, 0.0] #lat, lon, ht
        error[0] = self.setpoint_gps[0] - self.drone_pos_gps[0]
        error[1] = self.setpoint_gps[1] - self.drone_pos_gps[1]
        error[2] = self.setpoint_gps[2] - self.drone_pos_gps[2]

        
        self.Iterm[0] = (self.Iterm[0] + error[0]) * self.Ki[0]
        self.Iterm[1] = (self.Iterm[1] + error[1]) * self.Ki[1]
        self.Iterm[2] = (self.Iterm[2] + error[2]) * self.Ki[2]

        output = [0.0,0.0,0.0]

        output[0] = self.Kp[0] * error[0] + self.Iterm[0] + self.Kd[0] * (error[0] - self.prev_error[0])
        output[1] = self.Kp[1] * error[1] + self.Iterm[1] + self.Kd[1] * (error[1] - self.prev_error[1])
        output[2] = self.Kp[2] * error[2] + self.Iterm[2] + self.Kd[2] * (error[2] - self.prev_error[2])

        self.prev_error[0] = error[0]
        self.prev_error[1] = error[1]
        self.prev_error[2] = error[2]
    
        self.drone_cmd.rcRoll = 1500 + output[0]
        self.drone_cmd.rcPitch = 1500 + output[1]
        self.drone_cmd.rcYaw = 1500
        self.drone_cmd.rcThrottle = 1500 + output[2]

        self.lat_error.data = error[0]
        self.lon_error.data = error[1]
        self.z_error.data = error[2]


        self.lat_error_pub.publish(self.lat_error)
        self.lon_error_pub.publish(self.lon_error)
        self.z_error_pub.publish(self.z_error)

        if self.collision_detect():
            self.drone_cmd.rcRoll = 1500
            self.drone_cmd.rcPitch = 1450 # change pitch to pitch backwards
            self.drone_cmd.rcYaw = 1500
            self.drone_cmd.rcThrottle = 1500 + output[2]

        self.drone_cmd_pub.publish(self.drone_cmd)

        if self.isSetPoint(error):
            if self.barcode_data_read and len(self.setpoint_cmd) <= 2 and not self.grabbed_package:
                self.grabbed_package = True

                # gripper service

            elif self.setpoint_cmd:
                rospy.loginfo("yeay "+ str(len(self.setpoint_cmd)) + " more to go")
                self.setpoint_gps = self.setpoint_cmd.pop(0)
                self.initialise()
            elif not self.winner:
                self.winner = True
                rospy.loginfo("Completed")


        rospy.loginfo(self.setpoint_cmd)
        rospy.loginfo(self.setpoint_gps)


    def isSetPoint(self,err):
        lat_err = 0.000004517
        lon_err = 0.0000047487
        ht_err = 0.2

        if abs(err[0]) < lat_err and abs(err[1]) < lon_err and abs(err[2]) < ht_err:
            return True
        else:
            return False


    def collision_detect(self):
        # check for collisions ande return a bool
        return False
        self.initialise()

    def initialise(self):
        # making sure ki doesn't add up a lot
        self.Iterm = [0,0,0]



if __name__ == "__main__":
    pos = Position()
    r = rospy.Rate(1/pos.sample_time)
    while not rospy.is_shutdown():
        pos.posPid()
        r.sleep()
        


