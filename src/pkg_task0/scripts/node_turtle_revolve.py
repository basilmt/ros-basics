#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


VELOCITY = float(1)
RADIUS = float(1)
# mark -> Check if turtle has gone a particular distance
# To remove the false detection of initial to final
mark = False
# ranOnce -> Check if one revolution is completed
ranOnce = False

# publish to cmd/vel
velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()


def pose_callback(msg):

    global mark
    global ranOnce


    # Check if turtle has gone a particular distance
    if msg.theta > 1:
        mark = True

    # If one round completed, change linear and angular velocity to zero
    # make ranOnce to true
    if mark and not ranOnce and round(msg.theta,2) == 0 :
        global vel_msg
        global velocity_publisher
        global VELOCITY
        global RADIUS

        # GOAL REACHED
        ranOnce = True
        VELOCITY = 0
        vel_msg.linear.x = VELOCITY
        vel_msg.angular.z = VELOCITY/RADIUS

        velocity_publisher.publish(vel_msg)
        rospy.loginfo("goal reached")
        
    elif not ranOnce:
        rospy.loginfo("Moving in a circle")



def main():
    global velocity_publisher
    global vel_msg
    global ranOnce
    global VELOCITY
    global RADIUS

    rospy.init_node('turtle_revolve', anonymous=True)
    # Subscribe to pose
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    vel_msg.linear.x = VELOCITY
    vel_msg.angular.z = VELOCITY/RADIUS

    rate = rospy.Rate(10)

    # publish linear and angular velocities
    while not rospy.is_shutdown():
        # If one revolution completed
        # then break the while loop
        if ranOnce:
            break

        velocity_publisher.publish(vel_msg)
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass