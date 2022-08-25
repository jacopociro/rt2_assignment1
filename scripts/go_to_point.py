#! /usr/bin/env python

"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: go to point algorithm for the robot to reach the imposed goal.
    
.. moduleauthor:: Jacopo Ciro Soncini


This node is the node tasked with making sure that the robot reaches the goal, both in orientation and position.
The behaviour is handled with 3 states. The first one is tasked with aiming for the goal, using angular velocities. 
The second state makes sure the robot reaches the goal, giving linear velocities. The last state handles the orientation
given with the goal, using angular velocities. The angular and linear velocities are read on the /vel subscriber and sent 
to the simulation via /cmd_vel publisher. The goal is handled with an action. On the /target publisher I publish the number
of reached and aborted goal and, in case it is reached, the time to reach it.
 
Subscribes to:

/odom 

/vel 

Publishes to:

/cmd_vel 

/target

Service :

None

Action Service

/go_to_point

"""

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf import transformations
from rt2_assignment1.srv import Position
import math
import actionlib
import actionlib.msg
from rt2_assignment1 import msg
import time

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6


def clbk_odom(msg):    
    """
    
    The callback saves the current position and orientation of the robot on global variables, reading it from /odom.
    I needed to change the orientation from quaternion to euler.
    
    Args :
        Odometry
    
    Returns :
        None
        
    """
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):    
    """
    This function saves the passed parameter state to a global variable. Prints a message for 
    clear output.
    
    Args :
        state: Int
    
    Returns :
        None
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    """
    This function receives and angle, normalizes it in the interval [-pi, pi] and returns the
    new angle.
    
    Args :
        angle: Float
    
    Returns :
        Float
    """    
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """
 	This function calculates the yaw the robot needs to reach to face the goal and the publishes 
    the angular velocity to reach it. The algorithm is implemented with a treshold that needs to be 
    satisfied before changing state.
    
    Args :
    
        des_pos: array
    
    Returns :
    
        None
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = Velocity.angular.z 
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = Velocity.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = Velocity.angular.z
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """
 	This function publishes the velocity to reach the goal until the set treshold is satisfied.
    Then it changes the state.
    
    Args :
        des_pos: array
    
    Returns :
        None
        
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = Velocity.linear.x * err_pos
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = Velocity.linear.x

        twist_msg.angular.z = Velocity.angular.z * err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    """
 	This function publishes the velocity to reach the wanted orientation until the set treshold is satisfied.
    Then it changes the state.
    
    Args :
        des_yaw: Float
    
    Returns :
        None
    """
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = Velocity.angular.z 
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = Velocity.angular.z
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = Velocity.angular.z
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    """
 	This function is called when the robot has reached the goal and publishes all velocities
    as zero. 
    
    Args :
        None
    
    Returns :
        None
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)

def time_fun(t):
    """
 	This function publishes the parameter on /target. The parameter is either -1 if the goal is aborted
    or the time to reach the goal.
    
    Args :
        Float32
    
    Returns :
        None
    """
    pub_time.publish(t)

def go_to_point(req):
    """
 	This function is the callback for the action. It saves the goals to global variables. 
    It then enters a while loop where it handles the state and the various behaviours and the 
    eventual goal preemption.
    
    Args :
        goal: msg.MovAction
    
    Returns :
        Bool
    """
    # Timer
    timer_start = time.time()
    
    desired_position = Point()
    desired_position.x = req.target_pose.pose.position.x
    desired_position.y = req.target_pose.pose.position.y
    des_yaw = req.target_pose.pose.position.z
    change_state(0)
    while True:
        if action.is_preempt_requested():
            rospy.loginfo ("ERROR: ACTION PREEMEPTION")
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            pub_.publish(vel)

            t = time.time() - timer_start
            time_fun(-1)
            action.set_preempted()
            break
        elif state_ == 0:
            fix_yaw(desired_position)
        elif state_ == 1:
            go_straight_ahead(desired_position)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            t = time.time() - timer_start
            time_fun(t)
            action.set_succeeded()
            break

    return True

global Velocity
Velocity = Twist()
def clbk_vel(vel):
    """
 	This callback saves the velocited received on /vel on global variables that I will
    use where needed.
    
    Args :
        msg: Twist
    
    Returns :
        None
    """
    global Velocity
    Velocity.linear.x  = vel.linear.x
    Velocity.angular.z = vel.angular.z

    
def main():
    """
 	This is the main function. It initializes the node, the publisher, the subscribers and actionserver
    
    Args :
        None
    
    Returns :
        None
    """
    global pub_, action, pub_time
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_time = rospy.Publisher('/target', Float32, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_vel = rospy.Subscriber('/vel', Twist, clbk_vel)
    #service = rospy.Service('/go_to_point', Position, go_to_point)
    action = actionlib.SimpleActionServer('/go_to_point', msg.MovAction, go_to_point, auto_start=False)
    action.start()
    

    rospy.spin()

if __name__ == '__main__':
    main()
