#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
import turtlesim.srv
import random
from geometry_msgs.msg import Twist

class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_proprtional', anonymous=True)
        # Publisher for velocity
        self.velocity_publisher = rospy.Publisher('/turtle_spawn/cmd_vel',Twist, queue_size=10)

        #Subscriber to Pose
        self.pose_subscriber = rospy.Subscriber('/turtle_spawn/pose',Pose, self.pose_CB)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def pose_CB(self, data):

        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):

        return sqrt(pow((goal_pose.x - self.pose.x), 2) +pow((goal_pose.y - self.pose.y), 2))

    def linear_velocity(self, goal_pose):
        return self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_velocity(self, goal_pose):
        return (self.steering_angle(goal_pose) - self.pose.theta)


    #Spawn function
    def Turtle_spawn(self,location_x,location_y): 
        self.spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        self.spawner(location_x, location_y, 0, 'turtle_spawn') 

    def Turtle_kill(self,name): 
        self.killer=rospy.ServiceProxy('kill',turtlesim.srv.Kill)
        self.killer(name)



    def move2goal(self):

        self.Turtle_kill('turtle1')

        x = round(random.randrange(2,10),2)
        y = round(random.randrange(2,10),2)
        self.Turtle_spawn(x,y)
        goal_pose = Pose()


        goal_pose.x = rospy.get_param('turtlebot_proprtional/x')
        goal_pose.y = rospy.get_param('turtlebot_proprtional/y')

        vel_msg = Twist()

        while(True):


            # Linear velocity in the x-axis.
            vel_msg.linear.x = 1.5*self.linear_velocity(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 6*self.angular_velocity(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()
            if(self.linear_velocity(goal_pose) < 0.1):

                # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)


        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass