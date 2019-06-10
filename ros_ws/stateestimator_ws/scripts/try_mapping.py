#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
from math import pi


def main():
    pub_rate=10

    vx1=0.7
    yaw1=-1.0
    time1=3*pi/2

    vx2=1.0
    yaw2=0
    time2=2.0

    

    
    runtime=15.0
    totaltime=2*(time1+time2)

    reset_pose=rospy.Publisher('/rover/reset_pose', Pose, queue_size=10)
    reset_twist=rospy.Publisher('/rover/reset_twist', Twist, queue_size=10)
    cmd_pub=rospy.Publisher('rover/cmd_vel', TwistStamped, queue_size=10)
    
    reset_msg_pose=Pose()
    reset_msg_pose.position.x=0.0
    reset_msg_pose.position.y=0.0
    reset_msg_pose.orientation.w=1.0

    reset_msg_twist=Twist()
    reset_msg_twist.linear.x=0.0
    reset_msg_twist.angular.z=0.0
    
    start = rospy.get_rostime()
    r = rospy.Rate(pub_rate)
    while (rospy.get_rostime()-start).to_sec()<0.5:
        reset_pose.publish(reset_msg_pose)
        reset_twist.publish(reset_msg_twist)
        r.sleep()
    msg=TwistStamped()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'cmd'
    msg.twist.linear.x=0.0
    msg.twist.angular.z=0.0
    cmd_pub.publish(msg)
    
    r = rospy.Rate(pub_rate)
    start = rospy.get_rostime()

    while not rospy.is_shutdown():
        curtime = (rospy.get_rostime()-start).to_sec()
        if curtime >runtime:
            msg=TwistStamped()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'cmd'
            msg.twist.linear.x=0.0
            msg.twist.angular.z=0.0
            cmd_pub.publish(msg)
            rospy.signal_shutdown("Done")
            break
        else:
            loopstart=(rospy.Time.now()).to_sec()
            looptime=(rospy.Time.now()).to_sec()-loopstart
            while looptime<totaltime and not rospy.is_shutdown():
                looptime=(rospy.Time.now()).to_sec()-loopstart
                if looptime>( 2*time1+time2):
                    msg=TwistStamped()
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = 'cmd'
                    msg.twist.linear.x=vx2
                    msg.twist.angular.z=yaw2
                    print 'straight2'
                    cmd_pub.publish(msg)
                elif looptime>(time1+time2):
                    msg=TwistStamped()
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = 'cmd'
                    msg.twist.linear.x=vx1
                    msg.twist.angular.z=-yaw1
                    print 'turn2'
                    cmd_pub.publish(msg)
                elif looptime>time1:
                    msg=TwistStamped()
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = 'cmd'
                    msg.twist.linear.x=vx2
                    msg.twist.angular.z=yaw2
                    print 'straight1'
                    cmd_pub.publish(msg)
                else:
                    msg=TwistStamped()
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = 'cmd'
                    msg.twist.linear.x=vx1
                    msg.twist.angular.z=yaw1
                    print 'turn1'
                    cmd_pub.publish(msg) 
                r.sleep()

if __name__ == '__main__':
    rospy.init_node('pololu_action_server')
    main()
    rospy.spin()
