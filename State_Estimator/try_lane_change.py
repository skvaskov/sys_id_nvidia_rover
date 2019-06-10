#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from rover_controller.msg import LaneChangeParameters as LCP
from nav_msgs.msg import Odometry
from tf import transformations as TF
from std_msgs.msg import Header

def update_pose(data):
    #:param data: a State message describing the current state
    #:updates current_state: global variable containing [x,y,psi,w,vx]
    #:updates steering_input: recommended steering input to pololu command
    #:updates throttle_input: recommended throttle input to pololu command
    global current_pose
    global pose_flag
    current_pose=data.pose.pose
    pose_flag=True

def definition():
    global current_pose
    global pose_flag

    odom_quat = TF.quaternion_from_euler(0, 0, 0)
    current_pose = Pose(Point(0.0, 0.0, 0.), Quaternion(*odom_quat))
    pose_flag=False

def main():
    global current_pose
    global pose_flag

    cmd_pub=rospy.Publisher('rover/lane_change_cmd', LCP, queue_size=10)
    rospy.init_node('lane_change_node')
    #sub = rospy.Subscriber('mocap', PoseStamped, update_pose)
    sub =rospy.Subscriber('rover/odometry',Odometry,update_pose)

    msg = LCP()
    msg.b=0.0
    msg.V=1.0
    msg.Th=2
    msg.T=2
    msg.stop_time=1.375
    msg.stop_flag=False
    msg.road_heading=0.0
    t_move=1.375
    while not pose_flag:
        rospy.sleep(0.1)

    print("started")
    msg.initial_pose=current_pose
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    cmd_pub.publish(msg)
    rospy.sleep(t_move)

    msg.initial_pose=current_pose
    msg.b=1.0
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    cmd_pub.publish(msg)
    rospy.sleep(t_move)

    msg.initial_pose=current_pose
    msg.b=0.0
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    cmd_pub.publish(msg)
    rospy.sleep(t_move)

    msg.stop_flag=True
    cmd_pub.publish(msg)

    print("stopped")



if __name__ == '__main__':

    definition()
    main()
    rospy.spin()
