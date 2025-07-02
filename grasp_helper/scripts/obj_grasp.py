#! /usr/bin/env python3
#-*- coding: utf-8 -*-
import os
import tf
import sys
import rospy
import moveit_commander
from control_msgs.msg import GripperCommandActionGoal
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from yolov8_ros_msgs.msg import BoundingBoxes
from message_filters import ApproximateTimeSynchronizer, Subscriber
# from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class MoveRobot():
    def __init__(self):

        # 初始化 planning group
        self.robot = moveit_commander.robot.RobotCommander()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander(
            "arm")

        # 设置机械手臂的速度和加速度       速度0到1之间
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_max_velocity_scaling_factor(0.5)

        # 物体的位置
        self.Obj_pose = PoseStamped()
        self.Obj_pose.pose.position.x = 0
        self.find_enable = False
        self.Obj_class = ''
        self.add_collision_flag=False
        self.add_collision=''
        self.move_finish = False
        # # 物体吸附接口

        # self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
        #                                      Attach)
        # self.attach_srv.wait_for_service()

        # self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
        #                                      Attach)
        # self.detach_srv.wait_for_service()

        self.obj_pose_sub = Subscriber(
            "/objection_position_pose", Pose)
        
        self.yolo_sub = Subscriber("/yolov8/BoundingBoxes", BoundingBoxes)

		# Sync Subscribers
        self.ats = ApproximateTimeSynchronizer(
			[
				self.obj_pose_sub,
				self.yolo_sub
			],
			queue_size=5,
			slop=1,
			allow_headerless=True
		)

        self.ats.registerCallback(self.msg_filter_callback)
        self.current_pose_index = 0
        self.poses_array = []

        self.obj_pose_sub = rospy.Subscriber(
            "/transformed_poses_array", PoseArray, self.ObjectCallback)

    def msg_filter_callback(self, msg, yolo_msg):

        if self.find_enable:
            self.Obj_pose.pose = msg
            self.Obj_class = yolo_msg.bounding_boxes[0].Class

        if self.Obj_pose.pose.position.x != 0:
            self.find_enable = False

    def ObjectCallback(self, msg):
        # print(11)
        # 保存接收到的 PoseArray
        if self.find_enable:
            self.poses_array = msg.poses
            print(self.poses_array)

        if len(self.poses_array) > 0:
            self.find_enable=False
    def stop(self):
        moveit_commander.roscpp_initializer.roscpp_shutdown()



    def plan_cartesian_path(self, pose):
        """
            笛卡尔路径规划

            Parameters:
                pose - 目标pose

            Returns:
                None
            """
        waypoints = []
        waypoints.append(pose)

        # 设置机器臂当前的状态作为运动初始状态
        self.arm_group.set_start_state_to_current_state()

        # 计算轨迹
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,   # waypoint poses，路点列表，这里是5个点
            0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
            #0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
            False)        # avoid_collisions，避障规划

        self.arm_group.execute(plan, wait=True)

    def goSP(self):

        self.arm_group.set_joint_value_target([2.077,1.403, 1.625, 0.523, -1.419, -0.128 ])

        ###1.582, -0.0527,1.713,-0.15092505292892455, -0.15092505292892455, -1.57
        self.arm_group.go()
    def goSP2(self):

        self.arm_group.set_joint_value_target([3.02, -0.244,1.49,0, 1.34 ,-0.134 ])


        self.arm_group.go()
    def goSP3(self):

        self.arm_group.set_joint_value_target([1.545, -1.01,-1.74,-0.043, -0.92, -1.57  ])


        self.arm_group.go()
    def goSP4(self):

        self.arm_group.set_joint_value_target([0, 0.015,1.5,0, 1.22, 1.83  ])


        self.arm_group.go()
    def goSP5(self):

        self.arm_group.set_joint_value_target([0, 0.015,1.5,0, 1.22, -1.5355   ])


        self.arm_group.go()
    def goSP6(self):          

        self.arm_group.set_joint_value_target([1.71, 1.71,0.4366,0.17688, -0.52,-2 ])

        self.arm_group.go()
    def goSP7(self):          

        self.arm_group.set_joint_value_target([1.36, 1.59,0.69,0.0529, -0.55865,-1.36 ])
        self.arm_group.go()
    def goSP8(self):          

        self.arm_group.set_joint_value_target([1.04, 1.81,0.366,-0.89, -0.645,-0.73 ])

        self.arm_group.go()
    def goSP9(self):          

        self.arm_group.set_joint_value_target([1.27, 1.74,0.31,2.43,0.97,1.99 ])

        self.arm_group.go()
        if 0<=x0<=0.04:
           x1=x0+0.035
        elif 0.05<x0<=0.08:
           x1=x0+0.04
        elif 0.08<x0<=0.12:
           x1=x0+0.045
        elif x0>0.12:
           x1=x0+0.05
        else:
           x1=x0+0.05

    def grasp_obj1(self):

        print(self.Obj_pose)
        x1=self.Obj_pose.pose.position.x -0.05
        y1=self.Obj_pose.pose.position.y +0.2
        z1=self.Obj_pose.pose.position.z +0.05
        current_pose = self.arm_group.get_current_pose()
 
        x2=current_pose.pose.position.x
        y2=current_pose.pose.position.y
        z2=current_pose.pose.position.z
        self.arm_group.set_pose_target(self.Obj_pose.pose)
        self.plan_cartesian_path(self.Obj_pose.pose)        
        
        
        self.Obj_pose.pose.orientation = current_pose.pose.orientation

        self.Obj_pose.pose.position.x = x1
        self.Obj_pose.pose.position.y = y2
        self.Obj_pose.pose.position.z = z1
        self.arm_group.set_pose_target(self.Obj_pose.pose)
        self.plan_cartesian_path(self.Obj_pose.pose)         
        
        
        
        
        
        self.Obj_pose.pose.position.y = y1
        
        self.plan_cartesian_path(self.Obj_pose.pose)

 
    def grasp_obj(self):
 
        # 去到物体上方位置
       
        current_pose = self.arm_group.get_current_pose()
        self.Obj_pose.pose.orientation = current_pose.pose.orientation
        x0=self.Obj_pose.pose.position.x
        if 0<=x0<=0.05:
           x1=x0+0.035
        elif 0.05<x0<=0.08:
           x1=x0+0.04
        elif 0.08<x0<=0.12:
           x1=x0+0.045
        elif x0>0.12:
           x1=x0+0.05
        else:
           x1=x0+0.05
              
        x2=x1
        y2=self.Obj_pose.pose.position.y 
        z2=self.Obj_pose.pose.position.z 
        current_pose = self.arm_group.get_current_pose()
 
        x3=current_pose.pose.position.x
        y3=current_pose.pose.position.y
        z3=current_pose.pose.position.z
        
        self.Obj_pose.pose.position.x = x2 +0.08
        self.Obj_pose.pose.position.y = y3
        self.Obj_pose.pose.position.z = -0.02
        self.arm_group.set_pose_target(self.Obj_pose.pose)
        self.plan_cartesian_path(self.Obj_pose.pose)           

         
        self.Obj_pose.pose.position.y = y2-0.04
        
        self.plan_cartesian_path(self.Obj_pose.pose)
    def move_obj(self):

        # 去到物体上方位置

        current_pose = self.arm_group.get_current_pose()
        self.Obj_pose.pose.orientation = current_pose.pose.orientation



        self.Obj_pose.pose.position.z += 0.1

        self.plan_cartesian_path(self.Obj_pose.pose)
    def main_loop2(self):
        self.goSP()
        self.goSP2()
    def main_loop3(self):
        self.goSP()
    def main_loop(self):
       try:
            self.goSP()  # 1. 去到预抓取位置
            #os.system('python3 test1.py')
            self.find_enable = True
            rospy.sleep(1.5)  # 2. 识别当前的抓取位姿态(3s)
            if self.find_enable == False:
                print(self.poses_array)

                # 遍历 PoseArray 中的每个 Pose
                for pose in self.poses_array:
                    # 设置当前要抓取的物体位置
                    #print(pose_in_cam)
                    self.Obj_pose.pose = pose
                    # 执行抓取逻辑
                    self.grasp_obj()
                    #self.goSP()
                    os.system('cd /home/yhs/Desktop/rm_ws/src/grasp_helper/scripts && python3 test2.py')
                    #self.move_obj()
                    #self.goSP()
                    self.goSP2()
                    os.system('cd /home/yhs/Desktop/rm_ws/src/grasp_helper/scripts && python3 test1.py')
                    self.goSP()

#                self.move_finish = True
                    # 更新索引以指向下一个要抓取的物体
                    
                    #self.current_pose_index += 1

            else:
                rospy.logwarn('cant find object')
                #self.move_finish = True
                
       except Exception as e:
            rospy.logerr(str(e))

def main():
    rospy.init_node('grasp_demo', anonymous=True)
    rospy.loginfo('Start Grasp Demo')
    moverobot = MoveRobot()
    while(not rospy.is_shutdown() and not moverobot.move_finish):
        moverobot.main_loop()
        

if __name__ == "__main__":	

    main()
