#!/usr/bin/env python
# coding: UTF-8
#!/usr/bin/env python
# coding: UTF-8
#####################################################################################################################
#このノードは．人協働マニピュレーションモジュールを機能単位で作成したモジュールです．　　　　　　　　　　　　　　　 #
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2022/12/09
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#
#python,TFのライブラリを使用
import rospy, tf
#サービス通信で使用
from human_collaboration.srv  import work_recognition
from rospy.topics import Subscriber
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import  PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import moveit_commander

#オイラー角からクォータニオンへの変換
def euler_to_quaternion(role, pitch, yaw):
	q = quaternion_from_euler(role, pitch, yaw)
	return Quaternion(q[0], q[1], q[2], q[3])

class WorkRecognitionSystem():
    #型の定義と座標変換を行うためのListnerを用意しておく
    def __init__(self):
        self.reqest = PoseStamped()
        self.listener = tf.TransformListener()

    #ワークの位置姿勢を獲得する場所
    def WorkPoseRecognition(self):

        #ワーク検出サブシステムが起動するまで，待機する
        rospy.wait_for_service("wr_service")

        try:

            #サービスプロキシーの作成を行う
            proxy = rospy.ServiceProxy("wr_service", work_recognition)

            #receive  a reqest and make my object
            self.reqest = proxy()
            rospy.loginfo(self.reqest)
                    # print(self.reqest)
            br = tf.TransformBroadcaster()
            br.sendTransform((self.reqest.pose.pose.position.x, self.reqest.pose.pose.position.y, self.reqest.pose.pose.position.z),
                            (self.reqest.pose.pose.orientation.x, self.reqest.pose.pose.orientation.y, self.reqest.pose.pose.orientation.z,self.reqest.pose.pose.orientation.w),
                            rospy.Time.now(),
                            "marker_id_10",
                            "cam_frame")
            
            #値の出力
            rospy.loginfo("Result : " + str(self.reqest))


            print("arm_aruco")
            #マニピュレーションの設定
           
            robot = moveit_commander.RobotCommander()
            print("robot group:", robot.get_group_names())
            print ("")
        
            #グループの名前
            arm = moveit_commander.MoveGroupCommander("motoman_gp8")
            #ジョイントの名前
            print ("")
            print("arm joint name:", robot.get_joint_names("motoman_gp8"))
        
            arm.set_pose_reference_frame("base_link")
        
            print("Start Operations!")
            
            #ロボットのbase_linkから対象物を座標変換
            try:
                self.listener.waitForTransform('base_link', 'marker_id_10', rospy.Time(0),rospy.Duration(3.0))
                (trans,rot) = self.listener.lookupTransform('base_link', 'marker_id_10', rospy.Time(0))
            except (rospy.ROSInterruptException,tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("could not transform...")
            
            # ホームポーズへと移動
            rospy.loginfo("Home Pose")
            print("arm current pose states:", arm.get_current_pose().pose)
            joint_goal = arm.get_current_joint_values()
            joint_goal[0] = 0.0
            joint_goal[1] = 0.0
            joint_goal[2] = 0.0
            joint_goal[3] = 0.0
            joint_goal[4] = -1.5708
            joint_goal[5] = 0.0
            arm.go(joint_goal, wait=True)
            print ("Goal Home Pose")
            print ("      ")
        
            #ピック
            rospy.loginfo("Pick!!")
            print("arm current pose states:", arm.get_current_pose().pose)
            q = tf.transformations.quaternion_from_euler(0.0, 3.14, -1.57)
            aruco_pose = PoseStamped()
            aruco_pose.header.frame_id = "base_link"
            aruco_pose.pose.position.x = trans[0]
            aruco_pose.pose.position.y = trans[1]
            aruco_pose.pose.position.z = trans[2] + 0.1
            aruco_pose.pose.orientation.x = q[0]
            aruco_pose.pose.orientation.y = q[1]
            aruco_pose.pose.orientation.z = q[2]
            aruco_pose.pose.orientation.w = q[3]
            arm.set_joint_value_target(aruco_pose, True)
            arm.go(aruco_pose,wait=True)
            rospy.loginfo("Goal Pick Pose")
            print ("        ")
            
            # プレース
            rospy.loginfo("place")
            print("arm current pose states:", arm.get_current_pose().pose)
            joint_goal = arm.get_current_joint_values()
            joint_goal[0] = 1.5708
            joint_goal[1] =0.523599
            joint_goal[2] = 0.0
            joint_goal[3] = 0.0
            joint_goal[4] = -1.0472
            joint_goal[5] = 0.0
            arm.go(joint_goal, wait=True)
            print ("Goal place")
            print ("      ")
            
        except rospy.ServiceException as e:
            rospy.loginfo("ServiceException : %s" % e)



if __name__ == "__main__":
    rospy.init_node("HumanCollaboManipulation")
    wrs = WorkRecognitionSystem()

    wrs.WorkPoseRecognition()
