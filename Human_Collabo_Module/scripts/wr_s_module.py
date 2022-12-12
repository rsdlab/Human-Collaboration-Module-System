#!/usr/bin/env python
# coding: UTF-8
#####################################################################################################################
#このノードは，ArUco_ROSから得られるノードを人協働マニピュレーションモジュールに与えるためのブリッジモジュールです．#
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2022/12/09
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#
import rospy
#サービス通信を行うためのファイル
from human_collaboration.srv  import work_recognition, work_recognitionResponse
#クラスで実装するため
from rospy.topics import Subscriber
#データを受け取る型
from geometry_msgs.msg import  PoseStamped

#クラスの実装
class WorkRecogSubSystem:
    
    #ArUco_ROSから発行されているトピックを受け取る　 
    def __init__(self):
        self.aruco_sub = Subscriber('aruco_single/pose', PoseStamped, self.ArucoCallback)
        self.current_pose_ = PoseStamped()
    
    #コールバックでデータを受け取る
    def ArucoCallback(self,data):
        self.current_pose_ = data
        print(self.current_pose_)

    #人協働マニピュレーションモジュール側にマーカから取得した位置・姿勢を送る
    def on_request(self, req):
        print("Start work recognition!!")
        
        srv = work_recognitionResponse()
        srv.pose = self.current_pose_
        return srv

#サービス通信を行うための処理
if __name__ == "__main__":
    rospy.init_node("wr_s_module")

    wrss = WorkRecogSubSystem()

    rospy.Service("wr_service", work_recognition, wrss.on_request)
    print("send to cordinate_data")
    rospy.spin()

