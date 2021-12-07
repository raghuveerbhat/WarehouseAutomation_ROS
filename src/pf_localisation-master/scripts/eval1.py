#!/usr/bin/python3

import rospy
from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from tf.transformations import  euler_from_quaternion
import numpy as np
import time

class ParticleFilterLocalisationNodeEval(object):
    def __init__(self):
        
        self.rate=rospy.Rate(30)
        
        self.path_for_data_save = "/home/akshay/WarehouseAutomation_ROS/src/pf_localisation-master/scripts"


        # self.caminfo_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, callback=self.caminfo_callback)
        self.estimatedPose = None
        self.groundtruthPose = None
        self.pose_error = []
        self.Orientation_error = []

        self.pose_error1 = []
        self.Orientation_error1 = []
        self.pose_error2 = []
        self.Orientation_error2 = []
        self.pose_error3 = []
        self.Orientation_error3 = []
        self.pose_error4 = []
        self.Orientation_error4 = []
        self.pose_error5 = []
        self.Orientation_error5 = []
        self.pose_error6 = []
        self.Orientation_error6 = []
        self.start_time=0

        self.interval = []
        self.groundtruthList = []
        self.counter = 0
        self.landmark = [[1.4473416805267334, 0.41364067792892456, 0.0, 0.0, 0.0, -0.011503674603633503, 0.9999338305461085],
                            [1.0017170906066895, -1.3638901710510254, 0.0, 0.0, 0.0, 0.010521600528978107, 0.9999446464291454],
                            [0.9813948273658752, -3.1724913120269775, 0.0, 0.0, 0.0, 0.0013030230571348166, 0.999999151065096],
                            [0.9532657861709595, -4.968116283416748, 0.0, 0.0, 0.0, -0.0022372289058545302, 0.9999974974002799],      
                            [1.4260408878326416, -6.819857120513916, 0.0, 0.0, 0.0, -0.0015038960997956234, 0.9999988691476211],      
                            [1.5627377033233643, -8.728625297546387, 0.0, 0.0, 0.0, -0.02648123511897974, 0.9996493106017597]]
        self.main_process()


    

    def main_process(self):
        self.start_time = time.time() 
        while(rospy.has_param('/completed') and rospy.has_param('/path_id')):
            completed = rospy.get_param('/completed')
            path_id = rospy.get_param('/path_id')
            print(completed)
            print(path_id)

            if (completed == False): 
                              
                self.counter = self.counter+1
                rospy.sleep(1.0)
                self.estimatedPose = None
                self.groundtruthPose = None
                self.estimatedPose = rospy.wait_for_message("/estimatedpose",PoseStamped)
                if self.estimatedPose != None:
                    self.groundtruthPose = rospy.wait_for_message("/ground_truth/state",Odometry)
                    if self.groundtruthPose != None:
                        self.interval.append(self.counter)
                        print("EestimatedPose", self.estimatedPose)
                        print("ggroundtruthPose", self.groundtruthPose)
                        perror=round(((((self.estimatedPose.pose.position.x-self.groundtruthPose.pose.pose.position.x)**2)+((self.estimatedPose.pose.position.y-self.groundtruthPose.pose.pose.position.y)**2))/2),4)
                        #est_arr = [self.estimatedPose.pose.position.x,self.estimatedPose.pose.position.y,self.estimatedPose.pose.position.z,self.estimatedPose.pose.orientation.x,self.estimatedPose.pose.orientation.y,self.groundtruthPose.pose.pose.orientation.z,self.groundtruthPose.pose.pose.orientation.w]
                        self.pose_error.append(perror)
                        roll, pitch, yaw = euler_from_quaternion([self.estimatedPose.pose.orientation.x,self.estimatedPose.pose.orientation.y,self.estimatedPose.pose.orientation.z,self.estimatedPose.pose.orientation.w])
                        roll1, pitch1, yaw1 = euler_from_quaternion([self.groundtruthPose.pose.pose.orientation.x,self.groundtruthPose.pose.pose.orientation.y,self.groundtruthPose.pose.pose.orientation.z,self.groundtruthPose.pose.pose.orientation.w])
                        oerror=abs(round(((yaw-yaw1)),4))
                        self.Orientation_error.append(oerror)
                        if path_id==1:
                            roll1, pitch1, yaw1 = euler_from_quaternion([self.landmark[path_id-1][3],self.landmark[path_id-1][4],self.landmark[path_id-1][5],self.landmark[path_id-1][6]])
                            perror=round(((((self.estimatedPose.pose.position.x-self.landmark[path_id-1][0])**2)+((self.estimatedPose.pose.position.y-self.landmark[path_id-1][1])**2))/2),4)    
                            oerror=abs(round(((yaw-yaw1)),4))
                            print("1111111111")
                            self.pose_error1.append(perror)
                            self.Orientation_error1.append(oerror)
                        elif path_id==2:
                            roll1, pitch1, yaw1 = euler_from_quaternion([self.landmark[path_id-1][3],self.landmark[path_id-1][4],self.landmark[path_id-1][5],self.landmark[path_id-1][6]])
                            perror=round(((((self.estimatedPose.pose.position.x-self.landmark[path_id-1][0])**2)+((self.estimatedPose.pose.position.y-self.landmark[path_id-1][1])**2))/2),4)  
                            oerror=abs(round(((yaw-yaw1)),4))
                            print("2222222222")
                            self.pose_error2.append(perror)
                            self.Orientation_error2.append(oerror)
                        elif path_id==3:
                            roll1, pitch1, yaw1 = euler_from_quaternion([self.landmark[path_id-1][3],self.landmark[path_id-1][4],self.landmark[path_id-1][5],self.landmark[path_id-1][6]])
                            perror=round(((((self.estimatedPose.pose.position.x-self.landmark[path_id-1][0])**2)+((self.estimatedPose.pose.position.y-self.landmark[path_id-1][1])**2))/2),4)      
                            oerror=abs(round(((yaw-yaw1)),4))
                            print("33333333")
                            self.pose_error3.append(perror)
                            self.Orientation_error3.append(oerror)
                        elif path_id==4:
                            roll1, pitch1, yaw1 = euler_from_quaternion([self.landmark[path_id-1][3],self.landmark[path_id-1][4],self.landmark[path_id-1][5],self.landmark[path_id-1][6]])
                            perror=round(((((self.estimatedPose.pose.position.x-self.landmark[path_id-1][0])**2)+((self.estimatedPose.pose.position.y-self.landmark[path_id-1][1])**2))/2),4)    
                            oerror=abs(round(((yaw-yaw1)),4))
                            print("4444444")
                            self.pose_error4.append(perror)
                            self.Orientation_error4.append(oerror)

                        elif path_id==5:
                            roll1, pitch1, yaw1 = euler_from_quaternion([self.landmark[path_id-1][3],self.landmark[path_id-1][4],self.landmark[path_id-1][5],self.landmark[path_id-1][6]])
                            perror=round(((((self.estimatedPose.pose.position.x-self.landmark[path_id-1][0])**2)+((self.estimatedPose.pose.position.y-self.landmark[path_id-1][1])**2))/2),4)   
                            oerror=abs(round(((yaw-yaw1)),4))
                            print("55555555")
                            self.pose_error5.append(perror)
                            self.Orientation_error5.append(oerror)
                        elif path_id==6:
                            roll1, pitch1, yaw1 = euler_from_quaternion([self.landmark[path_id-1][3],self.landmark[path_id-1][4],self.landmark[path_id-1][5],self.landmark[path_id-1][6]])
                            perror=round(((((self.estimatedPose.pose.position.x-self.landmark[path_id-1][0])**2)+((self.estimatedPose.pose.position.y-self.landmark[path_id-1][1])**2))/2),4)      
                            oerror=abs(round(((yaw-yaw1)),4))
                            print("6666666666")
                            self.pose_error6.append(perror)
                            self.Orientation_error6.append(oerror)
            else :
                break       
            self.rate.sleep()
        self.save_data(self.pose_error1,self.Orientation_error1,"1")
        self.save_data(self.pose_error2,self.Orientation_error2,"2")
        self.save_data(self.pose_error3,self.Orientation_error3,"3")
        self.save_data(self.pose_error4,self.Orientation_error4,"4")
        self.save_data(self.pose_error5,self.Orientation_error5,"5")
        self.save_data(self.pose_error6,self.Orientation_error6,"6")
        self.save_data(self.pose_error,self.Orientation_error,"")
        self.save_graph(self.pose_error,self.Orientation_error,"")
        with open(self.path_for_data_save+'/time_taken.npy', 'wb') as f:
            np.save(f, time.time()-self.start_time)
        
    
    def save_data(self,perror,oerror,flag):
        
        with open(self.path_for_data_save+'/pose'+flag+'.npy', 'wb') as f:
            np.save(f, perror)
        with open(self.path_for_data_save+'/orien'+flag+'.npy', 'wb') as f:
            np.save(f, oerror)                 
        
            
    def save_graph(self,perror,oerror,flag):
        plt.figure()
        plt.errorbar(self.interval,perror,fmt='b-',label="pose_error")
        plt.xlabel('Interval per iteration')
        plt.ylabel('RMS Error')
        plt.tight_layout()
        plt.legend()
        plt.title('pose_error'+flag+' - EstimatedPose')
        plt.savefig("pose_error"+flag+".png")
        plt.show()
        plt.figure()
        plt.errorbar(self.interval,oerror,fmt='r-',label="Orientation_error")
        plt.xlabel('Interval per iteration')
        plt.ylabel('RMS Error')
        plt.tight_layout()
        plt.legend()
        plt.title('Orientation_error'+flag+' - EstimatedPose')
        plt.savefig("orien_error"+flag+".png")
        plt.show()

            
if __name__ == '__main__':

    rospy.init_node("pf_localisation")
    node = ParticleFilterLocalisationNodeEval()


