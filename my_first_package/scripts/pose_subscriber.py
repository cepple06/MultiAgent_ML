#! /usr/bin/env python3

# the main module for this program to interact with ROS
import rospy
# the message type for reading the controller 
from sensor_msgs.msg import Joy, Range, Image
from std_msgs.msg import String
# the message type for publishing(i.e sending) velocity commands to the robot
from geometry_msgs.msg import Twist
import message_filters
import pandas as pd
# import os.path
import os, os.path
from cv_bridge import CvBridge
from datetime import datetime
import cv2

bridge = CvBridge()
class save():
    def __init__(self):

        self.rate        = rospy.Rate(10)
        self.robot_name  = rospy.get_param("robot")
        
        self.sample_rate = rospy.get_param("sample_delta_time")
        self.prev_time = datetime.now()
        self.curr_time = 0
        # self.cwd = os.getcwd()

        self.start_demo = rospy.get_param("demo_collect_start")
        
        self.image_sub       = message_filters.Subscriber("/"+self.robot_name+"/laser_camera/image", Image)
        self.lidar_image_sub = message_filters.Subscriber("/"+self.robot_name+"/surface_normal_image", Image)
        self.laser_range_sub = message_filters.Subscriber("/"+self.robot_name+"/"+self.robot_name+"/laser_range", Range)
        self.velocity_sub    = message_filters.Subscriber("/"+self.robot_name+"/cmd_vel", Twist)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.laser_range_sub, self.lidar_image_sub, self.image_sub, self.velocity_sub],1,1, True)
        self.ts.registerCallback(self._callback)
        
        rollout_labels = ["laser_range_data", "lidar_path", "image_path", "lin_vel", "ang_vel"]#]#
        self.rollout_df = self.setup_df(rollout_labels)

        self.image_path=rospy.get_param("image_save_file_path")
        self.Lidar_image_path=rospy.get_param("Lidar_save_file_path")
        self.image_count = 0
        
        self.rollout_images = {}
        self.rollout_lidar_images = {}


        self.rollout_df_filename = rospy.get_param("Demo_data_path") 
        self.rollout_count = 0 
        self.flag = True
        
        if os.path.exists(self.rollout_df_filename):
            if len([name for name in os.listdir(self.rollout_df_filename) if os.path.isfile( os.path.join(self.rollout_df_filename, name))]):
                self.rollout_count = len([name for name in os.listdir(self.rollout_df_filename) if os.path.isfile( os.path.join(self.rollout_df_filename, name))]) +1
                print( self.rollout_df_filename+"demo_"+str(self.rollout_count)+".csv")
                print (len([name for name in os.listdir(self.rollout_df_filename) if os.path.isfile( os.path.join(self.rollout_df_filename, name))]) )
            

    def setup_df(self, rollout_labels):
        return pd.DataFrame(data=None, columns = rollout_labels)
    
    def _callback(self, range_msg, lidar, image, velocity):

        self.curr_time = datetime.now()
        if (self.curr_time - self.prev_time).total_seconds() >= self.sample_rate and self.start_demo:
            self.prev_time  = self.curr_time

            range_laser = 0
            if(range_msg.range == 0):
                range_laser=999#100.0
            else:
                range_laser = str(round(range_msg.range,3))
            
            self.complile_data(range_laser, 
                            bridge.imgmsg_to_cv2(lidar, "bgr8"), 
                            bridge.imgmsg_to_cv2(image, "bgr8"),
                            velocity.linear.x, 
                            velocity.angular.z)
            
            print("image_count: ",self.image_count)
        # self.rollout.append({  "lidar_path": None, "image_path": None, "lin_vel": None, "ang_vel": None}, ignore_index=True)

    def complile_data(self, laser_range, lidar_image, image, lin_vel, ang_vel):   

        self.rollout_images[self.image_path+"demo_"+str(self.rollout_count)+"_raw_image_"+str(self.image_count)+'.jpeg'] = image
        self.rollout_lidar_images[self.Lidar_image_path+"demo_"+str(self.rollout_count)+"_lidar_image_"+str(self.image_count)+'.jpeg'] = lidar_image
        # cv2.imshow("file_name", image)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     pass
        new_row_df = pd.DataFrame([{"laser_range_data": laser_range, 
                                    "lidar_path": self.Lidar_image_path+"demo_"+str(self.rollout_count)+"_lidar_image_"+str(self.image_count)+'.jpeg', 
                                    "image_path": self.image_path+"demo_"+str(self.rollout_count)+"_raw_image_"+str(self.image_count)+'.jpeg',
                                    "lin_vel": lin_vel,
                                    "ang_vel": ang_vel
                                    }])
        self.rollout_df = pd.concat([self.rollout_df, new_row_df], ignore_index=True)
        self.image_count+=1

    def save_rollout(self, max_steps=6):
        self.start_demo = False
        step_count      = self.rollout_df.shape[0]
        # print(step_count)
        # rollout_count = stats_df.shape[0]
        if step_count > max_steps:

            print( "SAVING Rollout:", self.rollout_df_filename)
            self.rollout_df.to_csv(self.rollout_df_filename+"/demo_"+str(self.rollout_count)+".csv" , index=False, encoding='utf-8')
            # clear the data frame
            self.rollout_df       = self.rollout_df.iloc[0:0]

            for file_name, image in self.rollout_lidar_images.items():
                print("SAVING LIDAR IMAGES: ",file_name)
                cv2.imwrite(file_name, image)
            self.rollout_lidar_images.clear()
            
            for file_name, image in self.rollout_images.items():
                print("SAVING IMAGES: ",file_name)
                cv2.imwrite(file_name, image)
            self.rollout_images.clear()

            self.image_count = 0
            self.rollout_count += 1
            # return rollout_df, stats_df, rollout_df.shape[0]

        else:
            print("[+]: ROLOUT LENGTH OF ",str(self.rollout_df.shape[0])," DOESN'T MEET MINIMUM REQURIREMENT OF", max_steps)
            # return rollout_df, stats_df, rollout_df.shape[0]

if __name__ == '__main__':
    rospy.init_node("save_data")
    save_obj = save()
    # while not rospy.is_shutdown():
    try:
        rospy.spin()
        save_obj.save_rollout()
    except KeyboardInterrupt:
        print("Shutting down")