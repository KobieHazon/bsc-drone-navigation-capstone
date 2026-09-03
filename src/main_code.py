#!/usr/bin/python2.7

import rospy
from drone_controller import drone_controller
from aux import Wifi_Finder
from tag_manager import tag_manager
from apriltag_ros.msg import AprilTagDetectionArray


# add auto connect and disconnect from tello wifi

def connect_to_drone(drone_name, drone_pass, network_interface):
    tello_finder = Wifi_Finder(server_name=drone_name,
                               password=drone_pass,
                               interface=network_interface)
    while tello_finder.run() is None and not rospy.is_shutdown():
        print("Connecting To Tello Wifi Network...")
        rospy.sleep(5)
    if rospy.is_shutdown():
        exit(1)


if __name__ == '__main__':
    rospy.init_node('our_node', anonymous=False)
    # connect_to_drone("Tello", "", "wlp3s0")
    # rospy.sleep(5)
    drone_controller = drone_controller()
    rospy.loginfo('Our_node has started!')
    # self.cleanup()
"""
    <node name="orb_slam2_mono" pkg="orb_slam2_ros" type="orb_slam2_ros_mono" output="screen" respawn="true">

      <remap from="/camera/image_raw" to="/tello/camera/image_raw" />

      <param name="publish_pointcloud" type="bool" value="true" />
      <param name="publish_pose" type="bool" value="true" />
      <param name="localize_only" type="bool" value="false" />
      <param name="reset_map" type="bool" value="false" />

      <!-- static parameters -->
      <param name="use_viewer" type="bool" value="false" />
      <param name="load_map" type="bool" value="false" />
      <param name="map_file" type="string" value="labaratory_rot.bin" />
      <param name="settings_file" type="string" value="$(find orb_slam2_ros)/orb_slam2/config/Tello.yaml" />
      <param name="voc_file" type="string" value="$(find orb_slam2_ros)/orb_slam2/Vocabulary/ORBvoc.txt" />

      <param name="pointcloud_frame_id" type="string" value="map" />
      <param name="camera_frame_id" type="string" value="camera_link" />
      <param name="min_num_kf_in_map" type="int" value="15" />
  </node>"""
