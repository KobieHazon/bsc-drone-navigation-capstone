import rospy
from sensor_msgs.msg import PointCloud2
import open3d
import open3d_convertor
import numpy as np
import time


class mapper:
    def __init__(self, controller):
        self.completed_room = False
        self.drone_controller = controller
        self.pcd = None
        self.num_points = 0
        self.time_new_points = None
        self.last_pose = None
        # self.mesh = None
        rospy.Subscriber("/orb_slam2_mono/map_points", PointCloud2, self.pcd_callback)
        rospy.Subscriber("/orb_slam2_mono/pose", PointCloud2, self.pose_callback)

    def pcd_callback(self, data):
        print("~~~~~~~~~~~~~~~~~~~~~~~~")
        pcd = open3d_convertor.convertCloudFromRosToOpen3d(data)
        if pcd == None:
            return
        pcd = pcd.voxel_down_sample(0.002)
        print(len(np.asarray(pcd.points)))
        pcd = pcd.remove_none_finite_points()
        pcd.estimate_normals()
        if self.pcd is None or len(np.asarray(pcd.points)) - len(np.asarray(self.pcd.points)) > 0:
            self.pcd = pcd
            self.num_points = len(np.asarray(pcd.points)) - len(np.asarray(self.pcd.points))
            self.time_new_points = time.time()
        print("In ORB callback: New Size is " + str(len(np.asarray(self.pcd.points))))
        """
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radius = 1.5 * avg_dist
        mesh = open3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, open3d.utility.DoubleVector([radius, radius * 2]))
        print(np.shape(mesh.triangles))"""

    def pose_callback(self, data):
        curr_x = data.pose.position.x
        curr_y = data.pose.position.y
        curr_z = data.pose.position.z
        if abs(curr_x - ((time.time() - self.last_pose_time)*self.last_vel[0])) > epsilon:
            print("Correcting movement")
        self.last_pose = (curr_x, curr_y, curr_z)

