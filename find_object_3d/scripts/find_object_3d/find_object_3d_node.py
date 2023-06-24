import rospy
import open3d as o3d
import numpy as np

from sensor_msgs.msg import PointCloud2


class FindObject3D:
    def __init__(self):
        self.point_cloud_sub = None
        self.centroids = None
    def callback(self, msg):
        pc_data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        xyz = pc_data[:, :3]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        # downsampled = pcd.voxel_down_sample(voxel_size=0.01)
        # outlier_removed, _ = pcd.remove_radius_outlier(nb_points=10, radius=0.1)
        outlier_removed, _ = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=2.0)

        _, inliers = outlier_removed.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
        plane_removed = outlier_removed.select_by_index(inliers, invert=True)

        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(plane_removed.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

        # max_label = labels.max()
        # print(f"point cloud has {max_label + 1} clusters")
        # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        # colors[labels < 0] = 0
        # plane_removed.colors = o3d.utility.Vector3dVector(colors[:, :3])

        self.centroids = []
        unique_labels = np.unique(labels)
        points = np.asarray(plane_removed.points)
        for label in unique_labels:
            cluster_indices = np.where(labels == label)[0]
            cluster_points = points[cluster_indices]

            centroid = np.mean(cluster_points, axis=0)
            self.centroids.append(centroid)
            # print("Cluster {}: Centroid = {}".format(label, centroid))
        
        # o3d.visualization.draw(plane_removed)  #for debug
        print(self.centroids)
    def result(self):
        return self.centroids

    def listen(self):
        self.point_cloud_sub = rospy.Subscriber('/segmentation/objects', PointCloud2, self.callback)
    

if __name__ == "__main__":
    rospy.init_node('open3d_processing_node')
    a = FindObject3D()
    a.listen()
    a.result()
    rospy.spin()