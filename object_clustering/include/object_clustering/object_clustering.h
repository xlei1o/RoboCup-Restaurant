#ifndef OBJECT_CLUSTERING_CLASS_H
#define OBJECT_CLUSTERING_CLASS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include  <objects_msgs/objects.h>
#include  <objects_msgs/single.h>

class ObjectClustering
{
public:
  typedef pcl::PointXYZ PointT;                // The Point Type
  typedef pcl::PointCloud<PointT> PointCloud;     // The PointCloud Type
  typedef PointCloud::Ptr CloudPtr;               // The PointCloud Pointer Type

public:
  ObjectClustering();
  ~ObjectClustering();

  bool initialize(ros::NodeHandle& nh);
  void update(const ros::Time &time);

private:
  bool preProcessCloud(CloudPtr& input, CloudPtr& output);
  bool segmentCloud(CloudPtr& input, CloudPtr& objects_cloud);
  std::vector<Eigen::Vector3d> clusterCloud(CloudPtr& input);
  Eigen::Vector3d clusterMaching(std::vector<Eigen::Vector3d> input, Eigen::Vector2d obj);

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
private:
  ros::ServiceServer service_;

  // std::string desire_object;
  
  ros::Subscriber point_cloud_sub_;   //!< Subscribers to the PointCloud data
  ros::Subscriber object_detections_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher objects_coordinate_pub_;  

  // internal pointclouds
  CloudPtr raw_cloud_;                  //!< Inital raw point cloud
  CloudPtr preprocessed_cloud_;         //!< after preprocessing
  CloudPtr objects_cloud_;              //!< points of objects
  std::vector<darknet_ros_msgs::BoundingBox> detections_;

  std::vector<Eigen::Vector2d> objects_;
  Eigen::Matrix3d K_; 
  // transformation
  tf::TransformListener tfListener_;    //!< access ros tf tree to get frame transformations
};

#endif
