#include <object_clustering/object_clustering.h>


ObjectClustering::ObjectClustering()
{
}

ObjectClustering::~ObjectClustering()
{
}

bool ObjectClustering::initialize(ros::NodeHandle& nh)
{ 
  is_cloud_updated_ = false;             
  is_boundingboxes_updated_ = false;
  has_camera_info_ = false;
  point_cloud_sub_ = nh.subscribe("/xtion/depth_registered/points", 1, &ObjectClustering::cloudCallback, this);
  // ros::topic::waitForMessage<sensor_msgs::PointCloud2ConstPtr>("/xtion/depth_registered/points");
  object_detections_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 1, &ObjectClustering::detectionCallback, this);
  // ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxesConstPtr>("/darknet_ros/bounding_boxes");
  camera_info_sub_ = nh.subscribe("/xtion/rgb/camera_info", 1, &ObjectClustering::cameraInfoCallback, this);
  // ros::topic::waitForMessage<sensor_msgs::CameraInfoConstPtr>("/xtion/rgb/camera_info");
  service_ = nh.advertiseService("/restaurant/get_object_coordinate", &ObjectClustering::getCoordinateCallback, this);
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);
  objects_cloud_.reset(new PointCloud);

  return true;
}

bool ObjectClustering::getCoordinateCallback(objects_msgs::getCoordinate::Request& req, 
                                             objects_msgs::getCoordinate::Response& res)
{ 
  // object_detections_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 1, &ObjectClustering::detectionCallback, this);
  if(is_cloud_updated_)
  {
    is_cloud_updated_ = false;
    if(!preProcessCloud(raw_cloud_, preprocessed_cloud_)){
      return false;
      }
    if(!segmentCloud(preprocessed_cloud_, objects_cloud_)){
      return false;
    }

    int number = detections_.size();
    for (int i = 0; i < number; i++)
    {
      if(detections_[i].Class == req.object){
        double x = (detections_[i].xmax - detections_[i].xmin)/2 + detections_[i].xmin;
        double y = (detections_[i].ymax - detections_[i].ymin)/2 + detections_[i].ymin;
        detection_ << x, y;
      }
    }

    if (is_boundingboxes_updated_ && has_camera_info_)
    {
      is_boundingboxes_updated_ = false;
      has_camera_info_ = false;
      Eigen::Vector3d output = clusterCloud(objects_cloud_, req.object);
      res.x = output[0];
      res.y = output[1];
      res.z = output[2];
    }
  }
  return true;
}


bool ObjectClustering::preProcessCloud(CloudPtr& input, CloudPtr& output)
{

  CloudPtr ds_cloud(new PointCloud);          
  
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(input);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter (*ds_cloud);

  CloudPtr transf_cloud(new PointCloud);       
  tf::StampedTransform transform;
  try 
  {
    tfListener_.lookupTransform("base_footprint", "xtion_rgb_optical_frame", ros::Time(0), transform);
  } 

  catch (tf::TransformException& ex) 
  {
    ROS_ERROR("%s", ex.what());
    return -1;
  }

  pcl_ros::transformPointCloud("base_footprint", *ds_cloud, *transf_cloud, tfListener_);

  pcl::PassThrough<PointT> pass_z;
  pass_z.setInputCloud(transf_cloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.2, 1.0);
  pass_z.filter(*output);
  
  pcl::PassThrough<PointT> pass_x;
  pass_x.setInputCloud(output);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (0.0, 1.0);
  pass_x.filter(*output);

  return true;
}


bool ObjectClustering::segmentCloud(CloudPtr& input, CloudPtr& objects_cloud)
{
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    return (-1);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);

  extract.setNegative(true);
  extract.filter(*objects_cloud);


  if(coefficients->values.empty())
    return false;

  Eigen::Vector3f n;
  n << coefficients->values[0], coefficients->values[1], coefficients->values[2];
  n.normalize();
  double d = coefficients->values[3];

  Eigen::Vector3f floor;
  floor << 0.0, 0.0, 1.0;
  Eigen::Quaternionf Q_plane_base; 

  Q_plane_base.setFromTwoVectors(n, floor);

  Eigen::Vector3f t_plane_base = d * n; 

  Eigen::Affine3f T_plane_base = Eigen::Affine3f::Identity();
  T_plane_base.rotate(Q_plane_base.toRotationMatrix());
  T_plane_base.translate(t_plane_base);
  
  CloudPtr transf_cloud(new PointCloud);
  pcl::transformPointCloud(*objects_cloud, *transf_cloud, T_plane_base);

  CloudPtr filterd_cloud(new PointCloud);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(transf_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.01, 0.15);
  pass.filter(*filterd_cloud);

  pcl::transformPointCloud(*filterd_cloud, *objects_cloud, T_plane_base.inverse());
  return true;
}


Eigen::Vector3d ObjectClustering::clusterCloud(CloudPtr& input, std::string& desire)
{
  pcl::EuclideanClusterExtraction<PointT> ec;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud(input);
  ec.extract(cluster_indices);

  std::vector<Eigen::Vector3d> centroids;
  Eigen::Vector3d centroid;

  int total_points = 0;
  for (const auto& cluster : cluster_indices)
  {
    centroid << 0.0, 0.0, 0.0;
    total_points = 0;
    for (const auto& idx : cluster.indices) 
    {
      centroid[0] += input->points[idx].x;
      centroid[1] += input->points[idx].y;
      centroid[2] += input->points[idx].z;
      total_points++;
    }
    centroid /= total_points;
    centroids.push_back(centroid);
  }

  Eigen::Affine3d T_base_camera; 
  tf::StampedTransform transform;
  try 
  {
    tfListener_.lookupTransform("xtion_rgb_optical_frame", "base_footprint", ros::Time(0), transform);
  } 
  catch (tf::TransformException& ex) 
  {
    ROS_ERROR("%s", ex.what());
  }
  tf::transformTFToEigen(transform, T_base_camera);

  double dist = 100;
  Eigen::Vector3d desire_coordinate;
  
  for(int index = 0; index < centroids.size(); index++)
  { 
    Eigen::Vector2d pixel_centroid; 
    Eigen::Vector3d inter;
    inter = K_ * T_base_camera * centroids[index];
    pixel_centroid << inter[0]/inter[2], inter[1]/inter[2];
    if((pixel_centroid-detection_).norm() < dist)
    {
      dist = (pixel_centroid-detection_).norm();
      desire_coordinate = centroids[index];
    }
  }
  return desire_coordinate;
}

void ObjectClustering::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  is_cloud_updated_ = true;
  pcl::fromROSMsg(*msg, *raw_cloud_);
}

void ObjectClustering::detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{ 
  is_boundingboxes_updated_ = true;
  int number = msg -> bounding_boxes.size();
  detections_.resize(number);
  for (int i = 0; i < number; i++)
  {   
    detections_.push_back(msg->bounding_boxes[i]);
  }
}

void ObjectClustering::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  has_camera_info_ = true;
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

  K << msg->K[0], 0.0, msg->K[2],
       0.0, msg->K[4], msg->K[5],
       0.0, 0.0, 1.0;
  K_ = K;
}