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
  // service_ = nh.advertiseService("/restaurant/get_object_coordinate", &ObjectClustering::getCoordinateCallback, this);
  objects_coordinate_pub_ = nh.advertise<objects_msgs::objects>("/restaurant/objects", 1);
  raw_cloud_.reset(new PointCloud);
  preprocessed_cloud_.reset(new PointCloud);
  objects_cloud_.reset(new PointCloud);

  return true;
}


void ObjectClustering::update(const ros::Time& time)
{
  if(is_cloud_updated_)
  {
    is_cloud_updated_ = false;
    if(!preProcessCloud(raw_cloud_, preprocessed_cloud_)){
      return;
      }
    if(!segmentCloud(preprocessed_cloud_, objects_cloud_)){
      return;
    }
    std::vector<Eigen::Vector3d> centorids = clusterCloud(objects_cloud_);
    
    objects_msgs::objects msg;
    msg.Objects.resize(detections_.size());
    for (int i = 0; i < detections_.size(); i++)
    { 
      double x = (detections_[i].xmax - detections_[i].xmin)/2 + detections_[i].xmin;
      double y = (detections_[i].ymax - detections_[i].ymin)/2 + detections_[i].ymin;
      Eigen::Vector2d obj_;
      obj_ << x, y;
      // ROS_INFO_STREAM(obj_);
      Eigen::Vector3d desire = clusterMaching(centorids, obj_);
      msg.Objects[i].name = detections_[i].Class;
      msg.Objects[i].x = desire[0];
      msg.Objects[i].y = desire[1];
      msg.Objects[i].z = desire[2];
      ROS_INFO_STREAM(desire);
    }
    objects_coordinate_pub_.publish(msg);
  }
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


std::vector<Eigen::Vector3d> ObjectClustering::clusterCloud(CloudPtr& input)
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
  return centroids;
}

Eigen::Vector3d ObjectClustering::clusterMaching(std::vector<Eigen::Vector3d>& input, Eigen::Vector2d& obj)
{
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

  double dist = 1000000;
  Eigen::Vector3d desire_coordinate;
  
  for(int index = 0; index < input.size(); index++)
  { 
    Eigen::Vector2d pixel_centroid; 
    Eigen::Vector3d inter;
    inter = K_ * T_base_camera * input[index];
    pixel_centroid << inter[0]/inter[2], inter[1]/inter[2];
    if((pixel_centroid-obj).norm() < dist)
    {
      // ROS_INFO_STREAM(pixel_centroid);
      dist = (pixel_centroid-obj).norm();
      desire_coordinate = input[index];
    }
  }
  // ROS_INFO_STREAM(desire_coordinate);
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
  detections_.clear();
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