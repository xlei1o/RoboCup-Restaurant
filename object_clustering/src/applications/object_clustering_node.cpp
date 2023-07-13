
#include <object_clustering/object_clustering.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_clustering_node");
  ros::NodeHandle nh;

  ObjectClustering clustering;
  
  // initialize the object
  if(!clustering.initialize(nh))
  {
    ROS_ERROR_STREAM("Error init service");
    return -1;
  }
  
  ros::Rate rate(30);
  while(ros::ok())
  {
    clustering.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }
}
