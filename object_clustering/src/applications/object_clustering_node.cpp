
#include <object_clustering/object_clustering.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_clustering_node");
  ros::NodeHandle nh;
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  ObjectClustering clustering;
  
  // initialize the object
  if(!clustering.initialize(nh))
  {
    ROS_ERROR_STREAM("Error init service");
    return -1;
  }
  
  while(ros::ok())
  {
    clustering.update(ros::Time::now());
    ros::spinOnce();
  }

  // ros::waitForShutdown();
}
