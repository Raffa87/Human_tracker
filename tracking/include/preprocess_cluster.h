#include "my_algebra_common.h"
#include "laser_clustering/ClusterArray.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tracking/PoseWithCovarianceStampedArray.h"

//#include <signal.h>
//#include "geometry_msgs/PoseArray.h"
//#include <visualization_msgs/Marker.h>


#define ssm_MAX_NUM_OF_PEOPLE 50
#define t_MAX_TRACKING_RANGE    4500 //[mm] 

#define NUM_CLUSTERS 200 //Maximum Number of Clusters
#define maxSizeDataUTM   1081 //Number of URG Data 

#define t_MAX_STEP_SIZE         650              // to compare the previous no of cluster with present and integration in mm
#define t_CYCLES_TO_WAIT        10              //10               // Number of cycles to wait before adding a new human to track


typedef struct{
  int counter[NUM_CLUSTERS]; //* MAX_LASER_CLUSTER_STREAMS];
}cycle_counter;

typedef struct{
        double x;
        double y;
}point_2d;

typedef struct{
  int numPoints;
  point_2d centroid ;
  double semi_major ;
  double semi_minor ;
  bool flag;
  double zVal;
  point_2d point[maxSizeDataUTM] ; //Points of the scan
}cluster_2d;

typedef struct{
  int numClusters;//Number of Clusters
  cluster_2d cluster[NUM_CLUSTERS];
}ClusterStruct;

ClusterStruct rob_cluster; 
ClusterStruct odo_cluster; 
ClusterStruct pre_cluster;

tf::TransformListener *transform_listener;

void ppl_cluster_arrayCallback(const laser_clustering::ClusterArray::ConstPtr& cluster_array);
void Cluster_masking(ClusterStruct *cl, cycle_counter *cnt, geometry_msgs::PointStamped *pose );
void cluster_data_asociation( ClusterStruct *pre_cl, ClusterStruct * cl, cycle_counter *pre_cnt, cycle_counter *cnt);
void copy_cluster_struct(ClusterStruct *org, ClusterStruct *dst );
void classify_clusters(ClusterStruct * cl, point_2d human_cluster[NUM_CLUSTERS], int *no_of_human_clusters, point_2d chunk_cluster[NUM_CLUSTERS], int *no_of_chunk_clusters);
void find_closest_cluster( int index, int *id, double *mindist, ClusterStruct *cl);


