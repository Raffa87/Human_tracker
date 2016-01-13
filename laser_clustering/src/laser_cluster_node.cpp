#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "my_algebra_common.h"


#include "laser_clustering/Cluster.h"
#include "laser_clustering/ClusterArray.h"

//#include <boost/lexical_cast.hpp>

#define maxSizeDataUTM   1081 //Number of URG Data 
#define NUM_CLUSTERS 200 //Maximum Number of Clusters

#define Cluster_Point_Dist_mm 100  //Distance between points to be clustered (grouped)


//#define Minimum_Points_Per_Cluster 3 //Minimum of points in a cluster
//#define Line_Length_mm 500//650 //Length of a straight line not considered human
//#define Small_Eig_Val_mm 35 //Small Eigen Value to Classify Cluster As A Line
//#define Big_Eig_Val_mm 1800 //Big Eigen Value to Classify Cluster As A Line

#define old_Minimum_Points_Per_Cluster 10 //Minimum of points in a cluster
#define old_Small_Eig_Val_mm 30 //Small Eigen Value to Classify Cluster As A Line
#define old_Big_Eig_Val_mm 180000 //Big Eigen Value to Classify Cluster As A Line

#define CONFIDENCE_INTERVAL sqrt (5.99 )

int Minimum_Points_Per_Cluster;
int Minimum_Points_Per_Cluster_default = 3; //Minimum of points in a cluster
int Line_Length_mm;
int Line_Length_mm_default = 800; //650 //Length of a straight line not considered human
int Small_Eig_Val_mm; 
int Small_Eig_Val_mm_default = 60; //35 //Small Eigen Value to Classify Cluster As A Line
int Big_Eig_Val_mm;  //Big Eigen Value to Classify Cluster As A Line
int Big_Eig_Val_mm_default = 1800; //Big Eigen Value to Classify Cluster As A Line





bool discard_map;	//exclude points already in map
				//useful to track unmapped persons/objects
bool discard_map_default = true;


bool search_around_map = true;  //when search on map, it searches also in the near cell to find occupancy
int num_cell_around_map = 7; 	//it searches up to num_cell_around_map near the center cell 

int map_occupancy_threshold = 70;	


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

typedef struct{
	double x_mm;
	double y_mm;
	double z_mm;
	double range_mm;
	double angle_rad;
	double intensity;
}URG_Transformed_Point;

typedef struct{
	int numPoints ;     //Number of Scan Points
	double timestamp;   //Time of finishing a scan
	///URG_Transformed_Point point[maxSizeDataUTM] ; //Points of the scan
	std::vector<URG_Transformed_Point> new_point;
	
}URG_Transformed;




void Make_Clusters_From_Scan(URG_Transformed *ss_urg_xy, std::vector<cluster_2d> &cl, URG_Transformed *residual );
int Moving_Object_Candidate( cluster_2d * cl );
bool get_parameter(std::string param_space, std::string param_name, std::string* param, std::string param_default);
bool get_parameter(std::string param_space, std::string param_name, bool* param, bool param_default);
bool get_parameter(std::string param_space, std::string param_name, int* param, int param_default);


ros::Publisher all_map_clusters_publisher;	//only for visualization purpose
ros::Publisher all_ppl_clusters_publisher;	//only for visualization purpose
ros::Publisher all_map_centroids_publisher;	//only for visualization purpose
ros::Publisher all_ppl_centroids_publisher;	//only for visualization purpose
 
ros::Publisher map_cluster_array_publisher;
ros::Publisher ppl_cluster_array_publisher;

laser_clustering::ClusterArray map_cluster_array_old;
laser_clustering::ClusterArray ppl_cluster_array_old;


std::string param_space;    

std::string global_frame;
std::string global_frame_default = "/map"; 

std::string laser_frame;
std::string laser_frame_default = "/laser";



laser_geometry::LaserProjection projector;
tf::TransformListener *transform_listener;
URG_Transformed ss_urg_xy;  //Sensor Coordinate Frame
URG_Transformed residual;  //Sensor Coordinate Frame
int non_classified_cnt = 0;

nav_msgs::OccupancyGrid static_map;

std::vector<cluster_2d> cl;

//void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
void laser_cloud_callback (const sensor_msgs::PointCloud::ConstPtr& msg)
{
  //cl.numClusters = 0;
  cl.clear();

  sensor_msgs::PointCloud laser_cloud = *msg;
  sensor_msgs::PointCloud cloud;

  if(discard_map)
  {
	cloud.header.stamp = laser_cloud.header.stamp;
  	cloud.header.frame_id = laser_cloud.header.frame_id;

	bool transformation_found = true;
	if(global_frame != cloud.header.frame_id)
	{
		ros::Time now = laser_cloud.header.stamp;
		transform_listener->waitForTransform (global_frame, laser_frame, now, ros::Duration(3.0)); 
		tf::StampedTransform transform;
    		try
		{
		   	transform_listener->lookupTransform(global_frame, laser_frame, now, transform);
 			transform_listener->transformPointCloud(global_frame, now, laser_cloud, laser_frame, laser_cloud);
		}
    		catch (...) 
		{
			transformation_found = false;
      			ROS_ERROR("unable to find trasformation between map and laser");
		}
	}	
	if (transformation_found)
	{
		for (int i=0; i<laser_cloud.points.size(); i++)
		{
			int grid_x = (unsigned int)((laser_cloud.points[i].x - static_map.info.origin.position.x) / static_map.info.resolution);
			int grid_y = (unsigned int)((laser_cloud.points[i].y - static_map.info.origin.position.y) / static_map.info.resolution);
			int cell_number = grid_y * static_map.info.width + grid_x;
			int occupancy = -1;
			if (search_around_map)
			{
				int temp_occupancy = 0;
				int temp_cell_number;
				for(int j=0 - num_cell_around_map; j <= 0 + num_cell_around_map;  j++)
				{
			
					for( int k=0 - num_cell_around_map; k <= 0 + num_cell_around_map; k++)
					{
						temp_cell_number =  (grid_y + j) * static_map.info.width + (grid_x + k);
						temp_occupancy = static_map.data[temp_cell_number];
						if (temp_occupancy > occupancy)
							occupancy = temp_occupancy;
					}	
				}	
			}
			else
				occupancy = static_map.data[cell_number];
		
			if (occupancy < map_occupancy_threshold)
			{
				cloud.points.push_back(laser_cloud.points[i]);
			}
		}
	}
 }
 else
 {
 	for (int i=0; i<laser_cloud.points.size(); i++)
		cloud.points.push_back(laser_cloud.points[i]);
 }

  //fill ss_urg_xy

  ss_urg_xy.numPoints = laser_cloud.points.size();
  ss_urg_xy.timestamp = laser_cloud.header.stamp.sec + laser_cloud.header.stamp.nsec / 1000000000;
  
  //for (int i=0; i<cloud.points.size(); i++)
  //{
  //	ss_urg_xy.point[i].x_mm = cloud.points[i].x * 1000;	//in [mm]
  //	ss_urg_xy.point[i].y_mm = cloud.points[i].y * 1000;	//in [mm]
  //	ss_urg_xy.point[i].z_mm = cloud.points[i].z * 1000;	//in [mm]
  //} 
  
  ss_urg_xy.new_point.clear();
  for (int i=0; i<cloud.points.size(); i++)
  {
	URG_Transformed_Point temp_point;
	temp_point.x_mm = cloud.points[i].x * 1000;	//in [mm]
	temp_point.y_mm = cloud.points[i].y * 1000;	//in [mm]
	temp_point.z_mm = cloud.points[i].z * 1000;	//in [mm]
	ss_urg_xy.new_point.push_back(temp_point);
  }

  std::vector<cluster_2d> map, ppl;
  //map.numClusters = ppl.numClusters = 0;

  Make_Clusters_From_Scan(&ss_urg_xy, cl, &residual); //Make Clusters from a scan line data    


  int res ;
    for (int k=0; k< cl.size(); k++){
      res = Moving_Object_Candidate(&cl[k]) ;
    
      if( res == 1){
	ppl.push_back(cl[k]);
	ppl.back().flag = true;
      }
      else if (res == -1) {
	map.push_back(cl[k]);
	map.back().flag = false;
      }
      else if( res ==0)
	non_classified_cnt++;

    }

    
    laser_clustering::ClusterArray map_cluster_array;
    ros::Time now = ros::Time::now();
	
    for (int i=0; i<map.size(); i++)
    {
	laser_clustering::Cluster my_cluster;
	sensor_msgs::PointCloud my_point_cloud;
        my_point_cloud.header.stamp = now;
	my_point_cloud.header.frame_id = global_frame;
 	for (int k=0; k<map[i].numPoints; k++)
        {
                geometry_msgs::Point32 point;
                point.x = map[i].point[k].x / 1000;     //back to m
                point.y = map[i].point[k].y / 1000;     //back to m
                point.z = 0;
                my_point_cloud.points.push_back(point);
	}
        geometry_msgs::PointStamped my_centroid;
        my_centroid.header.frame_id = global_frame;
        my_centroid.point.x = map[i].centroid.x / 1000;
        my_centroid.point.y = map[i].centroid.y / 1000;

	my_cluster.centroid = my_centroid.point;
	my_cluster.cluster = my_point_cloud;
        map_cluster_array.clusters.push_back(my_cluster);
    }

    laser_clustering::ClusterArray ppl_cluster_array;
    for (int i=0; i<ppl.size(); i++)
    {
	laser_clustering::Cluster my_cluster;

        geometry_msgs::PointStamped my_centroid;
        my_centroid.header.frame_id = global_frame;
        my_centroid.point.x = ppl[i].centroid.x / 1000;
        my_centroid.point.y = ppl[i].centroid.y / 1000;

	my_cluster.centroid = my_centroid.point;

	sensor_msgs::PointCloud my_point_cloud;
        my_point_cloud.header.stamp = now;
	my_point_cloud.header.frame_id = global_frame;
 	for (int k=0; k<ppl[i].numPoints; k++)
        {
                geometry_msgs::Point32 point;
                point.x = ppl[i].point[k].x / 1000;     //back to m
                point.y = ppl[i].point[k].y / 1000;     //back to m
                point.z = 0;
                my_point_cloud.points.push_back(point);
	}

        
	my_cluster.cluster = my_point_cloud;
        ppl_cluster_array.clusters.push_back(my_cluster);
   
    }

    if (map_cluster_array.clusters.size() > 0)
    	map_cluster_array_publisher.publish(map_cluster_array);
    if (ppl_cluster_array.clusters.size() > 0)
	 ppl_cluster_array_publisher.publish(ppl_cluster_array);
 
    
    sensor_msgs::PointCloud map_cloud_to_show;
    map_cloud_to_show.header.frame_id = global_frame;
    for(int i=0; i<map.size(); i++)
    {	
	for (int k=0; k<map[i].numPoints; k++)
	{
		geometry_msgs::Point32 point;
		point.x = map[i].point[k].x / 1000;	//back to m
		point.y = map[i].point[k].y / 1000;	//back to m
		point.z = 0;
		map_cloud_to_show.points.push_back(point);
	}	
    }
    all_map_clusters_publisher.publish(map_cloud_to_show); 

    sensor_msgs::PointCloud map_centroids_to_show;
    map_centroids_to_show.header.frame_id = global_frame;
    for(int i=0; i<map.size(); i++)
    {	
		geometry_msgs::Point32 point;
		point.x = map[i].centroid.x / 1000;	//back to m
		point.y = map[i].centroid.y / 1000;	//back to m
		point.z = 0;
		map_centroids_to_show.points.push_back(point);
	    }
    all_map_centroids_publisher.publish(map_centroids_to_show); 

    sensor_msgs::PointCloud ppl_cloud_to_show;
    ppl_cloud_to_show.header.frame_id = global_frame;
    for(int i=0; i<ppl.size(); i++)
    {	
	for (int k=0; k<ppl[i].numPoints; k++)
	{
		geometry_msgs::Point32 point;
		point.x = ppl[i].point[k].x / 1000;	//back to m
		point.y = ppl[i].point[k].y / 1000;	//back to m
		point.z = 0;
		ppl_cloud_to_show.points.push_back(point);
	}	
    }
    all_ppl_clusters_publisher.publish(ppl_cloud_to_show); 

    sensor_msgs::PointCloud ppl_centroids_to_show;
    ppl_centroids_to_show.header.frame_id = global_frame;
    for(int i=0; i<ppl.size(); i++)
    {	
		geometry_msgs::Point32 point;
		point.x = ppl[i].centroid.x / 1000;	//back to m
		point.y = ppl[i].centroid.y / 1000;	//back to m
		point.z = 0;
		ppl_centroids_to_show.points.push_back(point);
	    }
    all_ppl_centroids_publisher.publish(ppl_centroids_to_show); 

}



int main(int argc, char **argv)
{
 ros::init(argc, argv, "laser_cluster_node");

 param_space = ros::this_node::getName();

 transform_listener = new tf::TransformListener;
 ros::NodeHandle n;

get_parameter(param_space, "/global_frame", &global_frame, global_frame_default);
get_parameter(param_space, "/laser_frame", &laser_frame, laser_frame_default);
get_parameter(param_space, "/discard_map", &discard_map, discard_map_default);

get_parameter(param_space, "/Minimum_Points_Per_Cluster", &Minimum_Points_Per_Cluster, Minimum_Points_Per_Cluster_default);
get_parameter(param_space, "/Line_Length_mm", &Line_Length_mm, Line_Length_mm_default) ;
get_parameter(param_space, "/Small_Eig_Val_mm", &Small_Eig_Val_mm, Small_Eig_Val_mm_default);
get_parameter(param_space, "/Big_Eig_Val_mm", &Big_Eig_Val_mm, Big_Eig_Val_mm_default);




 transform_listener->waitForTransform (global_frame, laser_frame, ros::Time(0), ros::Duration(10.0)); 

 if(discard_map)
 {
	if (!ros::service::waitForService("/static_map", ros::Duration(60,0)))//wait for service to get map
	{
		ROS_ERROR("unable to find /static_map service for get the map");
		return -1;
	}
	ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("/static_map");
	nav_msgs::GetMap get_map_srv;
  	map_client.call(get_map_srv);
	static_map = get_map_srv.response.map;	
  	
 }

 
 //laser_cloud_publisher =  n.advertise<sensor_msgs::PointCloud>("laser_cloud", 10);
 //laser_cloud_publisher_without_map = n.advertise<sensor_msgs::PointCloud>("laser_cloud_without_map",10);
 all_map_clusters_publisher =  n.advertise<sensor_msgs::PointCloud>("all_map_clusters", 1);
 all_ppl_clusters_publisher =  n.advertise<sensor_msgs::PointCloud>("all_ppl_clusters", 1);
 all_map_centroids_publisher =  n.advertise<sensor_msgs::PointCloud>("all_map_centroids", 1);
 all_ppl_centroids_publisher =  n.advertise<sensor_msgs::PointCloud>("all_ppl_centroids", 1);
 
 map_cluster_array_publisher = n.advertise<laser_clustering::ClusterArray>("map_cluster_array", 1);
 ppl_cluster_array_publisher = n.advertise<laser_clustering::ClusterArray>("ppl_cluster_array", 1);

 ros::Subscriber laser_cloud_sub = n.subscribe("/laser_cloud", 1000, laser_cloud_callback);
 
 
 ros::spin();
 return 0;
}

//Creates sveral Clusters from a scan (Maximum of Max_Cluster_Num)

void Make_Clusters_From_Scan(URG_Transformed *ss_urg_xy, std::vector<cluster_2d> &cl, URG_Transformed *residual ) {

  //cl->numClusters = 0;
  //cl->cluster[cl->numClusters].numPoints = 0;
  //residual->numPoints = 0;

  cluster_2d new_cluster;
  new_cluster.numPoints = 0;
  
  if (ss_urg_xy->new_point.size() == 0)
	return;
  
  //for(int i = 0; i < ss_urg_xy->numPoints ; i++)
  for(int i=0; i< ss_urg_xy->new_point.size()-1; i++)
  {  //Condition to see if consecutive points distance is less than Cluster_Point_Dist_mm
    
    
    //if(Dist_Between_Points(ss_urg_xy->point[i].x_mm, ss_urg_xy->point[i+1].x_mm, ss_urg_xy->point[i].y_mm, ss_urg_xy->point[i+1].y_mm) < Cluster_Point_Dist_mm )
    if(Dist_Between_Points(ss_urg_xy->new_point[i].x_mm, ss_urg_xy->new_point[i+1].x_mm, ss_urg_xy->new_point[i].y_mm, ss_urg_xy->new_point[i+1].y_mm) < Cluster_Point_Dist_mm )
    {
      //printf("\n Dist %f ",Dist_Between_Points(ss_urg_xy->point[i].x_mm, ss_urg_xy->point[i+1].x_mm, ss_urg_xy->point[i].y_mm, ss_urg_xy->point[i+1].y_mm));
      
      //Copying scan points to cluster and the number of points!!
      
      //cl->cluster[cl->numClusters].point[cl->cluster[cl->numClusters].numPoints].x = ss_urg_xy->point[i].x_mm ;
      //cl->cluster[cl->numClusters].point[cl->cluster[cl->numClusters].numPoints].y = ss_urg_xy->point[i].y_mm ;

      new_cluster.point[new_cluster.numPoints].x = ss_urg_xy->new_point[i].x_mm;
      new_cluster.point[new_cluster.numPoints].y = ss_urg_xy->new_point[i].y_mm;
      new_cluster.numPoints++;
    }



    else{

      //if(cl->numClusters < MAX_NUM_CLUSTER && cl->cluster[cl->numClusters].numPoints > Minimum_Points_Per_Cluster){
      //if(cl->cluster[cl->numClusters].numPoints > Minimum_Points_Per_Cluster){
      if(new_cluster.numPoints > Minimum_Points_Per_Cluster){
	//if(cl->numClusters < MAX_NUM_CLUSTER ){
	cl.push_back(new_cluster);  //Add a New Cluster
      }
      new_cluster.numPoints = 0;
  
      //Add Point to Residual
      
      //residual->point[residual->numPoints] = ss_urg_xy->point[i] ;
      //residual->numPoints ++;
    }

   //cl->numClusters ++; 
   //printf("NumClusters %d \n",cl->numClusters);
   }

}

int Moving_Object_Candidate( cluster_2d * cl ){
  int i,lnum;
  double mx,my,mxy,mxx,myy;
  double mat[2][2],v1[2],v2[2],l1,l2;

  mx = 0;my = 0;mxx = 0;mxy = 0;myy = 0;
  //lz = 0;
    lnum = 0;
    for(i = 0;i < cl->numPoints;i++){
      mx  += cl->point[i].x; // m10
      my  += cl->point[i].y; // m01

      //lz  += ss_urg_xy->point[i].z_mm; 
      mxx += cl->point[i].x * cl->point[i].x;
      myy += cl->point[i].y * cl->point[i].y;
      mxy += cl->point[i].x * cl->point[i].y;
      lnum++;                 // m00
    }

    //if(lnum < Minimum_Points_Per_Cluster)return 0;
    mat[0][0] = (mxx - mx*mx/lnum) / lnum ;  //M20
    mat[1][1] = (myy - my*my/lnum) / lnum ;  //M02
    mat[0][1] = mat[1][0] =  (mxy - mx*my/lnum) / lnum ;  //M11

    cl->centroid.x = mx / lnum;   //xg
    cl->centroid.y = my / lnum;   //yg

    //ell->z = lz / lnum;

    eigenvector_matrix2d(mat,v1,v2,&l1,&l2);
    cl->semi_major = sqrt(l1) * CONFIDENCE_INTERVAL ;
    cl->semi_minor = sqrt(l2) * CONFIDENCE_INTERVAL ;

    //double dist = Dist_Between_Points(cl->point[0].x_mm, cl->point[i-1].x_mm, cl->point[0].y_mm, cl->point[i-1].y_mm);
    int return_flag = 0 ; 
    //0  default
    //-1 Not a human candidate
    //1  human candidate

    //if(dist > Line_Length_mm  || ( cl->semi_minor < Small_Eig_Val_mm && cl->semi_major > Big_Eig_Val_mm)  || cl->semi_major > Line_Length_mm)  {
    if( (cl->semi_minor < Small_Eig_Val_mm && cl->semi_major > Line_Length_mm) || cl->semi_major > Line_Length_mm)  {
      //printf("\n map: %f %f  ",cl->semi_minor, cl->semi_major);
      return_flag = -1;
      //return -1;
    }

    //if ( cl->semi_minor >= Small_Eig_Val_mm && cl->semi_major <= Line_Length_mm ){
    if ( cl->semi_minor >= 10 && cl->semi_major <= Line_Length_mm && cl->numPoints >=4){

   //printf("\nppl: %f %f  %f",cl->semi_minor, cl->semi_major,dist );
   return_flag = 1;
   //return 1;

 }

    //In any other case the cluster will be considered clutter
    return return_flag;
}

bool get_parameter(std::string param_space, std::string param_name, std::string* param, std::string param_default)
{
	std::string node_name = ros::this_node::getName();
	std::string temp;
	if (ros::param::get(param_space + param_name, temp))
 	{
       		*param = temp; 	
		ROS_INFO("[%s] got %s from param server, value is %s", node_name.c_str(), param_name.c_str(), param->c_str() );
		return true;
 	}
 	else
 	{
        	*param = param_default;
	        ROS_INFO("[%s] not found %s from param server, set to default value %s", node_name.c_str(), param_name.c_str(), param->c_str() );
 	}
	return false;
}

bool get_parameter(std::string param_space, std::string param_name, bool* param, bool param_default)
{
	std::string node_name = ros::this_node::getName();
	bool temp;
	if (ros::param::get(param_space + param_name, temp))
 	{
       		*param = temp; 	
		ROS_INFO("[%s] got %s from param server, value is %d", node_name.c_str(), param_name.c_str(), *param );
		return true;
 	}
 	else
 	{
        	*param = param_default;
	        ROS_INFO("[%s] not found %s from param server, set to default value %d", node_name.c_str(), param_name.c_str(), *param );
 	}
	return false;
}

bool get_parameter(std::string param_space, std::string param_name, int* param, int param_default)
{
	std::string node_name = ros::this_node::getName();
	int temp;
	if (ros::param::get(param_space + param_name, temp))
 	{
       		*param = temp; 	
		ROS_INFO("[%s] got %s from param server, value is %d", node_name.c_str(), param_name.c_str(), *param );
		return true;
 	}
 	else
 	{
        	*param = param_default;
	        ROS_INFO("[%s] not found %s from param server, set to default value %d", node_name.c_str(), param_name.c_str(), *param );
 	}
	return false;
}
