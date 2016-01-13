#include <ros/ros.h>

#include <signal.h>
//#include <ssm.h>
//#include <SsmStructures.h>
//#include "top_urg.h"
//#include "grid_common.h"
#include "on_board_human_tracker_2d.h"
#include "geometry_msgs/PoseArray.h"
#include "laser_clustering/ClusterArray.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <visualization_msgs/Marker.h>

//#define TEMP_NUMBER_OF_CLUSTER_STREAMS 2 //Number of cluster (laser sensor) streams



int counter       = 0;   //Counter for the number of cluster streams (laser sensors)
double time_count = 0;   //Counts the total time of the loop
int num_cluster_stream = 0; //Variable for the total number of cluster streams

cycle_counter cnt; //[MAX_LASER_CLUSTER_STREAMS];      
cycle_counter pre_cnt; //[MAX_LASER_CLUSTER_STREAMS]; 

ClusterStruct odo_cluster; //[MAX_LASER_CLUSTER_STREAMS]; //local (odometry) coordinate cluster scan
ClusterStruct pre_cluster; //[MAX_LASER_CLUSTER_STREAMS]; //pre local (odometry) coordinate cluster scan
ClusterStruct rob_cluster; //[MAX_LASER_CLUSTER_STREAMS]; //robot coordinate cluster scan

point_2d     human_cluster[NUM_CLUSTERS];
point_2d     chunk_cluster[NUM_CLUSTERS];
int     no_of_chunk_clusters = 0;
int     no_of_human_clusters   = 0;

double  pre_time_stamp = 0;
double  sampling_time   = 0;

//Variables related to the id of the humans
int t_Max_ID_Index =-1;
int t_People_ID[ssm_MAX_NUM_OF_PEOPLE] = {0};  //Variable that controls the id's of each tracked person (particle filter, it goes from 1 to t_MAX_ID)

//tracked entities variables
t_Entity human_pt;


int NUMBER_OF_CLUSTER_STREAMS = 1 ;
//GRID grid ;
bool LOCALIZATION_AVAILABLE = false;

tf::TransformListener *transform_listener;
ros::Publisher odo_cluster_before_leg_cluster_pub;
ros::Publisher humans_poses_pub;
ros::Publisher pose_array_debug_pub;
ros::Publisher humans_poses_before_filter_pub;
ros::Publisher humans_poses_before_leg_cluster_pub;
ros::Publisher humans_poses_before_create_humans_pub; 
ros::Publisher humans_poses_before_particle_likelihood_pub;
ros::Publisher humans_marker_pub;


std::string laser_frame;
std::string laser_frame_default = "/laser";

std::string cluster_frame;
std::string cluster_frame_default = "map";


void ppl_cluster_arrayCallback(const laser_clustering::ClusterArray::ConstPtr& cluster_array)
{
/*
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
*/

		
	//transform_listener->waitForTransform ("/map", "/laser", cluster_array->clusters[0].cluster.header.stamp, ros::Duration(3.0));
        tf::StampedTransform transform;
        //try
        //{
        //        transform_listener->lookupTransform("/map", "/laser", cluster_array->clusters[0].cluster.header.stamp, transform);
        //}
        //catch (...)
        //{
        //        ROS_ERROR("unable to find trasformation between map and laser");
        //        ros::Duration(1.0).sleep();
        //}

	sampling_time =  (cluster_array->clusters[0].cluster.header.stamp.sec + (double)cluster_array->clusters[0].cluster.header.stamp.nsec / 1000000000) - pre_time_stamp;
	pre_time_stamp = (cluster_array->clusters[0].cluster.header.stamp.sec + (double)cluster_array->clusters[0].cluster.header.stamp.nsec / 1000000000);

	 

	//fill clusterStruct
	rob_cluster.numClusters = cluster_array->clusters.size();
	for (int i=0; i<cluster_array->clusters.size(); i++)
	{
		sensor_msgs::PointCloud my_point_cloud = cluster_array->clusters[i].cluster;
                
		rob_cluster.cluster[i].numPoints = my_point_cloud.points.size();

		geometry_msgs::PointStamped my_centroid;
		my_centroid.header.frame_id = cluster_array->clusters[i].cluster.header.frame_id;
		my_centroid.point.x = cluster_array->clusters[i].centroid.x;
                my_centroid.point.y = cluster_array->clusters[i].centroid.y;

		//transform_listener->transformPoint("/map", my_centroid, my_centroid); 
		
		rob_cluster.cluster[i].centroid.x = my_centroid.point.x * 1000;
		rob_cluster.cluster[i].centroid.y = my_centroid.point.y * 1000;
		
		for (int k=0; k< my_point_cloud.points.size(); k++)
		{
			rob_cluster.cluster[i].point[k].x = my_point_cloud.points[k].x *1000;	//to [mm] 
                        rob_cluster.cluster[i].point[k].y = my_point_cloud.points[k].y *1000;	//to [mm]
		}
		rob_cluster.cluster[i].flag = true; 
		 
	}


	std::string rob_cluster_frame = cluster_array->clusters[0].cluster.header.frame_id;
	if(rob_cluster_frame != cluster_frame)
	{
		ROS_ERROR("[on_board_human_tracker_2d] WARNING: rob_cluster_frame is different from given cluster_frame");
		ROS_ERROR("[on_board_human_tracker_2d] rob_cluster_frame: %s, cluster_frame: %s", rob_cluster_frame.c_str(), cluster_frame.c_str());
	}
	
	ssm_Odometry_rl ssm_laser_pose;
	geometry_msgs::PointStamped laser_origin;
 	laser_origin.header.frame_id = laser_frame;
        laser_origin.point.x = 0.0;
        laser_origin.point.y = 0.0;
		
	if(rob_cluster_frame != laser_frame)
	{
	
		transform_listener->waitForTransform (rob_cluster_frame, laser_frame, ros::Time::now(), ros::Duration(3.0));
        	//tf::StampedTransform transform;
        	try
        	{
                	transform_listener->lookupTransform(rob_cluster_frame, laser_frame, ros::Time(0), transform);
        	}
        	catch (...)
        	{
                	ROS_ERROR("unable to find trasformation between %s and %s", rob_cluster_frame.c_str(), laser_frame.c_str());
                	ros::Duration(1.0).sleep();
        	}
	
	}
	ssm_laser_pose.x = laser_origin.point.x * 1000;		//in mm
        ssm_laser_pose.y = laser_origin.point.y * 1000; 	//in mm
	
	
	Cluster_masking(&rob_cluster, &cnt, &ssm_laser_pose);
        
	geometry_msgs::PoseArray pose_array_debug;
        pose_array_debug.header.frame_id = rob_cluster_frame;
	for (int i=0; i<odo_cluster.numClusters; i++)
        {
                //ROS_INFO("odo_cluster_centroid x:%f y:%f", odo_cluster.cluster[i].centroid.x, odo_cluster.cluster[i].centroid.y);
                geometry_msgs::Pose my_pose;
                my_pose.position.x = odo_cluster.cluster[i].centroid.x / 1000;  //in [m] for visualization purpose      
                my_pose.position.y = odo_cluster.cluster[i].centroid.y / 1000;  //in [m] for visualization purpose
                pose_array_debug.poses.push_back(my_pose);
        }
        if (pose_array_debug.poses.size() > 0)
	{	
                pose_array_debug_pub.publish(pose_array_debug);
	}

    	
	//Cluster_Coordinate_Transformation( &rob_cluster[counter], &odo_cluster[counter], &odo_pose ) ;      -skipped -do the transformation in ROS
      	ClusterStruct odo_cluster = rob_cluster;
	
	// Find consistent leg clusters ---------------
      	//This function asociates clusters comparing the cluster from the previous list to the current one.
      	//It increases the number of counts that the cluster has been seen
      	cluster_data_asociation(&pre_cluster, &odo_cluster, &pre_cnt, &cnt);

	//storing the clusters for the future time step
      	copy_cluster_struct(&odo_cluster, &pre_cluster);
      	pre_cnt = cnt;


      	// Classify clusters and find human positions -----------------------
      	//Classifies the clusters into humans and chunks of points
      	//Finds the position of a human by averaging the position of two closest clusters
      	classify_clusters(&odo_cluster, human_cluster, &no_of_human_clusters, chunk_cluster, &no_of_chunk_clusters);


 


	

	//RaffaeleL: try to keep in mm
	for (int i=0; i<odo_cluster.numClusters; i++)
	{
		odo_cluster.cluster[i].centroid.x = odo_cluster.cluster[i].centroid.x; //* 1000;
		odo_cluster.cluster[i].centroid.y = odo_cluster.cluster[i].centroid.y; //* 1000;
		for (int j=0; j<odo_cluster.cluster[i].numPoints; j++)
		{
			odo_cluster.cluster[i].point[j].x = odo_cluster.cluster[i].point[j].x; //* 1000;
			odo_cluster.cluster[i].point[j].y = odo_cluster.cluster[i].point[j].y; //* 1000;
		}
	}

     	/*
	geometry_msgs::PoseArray odo_cluster_before_leg_cluster;
        odo_cluster_before_leg_cluster.header.frame_id = "/laser";

        for (int i=0; i<odo_cluster.numClusters; i++)
        {
                //ROS_INFO("odo_cluster_centroid x:%f y:%f", odo_cluster.cluster[i].centroid.x, odo_cluster.cluster[i].centroid.y);
	        geometry_msgs::Pose my_pose;
                my_pose.position.x = odo_cluster.cluster[i].centroid.x / 1000;	//in [m] for visualization purpose	
                my_pose.position.y = odo_cluster.cluster[i].centroid.y / 1000;	//in [m] for visualization purpose
                odo_cluster_before_leg_cluster.poses.push_back(my_pose);
        }
        if (odo_cluster_before_leg_cluster.poses.size() > 0)
                odo_cluster_before_leg_cluster_pub.publish(odo_cluster_before_leg_cluster);
	*/



////RAFFAELE L - ok until here //////


 	// Identify new leg clusters -----------------------------------
      	//This function puts cluster.flag to false to all clusters whose centroid are close to the position of humans
      	
	t_identify_new_legs(&odo_cluster, &human_pt);	
	
	  				  
      	//Birth of humans
      	//Creates a human based on the counter of the cluster,if bigger than t_CYCLES_TO_WAIT
      	//Using only present clusters.  Sets cluster.flag to false
      	t_Create_Humans( &odo_cluster, &cnt, &human_pt, &t_Max_ID_Index, t_People_ID );

	
	geometry_msgs::PoseArray humans_poses_before_filter;
	humans_poses_before_filter.header.frame_id = cluster_frame ;
	for(int i=0; i<human_pt.numHumans; i++)	
	{
	      	//ROS_INFO("human_pt x:%f y:%f", human_pt.state[i].mean.x, human_pt.state[i].mean.y);
		geometry_msgs::Pose my_pose;
                my_pose.position.x = human_pt.state[i].mean.x /1000 ; //in [m] for visualization purpose    
                my_pose.position.y = human_pt.state[i].mean.y /1000 ; //in [m] for visualization purpose  
                my_pose.position.z = human_pt.state[i].mean.z /1000 ; //in [m] for visualization purpose
                double roll, pitch, yaw;
                roll = pitch = 0;
                yaw = human_pt.state[i].mean.theta;
                my_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, yaw);
		humans_poses_before_filter.poses.push_back(my_pose); 
	}
        if (humans_poses_before_filter.poses.size() > 0)
		humans_poses_before_filter_pub.publish(humans_poses_before_filter);





 	//Particle Filter Related Functions
    
    	// Prediction Step: Motion Model -------------------------------------
    	t_predict_particle_motion(&human_pt, sampling_time);
        	//Correction Step: Update
   


        geometry_msgs::PoseArray humans_poses_before_particle_likelihood;
        humans_poses_before_particle_likelihood.header.frame_id = cluster_frame;
        for(int i=0; i<human_pt.numHumans; i++)
        {
                //ROS_INFO("human_pt_before_particle_likelihood x:%f y:%f", human_pt.state[i].mean.x, human_pt.state[i].mean.y);
                geometry_msgs::Pose my_pose;
                my_pose.position.x = human_pt.state[i].mean.x ;
                my_pose.position.y = human_pt.state[i].mean.y ;
                my_pose.position.z = human_pt.state[i].mean.z ;
                double roll, pitch, yaw;
                roll = pitch = 0;
                yaw = human_pt.state[i].mean.theta;
                my_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, yaw);
                humans_poses_before_particle_likelihood.poses.push_back(my_pose);
        }
	if (humans_poses_before_particle_likelihood.poses.size() > 0)
		humans_poses_before_particle_likelihood_pub.publish(humans_poses_before_particle_likelihood);


 
    	// Calculate particle likelihood ------------------------------------
    	t_find_particle_likelihood(&human_pt, human_cluster, no_of_human_clusters, chunk_cluster, no_of_chunk_clusters, sampling_time);

    	// Update human states by selecting a particle that describes best the human states -----
    	t_update_human_states(&human_pt, sampling_time);
    	
    	geometry_msgs::PoseArray humans_poses_before_death;
	humans_poses_before_death.header.frame_id = cluster_frame;
	for(int i=0; i<human_pt.numHumans; i++)	
	{
	      	//ROS_INFO("human_pt_before_death x:%f y:%f", human_pt.state[i].mean.x, human_pt.state[i].mean.y);
		geometry_msgs::Pose my_pose;
                my_pose.position.x = human_pt.state[i].mean.x ;
                my_pose.position.y = human_pt.state[i].mean.y ;
                my_pose.position.z = human_pt.state[i].mean.z ;
                double roll, pitch, yaw;
                roll = pitch = 0;
                yaw = human_pt.state[i].mean.theta;
                my_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, yaw);
		humans_poses_before_death.poses.push_back(my_pose); 
	}

	// Death -------------------------------------------------------------------------------
    	// Remove humans embeded in the map
    
    	//if (LOCALIZATION_AVAILABLE)
    	//t_remove_human_from_map( &human_pt, t_People_ID, &grid, &map_par map );
	
	t_remove_uncertain_humans(&human_pt, t_People_ID );

    
	// Resample based on the weights -------------------------------------------------------
    	t_resample_particles(&human_pt);
	

	geometry_msgs::PoseArray humans_poses;
	humans_poses.header.frame_id = cluster_frame;
	//ROS_INFO("numHumans: %d", human_pt.numHumans);
	uint32_t shape = visualization_msgs::Marker::CYLINDER;
	for (int i=0; i<human_pt.numHumans; i++)
	{
		//ROS_INFO("human_pt after filter x:%f y:%f", human_pt.state[i].mean.x, human_pt.state[i].mean.y);
		geometry_msgs::Pose my_pose;
		my_pose.position.x = human_pt.state[i].mean.x / 1000;	
                my_pose.position.y = human_pt.state[i].mean.y / 1000;	
                my_pose.position.z = human_pt.state[i].mean.z / 1000;
		double roll, pitch, yaw;
		roll = pitch = 0;
		yaw = human_pt.state[i].mean.theta;
		my_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (roll, pitch, yaw);
		humans_poses.poses.push_back(my_pose);

		visualization_msgs::Marker marker;
		marker.header.frame_id = cluster_frame;
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
    		marker.id = i;
		marker.type = shape;
		marker.action = visualization_msgs::Marker::ADD;
		
		marker.pose.position = my_pose.position;
    		marker.pose.orientation = my_pose.orientation;
		marker.scale.x = 0.1;
    		marker.scale.y = 0.1;
    		marker.scale.z = 1.0;

		marker.color.r = 0.0f;
    		marker.color.g = 1.0f;
    		marker.color.b = 0.0f;
   		marker.color.a = 1.0;
		
		marker.lifetime = ros::Duration(sampling_time);
		humans_marker_pub.publish(marker);
	}
	if (humans_poses.poses.size() > 0)
		humans_poses_pub.publish(humans_poses);


}

int main(int argc, char **argv)
{
	t_initialize_entities(&human_pt);
	ros::init(argc, argv, "on_board_human_tracker_2d");
	ros::NodeHandle n;
	
 	if (n.getParam("/on_board_human_tracker_2d/cluster_frame", cluster_frame))
 	{
        	 ROS_INFO("[on_board_human_tracker_2d] got (/on_board_human_tracker_2d/cluster_frame) from param server, value is %s", cluster_frame.c_str() );
 	}
 	else
 	{
        	cluster_frame = cluster_frame_default;
        	ROS_INFO("[on_board_human_tracker_2d] not found (/on_board_human_tracker_2d/cluster_frame) from param server, set to default value %s", cluster_frame.c_str() );
 	}

        if (n.getParam("/on_board_human_tracker_2d/laser_frame", laser_frame))
        {
                 ROS_INFO("[on_board_human_tracker_2d] got (/on_board_human_tracker_2d/laser_frame) from param server, value is %s", laser_frame.c_str() );
        }
        else
        {
                laser_frame = laser_frame_default;
                ROS_INFO("[on_board_human_tracker_2d] not found (/on_board_human_tracker_2d/laser_frame) from param server, set to default value %s", laser_frame.c_str() );
        }



	transform_listener = new tf::TransformListener;
	if(cluster_frame != laser_frame)
	        transform_listener->waitForTransform (cluster_frame, laser_frame , ros::Time(0), ros::Duration(10.0));
	ros::Subscriber ppl_cluster_array_sub = n.subscribe("ppl_cluster_array", 10,  ppl_cluster_arrayCallback);
  	odo_cluster_before_leg_cluster_pub = n.advertise<geometry_msgs::PoseArray>("odo_cluster_before_leg_cluster", 10);
	humans_poses_before_leg_cluster_pub = n.advertise<geometry_msgs::PoseArray>("human_poses_from_laser_before_leg_cluster", 10);
	humans_poses_before_filter_pub = n.advertise<geometry_msgs::PoseArray>("human_poses_from_laser_before_filter", 10);
	humans_poses_before_create_humans_pub = n.advertise<geometry_msgs::PoseArray>("human_poses_from_laser_before_create_humans", 10); 
	humans_poses_before_particle_likelihood_pub = n.advertise<geometry_msgs::PoseArray>("human_poses_from_laser_before_particle_likelihood", 10); 
	humans_poses_pub = n.advertise<geometry_msgs::PoseArray>("human_poses_from_laser", 10);
	pose_array_debug_pub = n.advertise<geometry_msgs::PoseArray>("pose_array_debug", 10);
	humans_marker_pub = n.advertise<visualization_msgs::Marker>("humans_visualization_marker", 10);	
	ros::spin();	
}
