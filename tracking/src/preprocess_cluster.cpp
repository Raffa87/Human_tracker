#include "preprocess_cluster.h"
#include <ros/ros.h>


cycle_counter cnt;      
cycle_counter pre_cnt;  


std::string param_space;

std::string laser_frame;
std::string laser_frame_default = "/laser";

std::string cluster_frame;
std::string cluster_frame_default = "/laser";	//"/map";

bool chunck_only_for_tracking;
bool chunck_only_for_tracking_default = true;

bool couple_cluster;
bool couple_cluster_default = true;

point_2d     human_cluster[NUM_CLUSTERS];
point_2d     chunk_cluster[NUM_CLUSTERS];
int     no_of_chunk_clusters = 0;
int     no_of_human_clusters   = 0;
double  pre_time_stamp = 0;
double  sampling_time   = 0;

double  sigma;			//[m]
double  sigma_default = 0.150; 	//[m]

ros::Publisher human_cluster_publisher;
ros::Publisher chunk_cluster_publisher;



void ppl_cluster_arrayCallback(const laser_clustering::ClusterArray::ConstPtr& cluster_array)
{
	

	
        tf::StampedTransform transform;
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

	//Cluster_masking(&rob_cluster, &cnt, &laser_origin); 	skipped for now (RAFFAL) 

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


	tracking::PoseWithCovarianceStampedArray human_cluster_pose_array;
	tracking::PoseWithCovarianceStampedArray chunk_cluster_pose_array;	

	ros::Time now = ros::Time::now();
	if(no_of_human_clusters > 0)
	{	
		for (int i = 0; i < no_of_human_clusters; i++)
		{		
			geometry_msgs::PoseWithCovarianceStamped pose_temp;
			pose_temp.header.frame_id = cluster_frame;
			pose_temp.header.stamp = now;
			pose_temp.pose.pose.position.x = human_cluster[i].x / 1000; //[mm] back in [m]
			pose_temp.pose.pose.position.y = human_cluster[i].y / 1000; //[mm] back in [m]
			pose_temp.pose.covariance[0] = sigma; //[mm] back in [m]
			human_cluster_pose_array.poses.push_back(pose_temp);
		}

		human_cluster_pose_array.only_for_tracking.data = false;
		human_cluster_publisher.publish(human_cluster_pose_array);
		
		//ROS_INFO("human cluster size is %d", human_cluster_pose_array.poses.size());
		//std::cin.get();	
	}
	
	if(no_of_chunk_clusters > 0)
	{
		for (int i = 0; i < no_of_chunk_clusters; i++)
		{
			geometry_msgs::PoseWithCovarianceStamped pose_temp;
			pose_temp.header.frame_id = cluster_frame;
			pose_temp.header.stamp = now;
			pose_temp.pose.pose.position.x = chunk_cluster[i].x /1000 ; 
			pose_temp.pose.pose.position.y = chunk_cluster[i].y /1000 ;
			pose_temp.pose.covariance[0] = sigma;
			chunk_cluster_pose_array.poses.push_back(pose_temp);	
		}
	        chunk_cluster_pose_array.only_for_tracking.data = chunck_only_for_tracking;
		chunk_cluster_publisher.publish(chunk_cluster_pose_array);
	}


			
//        for (int i = 0; i < no_of_human_clusters; i++)
//                ROS_INFO("human cluster: %f, %f", human_cluster[i].x, human_cluster[i].y);
//        ROS_INFO("no_of_human_clusters: %d", no_of_human_clusters);
		
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "preprocess_cluster");
	ros::NodeHandle n;

 	param_space = ros::this_node::getName();

	
 	if (n.getParam(param_space + "/cluster_frame", cluster_frame))
 	{
        	 ROS_INFO("[%s] got (/cluster_frame) from param server, value is %s", param_space.c_str(), cluster_frame.c_str() );
 	}
 	else
 	{
        	cluster_frame = cluster_frame_default;
        	ROS_INFO("[%s] not found (/cluster_frame) from param server, set to default value %s", param_space.c_str(), cluster_frame.c_str() );
 	}

        if (n.getParam(param_space + "/laser_frame", laser_frame))
        {
                 ROS_INFO("[%s] got (/laser_frame) from param server, value is %s", param_space.c_str(), laser_frame.c_str() );
        }
        else
        {
                laser_frame = laser_frame_default;
                ROS_INFO("[%s] not found (/laser_frame) from param server, set to default value %s", param_space.c_str(), laser_frame.c_str() );
        }
	

        if (n.getParam(param_space + "/sigma", sigma))
        {
                 ROS_INFO("[%s] got (/sigma) from param server, value is %f", param_space.c_str(), sigma);
        }
        else
        {
                sigma = sigma_default;
                ROS_INFO("[%s] not found (/sigma) from param server, set to default value %f", param_space.c_str(), sigma );
        }

        if (n.getParam(param_space + "/chunck_only_for_tracking", chunck_only_for_tracking))
        {
                 ROS_INFO("[%s] got (/chunck_only_for_tracking) from param server, value is %d", param_space.c_str(), chunck_only_for_tracking);
        }
        else
        {
                chunck_only_for_tracking = chunck_only_for_tracking_default;
                ROS_INFO("[%s] not found (/chunck_only_for_tracking_default) from param server, set to default value %d", param_space.c_str(), chunck_only_for_tracking );
        }

        if (n.getParam(param_space + "/couple_cluster", couple_cluster))
        {
                 ROS_INFO("[%s] got (/couple_cluster) from param server, value is %d", param_space.c_str(), couple_cluster);
        }
        else
        {
                couple_cluster = couple_cluster_default;
                ROS_INFO("[%s] not found (/couple_cluster) from param server, set to default value %d", param_space.c_str(), couple_cluster );
        }




	transform_listener = new tf::TransformListener;
	if(cluster_frame != laser_frame)
	        transform_listener->waitForTransform (cluster_frame, laser_frame , ros::Time(0), ros::Duration(10.0));
	ros::Subscriber ppl_cluster_array_sub = n.subscribe("ppl_cluster_array", 10,  ppl_cluster_arrayCallback);
	human_cluster_publisher = n.advertise<tracking::PoseWithCovarianceStampedArray>("human_poses", 10); 
	chunk_cluster_publisher = n.advertise<tracking::PoseWithCovarianceStampedArray>("chunk_poses", 10); 
	ros::spin();	
}




void Cluster_masking(ClusterStruct *cl, cycle_counter *cnt, geometry_msgs::PointStamped *pose ){


  ClusterStruct tmp = *cl ;
    cl->numClusters = 0;
  double distance;
  double angle;


  for (int k=0; k < tmp.numClusters; k++){


    if(pose!=NULL){
      distance = Dist_Between_Points( tmp.cluster[k].centroid.x, pose->point.x, tmp.cluster[k].centroid.y *1000, pose->point.y *1000 );
    }
    else{
      distance = Dist_Between_Points( tmp.cluster[k].centroid.x, 0.0, tmp.cluster[k].centroid.y, 0.0 );
    }
     //RaffaeleL: removed the control on angle
     //if(distance <= t_MAX_TRACKING_RANGE && angle>= t_MIN_ANGLE && angle<= t_MAX_ANGLE){
     if(distance <= t_MAX_TRACKING_RANGE)
      {
        cl->cluster[cl->numClusters] = tmp.cluster[k] ;
        cnt->counter[cl->numClusters] =0 ;
        cl->numClusters ++ ;
      }//end if
      else
        printf("dist: %f ",distance);
   }//end for

}

void cluster_data_asociation( ClusterStruct *pre_cl, ClusterStruct * cl, cycle_counter *pre_cnt, cycle_counter *cnt){
    int i, j;

for(i=0; i<cl->numClusters; i++){
  double x = cl->cluster[i].centroid.x; 
  double y = cl->cluster[i].centroid.y; 
  double mindist = 100000;
  int id = -1;

  for(j=0; j<pre_cl->numClusters; j++) {
    double xp = pre_cl->cluster[j].centroid.x; 
    double yp = pre_cl->cluster[j].centroid.y; 
    double dist = Dist_Between_Points(x, xp, y, yp );
    if(dist < mindist) {
        mindist = dist;
        id = j;
    }//end if dist
  }//end for j

  if(mindist <= t_MAX_STEP_SIZE){
    cnt->counter[i] = pre_cnt->counter[id] + 1;

    if(cnt->counter[i] > t_CYCLES_TO_WAIT )
      cnt->counter[i] = t_CYCLES_TO_WAIT ;

  }
  else
    cnt->counter[i] = 0;
 }//end for i
}

void copy_cluster_struct(ClusterStruct *org, ClusterStruct *dst ){
  dst->numClusters   = org->numClusters;

  for (int k=0; k < dst->numClusters; k++){
    dst->cluster[k].numPoints     = org->cluster[k].numPoints;
    dst->cluster[k].semi_major    = org->cluster[k].semi_major;
    dst->cluster[k].semi_minor    = org->cluster[k].semi_minor;
    dst->cluster[k].flag          = org->cluster[k].flag;
    dst->cluster[k].centroid.x = org->cluster[k].centroid.x;
    dst->cluster[k].centroid.y = org->cluster[k].centroid.y;

    for (int i=0; i < dst->cluster[k].numPoints; i++){
      dst->cluster[k].point[i].x = org->cluster[k].point[i].x ;
      dst->cluster[k].point[i].y = org->cluster[k].point[i].y ;
    }
  }

}

//Classifies the clusters into humans and chunks of points
//Finds the position of a human by averaging the position of two closest clusters
//Humans are returned in human_cluster and no_of_human_clusters
void classify_clusters(ClusterStruct * cl,              //find_human_clusters(ClusterStruct * cl,
                          point_2d human_cluster[NUM_CLUSTERS], //* MAX_LASER_CLUSTER_STREAMS],
                          int *no_of_human_clusters,
                          point_2d chunk_cluster[NUM_CLUSTERS], //* MAX_LASER_CLUSTER_STREAMS],
                          int *no_of_chunk_clusters){
  int i;

  int cnt_human = -1;
  int cnt_cluster = -1 ;

  //printf("In: t_find_human_clusters  Clusters to process %d\n",cl->numClusters);

  if (couple_cluster)
  {
  	for(i=0; i< cl->numClusters; i++)   {

	    double mindist = 1000000;
	    int id = -1;
	    find_closest_cluster(i, &id, &mindist, cl);

	    //printf("\ncl[%d] id: %d  min_dist: %f",i, id, mindist);

	    if(id!=-1 && mindist<=t_MAX_STEP_SIZE){
	      cnt_human ++;

	      human_cluster[cnt_human].x = 0.5* (cl->cluster[i].centroid.x + cl->cluster[id].centroid.x );
	      human_cluster[cnt_human].y = 0.5* (cl->cluster[i].centroid.y + cl->cluster[id].centroid.y );

	      cl->cluster[i].flag = false ;
	      cl->cluster[id].flag = false ;

	    } //end if id

	    else{
	      if(cl->cluster[i].flag != false){
	        cnt_cluster ++;
	        chunk_cluster[cnt_cluster].x = cl->cluster[i].centroid.x ;
	        chunk_cluster[cnt_cluster].y = cl->cluster[i].centroid.y ;
	        cl->cluster[i].flag = false ;
	      }//end if flag != false
	
	    }//end else  if(id!=-1
   }	


  *no_of_human_clusters = cnt_human   +1 ;
  *no_of_chunk_clusters = cnt_cluster +1;
  
  }
  else
   {
  	for(i=0; i< cl->numClusters; i++)   {

	      if(cl->cluster[i].flag != false){
	        cnt_human ++;
	        human_cluster[cnt_human].x = cl->cluster[i].centroid.x ;
	        human_cluster[cnt_human].y = cl->cluster[i].centroid.y ;
	        cl->cluster[i].flag = false ;
	      }//end if flag != false
	
	    }//end else  if(id!=-1
   

  *no_of_human_clusters = cnt_human   +1 ;
   }	

  
  //printf("    no of humans = %d  no_of_chunks %d\n", *no_of_human_clusters, *no_of_chunk_clusters);

 for(i=0; i<cl->numClusters; i++)
    cl->cluster[i].flag = true;

}

void find_closest_cluster(
                       int index,
                       int *id,
                       double *mindist,
                       ClusterStruct *cl){
  *id = -1;
  *mindist = 1000000;

  double x1 = cl->cluster[index].centroid.x;
  double y1 = cl->cluster[index].centroid.y;

  for(int j=0; j < cl->numClusters; j++){
    if(index != j){
      if(cl->cluster[j].flag){
      double x2 = cl->cluster[j].centroid.x;
      double y2 = cl->cluster[j].centroid.y;
      double dist = Dist_Between_Points( x1, x2, y1, y2 );
      if(dist < *mindist){
        *mindist = dist;
        *id = j;
      }//end if dist

        }//end if flag

    }//end if index
  }//end for
}

