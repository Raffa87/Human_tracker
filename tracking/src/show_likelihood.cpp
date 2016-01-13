#include <ros/ros.h>
#include <tracking/Candidate.h>
#include <tracking/CandidateArray.h>
#include <visualization_msgs/Marker.h>

ros::Time pre_time;

uint32_t shape = visualization_msgs::Marker::CYLINDER;

ros::Publisher likelihood_publisher;

void candidates_arrayCallback(const tracking::CandidateArray::ConstPtr& candidates_array_msg)
{
       	ros::Time now = ros::Time::now(); 

	std::vector<tracking::Candidate> my_candidates = candidates_array_msg->candidates;
	std::vector<tracking::Candidate>::iterator it_candidates = my_candidates.begin();

	visualization_msgs::Marker marker;
        marker.header.frame_id = candidates_array_msg->candidates[0].pose.header.frame_id;
        marker.header.stamp = now;
        marker.ns = "basic_shapes";
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
	int i = 0;
	while(it_candidates != my_candidates.end())
        {	
		marker.pose = it_candidates->pose.pose.pose;
		marker.pose.position.x = marker.pose.position.x;
	        marker.pose.position.y = marker.pose.position.y;

                marker.scale.x = it_candidates->pose.pose.covariance[0];
        	marker.scale.y = marker.scale.x;
        	marker.scale.z = (1 * it_candidates->likelihood)/1000.0;//1.0;

        	marker.id = i;
		ros::Duration my_duration = marker.header.stamp - it_candidates->pose.header.stamp;
		double my_duration_factor = (1 + 18*(my_duration.sec + (double)my_duration.nsec/1000000000));
		//ROS_INFO("my_duration_factor is %f", my_duration_factor);
			
		marker.color.r = 0.0f;
        	marker.color.g = 1.0f;
        	marker.color.b = 0.0f;
       	 	marker.color.a = 1.0 / my_duration_factor;

        	marker.lifetime = now - pre_time;
        	likelihood_publisher.publish(marker);
		it_candidates++;
		i++;
	}
	pre_time = now;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "show_likelihood");
        ros::NodeHandle n;
        ros::Subscriber human_cluster_subscriber = n.subscribe("/all_human_candidate_tracking", 10,  candidates_arrayCallback);
	
        likelihood_publisher = n.advertise<visualization_msgs::Marker>("/likelihood_markers", 10);

	pre_time = ros::Time::now(); 
      ros::spin();
}

