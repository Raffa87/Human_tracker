#include "compute_likelihood.h"
#include <ros/ros.h>



//tracked entities variables
t_Entity human_pt;

std::vector<tracking::Candidate> my_candidates;
std::vector<tracking::Candidate> my_all_time_candidates;
//std::deque<uint> my_sliding_window_num_candidates; 


tracking::CandidateArray candidate_array_msg;
tracking::CandidateArray all_candidate_array_msg;


double duration_threshold_sec;
double duration_threshold_sec_default = 0.5;
ros::Duration duration_threshold;

double desired_value; //value of moltiplicator at duration_threshold 
double desired_value_default = 0.01;

double forgetting_factor; //= (1 - desired_value)/(desired_value*(duration_threshold.sec + (double)duration_threshold.nsec/1000000000));

std::string likelihood_frame;
std::string likelihood_frame_default = "/map";

ros::Publisher candidate_publisher;
ros::Publisher all_candidate_publisher;

double dist_gaussian_threshold = 1.0;

int counter_debug = 0;

tracking::PoseWithCovarianceStampedArray human_poses_old;

bool reset_likelihood(tracking::EmptyLikelihood::Request  &req,
         tracking::EmptyLikelihood::Response &res)
{ 
  my_all_time_candidates.empty();
  ROS_INFO("reset likelihood");
  return true;
}

void human_poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& human_pose)
{

	//Create candidates
	my_candidates.clear(); 
	
	tracking::Candidate temp_candidate;
	temp_candidate.pose = *human_pose;
	temp_candidate.flag = true;
	my_candidates.push_back(temp_candidate);


	t_identify_new_legs(my_candidates, &human_pt);
	update_all_time_candidates(my_candidates, my_all_time_candidates);
	
	compute_likelihood(my_candidates, my_all_time_candidates, forgetting_factor);

	std::vector<tracking::Candidate>::iterator publish_iterator = my_candidates.end();

	std::vector<tracking::Candidate>::iterator update_likelihood_iterator = my_all_time_candidates.end();
	
	while(publish_iterator != my_candidates.begin())
	{
		publish_iterator--;
		candidate_array_msg.candidates.push_back(*publish_iterator);
		
		update_likelihood_iterator--;
		update_likelihood_iterator->likelihood = publish_iterator->likelihood;
	}


}





void human_posesCallback(const tracking::PoseWithCovarianceStampedArray::ConstPtr& human_poses)
{

	if (human_poses->poses[0].header.frame_id != likelihood_frame)
	{
		ROS_ERROR("[compute_likelihood] received human_pose in %s frame but likelihood is computed in %s -  message dropped" , human_poses->poses[0].header.frame_id.c_str(), likelihood_frame.c_str());
		return;
	}
	if (human_poses->poses.size()==0)
	{
		ROS_ERROR("[compute_likelihood] empty message");
		return;
	}
	if (human_poses->only_for_tracking.data)
	{
		//ROS_ERROR("[compute_likelihood_detection] data only for tracking");
		return;
	}
	tracking::PoseWithCovarianceStampedArray human_poses_new = *human_poses;
	tracking::PoseWithCovarianceStampedArray human_poses_ok = *human_poses;
	
	
	int my_size = human_poses_ok.poses.size();
	for(int i=0; i<my_size; i++)
	{
		for(int j=0; j<human_poses_old.poses.size(); j++)
		{
			bool check_x = (human_poses_ok.poses[i].pose.pose.position.x == human_poses_old.poses[j].pose.pose.position.x); 
			bool check_y = (human_poses_ok.poses[i].pose.pose.position.y == human_poses_old.poses[j].pose.pose.position.y);
			if(check_x && check_y)
			{
				//ROS_ERROR("avoid bugs");
				human_poses_ok.poses.erase(human_poses_ok.poses.begin()+i);
				i = i-1;
				my_size = human_poses_ok.poses.size();
				break;
			}
		}

	}
	human_poses_old = human_poses_new;

	//Create candidates
	
	my_candidates.clear(); 
	
	for (int i = 0; i < human_poses_ok.poses.size(); i++)
	{
		tracking::Candidate temp_candidate;
		temp_candidate.pose = human_poses_ok.poses[i];
		temp_candidate.flag = true;
		temp_candidate.counter_debug = counter_debug;
		counter_debug++;
		
		my_candidates.push_back(temp_candidate);
	}


	t_identify_new_legs(my_candidates, &human_pt);
	update_all_time_candidates(my_candidates, my_all_time_candidates);
	compute_likelihood(my_candidates, my_all_time_candidates, forgetting_factor);

	std::vector<tracking::Candidate>::iterator publish_iterator = my_candidates.end();

	std::vector<tracking::Candidate>::iterator update_likelihood_iterator = my_all_time_candidates.end();
	
	while(publish_iterator != my_candidates.begin())
	{
		publish_iterator--;
		candidate_array_msg.candidates.push_back(*publish_iterator);
		
		update_likelihood_iterator--;
		update_likelihood_iterator->likelihood = publish_iterator->likelihood;
	}

}



int main(int argc, char **argv)
{
        t_initialize_entities(&human_pt);
	ros::init(argc, argv, "compute_likelihood_detection");
        ros::NodeHandle n;

        ros::Subscriber human_poses_subscriber = n.subscribe("human_poses", 10,  human_posesCallback);
	ros::Subscriber human_pose_subscriber = n.subscribe("human_pose", 10, human_poseCallback);
	candidate_publisher = n.advertise<tracking::CandidateArray>("human_candidate_detection", 10);
	all_candidate_publisher = n.advertise<tracking::CandidateArray>("all_human_candidate_detection", 10);


	if (n.getParam("/compute_likelihood/desired_value", desired_value))
        {
                 ROS_INFO("[compute_likelihood] got (/compute_likelihood/desired_value) from param server, value is %f", desired_value);
        }
        else
        {
                desired_value = desired_value_default;
                ROS_INFO("[compute_likelihood] not found (/compute_likelihood/desired_value) from param server, set to default value %f", desired_value);
        }



        if (n.getParam("/compute_likelihood/duration_threshold", duration_threshold_sec))
        {
                 ROS_INFO("[compute_likelihood] got (/compute_likelihood/duration_threshold_sec) from param server, value is %f", duration_threshold_sec);
        }
        else
        {
                duration_threshold_sec = duration_threshold_sec_default;
                ROS_INFO("[compute_likelihood] not found (/compute_likelihood/duration_threshold_sec) from param server, set to default value %f", duration_threshold_sec);
        }
	duration_threshold = ros::Duration(duration_threshold_sec);

	forgetting_factor = (1 - desired_value)/(desired_value*(duration_threshold.sec + (double)duration_threshold.nsec/1000000000));

        if (n.getParam("/compute_likelihood/likelihood_frame", likelihood_frame))
        {
                 ROS_INFO("[compute_likelihood] got (/compute_likelihood/likelihood_frame) from param server, value is %s", likelihood_frame.c_str());
        }
        else
        {
                likelihood_frame  = likelihood_frame_default;
                ROS_INFO("[compute_likelihood] not found (/compute_likelihood/likelihood_frame) from param server, set to default value %s", likelihood_frame.c_str());
        }
	


	ros::ServiceServer reset_service = n.advertiseService("reset_likelihood", reset_likelihood);
  	ROS_INFO("Ready to reset_likelihood.");
	
	ros::Rate loop_rate(10); //100ms
  	while (ros::ok())
	{


		 ros::Time now = ros::Time::now();
			
		 for (int i=0; i<candidate_array_msg.candidates.size(); i++)
		 {
		 	candidate_array_msg.candidates[i].pose.header.stamp = now;
			all_candidate_array_msg.candidates.push_back(candidate_array_msg.candidates[i]); 	
		 } 
		 if(candidate_array_msg.candidates.size() > 0)
		 {
			candidate_publisher.publish(candidate_array_msg);
		 	candidate_array_msg.candidates.clear();
		 } 


                 int my_size = all_candidate_array_msg.candidates.size();
		 for  (int i=0; i<my_size; i++)
		 {
			ros::Duration my_duration = now - all_candidate_array_msg.candidates[i].pose.header.stamp;
			if (my_duration > duration_threshold)
			{
				all_candidate_array_msg.candidates.erase(all_candidate_array_msg.candidates.begin()+i);
				my_size = all_candidate_array_msg.candidates.size();
			}
		 }
                 if(all_candidate_array_msg.candidates.size() > 0)
                 {
		 	all_candidate_publisher.publish(all_candidate_array_msg);
                 }

        	 ros::spinOnce();                    
        	 loop_rate.sleep();
	}
}

void t_initialize_entities(t_Entity *human){
  human->numHumans = 0;
  for(int i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++)
    human->state[i].ID = -1;

 }


//This function puts flag to false to all clusters whose centroid are close to the position of humans
void t_identify_new_legs(std::vector<tracking::Candidate> &candidates, t_Entity *human)
{
 for(int i=0; i< candidates.size(); i++)    {
    
    double xc = candidates[i].pose.pose.pose.position.x;
    double yc = candidates[i].pose.pose.pose.position.y;
     
    double mindist = 100000;

        for(int j=0; j<ssm_MAX_NUM_OF_PEOPLE; j++){
          if( human->state[j].ID >= 0 ){
            double xl = human->state[j].mean.x;
            double yl = human->state[j].mean.y;

            double d = Dist_Between_Points( xc, xl, yc, yl);
            if(d < mindist)
              mindist = d;
          }
        }

        if(mindist<=t_MAX_STEP_SIZE ){
          candidates[i].flag = false;
        }
  }
}

void update_all_time_candidates(std::vector<tracking::Candidate> &candidates, std::vector<tracking::Candidate>  &all_time_candidates)
{

	
	std::vector<tracking::Candidate>::iterator it_my_current_candidates = candidates.begin();
	while (it_my_current_candidates != candidates.end())
		all_time_candidates.push_back(*it_my_current_candidates++);
}	

//Function to compute the current poses likelihood using the sliding window containing past poses 
void compute_likelihood(std::vector<tracking::Candidate> &candidates, std::vector<tracking::Candidate> &all_candidates, double forgetting_factor)
{
	
	std::vector<tracking::Candidate>::iterator it_my_current_candidates = candidates.begin();
  	std::vector<tracking::Candidate>::iterator it_my_all_candidates;
	ros::Time now = ros::Time::now();

	while (it_my_current_candidates != candidates.end())
	{
		it_my_current_candidates->likelihood = 0;
		it_my_all_candidates = all_candidates.begin();
		std::vector<tracking::Candidate>::iterator it_my_all_candidates_end = all_candidates.end();
		
		while(it_my_all_candidates != it_my_all_candidates_end)
		{
			ros::Duration my_duration = now - it_my_all_candidates->pose.header.stamp;
			if (my_duration > duration_threshold)
			{
				//erase element
				all_candidates.erase(it_my_all_candidates);
				//refresh end pointer
				it_my_all_candidates_end = all_candidates.end();
			}
			else
			{
				double temp_distance = Dist_Between_Points( it_my_all_candidates->pose.pose.pose.position.x, it_my_all_candidates->pose.pose.pose.position.x, 
									    it_my_all_candidates->pose.pose.pose.position.y, it_my_all_candidates->pose.pose.pose.position.y);
				if(temp_distance > dist_gaussian_threshold)
				{
					break;
				}
				double temp_likelihood = gauss(temp_distance, it_my_all_candidates->pose.pose.covariance[0], 2); 
				double forgetting_element = double(1)/(1+double(forgetting_factor)*((double)my_duration.sec + (double)my_duration.nsec/1000000000));
				it_my_current_candidates->likelihood += temp_likelihood;
			}		
			it_my_all_candidates++;	
		}
		//ROS_INFO("updated_likelihood on candidate number: %d", it_my_current_candidates->counter_debug);			   
		it_my_current_candidates++;
	}

	
}

/*
    function name: gauss
    Description: Implements 2-D gaussian function.
    Inputs:
        (x, y)      : Vriables
        (xp, yp))   : Means of (x, y)
        (sx, sy)    : Standard deviations of (x, y)
    Return value:
        A non-normalized gaussian value
*/
double gauss(double x, double y, double xp, double yp, double sx, double sy)
{
    double A, X, Y, val;

    if(sx == 0) sx = 1;
    if(sy == 0) sy = 1;

    X = ((x-xp)/sx);
    Y = ((y-yp)/sy);
    A = 1.0;

    val = A*exp(-0.5*(X*X + Y*Y));

    return val;
} 

double gauss(double distance, double sigma, uint dimension)
{	
	double A, val;
	if (sigma == 0) sigma = 1;
	if (dimension == 2)
	{
		A = 1/(pow(sigma,2)*2*M_PI);
		val = A*exp(-0.5*(sqrt(distance)/(2*sigma)));
	}
	return val;
}
  
   


