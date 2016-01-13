
#include "tracking_particle_filter.h" 
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <ros/ros.h>


std::vector<t_Human> my_human;
std::vector<tracking::Candidate> my_candidates;

nav_msgs::OccupancyGrid static_map;


double  pre_time_stamp = 0;
double  sampling_time   = 0;

ros::Publisher humans_marker_pub;
ros::Publisher particles_pub;
ros::Publisher humans_pub;

double likelihood_threshold;
double likelihood_threshold_default = 10.0;

std::string human_tracked_frame;
std::string human_tracked_frame_default = "/laser";

int current_ID_human;

void likelihood_detectionCallback(const tracking::CandidateArray::ConstPtr& candidates_array_msg)
{
	my_candidates.clear();	
	my_candidates = candidates_array_msg->candidates;
	
	t_Create_Humans(my_candidates, my_human);
	

}


void likelihood_trackingCallback(const tracking::CandidateArray::ConstPtr& candidates_array_msg)
{
	my_candidates.clear();	
	my_candidates = candidates_array_msg->candidates;
	
	//t_Create_Humans(my_candidates, my_human);
	
	sampling_time = candidates_array_msg->candidates[0].pose.header.stamp.sec + (double)candidates_array_msg->candidates[0].pose.header.stamp.nsec/1000000000 - pre_time_stamp;
	pre_time_stamp = candidates_array_msg->candidates[0].pose.header.stamp.sec + (double)candidates_array_msg->candidates[0].pose.header.stamp.nsec/1000000000;  

        // Prediction Step: Motion Model 
        t_predict_particle_motion(my_human, sampling_time);

	//Correction Step: Update
	t_find_particle_likelihood(my_human, my_candidates, sampling_time);
	t_update_human_states(my_human, sampling_time);

	t_remove_uncertain_humans(my_human, my_candidates);
	t_remove_too_close_humans(my_human);
	t_remove_meaningless_particle(my_human);
		
	visualize_particles(my_human);
        visualize_human(my_human);
	publish_human(my_human);	
	//log_human(my_human);
	//print_human_info(my_human);
}



int main(int argc, char **argv)
{
	//t_initialize_entities(&human_pt);

	ros::init(argc, argv, "tracking_particle_filter");
        ros::NodeHandle n;

        ros::Subscriber detection_sub = n.subscribe("/human_candidate_detection", 10,  likelihood_detectionCallback);
	ros::Subscriber tracking_sub = n.subscribe("/human_candidate_tracking", 10,  likelihood_trackingCallback);
	

	humans_marker_pub = n.advertise<visualization_msgs::Marker>("/human_tracked_marker", 10);
	particles_pub = n.advertise<sensor_msgs::PointCloud>("/human_tracked_particles", 10);
	humans_pub = n.advertise<tracking::PoseWithCovarianceStampedArray>("/human_tracked", 10);
	
        if (n.getParam("/tracking_particle_filter/likelihood_threshold", likelihood_threshold))
        {
                 ROS_INFO("[tracking_particle_filter] got (/tracking_particle_filter/likelihood_threshold) from param server, value is %f", likelihood_threshold);
        }
        else
        {
                likelihood_threshold = likelihood_threshold_default;
                ROS_INFO("[tracking_particle_filter] not found (/tracking_particle_filter/likelihood_threshold) from param server, set to default value %f", likelihood_threshold);
        }


        if (n.getParam("/tracking_particle_filter/human_tracked_frame", human_tracked_frame))
        {
                 ROS_INFO("[/tracking_particle_filter] got (/tracking_particle_filter/human_tracked_frame) from param server, value is %s", human_tracked_frame.c_str());
        }
        else
        {
                human_tracked_frame = human_tracked_frame_default;
                ROS_INFO("[/tracking_particle_filter] not found (/tracking_particle_filter/human_tracked_frame) from param server, set to default value %s", human_tracked_frame_default.c_str());
        }
	
	//printf("\tTime\tx\ty\ttheta\n");

 	if (!ros::service::waitForService("/static_map", ros::Duration(60,0)))//wait for service to get map
        {
                ROS_ERROR("unable to find /static_map service for get the map");
                return -1;
        }
        ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("/static_map");
        nav_msgs::GetMap get_map_srv;
        map_client.call(get_map_srv);
        static_map = get_map_srv.response.map;



	ros::spin();
}


void t_Create_Humans(std::vector<tracking::Candidate> &candidates, std::vector<t_Human> &human)
{

  for( int i=0; i<candidates.size(); i++ ) 
  {
 
    double distance = find_closest_human(&candidates[i], human);
    if(distance>=t_MIN_DISTANCE_FROM_HUMAN && candidates[i].likelihood >= likelihood_threshold) 
    {
      t_init_human(human, &candidates[i]);

    }
    //else
    //{
    //	ROS_ERROR("cannot create a new human - too close to another one");
    //}
  }
}



double find_closest_human(tracking::Candidate *candidates, std::vector<t_Human> &human)
{
	double mindist = 1000000;
        double x1 = candidates->pose.pose.pose.position.x;
        double y1 = candidates->pose.pose.pose.position.y;
 
	for(int j=0; j < human.size(); j++)
        {
            //if(index != j)
            //{
                double x2 = human[j].mean.x;
                double y2 = human[j].mean.y;
                double dist = Dist_Between_Points( x1, x2, y1, y2 );
                if(dist < mindist)
                {	
                       mindist = dist;
                }
            //}
        }
	return mindist;

}    


//Initializes the humans and their particles
void t_init_human(std::vector<t_Human> &human, tracking::Candidate *candidate)
{
	
                t_Human my_new_human;
    
		my_new_human.ID = current_ID_human;
		current_ID_human++;		
		//Assign state variables
		my_new_human.mean.x      = candidate->pose.pose.pose.position.x;
		my_new_human.mean.y      = candidate->pose.pose.pose.position.y;
		my_new_human.mean.theta  = 0 ; // no prior knowledge
		my_new_human.mean.speed  = 0 ; // no prior knowledge
		my_new_human.mean.omega  = 0 ; // no prior knowledge

		//Assign other variables        ---------------------
		my_new_human.tracked_time = 0;
		my_new_human.dispersion = 0;

		for(int i=0; i<t_WEIGHT_MEMORY_BUFFER; i++)
      			my_new_human.weight_memory[i] = t_MIN_WEIGHT;
	    	my_new_human.weight_memory_count = 0;

		my_new_human.under_likelihood_threshold = false;

    		//Initialize particles        ---------------------
		my_new_human.particle.clear();
		t_init_human_particles(&my_new_human);
		human.push_back(my_new_human);
}


//Initializes the particles for a human
void t_init_human_particles(t_Human *human)
{

    while(human->particle.size() < t_NUM_OF_PARTICLE)
    {

      t_Particle my_particle;
      //Assign state variables
      my_particle.body.x = human->mean.x + rand_normal (0, t_INITIALIZATION_NOISE );
      my_particle.body.y = human->mean.y + rand_normal (0, t_INITIALIZATION_NOISE );
      my_particle.body.theta = human->mean.theta;
      my_particle.body.speed = human->mean.speed;
      my_particle.body.omega = human->mean.omega;
      my_particle.weight = 1.0/t_NUM_OF_PARTICLE;		
      human->particle.push_back(my_particle);
    }
  
}

double rand_normal(double mean, double stddev) {
    static double n2 = 0.0;
     static int n2_cached = 0;
     if (!n2_cached) {
         double x, y, r;
        do {
            x = 2.0*rand()/RAND_MAX - 1;
            y = 2.0*rand()/RAND_MAX - 1;

            r = x*x + y*y;
        } while (r == 0.0 || r > 1.0);

         {
         double d = sqrt(-2.0*log(r)/r);
        double n1 = x*d;
        n2 = y*d;

         double result = n1*stddev + mean;

         n2_cached = 1;
         return result;
         }
     } else {
         n2_cached = 0;
         return n2*stddev + mean;
     }
}



void t_predict_particle_motion(std::vector<t_Human> &human,  double ts){

  for(int i=0;  i < human.size();  i++){
    t_prediction_model(&human[i], ts);
  }
}

int t_prediction_model(t_Human *human, double ts)
{
    int error = 0;
    if(human == NULL) return 1;
    if(human->ID >= 0)
    {
        for(int j=0; j<human->particle.size(); j++){

          double theta = human->particle[j].body.theta;

          double kv = 1.0;
          double kn = 1.0;

          // ----------------------------------------------------------------------------------------------------
          human->particle[j].body.speed += rand_normal(0, 150)/1000.0; //in [m/s];
            if(human->particle[j].body.speed > t_MAX_TRAN_VELOCITY) human->particle[j].body.speed = t_MAX_TRAN_VELOCITY;

            human->particle[j].body.theta = rand_normal(human->particle[j].body.theta, deg2rad(5));

            human->particle[j].body.x += (kv*ts*human->particle[j].body.speed*cos(human->particle[j].body.theta)
                                         + kn*rand_normal(0, 20)/1000.0);
            human->particle[j].body.y += (kv*ts*human->particle[j].body.speed*sin(human->particle[j].body.theta)
                                         + kn*rand_normal(0, 20)/1000.0);

            human->particle[j].body.omega =  cal_ang_rad(human->particle[j].body.theta - theta)/ts;

        }
    }
//    printf("Out: t_prediction_model_3\n");
    return error;
}



void t_find_particle_likelihood(std::vector<t_Human> &human,
                           std::vector<tracking::Candidate> &candidates,
                           double ts
                           )
{
    for(int i=0; i<human.size(); i++)
    {
      
       	double sum = 0;
       	double xh = human[i].mean.x;
  	double yh = human[i].mean.y;

   	for(int j=0; j<human[i].particle.size(); j++)
	{
  		double xp = human[i].particle[j].body.x;
      		double yp = human[i].particle[j].body.y;

		std::vector<tracking::Candidate>::iterator it_candidates = candidates.begin();
		double distance = 1000;
		double x_center;
		double y_center;
		double sigma;
		

		//found the closest candidate
		while(it_candidates!=candidates.end())
		{
			double temp_distance = Dist_Between_Points( it_candidates->pose.pose.pose.position.x, xp,
                                                                    it_candidates->pose.pose.pose.position.y, yp);
			if (temp_distance < distance)
			{
				x_center = it_candidates->pose.pose.pose.position.x;
				y_center = it_candidates->pose.pose.pose.position.y;
				sigma = it_candidates->pose.pose.covariance[0];
				distance = temp_distance;
			}
			it_candidates++;				
		}			
	
		if (distance > 0.650) // candidate too far
		{
			human[i].particle[j].weight = 0;
		}
		else
		{	
			double temp_likelihood = gauss(distance, sigma, 2);
			human[i].particle[j].weight = temp_likelihood;
			//ROS_INFO("weigth is %f", temp_likelihood);
		}
			

		//check for near humans
/*		
		std::vector<t_Human>::iterator it_human = my_human.begin();
		while(it_human!=my_human.end())
		{
			:/eif(it_human-my_human.begin()==i)
			{
				it_human++;
				continue;
			}
			double human_x = it_human->mean.x;
			double human_y = it_human->mean.y;
			double distance_from_human = Dist_Between_Points( human_x, xp,
                                                                    human_y, yp);
			if (distance_from_human < 0.500)
			{
				ROS_ERROR("particle too close to another human");
	                        human[i].particle[j].weight = 0.0;
			}
			it_human++;
		}
*/

		if(human[i].particle[j].weight < 0)
        	{
          		human[i].particle[j].weight = 0;
        	}

  		// Sum up the weights for normalization -------------
      		sum += human[i].particle[j].weight;

    	} // end particle loop
  	
	for(int j=0; j<human[i].particle.size(); j++)
	{
      		//printf("Human %d  pt %d  sum %f\n",i,j, sum);
     		if(sum == 0)
        		human[i].particle[j].weight = 0.000001;
		else
		{
               		human[i].particle[j].weight = human[i].particle[j].weight/sum;
			//ROS_INFO("weight after normalization is %f", human[i].particle[j].weight);
		}
  	}
      	
    }// end human loop

}


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


// Update human states by selecting a particle that describes best the human states -------------
void t_update_human_states(std::vector<t_Human> &human, double ts)
{
    
    for(int i=0; i<human.size(); i++)
    {
      t_find_average_particle(&human[i], ts);
    }
    
}

void t_find_average_particle(t_Human *human, double ts){
    int j;

    double x=0, y=0;
    double temp_weight =0;

    double wsum = 0;
    for(j=0; j<human->particle.size(); j++)
      wsum += human->particle[j].weight;

    for(j=0; j<human->particle.size(); j++){
      x += human->particle[j].body.x*(human->particle[j].weight/wsum);
      y += human->particle[j].body.y*(human->particle[j].weight/wsum);

      temp_weight += human->particle[j].weight;
      //printf("\n pt %d  w:%f",j, human->particle[j].weight);
    }

    temp_weight /= human->particle.size();

    double xp = human->mean.x;
    double yp = human->mean.y;
    double thetap = human->mean.theta;

    human->mean.x = x;
    human->mean.y = y;
    human->mean.theta = atan2((human->mean.y-yp), (human->mean.x-xp));

    double distance = Dist_Between_Points( human->mean.x, xp, human->mean.y, yp);
    human->mean.speed = distance/ts;
    human->mean.omega = cal_ang_rad(human->mean.theta-thetap)/ts;

    //Finding the dispersion of the particle distribution
    t_find_dispersion(human);

    human->tracked_time += ts;


    human->weight_memory[human->weight_memory_count] = temp_weight;
    (human->weight_memory_count)++;
    if(human->weight_memory_count >= t_WEIGHT_MEMORY_BUFFER)
      human->weight_memory_count = 0;

}

void t_find_dispersion(t_Human *human){
  //int error = 0;
  int j;

  double meanx=0, meany=0;
  double sx=0, sy=0;
  for(j=0; j<human->particle.size(); j++){
    meanx += human->particle[j].body.x;
    meany += human->particle[j].body.y;
  }
  meanx /= human->particle.size();
  meany /= human->particle.size();
  for(j=0; j<human->particle.size(); j++){
    sx += pow((human->particle[j].body.x-meanx), 2);
    sy += pow((human->particle[j].body.y-meany), 2);
  }
  sx /= human->particle.size();
  sy /= human->particle.size();

  human->dispersion = sqrt(sx + sy);

  //  return error;
}

void t_remove_uncertain_humans(std::vector<t_Human> &human, std::vector<tracking::Candidate> &candidates)
{

  int my_size = human.size();
  for(int i=0; i<my_size; i++)
  {
    
      //evaluate likelihood in the center of human
	human[i].sum_likelihood = 0.0;
	float x = human[i].mean.x;
	float y = human[i].mean.y;
  
	for( int j=0; j<candidates.size(); j++ )
	{
		float distance = Dist_Between_Points(x, candidates[j].pose.pose.pose.position.x, y, candidates[j].pose.pose.pose.position.y);
		if (distance < 1.0)
		{
			human[i].sum_likelihood = human[i].sum_likelihood + gauss(distance, candidates[j].pose.pose.covariance[0], 2); 
		}
		if (human[i].sum_likelihood < 3.0)
		{
			if (human[i].under_likelihood_threshold == true)
				; //do nothing
			else
			{
				human[i].under_likelihood_threshold = true;
				human[i].under_likelihood_threshold_time = ros::Time::now();
			}
			
		}
		else
		{
			human[i].under_likelihood_threshold = false;
		}
		//ROS_INFO("sum_likelihood %f", sum_likelihood);
	}	 
	
       bool too_long_time_under_likelihood_threshold = false;
       if (human[i].under_likelihood_threshold == true)
       		if ( (ros::Time::now() - human[i].under_likelihood_threshold_time) > ros::Duration(1.0))
		{	
			too_long_time_under_likelihood_threshold = true;
		}	 
       
	if((human[i].dispersion > t_MIN_DISPERSION) || too_long_time_under_likelihood_threshold)
        {
	  human.erase(human.begin()+i);
	  i--;
	  my_size = human.size();
        }
		

    }
}


void t_remove_too_close_humans(std::vector <t_Human> &human)
{
	int my_size = human.size();	
	for (int i=0; i<my_size; i++)
	{
		double x1 = human[i].mean.x;
		double y1 = human[i].mean.y;
		for(int j=i+1; j<my_size; j++)
		{
			double x2 = human[j].mean.x;
			double y2 = human[j].mean.y;
			double dist = Dist_Between_Points( x1, x2, y1, y2 );
                	if(dist < 0.650)
			{
				//remove too close human
				human.erase(human.begin()+i);
			        i--;
          			my_size = human.size();
				break;
			}
		}
	}
		
}


void t_remove_meaningless_particle(std::vector<t_Human> &human)
{
	for (int i=0; i<human.size(); i++)
	{
		double min_weight = 1.0/(double)human[i].particle.size();
		for (int j=0; j<human[i].particle.size(); j++)		
		{
			if (human[i].particle[j].weight < min_weight)
			{
				//substitute meaningless particle with a default one
				human[i].particle[j].body.x = human[i].mean.x + rand_normal (0, t_INITIALIZATION_NOISE );
      				human[i].particle[j].body.y = human[i].mean.y + rand_normal (0, t_INITIALIZATION_NOISE );
      				human[i].particle[j].body.theta = human[i].mean.theta;
      				human[i].particle[j].body.speed = human[i].mean.speed;
      				human[i].particle[j].body.omega = human[i].mean.omega;
      				human[i].particle[j].weight = min_weight; 	//not necessary here
			}
		}
	}
}



uint32_t shape = visualization_msgs::Marker::CYLINDER;
void visualize_human(std::vector<t_Human> &human)
{
	for (int i=0; i<human.size(); i++)
	{
		geometry_msgs::Pose my_pose;
		my_pose.position.x = human[i].mean.x;
		my_pose.position.y = human[i].mean.y;
		my_pose.position.z = 0.0; //human_pt->state[i].mean.z;
	
		visualization_msgs::Marker marker;
		marker.header.frame_id = human_tracked_frame;
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
		marker.id = i;
		marker.type = shape;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.position = my_pose.position;
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 1.0;

		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration(sampling_time);
		humans_marker_pub.publish(marker);
		
	
	}
}

void visualize_particles(std::vector<t_Human> &human)
{
	sensor_msgs::PointCloud my_point_cloud;
	sensor_msgs::ChannelFloat32 my_channel;
	my_point_cloud.channels.push_back(my_channel);
	my_point_cloud.channels.back().name = "intensity";
	my_point_cloud.header.frame_id = human_tracked_frame;
	for(int i=0; i<human.size(); i++)
	{
		for(int j=0; j<human[i].particle.size(); j++)
		{
			geometry_msgs::Point32 my_point;
			my_point.x = human[i].particle[j].body.x;
                        my_point.y = human[i].particle[j].body.y;
			my_point.z = 0.0;
			my_point_cloud.points.push_back(my_point);
			
			float my_value = human[i].particle[j].weight;
			my_point_cloud.channels.back().values.push_back(my_value);
		}
	}
	particles_pub.publish(my_point_cloud);	
}
void publish_human(std::vector<t_Human> &human)
{
	tracking::PoseWithCovarianceStampedArray my_human_msg;
	for (int i=0; i<human.size(); i++)
        {
                geometry_msgs::PoseWithCovarianceStamped my_pose;
		my_pose.header.frame_id = human_tracked_frame;
		my_pose.header.stamp = ros::Time::now();
                my_pose.pose.pose.position.x = human[i].mean.x;
                my_pose.pose.pose.position.y = human[i].mean.y;
                my_pose.pose.pose.position.z = 0.0; //human_pt->state[i].mean.z;
		my_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(human[i].mean.theta);
		my_human_msg.poses.push_back(my_pose);
	}
	humans_pub.publish(my_human_msg);
}
void log_human(std::vector<t_Human> &human)
{
 	double time = (double)ros::Time::now().sec + (double)ros::Time::now().nsec/(double)1000000000;	


	for (int i=0; i<human.size(); i++)
        {
                geometry_msgs::Pose my_pose;
                my_pose.position.x = human[i].mean.x;
                my_pose.position.y = human[i].mean.y;
                my_pose.position.z = 0.0; //human_pt->state[i].mean.z;

		//int grid_x = (unsigned int)((laser_cloud.points[i].x - static_map.info.origin.position.x) / static_map.info.resolution);
                //int grid_y = (unsigned int)((laser_cloud.points[i].y - static_map.info.origin.position.y) / static_map.info.resolution);

		double map_x = (my_pose.position.x - static_map.info.origin.position.x)/static_map.info.resolution;

		double map_y = (my_pose.position.y - static_map.info.origin.position.y)/static_map.info.resolution;
		printf("\t%f\t%f\t%f\t%f\n", time, map_x, map_y, human[i].mean.theta);
  		
	}
}


void print_human_info(std::vector<t_Human> &human)
{

	ROS_INFO("new iteration");
	for(int i=0; i<human.size(); i++)
	{
		ROS_INFO("ID #%d", human[i].ID);
		ROS_INFO("position x:%f y:%f", human[i].mean.x, human[i].mean.y);
		ROS_INFO("sum likelihood: %f", human[i].sum_likelihood);
		if (human[i].under_likelihood_threshold == true)
		{
			ROS_INFO("under likelihood threshold: true");
			ROS_INFO("under threshold since %f", human[i].under_likelihood_threshold_time.toSec());
			ros::Duration duration = ros::Time::now() - human[i].under_likelihood_threshold_time;
			ROS_INFO("under threshold for %f", duration.toSec()); 
		}
		else
			ROS_INFO("under likelihood threshold: false");

		ROS_INFO(" ");		
	}
	ROS_INFO(" ");
	ROS_INFO(" ---------------- ");
	ROS_INFO("  ");

	for(int i=0; i<human.size(); i++)
	{	
		visualization_msgs::Marker text_marker;
		text_marker.header.frame_id = human_tracked_frame;
		text_marker.header.stamp = ros::Time::now();
		text_marker.ns = "text";
		text_marker.id = i;
		text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		text_marker.action = visualization_msgs::Marker::ADD;

		text_marker.pose.position.x = human[i].mean.x;
		text_marker.pose.position.y = human[i].mean.y;
		
		text_marker.scale.x = 0.05;
		text_marker.scale.y = 0.05;
		text_marker.scale.z = 1.0;

		text_marker.color.r = 0.0f;
		text_marker.color.g = 0.0f;
		text_marker.color.b = 0.0f;
		text_marker.color.a = 1.0;
	
		std::stringstream ss;
		ss << human[i].ID;
		std::string str = ss.str();	
		text_marker.text = str;

		text_marker.lifetime = ros::Duration(sampling_time);
		humans_marker_pub.publish(text_marker);
	}
	
}
