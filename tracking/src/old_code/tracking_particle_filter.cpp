#include "tracking_particle_filter.h" 
#include <ros/ros.h>

std::vector<tracking::Candidate> my_candidates;
std::vector<t_Human> my_human;

double  pre_time_stamp = 0;
double  sampling_time   = 0;

ros::Publisher humans_marker_pub;
ros::Publisher particles_pub;

double likelihood_threshold;
double likelihood_threshold_default = 10.0;

std::string human_tracked_frame = "/laser";

void likelihoodCallback(const tracking::CandidateArray::ConstPtr& candidates_array_msg)
{
	my_candidates = candidates_array_msg->candidates;

	t_Create_Humans(my_candidates, my_human);

	sampling_time = candidates_array_msg->candidates[0].pose.header.stamp.sec + (double)candidates_array_msg->candidates[0].pose.header.stamp.nsec/1000000000 - pre_time_stamp;
	pre_time_stamp = candidates_array_msg->candidates[0].pose.header.stamp.sec + (double)candidates_array_msg->candidates[0].pose.header.stamp.nsec/1000000000;  

        // Prediction Step: Motion Model 
        t_predict_particle_motion(my_human, sampling_time);

	//Correction Step: Update
	
	t_find_particle_likelihood(my_human, my_candidates, sampling_time);
	t_update_human_states(my_human, sampling_time);

	t_remove_uncertain_humans(my_human);

	visualize_particles(my_human);
        visualize_human(my_human);

}



int main(int argc, char **argv)
{
	//t_initialize_entities(&human_pt);

	ros::init(argc, argv, "tracking_particle_filter");
        ros::NodeHandle n;

        ros::Subscriber candidates = n.subscribe("/human_candidate", 10,  likelihoodCallback);
	humans_marker_pub = n.advertise<visualization_msgs::Marker>("/human_tracked_marker", 10);
	particles_pub = n.advertise<sensor_msgs::PointCloud>("/human_tracked_particles", 10);

        if (n.getParam("/tracking_particle_filter/likelihood_threshold", likelihood_threshold))
        {
                 ROS_INFO("[tracking_particle_filter] got (/tracking_particle_filter/likelihood_threshold) from param server, value is %f", likelihood_threshold);
        }
        else
        {
                likelihood_threshold = likelihood_threshold_default;
                ROS_INFO("[tracking_particle_filter] not found (/tracking_particle_filter/likelihood_threshold) from param server, set to default value %f", likelihood_threshold);
        }

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

    }//end if id
  }//end for i
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

    		//Initialize particles        ---------------------
		t_init_human_particles(&my_new_human);
		human.push_back(my_new_human);
}


//Initializes the particles for a human
void t_init_human_particles(t_Human *human)
{

    for(int j=0; j<t_NUM_OF_PARTICLE; j++)
    {

      //Assign state variables
      human->particle[j].body.x = human->mean.x + rand_normal (0, t_INITIALIZATION_NOISE );
      human->particle[j].body.y = human->mean.y + rand_normal (0, t_INITIALIZATION_NOISE );
      human->particle[j].body.theta = human->mean.theta;
      human->particle[j].body.speed = human->mean.speed;
      human->particle[j].body.omega = human->mean.omega;
      human->particle[j].weight = 1.0/t_NUM_OF_PARTICLE;

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
        for(int j=0; j<t_NUM_OF_PARTICLE; j++){

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
      
      	{
        	t_likelihood_model(&human[i], candidates, ts);
      	}
    }// end human loop

}

void t_likelihood_model(t_Human *human,
                        std::vector<tracking::Candidate> &candidates,
                        double ts
                        )
{
	double sum = 0;
        double xh = human->mean.x;
    	double yh = human->mean.y;

   	for(int j=0; j<t_NUM_OF_PARTICLE; j++)
	{
  		double xp = human->particle[j].body.x;
      		double yp = human->particle[j].body.y;

		std::vector<tracking::Candidate>::iterator it_candidates = candidates.begin();
		double min_distance = 1000;
		double x_center;
		double y_center;
		double distance;
		double sigma;
		while(it_candidates!=candidates.end())
		{
			double temp_distance = Dist_Between_Points( it_candidates->pose.pose.pose.position.x, xp,
                                                                    it_candidates->pose.pose.pose.position.y, yp);
			if (temp_distance < min_distance)
			{
				x_center = it_candidates->pose.pose.pose.position.x;
				y_center = it_candidates->pose.pose.pose.position.y;
				sigma = it_candidates->pose.pose.covariance[0];
				distance = temp_distance;
			}
			it_candidates++;				

		}			

		double temp_likelihood = gauss(distance, sigma, 2);
		human->particle[j].weight = temp_likelihood;

		if(human->particle[j].weight < 0)
        	{
          		human->particle[j].weight = 0;
        	}

      		// Sum up the weights for normalization -------------
      		sum += human->particle[j].weight;


    	} // end particle loop
  	
	for(int j=0; j<t_NUM_OF_PARTICLE; j++)
      	//printf("Human %d  pt %d  sum %f\n",i,j, sum);
     	 if(sum == 0)
        	human->particle[j].weight = 0.000001;
  	
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
    for(j=0; j<t_NUM_OF_PARTICLE; j++)
      wsum += human->particle[j].weight;

    for(j=0; j<t_NUM_OF_PARTICLE; j++){
      x += human->particle[j].body.x*(human->particle[j].weight/wsum);
      y += human->particle[j].body.y*(human->particle[j].weight/wsum);

      temp_weight += human->particle[j].weight;
      //printf("\n pt %d  w:%f",j, human->particle[j].weight);
    }

    temp_weight /= t_NUM_OF_PARTICLE;

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
  for(j=0; j<t_NUM_OF_PARTICLE; j++){
    meanx += human->particle[j].body.x;
    meany += human->particle[j].body.y;
  }
  meanx /= t_NUM_OF_PARTICLE;
  meany /= t_NUM_OF_PARTICLE;
  for(j=0; j<t_NUM_OF_PARTICLE; j++){
    sx += pow((human->particle[j].body.x-meanx), 2);
    sy += pow((human->particle[j].body.y-meany), 2);
  }
  sx /= t_NUM_OF_PARTICLE;
  sy /= t_NUM_OF_PARTICLE;

  human->dispersion = sqrt(sx + sy);

  //  return error;
}

void t_remove_uncertain_humans(std::vector<t_Human> &human)
{

  int my_size = human.size();
  for(int i=0; i<my_size; i++)
  {
    
        //------------------------------------------------------------------------------------
        double avgweight = 0;
        for(int j=0; j<t_WEIGHT_MEMORY_BUFFER; j++) {
          avgweight += human[i].weight_memory[j];
          //printf("j:%d  %f  %f \n",j, human->state[i].weight_memory[j], avgweight);
        }
        avgweight /= t_NUM_OF_PARTICLE;

        //printf("dispersion is %f\n", human->state[i].dispersion );
        //printf("avgweight is %f\n", avgweight);

        if(  (human[i].dispersion > t_MIN_DISPERSION) || avgweight < t_MIN_WEIGHT){

          if (human[i].dispersion > t_MIN_DISPERSION)
	  {
		;//ROS_INFO("human removed as human->state[i].dispersion > t_MIN_DISPERSION, dispersion %f: ", human->state[i].dispersion);
	  }
          if (avgweight < t_MIN_WEIGHT)
          {
                ;//ROS_INFO("human removed as avgweight < t_MIN_WEIGHT, avgweight %f: ", avgweight);
          }

	  human.erase(human.begin()+i);
	  i--;
	  my_size = human.size();
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
		for(int j=0; j<t_NUM_OF_PARTICLE; j++)
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
