#include "my_algebra_common.h"

#ifndef ON_BOARD_HUMAN_TRACKER_2D
#define ON_BOARD_HUMAN_TRACKER_2D

#define t_NUM_OF_PARTICLE       50               
#define t_WEIGHT_MEMORY_BUFFER  50
#define NUM_OF_PEOPLE           50

#define ssm_MAX_NUM_OF_PEOPLE 50

//#define MAX_LASER_CLUSTER_STREAMS 3 //Number of cluster (laser sensor) streams used for the correction step


#define t_MAX_TRACKING_RANGE    4500             //3500	// in mm
#define t_MAX_ANGLE             180 * M_PI / 180 //Angle range:    0 ~ 180
#define t_MIN_ANGLE             0 * M_PI / 180   //Angle range: -180 ~ 0
#define t_MAX_STEP_SIZE         650              // to compare the previous no of cluster with present and integration in mm
#define t_MAX_TRAN_VELOCITY     1700             // in mm/s
#define t_MEASUREMENT_NOISE     150              // in mm
#define t_INITIALIZATION_NOISE  150              // in mm
#define t_MAX_ID                99               //Maximum number of human ID number
#define t_MIN_DISPERSION        500              // Minimum dispersion of particles before disappearing in mm
#define t_MIN_WEIGHT            0.001		 
#define t_CYCLES_TO_WAIT        10		//10               // Number of cycles to wait before adding a new human to track

#define NUM_CLUSTERS 200 //Maximum Number of Clusters
#define maxSizeDataUTM   1081 //Number of URG Data 

typedef struct _ssm_Odometry_rl{
	double x;
	double y;
	double theta;
	double r_vel;
	double l_vel;
} ssm_Odometry_rl;

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
  double x;      // (x, y): position
  double y;
  double z;
  double theta;  // theta : heading direction
  double speed;  // speed
  double omega;  // omega : angular velocity
}t_Body;

typedef struct{
  t_Body body;
  double weight;
}t_Particle;
 

typedef struct{  
  int ID;                         // A unique ID assigned to each person
  t_Body mean;        // The average position of the human (weighted average of the ID^th particle filter)
  double tracked_time;            // Overall tracked time
  double dispersion;              // This variable checks what is the dispersion of the particles for each human
  int weight_memory_count;         //This counter variable acts like a ring buffer, when it arrives to t_WEIGHT_MEMORY_BUFFER it becomes zero and keepts track of the last 50 time steps of the particle filter weight
  double weight_memory[t_WEIGHT_MEMORY_BUFFER];  //This variable stores a weighting factor of t_WEIGHT_MEMORY_BUFFER (50) time steps
  t_Particle particle[t_NUM_OF_PARTICLE];
}t_Human;

typedef struct{  
  int numHumans;                  // The number of humans currently being tracked
  t_Human state[NUM_OF_PEOPLE];
}t_Entity;




typedef struct{
  int counter[NUM_CLUSTERS]; //* MAX_LASER_CLUSTER_STREAMS];
}cycle_counter;


//id related functions
void t_initialize_human_id(t_Human human[ssm_MAX_NUM_OF_PEOPLE]);
int t_assign_people_id(int *t_Max_ID_Index, int t_People_ID[ssm_MAX_NUM_OF_PEOPLE]  );

//Normalize the weights of the particles
void t_normalize(t_Entity *human);
double rand_normal(double mean, double stddev) ;

//Cluster related function
void Cluster_masking(ClusterStruct *cl, cycle_counter *cnt, ssm_Odometry_rl *pose ); //Masks the valid clusters using parameters: (t_MAX_TRACKING_RANGE && t_MIN_ANGLE && t_MAX_ANGLE)
void find_closest_cluster( int index, int *id, double *mindist, ClusterStruct *cl);

//Particle Related functions
void t_frequency_based_resample(t_Entity *human);
void t_resample_particles(t_Entity *human);

//Removing humans
void t_remove_uncertain_humans(t_Entity *human,    int t_People_ID[ssm_MAX_NUM_OF_PEOPLE] );
void t_remove_human_from_map( t_Entity *human,    int t_People_ID[ssm_MAX_NUM_OF_PEOPLE] ) ;


/*
void t_remove_human_from_map( t_Entity *human,    int t_People_ID[ssm_MAX_NUM_OF_PEOPLE], GRID *grid, MAP_Param *map_par ){
  int count = 0 ;
  int pre_num = human->numHumans;
  //printf("\n In remove_human_from_map");
  for(int i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++){
    if( human->state[i].ID >= 0 ){
      count++;
      
      int x_cell = xcoor2cell( human->state[i].mean.x, map_par) ;
      int y_cell = ycoor2cell( human->state[i].mean.y, map_par) ;
      int occp_buf_index = cell2buf(x_cell, y_cell, map_par) ;
      if( grid->grayval[ occp_buf_index ] < OCCUPIED_THRESHOLD ){
	 
	printf("\n Erasing human %d embeded in Map ",t_People_ID[human->state[i].ID]);//, grid->grayval[ occp_buf_index ]);
	
	//if( human->state[i].mean.x &&  human->state[i].mean.y ) is inside map erase the entity {
	t_People_ID[human->state[i].ID] = 0;
	human->state[i].ID = -1;
	human->numHumans--;
      }
      
      
      else
	for(int j=0; j<t_NUM_OF_PARTICLE; j++){
	  
	  x_cell = xcoor2cell( human->state[i].particle[j].body.x, map_par) ;
	  y_cell = ycoor2cell( human->state[i].particle[j].body.y, map_par) ;
	  occp_buf_index = cell2buf(x_cell, y_cell, map_par) ;
	  if( grid->grayval[ occp_buf_index ] < OCCUPIED_THRESHOLD ){
	    
	    //if ( human->state[i].particle[j].body.x && human->state[i].particle[j].body.y ) are inside a map grid
      human->state[i].particle[j].weight=0.0;
	  }
	}//end for j
      
      
    }

    if(count >= pre_num)
      break;    
  }
}
*/

void Cluster_masking(ClusterStruct *cl, cycle_counter *cnt, ssm_Odometry_rl *pose ){
  ClusterStruct tmp = *cl ;
    cl->numClusters = 0;
  double distance;
  double angle;
  

  for (int k=0; k < tmp.numClusters; k++){
         
    if(pose!=NULL){
      distance = Dist_Between_Points( tmp.cluster[k].centroid.x, pose->x, tmp.cluster[k].centroid.y, pose->y );
      angle = atan2(( tmp.cluster[k].centroid.y - pose->y ), ( tmp.cluster[k].centroid.x - pose->x )) ;
      angle = cal_ang_rad (angle - pose->theta) ;
    }
    else{
      distance = Dist_Between_Points( tmp.cluster[k].centroid.x, 0.0, tmp.cluster[k].centroid.y, 0.0 );
      angle = atan2(( tmp.cluster[k].centroid.y - 0.0 ), ( tmp.cluster[k].centroid.x - 0.0 )) ;
      angle = cal_ang_rad (angle - 0.0) ;
    }
     //RaffaeleL: removed the control on angle
     //if(distance <= t_MAX_TRACKING_RANGE && angle>= t_MIN_ANGLE && angle<= t_MAX_ANGLE){
     if(distance <= t_MAX_TRACKING_RANGE)
      {
	cl->cluster[cl->numClusters] = tmp.cluster[k] ;
	cnt->counter[cl->numClusters] =0 ;
	cl->numClusters ++ ;
      }//end if
      //else
    	//printf("dist: %f ang: %f  max:%f   min:%f \n",distance, angle * 180/M_PI, t_MAX_ANGLE*180/M_PI, t_MIN_ANGLE*180/M_PI);
   }//end for

}

void t_initialize_human_id(t_Human human[ssm_MAX_NUM_OF_PEOPLE]){
  for(int i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++)
    human[i].ID = -1;
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

int t_assign_people_id(int *t_Max_ID_Index, int t_People_ID[ssm_MAX_NUM_OF_PEOPLE]  ){
  int id = -1;
  
  for(int i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++)    {
    if(t_People_ID[i] == 0) {
      id = i;
      *t_Max_ID_Index = *t_Max_ID_Index + 1 ;

      if(*t_Max_ID_Index > t_MAX_ID)
	*t_Max_ID_Index = 0;
      
      t_People_ID[i] = *t_Max_ID_Index + 1;
      break;
    }
  }
  return id;
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
	  human->particle[j].body.speed += rand_normal(0, 150);
            if(human->particle[j].body.speed > t_MAX_TRAN_VELOCITY) human->particle[j].body.speed = t_MAX_TRAN_VELOCITY;

            human->particle[j].body.theta = rand_normal(human->particle[j].body.theta, deg2rad(5));

            human->particle[j].body.x += (kv*ts*human->particle[j].body.speed*cos(human->particle[j].body.theta)
                                         + kn*rand_normal(0, 20));
            human->particle[j].body.y += (kv*ts*human->particle[j].body.speed*sin(human->particle[j].body.theta)
                                         + kn*rand_normal(0, 20));
     
	    human->particle[j].body.omega =  cal_ang_rad(human->particle[j].body.theta - theta)/ts;

        }
    }
//    printf("Out: t_prediction_model_3\n");
    return error;
}





void catch_int( int sig_num ){
  printf("\n\nControl C pressed...\n Free Allocated Memory and Program Exit . . . . . \n\n\n");
  exit(1);  
}


void Cluster_Coordinate_Transformation(ClusterStruct *lc, ClusterStruct *gl, ssm_Odometry_rl *odo) {
  
  gl->numClusters = lc->numClusters ;

  double sin_yaw = sin(odo->theta *M_PI/180);
  double cos_yaw = cos(odo->theta *M_PI/180);

  for (int k=0; k < lc->numClusters; k++){

    gl->cluster[k].numPoints  = lc->cluster[k].numPoints;
    
    gl->cluster[k].centroid.x = lc->cluster[k].centroid.x * cos_yaw - lc->cluster[k].centroid.y * sin_yaw   + odo->x;
    gl->cluster[k].centroid.y = lc->cluster[k].centroid.x * sin_yaw + lc->cluster[k].centroid.y * cos_yaw   + odo->y;

    gl->cluster[k].semi_major = lc->cluster[k].semi_major;
    gl->cluster[k].semi_minor = lc->cluster[k].semi_minor;
    gl->cluster[k].flag       = lc->cluster[k].flag;   
   
    for(int i=0; i < lc->cluster[k].numPoints; i++){
      gl->cluster[k].point[i].x = lc->cluster[k].point[i].x * cos_yaw - lc->cluster[k].point[i].y * sin_yaw   + odo->x;
      gl->cluster[k].point[i].y = lc->cluster[k].point[i].x * sin_yaw + lc->cluster[k].point[i].y * cos_yaw   + odo->y;


    }
  }
}


/*
void Cluster_Coordinate_Transformation(ClusterStruct *lc, ClusterStruct *gl, ssm_Odometry_rl *odo) {
  
  gl->numClusters = lc->numClusters ;

  double sin_yaw = sin(odo->theta *M_PI/180);
  double cos_yaw = cos(odo->theta *M_PI/180);

  for (int k=0; k < lc->numClusters; k++){

    gl->cluster[k].numPoints = lc->cluster[k].numPoints;
    gl->cluster[k].flag = lc->cluster[k].numPoints;
    
    for(int i=0; i < lc->cluster[k].numPoints; i++){
      gl->cluster[k].point[i].x = lc->cluster[k].point[i].x * cos_yaw - lc->cluster[k].point[i].y * sin_yaw   + odo->x;
      gl->cluster[k].point[i].y = lc->cluster[k].point[i].x * sin_yaw + lc->cluster[k].point[i].y * cos_yaw   + odo->y;

      gl->cluster[k].centroid.x = lc->cluster[k].centroid.x * cos_yaw - lc->cluster[k].centroid.y * sin_yaw   + odo->x;
      gl->cluster[k].centroid.y = lc->cluster[k].centroid.x * sin_yaw + lc->cluster[k].centroid.y * cos_yaw   + odo->y;

    }
  }
}
*/


//This function asociates clusters comparing the cluster from the previous list to the current one.
//It increases the number of counts that the cluster has been seen
void cluster_data_asociation( ClusterStruct *pre_cl, ClusterStruct * cl, cycle_counter *pre_cnt, cycle_counter *cnt){
    int i, j;

for(i=0; i<cl->numClusters; i++){
  double x = cl->cluster[i].centroid.x; //lcluster[i].centroid.x;
  double y = cl->cluster[i].centroid.y; //lcluster[i].centroid.y;
  double mindist = 100000;
  int id = -1;

  for(j=0; j<pre_cl->numClusters; j++) {
    double xp = pre_cl->cluster[j].centroid.x; //pre_lcluster[j].centroid.x;
    double yp = pre_cl->cluster[j].centroid.y; //pre_lcluster[j].centroid.y;
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
    
    //printf("DataAsociation cl:%d  cnt:%d \n",i,cnt->counter[i] );
  }
  else
    cnt->counter[i] = 0;
 }//end for i
}

//originally t_find_human_positions
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

  //printf("    no of humans = %d  no_of_chunks %d\n", *no_of_human_clusters, *no_of_chunk_clusters);

  for(i=0; i<cl->numClusters; i++)
    cl->cluster[i].flag = true;

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



int t_init_particle(t_Human human, t_Particle *particle)
{
    int error=0;
    if(particle == NULL) return 1;
    if(human.ID >=0){

        particle->body.x = human.mean.x + rand_normal (0, t_INITIALIZATION_NOISE);
        particle->body.y = human.mean.y + rand_normal (0, t_INITIALIZATION_NOISE);
        particle->body.theta = human.mean.theta;
        particle->body.speed = human.mean.speed;
        particle->body.omega = human.mean.omega;
	//printf("particle body.x:%f  body.y%f \n", particle->body.x, particle->body.y);
    }
    return error;
}

int t_add_position_noise(t_Particle *particle, double mean, double std)
{
    int error = 0;
    double dx, dy;

    if(particle == NULL) return 1;

    dx = rand_normal (mean, std);
    dy = rand_normal (mean, std);
    particle->body.x += dx;
    particle->body.y += dy;

    return error;
}

int t_copy_particle(t_Particle src, t_Particle *dst)
{
    int error = 0;

    if(dst == NULL) return 1;

    dst->body.x = src.body.x;
    dst->body.y = src.body.y;
    dst->body.theta = src.body.theta;
    dst->body.speed = src.body.speed;
    dst->body.omega = src.body.omega;

    return error;
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


void t_initialize_entities(t_Entity *human){
  human->numHumans = 0;
  for(int i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++)
    human->state[i].ID = -1;
  
 }

void t_predict_particle_motion(t_Entity *human, double ts){

  for(int i=0;  i < human->numHumans;  i++){
    if( human->state[i].ID >= 0 )
      t_prediction_model(&human->state[i], ts);
  }
}


double t_find_force(t_Entity *human, 
                 int i,
                 double x,
                 double y,
                 double sx,
                 double sy
		     ){
  int m, local_count=0;
  double force=0;
  for(m=0; m<ssm_MAX_NUM_OF_PEOPLE; m++) {
    //if(t_People_ID[m] > 0 && human->state[m].ID != human->state[i].ID) {
    if(human->state[m].ID >= 0 && human->state[m].ID != human->state[i].ID) {

      local_count++;
      
      force += gauss(x, y, human->state[m].mean.x, human->state[m].mean.y,
		       sx, sy);
      
        }
    
    if(local_count >= human->numHumans)
      break;
        
    }
    return force;
}


void t_likelihood_model(t_Entity *human,
                         int i,
			point_2d human_clusters[NUM_CLUSTERS], //* MAX_LASER_CLUSTER_STREAMS],
                         int no_of_human_clusters,
                         point_2d chunk_clusters[NUM_CLUSTERS], //* MAX_LASER_CLUSTER_STREAMS],
                         int no_of_chunk_clusters,
                         double ts
                        ){
  int j, k;
  
  if(human->state[i].ID >= 0){
    double sum = 0;
    double kbody    =  0.9; // constant for body position attractor
    double kcluster =  0.1; // constant for cluster position attractor
    double kforce   = -2.0*(kbody+kcluster); // constant for force repulser
    
    double pbody, pcluster, pforce, popen;
    
    double xh = human->state[i].mean.x;
    double yh = human->state[i].mean.y;
    
    for(j=0; j<t_NUM_OF_PARTICLE; j++){
      double xp = human->state[i].particle[j].body.x;
      double yp = human->state[i].particle[j].body.y;

      //----------------------------------------
      popen = 0.001;
      //--------------------------------------------------------------------------------------
      pbody    = 0;
      for(k=0; k<no_of_human_clusters; k++){
	double x = human_clusters[k].x;
	double y = human_clusters[k].y;
	pbody += (kbody* gauss(x, y, xp, yp, t_MEASUREMENT_NOISE, t_MEASUREMENT_NOISE));

      }
      //--------------------------------------------------------------------------------------
      pcluster = 0;
      for(k=0; k<no_of_chunk_clusters; k++){
	double x = chunk_clusters[k].x;
	double y = chunk_clusters[k].y;
	
	double dist = Dist_Between_Points( x, xh, y, yh);
	//printf("\n chunk %d dist %f  chnk: %f %f  hmn[%d] %f %f", k, dist,  chunk_clusters[k].x, chunk_clusters[k].y, i,xh, yh);
	//printf("\nchunk %d dist %f  chnk: %f %f  hmn %f %f", k, dist,  chunk_clusters[k].x, chunk_clusters[k].y, xh, yh);
	//printf("\nchunk %d dist %f TH:%f chnk: %f %f  hmn %f %f",k, dist, 1.5*t_MAX_STEP_SIZE,  chunk_clusters[k].x, chunk_clusters[k].y, xh, yh);
	if(dist<=1.5*t_MAX_STEP_SIZE)
	  pcluster += ( kcluster* gauss(x, y, xp, yp, 0.5*t_MAX_STEP_SIZE, 0.5*t_MAX_STEP_SIZE));
	
      }//end for chunks

      // --------------------------------------------------------------------------------------
      pforce = kforce * t_find_force(human, i, xp, yp, 0.5*t_MAX_STEP_SIZE, 0.5*t_MAX_STEP_SIZE);
      
      //-----------------------------------------------------------------------------------------
      human->state[i].particle[j].weight = popen + pbody + pcluster + pforce;

      
      //printf("  pt %d w %f  popen %f pbody %f pcluster %f pforce %f\n",j, human[i].particle[j].weight, popen, pbody, pcluster, pforce);
      //printf("Human %d  pt %d w %f  sum %f\n",i,j, human[i].particle[j].weight, sum);
      if(human->state[i].particle[j].weight < 0)
	{
	  human->state[i].particle[j].weight = 0;
	}
      
      // Sum up the weights for normalization -------------
      sum += human->state[i].particle[j].weight;

      
    } // end particle loop
    
    for(j=0; j<t_NUM_OF_PARTICLE; j++)
      //printf("Human %d  pt %d  sum %f\n",i,j, sum);
      if(sum == 0)	    
	human->state[i].particle[j].weight = 0.000001;
  }
  
}



void t_find_particle_likelihood(t_Entity *human,
                           point_2d human_clusters[NUM_CLUSTERS], //* MAX_LASER_CLUSTER_STREAMS],
                           int no_of_human_clusters,
                           point_2d chunk_clusters[NUM_CLUSTERS], //* MAX_LASER_CLUSTER_STREAMS],
                           int no_of_chunk_clusters,
                           double ts
                           ){
    int count = 0;

    for(int i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++){
      //if(t_People_ID[i] > 0){
      if( human->state[i].ID >= 0 ){
	count++;
	t_likelihood_model(human, i, human_clusters, no_of_human_clusters, chunk_clusters, no_of_chunk_clusters, ts);
      }
      
      if(count >= human->numHumans)
	break;
        
    }// end human loop

}


// Update human states by selecting a particle that describes best the human states -------------
void t_update_human_states(t_Entity * human, double ts){
    int i;
    int count = 0;

    for(i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++){
      if( human->state[i].ID >= 0 ){
	count++;	    
	t_find_average_particle(&human->state[i], ts);
      }
      if( count >= human->numHumans )
	break;
      
    }
}

void t_remove_uncertain_humans(t_Entity *human,    int t_People_ID[ssm_MAX_NUM_OF_PEOPLE] ){
  int count = 0;
  int pre_num = human->numHumans;
  
  for(int i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++){
    if( human->state[i].ID >= 0 ){
	count++;
	
	//------------------------------------------------------------------------------------
	double avgweight = 0;
	for(int j=0; j<t_WEIGHT_MEMORY_BUFFER; j++) {
	  avgweight += human->state[i].weight_memory[j];
	  //printf("j:%d  %f  %f \n",j, human->state[i].weight_memory[j], avgweight);
	}
	avgweight /= t_NUM_OF_PARTICLE;
	
	//printf("dispersion is %f\n", human->state[i].dispersion );
        //printf("avgweight is %f\n", avgweight);
	
	if(  (human->state[i].dispersion > t_MIN_DISPERSION) || avgweight < t_MIN_WEIGHT){
	
	  //if (human->state[i].dispersion > t_MIN_DISPERSION)
	  	//printf("I'm removing a human because dispersion: %f is > t_MIN_DISPERSION \n", human->state[i].dispersion);
	  //if (avgweight < t_MIN_WEIGHT)
                //printf("I'm removing a human because avgweight: %f is < t_MIN_WEIGHT \n", avgweight);

	  t_People_ID[human->state[i].ID] = 0;
	  human->state[i].ID = -1;
	  human->numHumans--;
	}
	
    }
        if(count >= pre_num)
	  break;
        
  }

}


//WENTITY
void update_ssm_entity(t_Entity *human, 
		       //		       People *global_people_data,
		       ssm_Odometry_rl  *raw_odo,
		       ssm_Odometry_rl *corr_odo,
		       t_Entity *ssm_buf,
		       int t_People_ID[ssm_MAX_NUM_OF_PEOPLE] 
		       ){
  int i;
  int count=0;
  ssm_buf->numHumans = human->numHumans;
  
    for(i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++) {
      if( human->state[i].ID >= 0 ){

	//This is the condition to only copy back the humans tracked in raw odometry coordinate frame
	if( (raw_odo!=NULL) && (corr_odo == NULL)) {

	  ssm_buf->state[count] = human->state[i] ;
	  ssm_buf->state[count].ID = (int)t_People_ID[i];
	}

	//Condition to return the tracked humans in global coordinate frame
	else if((raw_odo!=NULL) && (corr_odo!=NULL)) {
	  double x = human->state[i].mean.x - raw_odo->x;
	  double y = human->state[i].mean.y - raw_odo->y;
	  double odotheta = cal_ang_rad(deg2rad(raw_odo->theta));
	  double corrtheta = cal_ang_rad(deg2rad(corr_odo->theta));
	  double theta = cal_ang_rad(corrtheta - odotheta);
	  
	  //printf("\n In global %d  ht_lc %f %f   ht_odo %f %f  odo %f %f",i, x, y, ssm_buf->state[i].mean.x, ssm_buf->state[i].mean.y, raw_odo->x, raw_odo->y  );
	  
	  ssm_buf->state[count] = human->state[i] ;
	  ssm_buf->state[count].ID = (int)t_People_ID[i];
	  
	  ssm_buf->state[count].mean.x = corr_odo->x + (x*cos(theta) - y*sin(theta));
	  ssm_buf->state[count].mean.y = corr_odo->y + (x*sin(theta) + y*cos(theta));


	  
	  for(int j=0; j<t_NUM_OF_PARTICLE; j++){
	    x = human->state[i].particle[j].body.x - raw_odo->x;
	    y = human->state[i].particle[j].body.y - raw_odo->y;
	    corrtheta = cal_ang_rad(deg2rad(corr_odo->theta));
	    theta = cal_ang_rad(corrtheta - odotheta);
	    
	    ssm_buf->state[count].particle[j].body.x = corr_odo->x + (x*cos(theta) - y*sin(theta));
	    ssm_buf->state[count].particle[j].body.y = corr_odo->y + (x*sin(theta) + y*cos(theta));
	  }
	} //end if corr_odo != NULL
	  

	count++;
      }//end if people id > 0
	
      if(count >= human->numHumans)
	break;
	
      }//end for

}



void t_normalize(t_Entity *human){
  int i, j, count=0;
    for(i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++){
      if( human->state[i].ID >= 0 ){
            count++;
            double sum = 0;
            for(j=0; j<t_NUM_OF_PARTICLE; j++)
	      sum += human->state[i].particle[j].weight;

            if(sum>0){
                for(j=0; j<t_NUM_OF_PARTICLE; j++)
		  human->state[i].particle[j].weight /= sum;
            }
        }
	if(count >= human->numHumans)
	  break;
        
    }//end for i=0
}


void t_frequency_based_resample(t_Entity *human){
    int i, j, k;
    int count = 0;
    int N = 0;
    int index = 0;
    t_Particle particle[t_NUM_OF_PARTICLE];

    for(i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++){
      if( human->state[i].ID >= 0 ){
	
	count++;
	index = 0;
	double wsum = 0;
	double m = 0;
	
	for(j=0; j<t_NUM_OF_PARTICLE; j++)
	  m += round(t_NUM_OF_PARTICLE*human->state[i].particle[j].weight); //printf("    m = %f\n", m);
	
	if(m>0)
	  m = (double)t_NUM_OF_PARTICLE/m;
	
	//            printf("    m = %f\n", m);
	// Resample particles proportional to the particle weight.
	for(j=0; j<t_NUM_OF_PARTICLE; j++){
	  N = (int)round(m*(t_NUM_OF_PARTICLE*human->state[i].particle[j].weight));
	  int tmp = index;
	  for(index=tmp; index<(tmp+N); index++){
	    //t_copy_particle(human[i].particle[j], &particle[index]);
	    particle[index].body.x = human->state[i].particle[j].body.x;
	    particle[index].body.y = human->state[i].particle[j].body.y;
	    particle[index].body.theta = human->state[i].particle[j].body.theta;
	    particle[index].body.speed = human->state[i].particle[j].body.speed;
	    particle[index].body.omega = human->state[i].particle[j].body.omega;
	    
	    particle[index].weight = human->state[i].particle[j].weight;
	    
	    // Add position noise to the copies of the same particle ---------
	    if(index>tmp && index<(tmp+N)){
	      t_add_position_noise(&particle[index], 0, 20);
	    }
	    
	    // -------------------------------------
	    wsum += particle[index].weight;
	    if(index == t_NUM_OF_PARTICLE-1) break;
	  }//end for index=tmp
	  if(index == t_NUM_OF_PARTICLE-1) break;
	}//end for(j=0; j<t_NUM_OF_PARTICLE; j++)

	//            printf("    index = %d\n", index);
	for(k=index; k<t_NUM_OF_PARTICLE; k++)  {
	  t_init_particle(human->state[i], &particle[k]);
	  wsum += particle[index].weight;
	}
	//            printf("    index = %d k = %d\n", index, k);
	for(k=0; k<t_NUM_OF_PARTICLE; k++){
	  //                printf("    weight = %f\n", particle[j].weight);
	  t_copy_particle(particle[k], &human->state[i].particle[k]);
	  human->state[i].particle[k].body.x     = particle[k].body.x;
	  human->state[i].particle[k].body.y     = particle[k].body.y;
	  human->state[i].particle[k].body.theta = particle[k].body.theta;
	  human->state[i].particle[k].body.speed = particle[k].body.speed;
	  human->state[i].particle[k].body.omega = particle[k].body.omega;
	  human->state[i].particle[k].body.omega = particle[k].weight;
	  human->state[i].particle[k].weight /= wsum;
	} //end for(k=0; k<t_NUM_OF_PARTICLE; k++)

      }//end if(human.id >0)

      // Update the human particles
      if(count >= human->numHumans)
	break;

    }//end for(i=0; i<ssm_MAX_NUM_OF_PEOPLE; i++)

}


void t_resample_particles(t_Entity *human){
  
  t_normalize(human);
  
  // Resample based on the particle weights --------
  t_frequency_based_resample(human);
}

//This function puts flag to false to all clusters whose centroid are close to the position of humans
void t_identify_new_legs(ClusterStruct *cl, t_Entity *human){

  for(int i=0; i< cl->numClusters; i++)    {
    double xc = cl->cluster[i].centroid.x; 
    double yc = cl->cluster[i].centroid.y; 
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
	  cl->cluster[i].flag  = false;
	  //printf("lcluster[%d].flag = %d\n", i, cl->cluster[i].flag);	  
	}
  }//end for i
}

//Initializes the particles for a human
void t_init_human_particles(t_Human *human){
  if(human->ID >=0){
    for(int j=0; j<t_NUM_OF_PARTICLE; j++){
      
      //Assign state variables
      human->particle[j].body.x = human->mean.x + rand_normal (0, t_INITIALIZATION_NOISE );
      human->particle[j].body.y = human->mean.y + rand_normal (0, t_INITIALIZATION_NOISE );
      human->particle[j].body.theta = human->mean.theta;
      human->particle[j].body.speed = human->mean.speed;
      human->particle[j].body.omega = human->mean.omega;
      human->particle[j].weight = 1.0/t_NUM_OF_PARTICLE;
      
    }
  } 
}

//Initializes the humans and their particles
void t_init_human(t_Entity *human,
		 ClusterStruct *cl, 
		 int id1,
		  int id2,
		  int *t_Max_ID_Index,
		  int t_People_ID[ssm_MAX_NUM_OF_PEOPLE] 
		  ){
  int id, i;
  double x1, y1, x2, y2;
  
  id = t_assign_people_id(t_Max_ID_Index, t_People_ID);

  if(id != -1 ){
    human->numHumans ++;
    x1 = cl->cluster[id1].centroid.x; 
    y1 = cl->cluster[id1].centroid.y; 
    x2 = cl->cluster[id2].centroid.x; 
    y2 = cl->cluster[id2].centroid.y; 
  

    //Assign state variables
    human->state[id].mean.x      = ( x1 + x2 ) / 2.0;
    human->state[id].mean.y      = ( y1 + y2 ) / 2.0;
    human->state[id].mean.theta  = 0 ; // no prior knowledge
    human->state[id].mean.speed  = 0 ; // no prior knowledge
    human->state[id].mean.omega  = 0 ; // no prior knowledge

    //Assign other variables        ----------------------

    human->state[id].ID = id;
    //human[id].count = 0; //Counter of the number of human

    human->state[id].tracked_time = 0;
  
    //human[id].weight = 1;
    human->state[id].dispersion = 0;
  
    for(i=0; i<t_WEIGHT_MEMORY_BUFFER; i++)
      human->state[id].weight_memory[i] = t_MIN_WEIGHT;
    human->state[id].weight_memory_count = 0;

    //  Initialize particles        ---------------------
    
    t_init_human_particles(&human->state[id]);

}

}

//Birth of humans 
//Creates a human based on the counter of the cluster,if bigger than t_CYCLES_TO_WAIT
//Using only present clusters.  Sets cluster.flag to false

void t_Create_Humans(ClusterStruct *cl, cycle_counter *cnt, t_Entity *human, int *t_Max_ID_Index,  int t_People_ID[ssm_MAX_NUM_OF_PEOPLE]     ){
  double mindist = 1000000;
  int id = -1;

  for( int i=0; i<cl->numClusters; i++ ) {

    find_closest_cluster(i, &id, &mindist, cl);
    if( id!=-1 && mindist<=t_MAX_STEP_SIZE && cnt->counter[i]>=t_CYCLES_TO_WAIT && cnt->counter[id]>=t_CYCLES_TO_WAIT ){

      t_init_human( human, cl, i, id, t_Max_ID_Index, t_People_ID );

    //printf("\n id:%d  i:%d mindist:%f  count: %d",id, i, mindist, cnt->counter[i]);
      
    cl->cluster[i].flag = false;
    cl->cluster[id].flag = false;
    }//end if id
  }//end for i
  
}



#endif
