#include "my_algebra_common.h"
#include <tracking/Candidate.h>
#include <tracking/CandidateArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>

#define t_NUM_OF_PARTICLE       50               
#define t_WEIGHT_MEMORY_BUFFER  50

#define t_MIN_DISTANCE_FROM_HUMAN         0.500		 //in m		//to compare the current candidate to old one - if it is to close, it cannot be a new human
#define t_MIN_WEIGHT            0.001            
#define t_INITIALIZATION_NOISE  0.150		 //in m		//150              // in mm
#define t_MAX_TRAN_VELOCITY     1.7		 //in m/s	//1700             // in mm/s
#define t_MIN_DISPERSION        0.500		 //in m		//500              // Minimum dispersion of particles before disappearing in mm


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

void t_Create_Humans(std::vector<tracking::Candidate> &candidates, std::vector<t_Human> &human);
//void find_closest_cluster(int index, int *id, double *mindist, std::vector<tracking::Candidate> &candidates);
double find_closest_human(tracking::Candidate *candidates, std::vector<t_Human> &human);


//int t_assign_people_id(int *t_Max_ID_Index, int t_People_ID[NUM_OF_PEOPLE] );
void t_init_human(std::vector<t_Human> &human, tracking::Candidate *candidate);
void t_init_human_particles(t_Human *human);
double rand_normal(double mean, double stddev);

void t_predict_particle_motion(std::vector<t_Human> &human, double ts);
int t_prediction_model(t_Human *human, double ts);


void t_find_particle_likelihood(std::vector<t_Human> &human, std::vector<tracking::Candidate> &candidates, double ts);
void t_likelihood_model(t_Human *human, std::vector<tracking::Candidate> &candidates, double ts);


void t_update_human_states(std::vector<t_Human> &human, double ts);
void t_find_average_particle(t_Human *human, double ts);
void t_find_dispersion(t_Human *human);

void t_remove_uncertain_humans(std::vector<t_Human> &human);


double gauss(double x, double y, double xp, double yp, double sx, double sy);
double gauss(double distance, double sigma, uint dimension);

void visualize_human(std::vector<t_Human> &human);
void visualize_particles(std::vector<t_Human> &human);




