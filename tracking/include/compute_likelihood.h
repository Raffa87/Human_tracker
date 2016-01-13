#include "my_algebra_common.h"
#include <deque>
#include <tracking/PoseWithCovarianceStampedArray.h>
#include <tracking/Candidate.h>
#include <tracking/CandidateArray.h>
#include <tracking/EmptyLikelihood.h>


#define t_NUM_OF_PARTICLE       50               
#define t_WEIGHT_MEMORY_BUFFER  50
#define NUM_OF_PEOPLE           50
#define ssm_MAX_NUM_OF_PEOPLE 50
#define t_MAX_STEP_SIZE         650              // to compare the previous no of cluster with present and integration in mm


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

void t_initialize_entities(t_Entity *human);
void t_identify_new_legs(std::vector<tracking::Candidate> &candidates, t_Entity *human);
void update_all_time_candidates(std::vector<tracking::Candidate> &candidates, std::vector<tracking::Candidate>  &all_candidates);
void compute_likelihood(std::vector<tracking::Candidate> &candidates, std::vector<tracking::Candidate> &all_candidates, double forgetting_factor);



double gauss(double x, double y, double xp, double yp, double sx, double sy);
double gauss(double distance, double sigma, uint dimension);

