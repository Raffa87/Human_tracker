/*************************************************************
Author: Rajibul Huq
Date: December 24, 2009
Description: Header file for tracking library.

Copyright (c) 2009 ATR
************************************************************* */
#ifndef __TRACKING_H__
#define __TRACKING_H__

/*
#ifdef __cplusplus
extern "C" {
#endif
*/

#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <math.h>
#include <SsmStructures.h>

/*
#include <cloud.h>
#include <ssm_kinect.h>
#include <obstacle.h>
*/
//To erase:
#define NUMBER_OF_OBSTACLES 150	
typedef struct _kinect_point_data *Kinect_Point_DataPtr;
typedef struct _kinect_point_data{
	float X_mm;
	float Y_mm;
	float Z_mm;
	float coor(int coord)
	{
		switch(coord)
		{
		case 0:
			return X_mm;
			break;
		case 1:
			return Y_mm;
			break;
		case 2:
			return Z_mm;
			break;
		default:
			return 0;
		}
	}
	bool isempty()
	{
		if(X_mm == 0 && Y_mm == 0 && Z_mm == 0)
			return true;
		else
			return false;
	}
	unsigned char traverse;
}Kinect_Point_Data;

typedef struct obstacle_data {
	int obstacle_id;
	float miny;
	float maxy;
	float minz;
	float maxz;
	Kinect_Point_Data centroid;
}Obstacle_Data;



using namespace std;


/* ---- defines ---- */
#define t_PI                    M_PI
#define t_URG_DATA_POINTS       1081
#define t_URG_COVERAGE          (3*t_PI/2)//(3*t_PI/2)                                              // in radian
#define t_URG_RESOLUTON         (t_URG_COVERAGE/t_URG_DATA_POINTS)                      // in radian
#define t_URG_ANG_OFFSET        (3*t_PI/4)//(3*t_PI/4)                                              // in radian
#define t_URG_LOWER_LIMIT       200 //500    shortest urg point                          // in mm
#define t_URG_UPPER_LIMIT       20000//25000 farthest urg point                         // in mm
#define t_URG_X_OFFSET          150//200//R3                                                     // in mm
#define t_URG_Y_OFFSET          0                                                       // in mm
#define t_URG_TH_OFFSET         0     // in rad
#define t_BACK_URG_X_OFFSET     -260//-370//R3                                                     // in mm
#define t_BACK_URG_Y_OFFSET     0                                                       // in mm
#define t_BACK_URG_TH_OFFSET     180*M_PI/180                                                       // in rad// 183.5*M_PI/180                                                       // in rad
#define t_MAX_NUM_OF_CLUSTER    t_URG_DATA_POINTS*2
#define t_MAX_NUM_OF_PEOPLE     ssm_MAX_NUM_OF_PEOPLE
#define t_MAX_TRACKING_RANGE    8000                                                    // in mm
#define t_MAX_LEFT_ANGLE        3*t_PI//(3*t_PI/4.0)                                            // in radian
#define t_MAX_RIGHT_ANGLE       3*-t_PI//(-3*t_PI/4.0)                                           // in radian
#define t_THETA_STEP            t_PI/180.0                                              // in radian
#define t_INVALID_COORD         0.05
#define t_MAX_STEP_SIZE         750//600    to compare the previous no of cluster with present and integration // in mm
#define t_NUM_OF_PARTICLE       50
#define t_MAX_TRAN_VELOCITY     1700                                         // in mm/s
//#define t_MAX_ROT_VELOCITY      3*t_PI                                                    // in rad/s
//#define t_HUMAN_ACCELERATION    2*t_MAX_TRAN_VELOCITY                                   // in mm/s^2
#define t_MEASUREMENT_NOISE     150                                                // in mm
#define t_MAX_ID                1000
#define t_INITIALIZATION_NOISE  150 // in mm
#define t_WEIGHT_BUFFER_LENGTH  50
#define t_SPEED_BUFFER_LENGTH   5
#define t_THETA_BUFFER_LENGTH   5
#define t_OMEGA_BUFFER_LENGTH   5
#define t_MIN_DISPERSION        400 // in mm
#define t_MIN_WEIGHT            0.01
#define t_MAP_XMIN              -30000
#define t_MAP_YMIN              -40000
#define t_CYCLES_TO_WAIT        10

#define t_CLUSTER_MIN_WIDTH     35
#define t_CLUSTER_MAX_WIDTH     650//400


#define t_deg_to_rad(deg)       (deg)*t_PI/180.0
#define t_rad_to_deg(rad)       (rad)*180.0/t_PI
#define t_find_max_pt(dis)      (t_MAX_MAJORAX_LENGTH)/(t_URG_RESOLUTON*dis)


/* ---- typedefs ---- */
// Point structure that refers to a (x, y) location
typedef struct{
    double x;
    double y;
}t_Point;

// Pose structure that refers to a location (x, y) with heading direction
typedef struct{
    double x;
    double y;
    //t_Point location;
    double theta;
}t_Pose;

typedef struct{
    double majorax;
    double minorax;
    double theta;
}t_Ellipse;


/*
    Cluster structure that provides the following information:
        1) Number of data points in the cluster
        2) Centroid of the cluster
        3) Lengths of major and minor axes of the optimal ellipsoid that surrounds the cluster in mm
        4) Orientation of the cluster in radian
        5) A flag which is set to true if the cluster matches a specific pattern.
*/
typedef struct{
    int N;
    t_Point centroid;
    double  majorax;
    double  minorax;
    double  theta;
    bool    flag;
    double  width;
    double  length;
    double  mid_ini_length;
    double  mid_end_length;
    double  xc;
    double  yc;
    double  r;
    double  n;
    /*double  d;*/
    double th1;
    double th2;
}t_Cluster;


typedef struct{
    /*
        (x, y)  : position
        (v, vtheta): velocity
    */
    double x;
    double y;
    double vx;
    double vy;
    double d;
    double moving_time;
    bool is_moving;
}t_Leg;

typedef struct{
    /*
        (x, y): position
        theta : heading direction
        omega : angular velocity
    */
    double x;
    double y;
    double theta;
    double speed;
    double omega;
}t_Body;


typedef struct{

    t_Body body;
    double weight;

}t_Particle;


typedef struct{

    /*
        State variables
    */
    t_Body body;


    /*
        Other variables (We may remove the following variable(s) if not necessary)
    */
    int ID;                         // A unique ID assigned to each person
    double tracked_time;               // Overall tracked time
    t_Ellipse surrounding_ellipse;  // Parameters for drawing an ellipse surrounding a person
    double weight;
    double dispersion;
    double leg_to_leg_distance;
    double wbuffer[t_WEIGHT_BUFFER_LENGTH];
    int wcount;
    double sbuffer[t_SPEED_BUFFER_LENGTH];
    int scount;
    double tbuffer[t_THETA_BUFFER_LENGTH];
    int tcount;
    double obuffer[t_OMEGA_BUFFER_LENGTH];
    int ocount;
    /*
        Multiple hypotheses
    */
    t_Particle particle[t_NUM_OF_PARTICLE];


}t_Human;

/* ---- function prototypes ---- */

inline t_Point tPoint(double x, double y){
    t_Point point;
    point.x = x;
    point.y = y;
    return point;
}

inline t_Pose tPose(double x, double y, double theta)
{
    t_Pose pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    return pose;
}

// Structure for writing to SSM -------------
typedef struct{
    int n;
    int id[t_MAX_NUM_OF_PEOPLE];
    double x[t_MAX_NUM_OF_PEOPLE];
    double y[t_MAX_NUM_OF_PEOPLE];
    double theta[t_MAX_NUM_OF_PEOPLE];
}People;

typedef struct{
    t_Human     *human;
    int         no_of_humans;
    t_Cluster   *lcluster;
    int         no_of_lclusters;
    t_Point     *global_xy;
    t_Pose      pose;
    People      *people;
    double      ts;
    bool        updated;
    //int         updated;
}t_Tracking_Data;

typedef struct{
	t_Human  human_data[t_MAX_NUM_OF_PEOPLE];
	int k;
}write_ssm_Human_2d;

extern int t_People_ID[];
extern int t_Max_ID_Index;
//extern int t_Tmp_People_ID[];

/*
    Function name: t_getch
    Description: This function implements the DOS getch().
*/
extern int t_getch(void);

/*
    Function name: t_get_range_to_local_xy_coordinate
    Description: This function takes laser range data as the input and produces
        corresponding (x, y) positions in the local coordinate system. The range
        data within the range of t_URG_LOWER_LIMIT and t_URG_UPPER_LIMIT are
        transformed into (x, y) positions; the rest are set to
        (t_URG_UPPER_LIMIT+1).
    Inputs:
        range:      Laser range in mm with the data length t_URG_DAtA_POINTS
        local_xy:   Transformed (x, y) positions in the local coordinate system with
                    data length t_URG_DAtA_POINTS
    Return value:
        This function returns 1 in case of error, 0 otherwise.
*/
extern int t_get_range_to_local_xy_coordinate(int range[t_URG_DATA_POINTS], t_Point local_xy[t_URG_DATA_POINTS]);
extern int t_omni_get_range_to_local_xy_coordinate(int front_range[t_URG_DATA_POINTS], int back_range[t_URG_DATA_POINTS], t_Point local_xy[t_URG_DATA_POINTS*2]);


/*
    Function name: t_find_clusters
    Description: This function takes a set of (x, y) data points and partitions
        the data points according to point-to-point distanc and angular deviation.
    Inputs:
        xy              : A set of (x, y) data points
        dis_th          : A distance threshold in mm
        ang_th          : An angular threshold in radian
        clusters        : The clusters obtained using the given thresholds
        no_of_cluster   : The total number of clusters
    Return value:
        This function returns 1 in case of error, 0 otherwise.
*/
extern int t_find_clusters(t_Point xy[t_URG_DATA_POINTS], double dis_th, double ang_th, t_Cluster cluster[t_MAX_NUM_OF_CLUSTER], int *no_of_cluster);
extern int t_omni_find_clusters(t_Point xy[t_URG_DATA_POINTS*2], double dis_th, double ang_th, t_Cluster cluster[t_MAX_NUM_OF_CLUSTER], int *no_of_cluster);

/*
    Function name: t_initialize_cluster
    Description: This function initializes a cluster with the default values.
    Inputs:
        cluster: A cluster.
    Reurn value:
        This function returns 1 in case of error, 0 otherwise.
*/
extern int t_initialize_cluster(t_Cluster *cluster);


/*
    Function name: t_find_euclidean_distance
    Description: This function calculates the Euclidean distance between two points.
    Inputs:
        point1: The first point.
        point1: The second point.
    Reurn value:
        The Euclidean distance between the points.
*/
extern double t_find_euclidean_distance(t_Point point1, t_Point point2);


/*
    Function name: t_initialize_eigen_matrix
    Description: This function initializes a matrix to obtain the eigen values.
    Inputs:
        emat: The matrix to be initialized
        sx: Summation of x
        sxx: Summation of x^2
        sy: Summation of y
        syy: Summation of y^2
        sxy: Summation of xy
        n: The number of data points
    Reurn value:
        0: No error
        1: In case of error, e.g. the number of data points is zero.
*/
extern int t_initialize_eigen_matrix(double emat[][2], double sx, double sxx, double sy, double syy, double sxy, int n);

/*
    Function name: t_find_axes_theta
    Description: This function calculates the eigen values of the input matrix. It also determines orientation of
        major axis.
    Inputs:
        emat: The matrix used to calculate eigen values
        majorax: Length of major axis
        minorax: Length of minor axis
        theta: Orientation of the major axis
    Reurn value:
        0: No error
        1: In case of error, e.g. if real solutions do not exist.
*/
extern int t_find_axes_theta(double emat[][2], double *majorax, double *minorax, double *theta);


/*
    Function name: t_update_cluster
    Description: It updates the cluster information.
    Inputs:
        emat: The matrix used to calculate eigen values
        majorax: Length of major axis
        minorax: Length of minor axis
        theta: Orientation of the major axis
    Reurn value:
        0: No error
        1: In case of error, e.g. if real solutions do not exist.
*/
extern int t_update_cluster(double sx, double sxx, double sy, double syy, double sxy, t_Cluster *cluster);


/*
    Function name: t_show_cluster_info
    Description: It shows the cluster information.
    Inputs:
        cluster: A cluster structure
        str: A string to be
    Reurn value: None
*/
extern void t_show_cluster_info(t_Cluster cluster);


/*
    Function name: t_is_leg
    Description: This function detects legs of people from the input clusters.
    Inputs:
        cluster         : Clusters obtained from the range data.
        no_of_cluster   : The number of the clusters
        leg             : the clusters classfied as legs
        no_of_legs      : The number of legs
        robot           : The robot's position
    Reurn value: None
*/
extern int t_is_leg(t_Cluster cluster[t_MAX_NUM_OF_CLUSTER],
                    int no_of_cluster,
                    t_Cluster leg[t_MAX_NUM_OF_CLUSTER],
                    int *no_of_legs, t_Pose robot
                   );



/*
    Function name: t_copy_cluster
    Description: This function copies the informaton from the source cluster to the destination cluster.
    Inputs:
        src: The source cluster
        dst: The destination cluster
    Return value:
        It returns 1 in case of NULL destination cluster; 0 otherwise.
*/
extern int t_copy_cluster(t_Cluster src, t_Cluster *dst);


/*
    Function name: t_find_rmse_of_scans
    Description: It calculates the root mean square error of two range scans.
    Inputs:
        scan1_xy: (x, y) coordinates of scan 1
        scan2_xy: (x, y) coordinates of scan 2
    Return value:
        It returns the root mean square error of the scans.
*/
extern double t_find_rmse_of_scans(t_Point scan1_xy[t_URG_DATA_POINTS], t_Point scan2_xy[t_URG_DATA_POINTS]);

/*
    Function name: t_copy_scan
    Description: It copies the source scan to the destination scan.
    Inputs:
        src: (x, y) coordinates of the source scan
        dst: (x, y) coordinates of the destination scan
    Return value: None
*/
inline void t_copy_scan(t_Point src[t_URG_DATA_POINTS], t_Point dst[t_URG_DATA_POINTS])
{
    int i;
    for(i=0; i<t_URG_DATA_POINTS; i++)
    {
        dst[i].x = src[i].x;
        dst[i].y = src[i].y;
    }
}

inline void t_omni_copy_scan(t_Point src[t_URG_DATA_POINTS*2], t_Point dst[t_URG_DATA_POINTS*2])
{
    int i;
    for(i=0; i<t_URG_DATA_POINTS*2; i++)
    {
        dst[i].x = src[i].x;
        dst[i].y = src[i].y;
    }
}



/*
    function name: t_gauss
    Description: Implements 2-D gaussian function.
    Inputs:
        (x, y)      : Vriables
        (xp, yp))   : Means of (x, y)
        (sx, sy)    : Standard deviations of (x, y)
    Return value:
        A non-normalized gaussian value
*/
inline double t_gauss(double x, double y, double xp, double yp, double sx, double sy)
{
    double A, X, Y, val;

    if(sx == 0) sx = 1;
    if(sy == 0) sy = 1;

    X = ((x-xp)/sx);
    Y = ((y-yp)/sy);
    A = 1.0;///(2*t_PI*sx*sy);

    val = A*exp(-0.5*(X*X + Y*Y));

    return val;
}

inline double t_gauss_1d(double x, double xp, double sx)
{
    double A, X, val;

    if(sx == 0) sx = 1;

    X = ((x-xp)/sx);
    A = 1.0;///(2*t_PI*sx);

    val = A*exp(-0.5*(X*X));

    return val;
}

inline double t_gauss_2d_polar(double x,
                               double y,
                               double xp,
                               double yp,
                               double sx,
                               double sy,
                               double theta
                               )
{
    double A, X, Y, val;
    double a, b,c;

    if(sx == 0) sx = 1;
    if(sy == 0) sy = 1;

    double cosT = cos(theta);
    double sinT = sin(theta);
    double sin2T = sin(2*theta);

    A = 1.0;///(2*t_PI*sx*sy);
    a = (cosT*cosT)/(2*sx*sx) + (sinT*sinT)/(2*sy*sy);
    b = -(sin2T/(4*sx*sx)) + sin2T/(4*sy*sy) ;
    c = (sinT*sinT)/(2*sx*sx) + (cosT*cosT)/(2*sy*sy);

    X = x - xp;
    Y = y - yp;

    val = A*exp(-(a*X*X + 2*b*X*Y + c*Y*Y));

    return val;
}

/*
    function name: t_transform_to_global_coordinate
    Description: Transforms laser data in 2-D space
    Inputs:
        (x, y)  : Offset values of (x, y)
        src     : Source laser data
        dst     : Destination laser data
    Return value: None
*/
extern void
t_transform_to_global_coordinate(double x,
                                 double y,
                                 double theta,
                                 t_Point src[t_URG_DATA_POINTS],
                                 t_Point dst[t_URG_DATA_POINTS]
                                );

extern void
t_omni_transform_to_global_coordinate(double x,
                                 double y,
                                 double theta,
				 t_Point src[t_URG_DATA_POINTS*2],
                                 t_Point dst[t_URG_DATA_POINTS*2]
                                );

//extern void t_kinect_transform_to_global_coordinate(double x, double y, double theta, Obstacle_Data src[NUMBER_OF_OBSTACLES], Obstacle_Data dst[NUMBER_OF_OBSTACLES]);

/*
    function name: t_check_angle
    Description: Confines an angle value from 0 to 2*pi.
    Inputs:
        x: An input angle in radian
    Return value:
        The output angle in radian
*/
extern double t_check_angle(double x);

/*
    Function name: t_3P_circle
    Descrtption: This function determines the center and radius of a circle formed by the given points.
    Inputs:
        p: An array of three points
        xc: x-coordinate of the circle center
        yc: y-coordinate of the circle center
        r : radius of the circle; r is -1 if a circle can not be formed using the points p1, p2, and p3.
    Return value:
        0: No errors
        1: Invalid pointer
*/
extern int t_3P_circle(t_Point p[3], double *xc, double *yc, double *r);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_copy_particle(t_Particle src, t_Particle *dst);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_assign_leg_id(void);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_assign_people_id(void);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_detect_leg_clusters(t_Point xy[t_URG_DATA_POINTS], t_Cluster leg[t_MAX_NUM_OF_CLUSTER], int *no_of_legs, t_Pose robot);
extern int t_omni_detect_leg_clusters(t_Point xy[t_URG_DATA_POINTS*2], t_Cluster leg[t_MAX_NUM_OF_CLUSTER], int *no_of_legs, t_Pose robot);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_find_closest_cluster(
                       int index,
                       int *id,
                       double *mindist,
                       t_Cluster lcluster[t_MAX_NUM_OF_CLUSTER],
                       int no_of_clusters
                      );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_init_human(t_Human human[t_MAX_NUM_OF_PEOPLE],
                 int *no_of_humans,
                 t_Cluster lcluster[t_MAX_NUM_OF_CLUSTER],
                 int id1,
                 int id2,
                 int hid,
                 bool flag
                 );


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int
t_create_humans(t_Cluster lcluster[t_MAX_NUM_OF_CLUSTER],
                int no_of_lclusters,
                t_Human human[t_MAX_NUM_OF_PEOPLE],
                int *no_of_humans,
                bool flag);


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
inline void
t_initialize_human_id(t_Human human[t_MAX_NUM_OF_PEOPLE])
{
    for(int i=0; i<t_MAX_NUM_OF_PEOPLE; i++)
    {
        human[i].ID = -1;
    }
}

/*
    Function name: t_max
    Description:
    Inputs:
    Return value:
*/
inline double
t_max(double x, double y)
{
    double z;
    if(x >= y)
    {
        z = x;
    }else{
        z = y;
    }
    return z;
}

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
inline double
t_min(double x, double y)
{
    double z;
    if(x <= y)
    {
        z = x;
    }else{
        z = y;
    }
    return z;
}


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
inline double t_rand(double x)
{
    //srand(time(0));
    return x*((rand()/(RAND_MAX + 1.0)));
}

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern double t_nrand(double mean, double std);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void t_predict_particle_motion(t_Human human[t_MAX_NUM_OF_PEOPLE], int no_of_humans, double ts);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void t_update_human_states(t_Human human[t_MAX_NUM_OF_PEOPLE], int no_of_humans, double ts);


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void t_remove_uncertain_humans(t_Human human[t_MAX_NUM_OF_PEOPLE], int *no_of_humans);


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_write_to_file(t_Pose robot, double laser_time_stamp, int range[t_URG_DATA_POINTS], FILE *file);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_read_file(FILE *file, t_Pose *robot, double *laser_time_stamp, int range[t_URG_DATA_POINTS], int *eof);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int
t_write_human_states(FILE *sfile,
                     t_Human human[t_MAX_NUM_OF_PEOPLE],
                     int no_of_humans,
                     long int frame_count
                     );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern double t_find_force(t_Human human[t_MAX_NUM_OF_PEOPLE],
                 int no_of_humans,
                 int i,
                 double x,
                 double y,
                 double sx,
                 double sy
                );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_find_human_positions(
                                 t_Cluster lcluster[t_MAX_NUM_OF_CLUSTER],
                                 int no_of_lclusters,
                                 t_Point human_position[t_MAX_NUM_OF_CLUSTER],
                                 int *no_of_human_positions,
                                 t_Point cluster_position[t_MAX_NUM_OF_CLUSTER],
                                 int *no_of_cluster_positions,
                                 t_Human human[t_MAX_NUM_OF_PEOPLE],
                                 int no_of_humans
                                );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int
t_init_human_particles(t_Human *human);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int
t_prediction_model(t_Human *human, double ts);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_find_particle_likelihood(t_Human human[t_MAX_NUM_OF_PEOPLE],
                           int no_of_humans,
                           t_Point human_positions[t_MAX_NUM_OF_CLUSTER],
                           int no_of_human_positions,
                           t_Point cluster_positions[t_MAX_NUM_OF_CLUSTER],
                           int no_of_cluster_positions,
                           t_Pose robot,
                           int range[t_URG_DATA_POINTS],
                           double ts
                           );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void t_likelihood_model(t_Human human[t_MAX_NUM_OF_PEOPLE],
                         int no_of_humans,
                         int i,
                         t_Point human_positions[t_MAX_NUM_OF_CLUSTER],
                         int no_of_human_positions,
                         t_Point cluster_positions[t_MAX_NUM_OF_CLUSTER],
                         int no_of_cluster_positions,
                         t_Pose robot,
                         int range[t_URG_DATA_POINTS],
                         double ts
                        );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_resample_particles(t_Human human[t_MAX_NUM_OF_PEOPLE], int no_of_humans);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_identify_new_legs(t_Cluster lcluster[t_MAX_NUM_OF_CLUSTER],
                    int no_of_lclusters,
                    t_Human human[t_MAX_NUM_OF_PEOPLE],
                    bool flag=true
                    );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_find_dipersion(t_Human *human);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_find_max_weight_particle(t_Human human, int *index, double *stdx, double *stdy);


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int t_find_average_particle(t_Human *human, double ts);

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_find_consistent_clusters(t_Cluster pre_lcluster[t_MAX_NUM_OF_CLUSTER],
                           int no_of_pre_lclusters,
                           t_Cluster lcluster[t_MAX_NUM_OF_CLUSTER],
                           int no_of_lclusters
                          );


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int
t_update_ssm_buffer(t_Human human[t_MAX_NUM_OF_PEOPLE],
                    int no_of_humans,
                    People *people,
		    People *global_people_data,
                    const t_Pose *raw_odo,
                    const t_Pose *corr_odo
                   );


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int
t_compare_scan_with_map(int *gridmap,
                        int width,
                        int height,
                        int xmin,
                        int ymin,
                        int xres,
                        int yres,
                        t_Point xy[t_URG_DATA_POINTS],
                        const t_Pose *raw_odo,
                        const t_Pose *corr_odo
                       );


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int
t_human_tracking(
           const t_Pose *raw_odo,
           const t_Pose *corr_odo,
           int          range[t_URG_DATA_POINTS],
           double       time_stamp,
           t_Human      person[t_MAX_NUM_OF_PEOPLE],
           int          *no_of_persons,
           t_Cluster    leg[t_MAX_NUM_OF_CLUSTER],
           int          *no_of_legs,
           t_Point      globalxy[t_URG_DATA_POINTS],
           double       *Ts,
           People       *people,
           FILE         *sfile/*,
           t_Point      *filtered_xy=NULL*/
          );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern int
t_omni_human_tracking(
           const t_Pose *raw_odo,
           const t_Pose *corr_odo,
           int          range[t_URG_DATA_POINTS],
           double       time_stamp,
           int          back_range[t_URG_DATA_POINTS],
           double       back_time_stamp,
           t_Human      person[t_MAX_NUM_OF_PEOPLE],
           int          *no_of_persons,
           t_Cluster    leg[t_MAX_NUM_OF_CLUSTER],
           int          *no_of_legs,
           t_Point      globalxy[t_URG_DATA_POINTS*2],
           double       *Ts,
           People       *people,
	   People       *global_people_data,
           FILE         *sfile/*,
           t_Point      *filtered_xy=NULL*/
          );


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_copy_clusters(
           t_Cluster src[t_MAX_NUM_OF_CLUSTER],
           t_Cluster dst[t_MAX_NUM_OF_CLUSTER],
           int no_of_clusters
          );


/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_copy_humans(
           t_Human src[t_MAX_NUM_OF_PEOPLE],
           t_Human dst[t_MAX_NUM_OF_PEOPLE],
           int no_of_humans
          );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_max_filter(
           int  src[t_URG_DATA_POINTS],
           int  dst[t_URG_DATA_POINTS],
           int  n
          );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_min_filter(
           int  src[t_URG_DATA_POINTS],
           int  dst[t_URG_DATA_POINTS],
           int  n
          );

/*
    Function name:
    Description:
    Inputs:
    Return value:
*/
extern void
t_mean_filter(
           int  src[t_URG_DATA_POINTS],
           int  dst[t_URG_DATA_POINTS],
           int  n
          );



/*
#ifdef __cplusplus
}
#endif
*/

#endif
