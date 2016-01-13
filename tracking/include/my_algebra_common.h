#include <math.h>
#include <stdio.h>
#include <stdlib.h>
//#include <SsmStructures.h>

#ifndef  ALGEBRA_COMMON
#define ALGEBRA_COMMON

typedef struct{
	float x;
	float y;
	float z;
	float r;  //range of the point
	float i;  //intensity
} pct_Point;




#define deg2rad(deg) (deg)*M_PI/180.0
#define rad2deg(rad) (rad)*180.0/M_PI

double pct_Dist_Between_Points(  pct_Point p1,  pct_Point p2);
double Dist_Between_Points( double x1, double x2, double y1, double y2);
double Dist_Between_Points_3D( double x1, double x2, double y1, double y2, double z1, double z2);

double cal_ang_rad(double th);
double cal_ang_deg(double th);

double Prob_2_logOdd ( double p );
double logOdd_2_Prob ( double logOdd );

int eigenvalue_matrix2d(double mat[2][2],double *l1, double *l2);
int eigenvector_matrix2d(double mat[2][2],
			 double *v1, double *v2,double *l1,double *l2);

//void point_to_point_transform( pct_Point *src_point,  pct_Point* dst_point, pct_Pose* pose);
double dist_point_to_line ( double px, double py, double x1, double y1, double x2, double y2);
double pct_dist_point_to_line ( pct_Point point, pct_Point lp1, pct_Point lp2);

double nrand(double n);
double nrandDiff(double n);

#endif
