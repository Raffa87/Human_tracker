#include "../include/my_algebra_common.h"
#include <stdlib.h>


double Dist_Between_Points( double x1, double x2, double y1, double y2){
  return ( sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) ) );
}


double pct_Dist_Between_Points(  pct_Point p1,  pct_Point p2){
  return ( sqrt( (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) ) );
}



double Dist_Between_Points_3D( double x1, double x2, double y1, double y2, double z1, double z2){
  return ( sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1) ) );
}


double cal_ang_rad(double th){
  while (th > M_PI) 
    th -= M_PI * 2 ;
    
  while (th < -M_PI) 
    th += M_PI * 2 ;
  
  return(th);
}

double cal_ang_deg(double th){
  while ( th > rad2deg(M_PI) ) 
    th -= rad2deg(M_PI * 2) ;
    
  while (th < rad2deg(-M_PI) ) 
    th += rad2deg(M_PI * 2) ;
  
  return(th);
}

double Prob_2_logOdd ( double p ){
  return (  log( p/(1-p) )  );
}

double logOdd_2_Prob ( double logOdd ){
  return (  1 - ( 1 / (1 + exp(logOdd) ) )  );
}





/*eigen value*/
int eigenvalue_matrix2d(double mat[2][2],double *l1, double *l2){
  double a,b,c,x;
  
  a =  1.0;
  b = -mat[0][0]-mat[1][1];
  c =   mat[0][0]*mat[1][1] - mat[1][0]*mat[0][1];
  
//	if(fabs(a) < E_ERROR)return(0);

  x = sqrt(b*b - 4* a* c);
  *l1 = (-b+x)/2*a;
  *l2 = (-b-x)/2*a;
  if(fabs(*l1) < fabs(*l2)){
    x  = *l1;
    *l1 = *l2;
    *l2 = *l1;
  }
  return 1;
}




/*eigen vector*/
int eigenvector_matrix2d(double mat[2][2],
			 double *v1, double *v2,double *l1,double *l2){
  double a;
  
  if(!eigenvalue_matrix2d(mat,l1,l2))return 0;
  
  v1[0] = mat[0][1];
  v1[1] = -(mat[0][0]-*l1);
  a = sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
  v1[0] = v1[0]/a;
  v1[1] = v1[1]/a;
  
  v2[0] = mat[1][1]-*l2;
  v2[1] = -mat[1][0];
  a = sqrt(v2[0]*v2[0] + v2[1]*v2[1]);
  v2[0] = v2[0]/a;
  v2[1] = v2[1]/a;
  
  return 1;
}
/*
void point_to_point_transform( pct_Point *src_point,  pct_Point* dst_point, pct_Pose* pose)
{
  float sinYaw = sin(pose->yaw_deg * M_PI / 180);
  float cosYaw = cos(pose->yaw_deg * M_PI / 180);
  float sinPitch = sin(pose->pitch_deg * M_PI / 180);
  float cosPitch = cos(pose->pitch_deg * M_PI / 180);
  float sinRoll = sin(pose->roll_deg * M_PI / 180);
  float cosRoll = cos(pose->roll_deg * M_PI / 180);
	
  //if points dont exist, they still dont exist
  if( src_point->x == 0 && src_point->y == 0 && src_point->z == 0)
    {	
      dst_point->x = 0;
      dst_point->y = 0;
      dst_point->z = 0;
    }
  
	else
	{
	dst_point->x = src_point->x * cosYaw * cosPitch
		- src_point->y * sinYaw * cosRoll
		+ src_point->y * cosYaw * sinPitch * sinRoll
		+ src_point->z * sinYaw * sinRoll
		+ src_point->z * cosYaw * sinPitch * cosRoll
		+ pose->x_mm;

	dst_point->y = src_point->x * sinYaw * cosPitch
		+ src_point->y * cosYaw * cosRoll
		+ src_point->y * sinYaw * sinPitch * sinRoll
		- src_point->z * cosYaw * sinRoll
		+ src_point->z * sinYaw * sinPitch * cosRoll
		+ pose->y_mm;

	dst_point->z = - src_point->x * sinPitch
		+ src_point->y * cosPitch * sinRoll
		+ src_point->z * cosPitch * cosRoll
		+ pose->z_mm;
	}
}
*/


double dist_point_to_line ( double px, double py, double x1, double y1, double x2, double y2){
  return ( 
	  abs( (x2-x1)*(y1-py) - (x1-px)*(y2-y1) ) / sqrt( (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1) )
	   );
}

/*
double nrandDiff(double n)
{
  double r = nrand(n);
  int num = rand();
  
  if(num%2 == 0)
    r*=-1;
  return r ;
}
*/

double pct_dist_point_to_line ( pct_Point point, pct_Point lp1, pct_Point lp2){
  return ( 
	  abs( (lp2.x-lp1.x)*(lp1.y-point.y) - (lp1.x-point.x)*(lp2.y-lp1.y) ) / sqrt( (lp2.x-lp1.x)*(lp2.x-lp1.x)+(lp2.y-lp1.y)*(lp2.y-lp1.y) )
	   );
}
