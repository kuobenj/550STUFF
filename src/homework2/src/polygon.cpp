#include "polygon.h"
#include <cmath>
#include <stdio.h>

My_Polygon::
My_Polygon(){

}

My_Polygon::
My_Polygon(float init_x, float init_y, float init_width, float init_height){
	printf("%f %f %f %f\n", init_x, init_y, init_width, init_height);
	center_x = init_x;
	center_y = init_y;
	width = init_width;
	height = init_height;

	corners[0][0] = center_x-(width/2.0);
	corners[0][1] = center_y-(height/2.0);

	corners[1][0] = center_x+(width/2.0);
	corners[1][1] = center_y-(height/2.0);

	corners[2][0] = center_x+(width/2.0);
	corners[2][1] = center_y+(height/2.0);

	corners[3][0] = center_x-(width/2.0);
	corners[3][1] = center_y+(height/2.0);

	corners[4][0] = center_x-(width/2.0);
	corners[4][1] = center_y-(height/2.0);
}

My_Polygon::
~My_Polygon(){

}

void
My_Polygon::
update_center(float new_x, float new_y){

	center_x = new_x;
	center_y = new_y;

	corners[0][0] = center_x-(width/2.0);
	corners[0][1] = center_y-(height/2.0);

	corners[1][0] = center_x+(width/2.0);
	corners[1][1] = center_y-(height/2.0);

	corners[2][0] = center_x+(width/2.0);
	corners[2][1] = center_y+(height/2.0);

	corners[3][0] = center_x-(width/2.0);
	corners[3][1] = center_y+(height/2.0);

	corners[4][0] = center_x-(width/2.0);
	corners[4][1] = center_y-(height/2.0);
}

float
My_Polygon::
distance_to_point(float point_x, float point_y){

	float min_dist = 1000.0;
	float dist;
	for (int i = 0; i < 4; i++)
	{
		//algorithm to find distance to a line segment: http://stackoverflow.com/questions/10983872/distance-from-a-point-to-a-polygon
		float vector1[2];
		vector1[0] = this->corners[i+1][0]-this->corners[i][0];
		vector1[1] = this->corners[i+1][1]-this->corners[i][1];

		float vector2[2];
		vector2[0] = point_x-this->corners[i][0];
		vector2[1] = point_y-this->corners[i][1];

		float vector3[2];
		vector3[0] = this->corners[i+1][0]-point_x;
		vector3[1] = this->corners[i+1][1]-point_y;

		float dot_product = vector1[0]*vector2[0]+vector1[1]*vector2[1];

		//normalize

		float magnitude = sqrt(vector2[0]*vector2[0]+vector2[1]*vector2[1]);

		dot_product = dot_product/magnitude;

		

		if (dot_product < 0)
			dist = sqrt(vector2[0]*vector2[0]+vector2[1]*vector2[1]);
		else if (dot_product > 1)
			dist = sqrt(vector3[0]*vector3[0]+vector3[1]*vector3[1]);
		else
			dist = sqrt(vector2[0]*vector2[0]+vector2[1]*vector2[1] - dot_product * (vector1[0]*vector1[0]+vector1[1]*vector1[1]));

		if (dist < min_dist)
			min_dist = dist;
	}

	return dist;
}

void 
My_Polygon::
closest_point(float point_x, float point_y, float &closest_x, float &closest_y)
{

	float min_dist = 1000.0;
	float dist;
	float close_x;
	float close_y;
	for (int i = 0; i < 4; i++)
	{
		//algorithm to find distance to a line segment: http://stackoverflow.com/questions/10983872/distance-from-a-point-to-a-polygon
		float vector1[2];
		vector1[0] = this->corners[i+1][0]-this->corners[i][0];
		vector1[1] = this->corners[i+1][1]-this->corners[i][1];

		float vector2[2];
		vector2[0] = point_x-this->corners[i][0];
		vector2[1] = point_y-this->corners[i][1];

		float vector3[2];
		vector3[0] = this->corners[i+1][0]-point_x;
		vector3[1] = this->corners[i+1][1]-point_y;

		float dot_product = vector1[0]*vector2[0]+vector1[1]*vector2[1];

		//normalize

		float magnitude = sqrt(vector2[0]*vector2[0]+vector2[1]*vector2[1]);

		dot_product = dot_product/magnitude;

		

		if (dot_product < 0)
		{
			dist = sqrt(vector2[0]*vector2[0]+vector2[1]*vector2[1]);
			close_x = this->corners[i][0];
			close_y = this->corners[i][1];
		}
		else if (dot_product > 1)
		{
			dist = sqrt(vector3[0]*vector3[0]+vector3[1]*vector3[1]);
			close_x = this->corners[i+1][0];
			close_y = this->corners[i+1][1];
		}
		else
		{
			dist = sqrt(vector2[0]*vector2[0]+vector2[1]*vector2[1] - dot_product * (vector1[0]*vector1[0]+vector1[1]*vector1[1]));
			close_x = this->corners[i][0] + dot_product*vector1[0];
			close_y = this->corners[i][1] + dot_product*vector1[1];
		}

		if (dist < min_dist)
		{
			min_dist = dist;
			closest_x = close_x;
			closest_y = close_y;
		}
	}
}