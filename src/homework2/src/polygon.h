#ifndef MY_POLYGON_H_
#define MY_POLYGON_H_


//my polygon class limited to only rectangles

class My_Polygon
{

public:

	My_Polygon();
	My_Polygon(float center_x, float center_y, float width, float height);
	~My_Polygon();

	void update_center(float new_x, float new_y);

	float distance_to_point(float point_x, float point_y);

	void closest_point(float point_x, float point_y, float &closest_x, float &closest_y);

	float center_x;
	float center_y;
	float width;
	float height;
	float corners[5][2];

private:

};

#endif // MY_POLYGON_H_