#pragma once

#include "common_include.h"

class Patch
{
public:
	vector<Point> points;
	vector<Face> faces;
	map<double, int> feature_map;
	feature_t* feature_value;
	float* feature_weight;
	bool valid = true;
	int seednode = 0;
	Point computeCentroid()
	{

		double sumx = 0, sumy = 0, sumz = 0;
		for (int i = 0; i < points.size(); i++)
		{
			sumx += points[i].x(); sumy += points[i].y(); sumz += points[i].z();
		}
		Point p(sumx / (float)points.size(), sumy / (float)points.size(), sumz / (float)points.size());
		return p;
	}
};
