#pragma once
#include "common_include.h"

void obj2off(string obj, string off)
{
	ifstream fin(obj);
	string null, type, s1, s2, s3, s4;
	vector<vector<int>> faces;
	vector<Point> points;
	vector<Edge> edges;


	while (fin >> type)
	{
		if (type == "v")
		{
			fin >> s1 >> s2 >> s3;
			double x = atof(s1.c_str());
			double y = atof(s2.c_str());
			double z = atof(s3.c_str());
			Point p(x, y, z);
			points.push_back(p);
		}
		if (type == "e")
		{

		}
		if (type == "f")
		{
			fin >> s1 >> s2 >> s3;
			int p1 = atoi(s1.c_str()) - 1;
			int p2 = atoi(s2.c_str()) - 1;
			int p3 = atoi(s3.c_str()) - 1;
			vector<int> f = { p1,p2,p3 };
			//this->facemap.insert(pair<Face,int>(f, this->faces.size()));
			faces.push_back(f);

		}
	}


	std::ofstream out(off, std::ios::out);
	out << "OFF" << endl;
	out << points.size() << " " << faces.size() << " 0" << endl;

	for (int i = 0; i < points.size(); i++)
	{
		out << points[i][0] << " " << points[i][1] << " " << points[i][2] << endl;

	}
	for (int i = 0; i < faces.size(); i++)
	{
		out << "3 " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] << endl;

	}

	out.close();
}

void off2obj(string off, string obj)
{
	ifstream fin(off);
	string null, type, s1, s2, s3, s4;
	int pnum, fnum;
	vector<vector<int>> faces;
	vector<Point> points;
	vector<Edge> edges;

	fin >> null >> s1 >> s2 >> null;

	pnum = atoi(s1.c_str());
	fnum = atoi(s2.c_str());

	for (int i = 0; i < pnum; i++)
	{
		fin >> s1 >> s2 >> s3;
		double x = atof(s1.c_str());
		double y = atof(s2.c_str());
		double z = atof(s3.c_str());
		Point p(x, y, z);
		points.push_back(p);
	}

	for (int i = 0; i < fnum; i++)
	{
		fin >> null >> s1 >> s2 >> s3;
		int p1 = atoi(s1.c_str()) + 1;
		int p2 = atoi(s2.c_str()) + 1;
		int p3 = atoi(s3.c_str()) + 1;
		vector<int> f = { p1,p2,p3 };
		//this->facemap.insert(pair<Face,int>(f, this->faces.size()));
		faces.push_back(f);
	}



	std::ofstream out(obj, std::ios::out);
	for (int i = 0; i < points.size(); i++)
	{
		out << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2] << endl;

	}
	for (int i = 0; i < faces.size(); i++)
	{
		out << "f " << faces[i][0] << " " << faces[i][1] << " " << faces[i][2] << endl;

	}

	out.close();
}