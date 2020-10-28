#pragma once
#include "MAT.h"
#include "ombb/gdiam.hpp"

class Decomposer
{
public:
	vector<int> final_facelabel;
	Mesh readMesh(string path);
	void decompose3Dshape(MAT& mat, MAT& smat, Mesh& mesh, float growing_threshold, float min_region);
	void transfer_MAT_mesh(MAT& mat, Mesh& mesh, float weight);
	void primitiveAbstraction(MAT& mat, Mesh& mesh, vector<vector<float>> colors, string outputpath);
	void saveColoredMesh(MAT& mat, Mesh& mesh, vector<vector<float>> colors, string outputpath);
	void saveSegResult(MAT& mat, Mesh& mesh, string outputpath);

private:
	double compute_face_angle(Face& f1, Face& f2);
	double compute_bend_angle_degree(Face& f1, Face& f2);
	bool check_concave(Face& f1, Face& f2, Edge& sharededge);
	bool checkTriangleConnect(Face& f1, Face& f2, Edge& sharededge);
	vector<Eigen::Vector3d> box_fitting(gdiam_real  * points, int  num);
	vector<Eigen::Vector3d> computeOrientedBox(std::vector<Eigen::Vector3d>& vertices);

};