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
	double compute_face_angle(Face& f1, Face& f2)
	{
		if (f1.is_degenerate() || f2.is_degenerate())return 0;
		Vector3 n1;
		Vector3 n2;

		n1 = CGAL::unit_normal(f1[0], f1[1], f1[2]);
		n2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);
		double cos_normal = n1*n2;
		return cos_normal;
	}
	double compute_bend_angle_degree(Face& f1, Face& f2)
	{
		if (f1.is_degenerate() || f2.is_degenerate())return 0;
		Vector3 n1;
		Vector3 n2;

		n1 = CGAL::unit_normal(f1[0], f1[1], f1[2]);
		n2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);
		double cos_normal = n1*n2;
		return acos(cos_normal) / PI * 180;
	}
	bool check_concave(Face& f1, Face& f2, Edge& sharededge)
	{
		if (f1.is_degenerate() || f2.is_degenerate())return true;

		Point c1 = CGAL::centroid(f1);
		Point c2 = CGAL::centroid(f2);
		Point ce((sharededge[0][0] + sharededge[1][0])*0.5f, (sharededge[0][1] + sharededge[1][1])*0.5f, (sharededge[0][2] + sharededge[1][2])*0.5f);
		Point cn((c1[0] + c2[0])*0.5f, (c1[1] + c2[1])*0.5f, (c1[2] + c2[2])*0.5f);
		Vector3 v = cn - ce;
		Vector3 nf = CGAL::unit_normal(f1[0], f1[1], f1[2]);
		if (nf*v <= 0.000001f)
			return false;
		else
			return true;

	}
	bool checkTriangleConnect(Face& f1, Face& f2, Edge& sharededge)
	{
		//check if triangles share an edge
		if (f1.is_degenerate() || f2.is_degenerate())
			return false;

		vector<Point> p;
		if (f1[0] == f2[0] || f1[0] == f2[1] || f1[0] == f2[2])
			p.push_back(f1[0]);

		if (f1[1] == f2[0] || f1[1] == f2[1] || f1[1] == f2[2])
			p.push_back(f1[1]);

		if (f1[2] == f2[0] || f1[2] == f2[1] || f1[2] == f2[2])
			p.push_back(f1[2]);

		if (p.size() == 2)
		{
			sharededge = Edge(p[0], p[1]);
			return true;
		}
		else
			return false;

	}
	vector<Eigen::Vector3d> box_fitting(gdiam_real  * points, int  num)
	{
		GPointPair   pair;
		//printf("Computing the diameter for %d points selected "
		//	"uniformly from the unit cube\n", num);

		pair = gdiam_approx_diam_pair((gdiam_real *)points, num, 0.0);
		//printf("Diameter distance: %g\n", pair.distance);
		//printf("Points realizing the diameter\n"
		//	"\t(%g, %g, %g) - (%g, %g, %g)\n",
		//	pair.p[0], pair.p[1], pair.p[2],
		//	pair.q[0], pair.q[1], pair.q[2]);


		gdiam_point  * pnt_arr;
		gdiam_bbox   bb;

		pnt_arr = gdiam_convert((gdiam_real *)points, num);

		//printf("Computing a tight-fitting bounding box of the point-set\n");
		bb = gdiam_approx_mvbb_grid_sample(pnt_arr, num, 5, 400);

		vector<vector<double>> bb_points = bb.my_dump();
		vector<Eigen::Vector3d> bb_output;
		for (int i = 0; i < 8; i++)
			bb_output.push_back(Eigen::Vector3d(bb_points[i][0], bb_points[i][1], bb_points[i][2]));

		return bb_output;

	}
	vector<Eigen::Vector3d> computeOrientedBox(std::vector<Eigen::Vector3d>& vertices)
	{
		gdiam_real  * points;
		int  num = vertices.size();

		points = (gdiam_point)malloc(sizeof(gdiam_point_t) * num);
		assert(points != NULL);

		// Pick randomly points from the unit cube */
		for (int ind = 0; ind < num; ind++) {
			points[ind * 3 + 0] = vertices[ind][0];
			points[ind * 3 + 1] = vertices[ind][1];
			points[ind * 3 + 2] = vertices[ind][2];
		}

		return box_fitting(points, num);
	}

};