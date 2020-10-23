#pragma once
#include "common_include.h"
#include "Patch.h"
#include "graphcut\GCoptimization.h"

class MAT
{
public:

	//build 
	MAT(){};
	MAT(string matfile, Mesh& mesh);
	~MAT() {};

	//data
	vector<Face> faces;
	vector<Point> points;
	vector<Edge> edges;
	vector<Edge> pedges;
	vector<Vector3> normals;
	vector<double> radius;
	map<Point, int> pointmap;
	map<int, Point> face2mat;
	vector<double> tag_ratio;

	//mesh
	Tree* meshTree;
	Mesh mesh;

	//patch
	vector<Patch> coarse_patch;
	vector<Patch> dense_patch;
	vector<Patch> tiny_patch;
	vector<Patch> final_patch;

	//for smat structure
	vector<Patch> thin_parts;
	vector<Patch> normal_parts;

	//attributes
	double max_radius;
	double boundingbox;
	vector<double> centroid;

	//build connection relationships
	vector<vector<int>> buildMATGraph(double t);
	vector<vector<int>> buildSMATGraph();
	vector<vector<int>> buildVisGraph(vector<Patch> patches, double vis);
	vector<vector<float>> buildEmdGraph(vector<vector<int>>& patchgraph);
	vector<vector<int>> buildMATGraph_pc(int N);
	vector<int> transfer_SMAT_MAT(vector<Patch>& thin_parts, vector<Patch>& normal_parts);

	//junction rule checking and processing
	bool StructureDecompose(float is_thin = 8.0);
	void densifyJunction(vector<Patch>& patches, MAT& mat, int num);

	//region growing
	void RegionGrowing(vector<vector<int>>& graph, vector<int>& tags, float growing_threshold, float min_region, bool is_pointcloud = false);

	//region merging
	void MergeTinyPatches();
	void MergePatches(vector<vector<int>>& patchgraph,  vector<vector<float>>& emd_values, float max_emd, float merge_para);
	void MergeIterations(bool use_vis);


private:
	//for checking the non-manifoldness
	map<Point, int> point_share;
	vector<vector<int>> edge_share;

	//computation during growing
	bool checkFaceGrowing(int tag1, int tag2, int i, int j, float growing_threshold);
	bool checkPointCloudGrowing(int i, int j, float growing_threshold);

	//earth mover distance compuatation functions
	signature_t getPatchSignature(Patch p1);
	void setPatchEMD(int num, vector<Patch>& patches);
	float getMaxEmd(vector<vector<float>> emd_values);

	//basic geometric checking process
	bool checkPatchConnect(Patch& pa1, Patch& pa2, int mode);
	bool checkMATTriangleConnect(Face& f1, Face& f2);
	bool checkFaceConnect(Face& f1, Face f2);
	bool checkFaceCentroidContained(Face& f, vector<Face>& group);
	bool checkFacePointContained(Face& f, vector<Face>& group);
	bool checkMATtriangleStructureConnect_forPoint2Skel(Face & f1, Face & f2);
	bool checkMATtriangleStructureConnect(Face& f1, Face& f2);
	bool checkManifold(vector<Face>& faces);
	bool checkEdgeOnFace(Edge& e, Face& f);
	bool checkMedialPointAllVisible(Patch & pa1, Patch & pa2);
	bool checkPatchVisibility(Patch& pa1, Patch& pa2, float visratio);
	bool checkFaceConvexwithTwoNormals(Face f1, Face f2, Vector3 n1, Vector3 n2);

	//basic geomertic computing algorithm
	void  getPointShareNum();
	void getEdgeShareNum();
	double computeBoundingBox();
	double compute_face_mean_r(Face f);
	double compute_triangle_area(Face& f);
	vector<Vector3> compute_face_consistent_normal(Face& f1, Face& f2);
	double compute_group_mean_r(vector<Face> group);
	double compute_point_patch_distance(Point& p, Patch pa);
	double compute_patch_closest_euclidean_distance(Patch& pa1, Patch& pa2);
	double compute_face_angle(Face& f1, Face& f2);
	double compute_face_radius_difference(Face& f1, Face& f2);
	double compute_face_angle_degree(Face& f1, Face& f2);
	double compute_face_radius_gradient(Face& f1, Face& f2);
	vector<Point> computeJointPoint(Face& f1, Face& f2);
	Vector3 compute_face_normal(Face& f1);
	int getLargestFaceIndexOfPatch(Patch& pa1);
	double compute_face_slab_angle(Face& f1, Face& f2);
	bool compute_distance_to_line(Vector3&  p, Vector3& v0, Vector3& v1, double& dist, Vector3& fp)
	{
		//only used for compute slab
		Vector3 v0v1(v1 - v0), pv0(v0 - p), pv1(v1 - p);
		double area = fabs(pow((CGAL::cross_product(v0v1, pv0)).squared_length(), 0.5));
		if (pow(v0v1.squared_length(), 0.5)> 1e-12)
		{
			dist = area / pow(v0v1.squared_length(), 0.5);
			double t = (pv0*pv0 - pv0*pv1) / (pv0*pv0 + pv1*pv1 - 2 * pv0*pv1);
			fp = (1 - t)*v0 + t*v1;
			return true;
		}
		else
			return false;
	}
	bool compute_slab(Face f, Vector3 normal, vector<double> r, vector<Face>& slab, vector<Vector3>& slabnormals);


};