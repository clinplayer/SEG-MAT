#include "Decomposer.h"

Mesh Decomposer::readMesh(string path) {
	Mesh mesh;
	std::ifstream input(path);
	input >> mesh;
	return mesh;
}

void Decomposer::decompose3Dshape(MAT& mat, MAT& smat, Mesh& mesh, float growing_threshold, float min_region)
{
	//intialize mesh tree
	mat.mesh = mesh;
	Tree tree(faces(mesh).first, faces(mesh).second, mesh);
	mat.meshTree = &tree;

	//strucural decomposition
	cout << "structural decomposition ..." << endl;
	smat.StructureDecompose();
	mat.tag_ratio = smat.tag_ratio;
	smat.densify(smat.thin_parts, smat);
	smat.densify(smat.normal_parts, smat);
	vector<int> labels = mat.transfer_SMAT_MAT(smat.thin_parts, smat.normal_parts);

	cout << "geometrical decomposition (region growing) ..." << endl;
	//geometrical decomposition
	vector<vector<int>> graph = mat.buildMATGraph();
	mat.RegionGrowing(graph, labels, growing_threshold, min_region);

	cout << "merging  ..." << endl;
	mat.MergeTinyPatches();
	mat.MergeIterations();
}

void Decomposer::saveColoredMesh(MAT& mat, Mesh& mesh, vector<vector<float>> colors, string outputpath)
{
	//save to file
	std::ofstream out(outputpath, std::ios::out);
	out << "OFF" << endl;
	out << mesh.vertices().size() << " " << mesh.faces().size() << " 0" << endl;

	BOOST_FOREACH(Vertexi v_index, mesh.vertices()) {
		Point meshp = mesh.point(v_index);
		{
			out << meshp.x() << " " << meshp.y() << " " << meshp.z() << endl;
		}
	}

	BOOST_FOREACH(Facei f_index, mesh.faces()) {
		out << "3";
		BOOST_FOREACH(Vertexi v_index, vertices_around_face(mesh.halfedge(f_index), mesh)) {
			Point meshp = mesh.point(v_index);
			out << " " << int(v_index);
		}
		int label = final_facelabel[(int)f_index];
		if (label == -1)
			out << " " << 0 << " " << 0 << " " << 0 << " " << 0;
		else
		{
			label = label%colors.size();
			out << " " << int(colors[label % 18][0]) << " " << int(colors[label % 18][1]) << " " << int(colors[label % 18][2]) << " " << int(0);
		}
		out << endl;
	}
	out.close();
};

void Decomposer::saveSegResult(MAT & mat, Mesh & mesh, string outputpath)
{
	std::ofstream out(outputpath, std::ios::out);

	BOOST_FOREACH(Facei f_index, mesh.faces()) {

		int minindex = final_facelabel[(int)f_index];
		out << minindex << endl;
	}
	out.close();
}

void Decomposer::primitiveAbstraction(MAT& mat, Mesh& mesh, vector<vector<float>> colors, string outputpath)
{
	vector<Eigen::Vector3d> prim_v;
	vector<Eigen::Vector4i> prim_f;

	int color_num = colors.size();
	for (int l = 0; l < color_num; l++)
	{
		vector<Eigen::Vector3d> vertices;
		//find all the faces with label l
		BOOST_FOREACH(Facei f_index, mesh.faces()) {
			int label = final_facelabel[(int)f_index];
			if (label == l) {
				BOOST_FOREACH(Vertexi v_index, vertices_around_face(mesh.halfedge(f_index), mesh)) {
					Point meshp = mesh.point(v_index);
					Eigen::Vector3d v(meshp.x(), meshp.y(), meshp.z());
					float t = 0.0005f;
					Eigen::Vector3d v_add(rand() / double(RAND_MAX), rand() / double(RAND_MAX), rand() / double(RAND_MAX));
					v_add = v_add*t;
					vertices.push_back(v + v_add);
				}
			}
		}
		if (vertices.size() == 0)
			break;
		Eigen::Vector3d center(0, 0, 0);
		for (int i = 0; i < vertices.size(); i++)
		{
			center += vertices[i];
		}
		center = center / float(vertices.size());
		for (int i = 0; i < vertices.size(); i++)
		{
			vertices[i] = center + 0.85*(vertices[i] - center);
		}

		Eigen::Vector4i base(prim_v.size(), prim_v.size(), prim_v.size(), prim_v.size());

		vector<Eigen::Vector3d> bbox_points = computeOrientedBox(vertices);
		for (int i = 0; i < 8; i++) {
			prim_v.push_back(bbox_points[i]);
		}

		Eigen::Vector4i f1(0, 1, 2, 3);
		Eigen::Vector4i f2(4, 5, 6, 7);
		Eigen::Vector4i f3(0, 4, 5, 1);
		Eigen::Vector4i f4(3, 7, 6, 2);
		Eigen::Vector4i f5(3, 7, 4, 0);
		Eigen::Vector4i f6(2, 6, 5, 1);

		prim_f.push_back(f1 + base);
		prim_f.push_back(f2 + base);
		prim_f.push_back(f3 + base);
		prim_f.push_back(f4 + base);
		prim_f.push_back(f5 + base);
		prim_f.push_back(f6 + base);

	}


	std::ofstream out(outputpath, std::ios::out);
	out << "OFF" << endl;
	out << prim_v.size() << " " << prim_f.size() << " 0" << endl;
	for (int i = 0; i < prim_v.size(); i++)
	{
		out << prim_v[i][0] << " " << prim_v[i][1] << " " << prim_v[i][2] << endl;
	}

	for (int i = 0; i < prim_f.size(); i++)
	{
		int group = int(i / 6);
		out << "4 " << prim_f[i][0] << " " << prim_f[i][1] << " " << prim_f[i][2] << " " << prim_f[i][3] << " " << int(colors[group][0]) << " " << int(colors[group][1]) << " " << int(colors[group][2]) << " " << int(colors[group][3]) << endl;

	}



}


vector<vector<int>>* smooth_term;
vector<vector<int>>* graph;

//fm:GC node to faceid  fv:faceid to GC node
map<int, int> fm;
map<int, int> fv;

int smoothFn(int p1, int p2, int l1, int l2)
{
	if (l1 == l2)
		return 0;
	else
	{
		vector<int>::iterator it;
		vector<int> neighbor = (*graph)[p1];
		it = find(neighbor.begin(), neighbor.end(), p2);
		int index = std::distance(begin(neighbor), it);
		if (it != neighbor.end())
			return (*smooth_term)[p1][index];
		else
			return 0;
	}
}

void Decomposer::transfer_MAT_mesh(MAT& mat, Mesh& mesh, float weight)
{
	int face_num = mesh.number_of_faces();
	int label_num = mat.final_patch.size();
	vector<Face> all_faces(face_num);
	final_facelabel.resize(face_num);

	//faces need recompute
	vector<vector<int>> data_term(face_num, vector <int>(label_num));
	int enlarge = 10e4;
	int begin = clock();


	//data term
	BOOST_FOREACH(Facei f_index, mesh.faces()) {
		Point fpoints[3];
		int count = 0;
		BOOST_FOREACH(Vertexi v_index, vertices_around_face(mesh.halfedge(f_index), mesh)) {
			Point meshp = mesh.point(v_index);
			fpoints[count++] = meshp;
		}

		Face f(fpoints[0], fpoints[1], fpoints[2]);
		all_faces[f_index] = f;

		Point cp = CGAL::centroid(f);

		for (int i = 0; i < mat.final_patch.size(); i++)
		{
			double mindis = 9999999.99f;
			for (int j = 0; j < mat.final_patch[i].points.size(); j++)
			{
				Point nowp = mat.final_patch[i].points[j];
				double nowdis = CGAL::squared_distance(cp, nowp);
				nowdis = pow(nowdis, 0.5) - mat.radius[mat.pointmap[nowp]];
				if (nowdis < mindis)
					mindis = nowdis;
			}
			if (mindis < 0)
				mindis = 0;
			data_term[(int)f_index][i] = int(mindis / mat.boundingbox * enlarge);

		}

	}

	if (label_num == 1)
	{
		final_facelabel = vector<int>(face_num, 0);
		return;
	}

	//smooth term
	vector<vector<int>> boundary_term;
	boundary_term.resize(face_num);

	vector<vector<int>> face_graph;
	face_graph.resize(face_num);


	BOOST_FOREACH(Facei i, mesh.faces()) {

		Face f1 = all_faces[i];
		BOOST_FOREACH(Facei j, faces_around_face(mesh.halfedge(i), mesh))
		{
			Face f2 = all_faces[(int)j];

			Edge sharededge;
			bool connect = checkTriangleConnect(all_faces[(int)i], all_faces[(int)j], sharededge);
			bool concave = check_concave(all_faces[(int)i], all_faces[(int)j], sharededge);
			double angle = 180 - compute_bend_angle_degree(all_faces[(int)i], all_faces[(int)j]);

			if (!connect) continue;

			face_graph[(int)i].push_back((int)j);

			if (concave)
			{
				int aterm = int(weight * angle / 180.0f * enlarge);
				aterm = aterm < 0 ? 0 : aterm;

				boundary_term[(int)i].push_back(aterm);
			}
			else
			{
				int aterm = int(weight *(180 + angle) / 180.0f * enlarge);
				aterm = aterm < 0 ? 0 : aterm;

				boundary_term[(int)i].push_back(aterm);
			}
		}
	}

	smooth_term = &boundary_term;
	graph = &face_graph;

	// stores result of optimization
	int *result = new int[face_num];

	// first set up the array for data costs
	int *data = new int[face_num*label_num];
	for (int i = 0; i < face_num; i++)
		for (int l = 0; l < label_num; l++)
		{
			data[i*label_num + l] = data_term[i][l];
		}


	GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(face_num, label_num);
	gc->setDataCost(data);
	gc->setSmoothCost(&smoothFn);

	// now set up a grid neighborhood system
	for (int y = 0; y < face_num; y++)
		for (int i = 0; i < face_graph[y].size(); i++) {
			int x = face_graph[y][i];
			gc->setNeighbors(y, x, 1);
		}

	//printf("\nBefore graph-cut optimization energy is %d", gc->compute_energy());
	gc->expansion(2);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
	//printf("\nAfter graph-cut optimization energy is %d", gc->compute_energy());

	//pre-process the tag
	map<int, int>::iterator it;
	vector<int> tag_label_exist(label_num, 0);
	vector<int> space(label_num, 0);

	for (int i = 0; i < face_num; i++)
	{
		int label = gc->whatLabel(i);
		tag_label_exist[label] = 1;
	}
	//find the label missing number
	for (int i = 0; i < label_num; i++)
	{
		if (tag_label_exist[i] == 0)
			for (int j = i + 1; j < label_num; j++)
				space[j]++;
	}


	for (int i = 0; i < face_num; i++)
	{
		int label = gc->whatLabel(i);
		label = label - space[label];
		final_facelabel[i] = label;

	}
	delete gc;
	delete[] result;
	delete[] data;

}

double Decomposer::compute_face_angle(Face& f1, Face& f2)
{
	if (f1.is_degenerate() || f2.is_degenerate())
		return 0;

	Vector3 n1;
	Vector3 n2;

	n1 = CGAL::unit_normal(f1[0], f1[1], f1[2]);
	n2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);
	double cos_normal = n1*n2;
	return cos_normal;
}

double Decomposer::compute_bend_angle_degree(Face& f1, Face& f2)
{
	if (f1.is_degenerate() || f2.is_degenerate())return 0;
	Vector3 n1;
	Vector3 n2;

	n1 = CGAL::unit_normal(f1[0], f1[1], f1[2]);
	n2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);
	double cos_normal = n1*n2;
	return acos(cos_normal) / PI * 180;
}

bool Decomposer::check_concave(Face& f1, Face& f2, Edge& sharededge)
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

bool Decomposer::checkTriangleConnect(Face& f1, Face& f2, Edge& sharededge)
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

vector<Eigen::Vector3d> Decomposer::box_fitting(gdiam_real  * points, int  num)
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

vector<Eigen::Vector3d> Decomposer::computeOrientedBox(std::vector<Eigen::Vector3d>& vertices)
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


