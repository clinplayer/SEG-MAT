#include "MAT.h"


//initialize MAT through loading file
MAT::MAT(string matfile, Mesh& mesh)
{
	ifstream fin(matfile);
	string null, type, s1, s2, s3, s4;

	//fin >> null >> null >> null;
	this->points.reserve(5000);
	this->radius.reserve(5000);
	this->faces.reserve(8000);
	double maxr = 0;
	double xsum = 0, ysum = 0, zsum = 0;
	int count = 0;
	while (fin >> type)
	{
		if (type == "v")
		{
			fin >> s1 >> s2 >> s3 >> s4;
			double x = atof(s1.c_str());
			double y = atof(s2.c_str());
			double z = atof(s3.c_str());
			double r = atof(s4.c_str());
			Point p(x, y, z);
			this->pointmap.insert(pair<Point, int>(p, this->points.size()));
			this->points.push_back(p);
			this->radius.push_back(r);
			xsum += x, ysum += y, zsum += z;
			if (r > maxr)maxr = r;
		}
		if (type == "e")
		{
			fin >> s1 >> s2;
			int p1 = atoi(s1.c_str());
			int p2 = atoi(s2.c_str());
			Point pa = this->points[p1], pb = this->points[p2];
			Edge e(pa, pb);
			this->edges.push_back(e);
		}
		if (type == "f")
		{
			fin >> s1 >> s2 >> s3;
			int p1 = atoi(s1.c_str());
			int p2 = atoi(s2.c_str());
			int p3 = atoi(s3.c_str());
			Point pa = this->points[p1], pb = this->points[p2], pc = this->points[p3];
			Face f(pa, pb, pc);
			this->faces.push_back(f);

		}

		if (type == "n")
		{
			fin >> s1 >> s2 >> s3;
			int p1 = atof(s1.c_str());
			int p2 = atof(s2.c_str());
			int p3 = atof(s3.c_str());
			Vector3 n(p1, p2, p3);
			this->normals.push_back(n);

		}

	}

	//find pure edges (edges that are not the sides of faces)
	vector<Edge> pedges;
	for (int i = 0; i < this->edges.size(); i++)
	{
		Edge e = this->edges[i];
		bool pure_edge = true;
		for (int j = 0; j < this->faces.size(); j++)
		{
			Face f = this->faces[j];
			if (checkEdgeOnFace(e, f))
			{
				pure_edge = false;
				break;
			}
		}

		if (pure_edge)
		{
			this->pedges.push_back(e);
			Face f(e[0], e[1], e[0]);
			this->faces.push_back(f);
		}
	}

	this->max_radius = maxr;
	vector<double> ce(3);
	ce[0] = xsum / (double)this->points.size();
	ce[1] = ysum / (double)this->points.size();
	ce[2] = zsum / (double)this->points.size();
	this->centroid = ce;
	this->boundingbox = computeBoundingBox();

}

//build MAT Graph
vector<vector<int>> MAT::buildMATGraph(double t)
{
	int graphsize = faces.size();
	vector<vector<int>> graph(graphsize);
	for (int i = 0; i < graphsize; i++)
	{
		Face f1 = faces[i];
		for (int j = i + 1; j < graphsize; j++)
		{
			Face f2 = faces[j];
			double r1 = compute_face_mean_r(f1);
			if (checkMATTriangleConnect(f1, f2))
			{
				double r2 = compute_face_mean_r(f2);

				if (r1 >= r2)
				{
					double radius_dis = compute_face_radius_difference(f1, f2);

					if (radius_dis < (1.0f + t))
					{
						graph[i].push_back(j);
						graph[j].push_back(i);
					}
				}

				if (r1 < r2)
				{
					double radius_dis = compute_face_radius_difference(f2, f1);

					if (radius_dis < (1.0f + t))
					{
						graph[i].push_back(j);
						graph[j].push_back(i);
					}
				}
			}
		}
	}
	return graph;
}

//SMAT Graph for structural decomposition of SMAT (topological joints)
vector<vector<int>> MAT::buildSMATGraph()
{
	int graphsize = faces.size();
	vector<vector<int>> graph(graphsize);

	getPointShareNum();
	getEdgeShareNum();
	for (int i = 0; i < graphsize; i++)
	{
		Face f1 = faces[i];
		for (int j = i + 1; j < graphsize; j++)
		{
			Face f2 = faces[j];
			if (checkMATtriangleStructureConnect(f1, f2))
			{
				graph[i].push_back(j);
				graph[j].push_back(i);
			}
		}
	}
	return graph;
}

//strctural decomposition of SMAT graph
bool MAT::StructureDecompose(float is_thin)
{
	vector<vector<int>> graph = buildSMATGraph();
	vector<vector<Face>> structure_face_group;
	vector<vector<Face>> non_structure_face_group;
	tag_ratio.push_back(0);

	vector<int> visit(faces.size(), 0);
	int minsize = 1;
	queue<int> q;

	bool linesheets = true;

	for (int i = 0; i < faces.size(); i++)
	{
		if (visit[i] != 0)continue;
		vector<Face> subgroup;
		q.push(i);
		visit[i] = 1;

		vector<int> temp_visit;
		temp_visit.push_back(i);

		while (!q.empty())
		{
			int now = q.front();
			visit[now] = 1;
			subgroup.push_back(faces[now]);
			Face f1 = faces[now];
			for (int j = 0; j < graph[now].size(); j++)
			{

				if (visit[graph[now][j]] == 0)
				{
					Face f2 = faces[graph[now][j]];
					q.push(graph[now][j]);
					visit[graph[now][j]] = 1;
					temp_visit.push_back(graph[now][j]);

				}
			}
			q.pop();
		}


		int subgroup_size_now = subgroup.size();

		//dealing with the group with enough faces
		bool validline = false;
		bool validsheet = false;
		double ratio = 1.0f;
		if (subgroup_size_now >= minsize)
		{
			//this group is a line group
			if (subgroup[0].is_degenerate())
			{

				double sumlength = 0;
				double maxr = 0;
				for (int k = 0; k < subgroup.size(); k++)
				{
					maxr = radius[pointmap[subgroup[k][0]]] > maxr ? radius[pointmap[subgroup[k][0]]] : maxr;
					maxr = radius[pointmap[subgroup[k][1]]] > maxr ? radius[pointmap[subgroup[k][1]]] : maxr;
				}
				double sumr = 0;
				for (int j = 0; j < subgroup.size(); j++)
				{
					Point p1 = subgroup[j][0], p2 = subgroup[j][1];
					Edge segment_query(p1, p2);
					sumlength += pow(CGAL::squared_distance(p1, p2), 0.5);
				}

				ratio = sumlength / maxr;

				if (ratio > 100.0f)
					continue;

				if (ratio > is_thin)
					validline = true;
				else
					validline = false;
			}
			//this group is a sheet group
			else
			{
				if (!checkManifold(subgroup))
				{
					validsheet = false;
				}
				else
				{
					double sumsize = 0;
					double maxr = 0;
					for (int k = 0; k < subgroup.size(); k++)
					{
						maxr = radius[pointmap[subgroup[k][0]]] > maxr ? radius[pointmap[subgroup[k][0]]] : maxr;
						maxr = radius[pointmap[subgroup[k][1]]] > maxr ? radius[pointmap[subgroup[k][1]]] : maxr;
						maxr = radius[pointmap[subgroup[k][2]]] > maxr ? radius[pointmap[subgroup[k][2]]] : maxr;

					}

					for (int j = 0; j < subgroup.size(); j++)
					{
						sumsize += compute_triangle_area(subgroup[j]);
					}

					ratio = pow(sumsize, 0.5f) / maxr;

					if (ratio > is_thin)
						validsheet = true;
					else
						validsheet = false;
				}
			}

			// growing threshold adjustment
			if (validline || validsheet)
			{
				structure_face_group.push_back(subgroup);
				ratio = log(ratio) / log(2.0f);
				tag_ratio.push_back(ratio);
			}
			else
			{
				non_structure_face_group.push_back(subgroup);
			}
		}
		else
		{
			//restore the visited nodes
			for (int j = 0; j < temp_visit.size(); j++)
			{
				visit[temp_visit[j]] = 0;
			}
		}
	}


	//only one line, invalid
	if (structure_face_group.size() == 1 && structure_face_group[0][0].is_degenerate())
		linesheets = false;

	for (int i = 0; i < structure_face_group.size(); i++)
	{

		set<Point> points_set;
		Patch patch;
		patch.faces = structure_face_group[i];
		for (int j = 0; j < structure_face_group[i].size(); j++)
		{
			Face f = structure_face_group[i][j];
			points_set.insert(f[0]);
			points_set.insert(f[1]);
			points_set.insert(f[2]);
		}
		vector<Point> non_repeat_points;
		non_repeat_points.insert(non_repeat_points.end(), points_set.begin(), points_set.end());
		patch.points = non_repeat_points;
		thin_parts.push_back(patch);
	}

	for (int i = 0; i < non_structure_face_group.size(); i++)
	{

		set<Point> points_set;
		Patch patch;
		patch.faces = non_structure_face_group[i];
		for (int j = 0; j < non_structure_face_group[i].size(); j++)
		{
			Face f = non_structure_face_group[i][j];
			points_set.insert(f[0]);
			points_set.insert(f[1]);
			points_set.insert(f[2]);
		}
		vector<Point> non_repeat_points;
		non_repeat_points.insert(non_repeat_points.end(), points_set.begin(), points_set.end());
		patch.points = non_repeat_points;
		normal_parts.push_back(patch);
	}

	return linesheets;


}

//densify the joints on SMAT for better accuracy
void MAT::densify(vector<Patch>& patches, MAT& mat, int interpolation)
{
	float step = 1.0f / double(interpolation);
	for (int i = 0; i < patches.size(); i++)
	{
		for (int j = 0; j < patches[i].faces.size(); j++)
		{
			Face f = patches[i].faces[j];
			Point p1 = f[0];
			Point p2 = f[1];
			Point p3 = f[2];
			Point nullp(0.0f, 0.0f, 0.0f);
			double r1 = mat.radius[mat.pointmap[f[0]]], r2 = mat.radius[mat.pointmap[f[1]]], r3 = mat.radius[mat.pointmap[f[2]]];

			if (f.is_degenerate())
			{
				for (float ii = 0; ii <= 1.0f; ii += step)
				{
					float a = ii;
					float b = 1.0f - a;

					float px = a*p1.x() + b*p2.x(),
						py = a*p1.y() + b*p2.y(),
						pz = a*p1.z() + b*p2.z(),
						pr = a*r1 + b*r2;
					Point p(px, py, pz);

					mat.pointmap[p] = mat.points.size();
					mat.points.push_back(p);
					mat.radius.push_back(pr);

					patches[i].points.push_back(p);
				}
			}
			else for (float ii = 0; ii < 1.0f; ii += step)
				for (float jj = 0; jj < 1.0f; jj += step)
				{
					float a = ii;
					float b = jj;
					if (a + b >= 1.0f)continue;
					float c = 1 - a - b;
					double px = a*p1.x() + b*p2.x() + c*p3.x(),
						py = a*p1.y() + b*p2.y() + c*p3.y(),
						pz = a*p1.z() + b*p2.z() + c*p3.z(),
						pr = a*r1 + b*r2 + c*r3;
					Point p(px, py, pz);

					mat.pointmap[p] = mat.points.size();
					mat.points.push_back(p);
					mat.radius.push_back(pr);

					patches[i].points.push_back(p);
				}
		}
	}
}

//Transfer segmentation results of SMAT joints to MAT graph
vector<int> MAT::transfer_SMAT_MAT(vector<Patch>& thin_parts, vector<Patch>& normal_parts)
{

	int graphsize = faces.size();
	vector<int> tags(graphsize, 0);
	int snum = thin_parts.size();

	for (int i = 0; i < graphsize; i++)
	{
		Point cp1 = CGAL::centroid(faces[i]);
		double mindis = 9999999.9f;
		int minindex = 0;

		for (int k = 0; k < thin_parts.size(); k++)
		{
			Patch pa = thin_parts[k];
			for (int j = 0; j < pa.points.size(); j++)
			{
				Point nowp = pa.points[j];
				double dis = pow(CGAL::squared_distance(cp1, nowp), 0.5);
				if (dis < mindis)
				{
					mindis = dis;
					minindex = k;
				}
			}
		}

		for (int k = 0; k < normal_parts.size(); k++)
		{
			Patch pa = normal_parts[k];
			for (int j = 0; j < pa.points.size(); j++)
			{
				Point nowp = pa.points[j];
				double dis = pow(CGAL::squared_distance(cp1, nowp), 0.5);

				if (dis < mindis)
				{
					mindis = dis;
					minindex = snum + k;
				}
			}
		}


		if (minindex < snum)
			tags[i] = minindex + 1;
		else
			tags[i] = 0;
	}


	return tags;

}

//KNN for MAT Graph of point clouds
vector<vector<int>> MAT::buildMATGraph_pc(int N)
{
	std::list<Point> pc;
	for (int i = 0; i < points.size(); i++)
		pc.push_back(Point(points[i][0], points[i][1], points[i][2]));

	NNTree tree(pc.begin(), pc.end());

	int graphsize = faces.size();
	vector<vector<int>> graph(graphsize);
	for (int i = 0; i < graphsize; i++)
	{
		Point query = points[i];
		Neighbor_search search(tree, query, N);
		for (Neighbor_search::iterator it = search.begin(); it != search.end(); ++it) {
			int j = pointmap[it->first];
			graph[i].push_back(j);
			graph[j].push_back(i);
		}
	}
	return graph;
}

//for face sorting
struct face_node
{
	Face f;
	int index;
	double r;
};
bool cmpf(face_node f1, face_node f2)
{
	return f1.r > f2.r;
};

//Region growing process
void MAT::RegionGrowing(vector<vector<int>>& graph, vector<int>& tags, float growing_threshold, float min_region, bool is_pointcloud)
{

	int min_matp_num = min_region * points.size();
	int graphsize = faces.size();

	vector<int> visit(graphsize, 0);
	vector<vector<Face>> coarse_face_group;
	vector<vector<Face>> dense_face_group;
	vector<vector<Face>> tiny_face_group;
	vector<int> seed_nodes;

	vector<face_node> sorted_face;
	double max_r = 0;
	int max_face_index = 0;
	for (int i = 0; i < graphsize; i++)
	{
		face_node fn;
		fn.f = faces[i];
		fn.index = i;
		fn.r = compute_face_mean_r(faces[i]);
		sorted_face.push_back(fn);
	}

	sort(sorted_face.begin(), sorted_face.end(), cmpf);

	//region growing by finding the largest face
	int regioncount = 0;
	for (int k = 0; k < sorted_face.size(); k++)
	{

		if (visit[sorted_face[k].index] == 0)
			max_face_index = sorted_face[k].index;
		else
			continue;


		queue<int> q;
		vector<Face> subgroup;
		vector<Face> dense_subgroup;
		int seedNode = sorted_face[k].index;
		q.push(max_face_index);
		visit[max_face_index] = 1;


		vector<int> temp_visit;
		temp_visit.push_back(max_face_index);

		while (!q.empty())
		{
			int now = q.front();
			subgroup.push_back(faces[now]);
			Face f1 = faces[now];
			for (int j = 0; j < graph[now].size(); j++)
			{

				if (visit[graph[now][j]] == 0)
				{
					Face f2 = faces[graph[now][j]];
					if (!is_pointcloud) {
						if (checkFaceGrowing(tags[now], tags[graph[now][j]], now, graph[now][j], growing_threshold))
						{
							q.push(graph[now][j]);
							visit[graph[now][j]] = 1;
							temp_visit.push_back(graph[now][j]);
						}
					}
					else
					{
						if (checkPointCloudGrowing(now, graph[now][j], growing_threshold))
						{
							q.push(graph[now][j]);
							visit[graph[now][j]] = 1;
							temp_visit.push_back(graph[now][j]);
						}
					}

				}
			}
			q.pop();
		}

		int subgroup_size_now = subgroup.size();
		//dealing with the group with enough faces
		if (subgroup_size_now >= min_matp_num)
		{

			regioncount++;
			//cout << "Region:" << regioncount<<" Size:"<<subgroup_size_now << endl;
			seed_nodes.push_back(seedNode);
			dense_subgroup.insert(dense_subgroup.end(), subgroup.begin(), subgroup.end());
			double group_mean_r = compute_group_mean_r(subgroup);

			//Swallowing: find the MAT faces contained in this grown area
			for (int i = 0; i < graphsize; i++)
			{
				if (visit[i] == 1)continue;
				//check if face[i] contained in subgroup
				Face f = faces[i];
				if (checkFacePointContained(f, subgroup))
				{
					visit[i] = 1;
					dense_subgroup.push_back(f);
				}
			}
			coarse_face_group.push_back(subgroup);
			dense_face_group.push_back(dense_subgroup);


		}
		else
		{
			//restore the visited nodes
			for (int i = 0; i < temp_visit.size(); i++)
			{
				visit[temp_visit[i]] = 0;
			}
		}
	}


	//find unreached faces as tinny patches
	queue<int> unreach_q;
	for (int i = 0; i < graphsize; i++)
	{
		if (visit[i] == 0)
		{
			vector<Face> tiny_subgroup;
			unreach_q.push(i);
			while (!unreach_q.empty())
			{
				int now = unreach_q.front();
				tiny_subgroup.push_back(faces[now]);
				Face f1 = faces[now];
				for (int j = 0; j < graph[now].size(); j++)
				{

					if (visit[graph[now][j]] == 0)
					{
						Face f2 = faces[graph[now][j]];
						unreach_q.push(graph[now][j]);
						visit[graph[now][j]] = 1;
					}
				}
				unreach_q.pop();
			}
			tiny_face_group.push_back(tiny_subgroup);
		}
	}


	//generate coarse patches from group
	for (int i = 0; i < coarse_face_group.size(); i++)
	{
		set<Point> points_set;
		Patch patch;
		//patch.seednode = seed_nodes[i];
		patch.faces = coarse_face_group[i];
		for (int j = 0; j < coarse_face_group[i].size(); j++)
		{
			Face f = coarse_face_group[i][j];
			points_set.insert(f[0]);
			points_set.insert(f[1]);
			points_set.insert(f[2]);
		}
		vector<Point> non_repeat_points;
		non_repeat_points.insert(non_repeat_points.end(), points_set.begin(), points_set.end());
		patch.points = non_repeat_points;

		coarse_patch.push_back(patch);
	}

	//generate dense patches from group
	for (int i = 0; i < dense_face_group.size(); i++)
	{
		set<Point> points_set;
		Patch patch;
		patch.faces = dense_face_group[i];
		for (int j = 0; j < dense_face_group[i].size(); j++)
		{
			Face f = dense_face_group[i][j];
			points_set.insert(f[0]);
			points_set.insert(f[1]);
			points_set.insert(f[2]);
		}
		vector<Point> non_repeat_points;
		non_repeat_points.insert(non_repeat_points.end(), points_set.begin(), points_set.end());
		patch.points = non_repeat_points;

		dense_patch.push_back(patch);
	}

	//generate unreached tiny patches
	for (int i = 0; i < tiny_face_group.size(); i++)
	{
		set<Point> points_set;
		Patch patch;
		patch.faces = tiny_face_group[i];
		for (int j = 0; j < tiny_face_group[i].size(); j++)
		{
			Face f = tiny_face_group[i][j];
			points_set.insert(f[0]);
			points_set.insert(f[1]);
			points_set.insert(f[2]);
		}
		vector<Point> non_repeat_points;
		non_repeat_points.insert(non_repeat_points.end(), points_set.begin(), points_set.end());
		patch.points = non_repeat_points;

		tiny_patch.push_back(patch);
		/*dense_patch.push_back(patch);
		coarse_patch.push_back(patch);*/
	}

	final_patch = coarse_patch;
}

//EMD feature distance
float dist(feature_t *F1, feature_t *F2) {
	return abs(*F1 - *F2);
}
//EMD graph for merging
vector<vector<float>> MAT::buildEmdGraph(vector<vector<int>>& patchgraph)
{
	setPatchEMD(20, dense_patch);
	int patches_num = dense_patch.size();

	vector<vector<float>> emd_values(200, vector<float>(200, 0.0f));
	float max_emd_value = 0.0f;

	for (int i = 0; i < patches_num; i++)
	{
		Patch pa1 = dense_patch[i];
		signature_t s1 = getPatchSignature(pa1);

		for (int j = 1; j < patches_num; j++)
		{
			Patch pa2 = dense_patch[j];
			signature_t s2 = getPatchSignature(pa2);
			float emdvalue = emd(&s1, &s2, dist, 0, 0);
			emd_values[i][j] = emdvalue;
		}
	}
	return emd_values;
};

//Visiblity for merging: build a graph with edges indicating if the visibility of two patches exceeds a vis-ratio 
vector<vector<int>> MAT::buildVisGraph(vector<Patch> patches, double vis)
{
	double ratio = vis;
	//intial graph
	vector<vector<int>> patchgraph(patches.size());
	for (int i = 0; i < patches.size(); i++)
	{
		Patch pa1 = patches[i];
		for (int j = i + 1; j < patches.size(); j++)
		{
			Patch pa2 = patches[j];

			if (checkPatchConnect(pa1, pa2, 1))
			{
				patchgraph[i].push_back(j);
				patchgraph[j].push_back(i);
			}
		}
	}

	vector<vector<int>> finalpatchgraph(patches.size());
	for (int i = 0; i < patches.size(); i++)
	{

		//get all neighbors of i
		set<int> set1, set2, set3;

		set1.insert(patchgraph[i].begin(), patchgraph[i].end());

		for (int j = 0; j < patchgraph[i].size(); j++)
		{
			int k = patchgraph[i][j];

			if (!checkPatchVisibility(patches[i], patches[k], ratio))
			{
				continue;
			}
			finalpatchgraph[i].push_back(k);
		}
	}
	return finalpatchgraph;

};

//region merging process
void MAT::MergeTinyPatches() {

	setPatchEMD(20, tiny_patch);
	setPatchEMD(20, dense_patch);
	for (int i = 0; i < tiny_patch.size(); i++)
	{
		Patch tp = tiny_patch[i];
		Point tp_c = tp.computeCentroid();

		double max_point_dis = 0.0f, min_point_dis = 9999999.9f;
		double max_emd_dis = 0.0f, min_emd_dis = 9999999.9f;
		//int min_index = -1;

		vector<double> pointdis_list(dense_patch.size()), emddis_list(dense_patch.size());
		for (int j = 0; j < dense_patch.size(); j++)
		{

			Patch dp = dense_patch[j];
			if (!checkPatchConnect(tp, dp, 0))continue;

			double pointdis = compute_point_patch_distance(tp_c, dp);

			pointdis_list[j] = pointdis;
			if (pointdis > max_point_dis)max_point_dis = pointdis;
			if (pointdis < min_point_dis)
			{
				min_point_dis = pointdis;
			}


			signature_t pt1 = getPatchSignature(tp);
			signature_t pt2 = getPatchSignature(dp);
			float emddis = emd(&pt1, &pt2, dist, 0, 0);

			emddis_list[j] = emddis;

			if (emddis > max_emd_dis)max_emd_dis = emddis;
			if (emddis < min_emd_dis)
			{
				min_emd_dis = emddis;
			}
		}

		//just have one face,  no need to do such operation
		if (max_emd_dis == min_emd_dis)continue;
		int min_index = -1;

		double min_totaldis = 99999.9f;
		for (int j = 0; j < dense_patch.size(); j++)
		{
			Patch dp = dense_patch[j];
			if (!checkPatchConnect(tp, dp, 0))continue;

			double totaldis = 0.5f* (emddis_list[j] - min_emd_dis) / (max_emd_dis - min_emd_dis) + 0.5f*(pointdis_list[j] - min_point_dis) / (max_point_dis - min_point_dis);

			if (totaldis < min_totaldis)
			{
				min_totaldis = totaldis;
				min_index = j;
			}
		}

		//this tiny patch has no neighborhood
		if (min_index == -1)
		{
			//then find the closest dense part to merge
			double min_dis = 999999.999f;
			for (int j = 0; j < dense_patch.size(); j++)
			{
				Patch dp = dense_patch[j];
				double nowdis = compute_patch_closest_euclidean_distance(tp, dp);
				if (nowdis < min_dis)
				{
					min_dis = nowdis;
					min_index = j;
				}
			}
		}
		if (min_index != -1)
		{
			dense_patch[min_index].points.insert(dense_patch[min_index].points.end(), tp.points.begin(), tp.points.end());
			dense_patch[min_index].faces.insert(dense_patch[min_index].faces.end(), tp.faces.begin(), tp.faces.end());
		}
	}
}
void MAT::MergeIterations(bool use_vis)
{
	//merging steps
	double emd1 = 0.1;
	double emd2 = 0.2;
	double emd3 = 0.3;
	double vis1 = 0.8;
	double vis2 = 0.9;
	double vis3 = 1.0;

	vector<vector<int>> patchgraph;

	if (use_vis)
	{
		patchgraph = buildVisGraph(dense_patch, vis1);
	}
	else
	{
		patchgraph = vector<vector<int>>(dense_patch.size());
		for (int i = 0; i < dense_patch.size(); i++)
		{
			Patch pa1 = dense_patch[i];
			for (int j = i + 1; j < dense_patch.size(); j++)
			{
				Patch pa2 = dense_patch[j];

				if (checkPatchConnect(pa1, pa2, 0))
				{
					patchgraph[i].push_back(j);
					patchgraph[j].push_back(i);
				}
			}
		}
	}


	vector<vector<float>> emdvalues = buildEmdGraph(patchgraph);
	float maxemd = getMaxEmd(emdvalues);
	MergePatches(patchgraph, emdvalues, maxemd, emd1);

	if (use_vis)
	{
		patchgraph = buildVisGraph(dense_patch, vis2);
	}
	else
	{
		patchgraph = vector<vector<int>>(dense_patch.size());
		for (int i = 0; i < dense_patch.size(); i++)
		{
			Patch pa1 = dense_patch[i];
			for (int j = i + 1; j < dense_patch.size(); j++)
			{
				Patch pa2 = dense_patch[j];

				if (checkPatchConnect(pa1, pa2, 0))
				{
					patchgraph[i].push_back(j);
					patchgraph[j].push_back(i);
				}
			}
		}
	}

	emdvalues = buildEmdGraph(patchgraph);
	MergePatches(patchgraph, emdvalues, maxemd, emd2);

	if (use_vis)
	{
		patchgraph = buildVisGraph(dense_patch, vis3);
	}
	else
	{
		patchgraph = vector<vector<int>>(dense_patch.size());
		for (int i = 0; i < dense_patch.size(); i++)
		{
			Patch pa1 = dense_patch[i];
			for (int j = i + 1; j < dense_patch.size(); j++)
			{
				Patch pa2 = dense_patch[j];

				if (checkPatchConnect(pa1, pa2, 0))
				{
					patchgraph[i].push_back(j);
					patchgraph[j].push_back(i);
				}
			}
		}
	}
	emdvalues = buildEmdGraph(patchgraph);
	MergePatches(patchgraph, emdvalues, maxemd, emd3);

}
void MAT::MergePatches(vector<vector<int>>& patchgraph, vector<vector<float>>& emd_values, float max_emd, float merge_para)
{
	int patches_num = dense_patch.size();
	for (int i = 0; i < patches_num; i++)
	{
		vector<int> kept_nodes;
		Patch pa1 = dense_patch[i];

		for (int j = 0; j < patchgraph[i].size(); j++)
		{
			int pindex = patchgraph[i][j];
			Patch pa2 = dense_patch[pindex];

			//merge the part
			if (emd_values[i][pindex] < merge_para * max_emd)
				kept_nodes.push_back(pindex);

		}
		//update the merged nodes
		patchgraph[i] = kept_nodes;
	}

	//Merge the patches by connected components computation 
	int graphsize = dense_patch.size();
	vector<int> visit(graphsize, 0);
	queue<int> q;

	vector<Patch> temp_dense_patch, temp_coarse_patch;
	for (int i = 0; i < graphsize; i++)
	{
		if (visit[i] == 1)
			continue;

		vector<Patch> coarse_subgroup;
		vector<Patch> dense_subgroup;
		q.push(i);
		visit[i] = 1;

		while (!q.empty())
		{
			int now = q.front();
			//get final patches based on coarse patch or dense patch
			dense_subgroup.push_back(dense_patch[now]);
			coarse_subgroup.push_back(coarse_patch[now]);
			for (int j = 0; j < patchgraph[now].size(); j++)
			{

				if (visit[patchgraph[now][j]] == 0)
				{
					q.push(patchgraph[now][j]);
					visit[patchgraph[now][j]] = 1;
				}
			}
			q.pop();
		}

		Patch newPatch_dense;
		Patch newPatch_coarse;
		for (int j = 0; j < dense_subgroup.size(); j++)
		{
			newPatch_dense.points.insert(newPatch_dense.points.end(), dense_subgroup[j].points.begin(), dense_subgroup[j].points.end());
			newPatch_dense.faces.insert(newPatch_dense.faces.end(), dense_subgroup[j].faces.begin(), dense_subgroup[j].faces.end());
		}
		for (int j = 0; j < coarse_subgroup.size(); j++)
		{
			newPatch_coarse.points.insert(newPatch_coarse.points.end(), coarse_subgroup[j].points.begin(), coarse_subgroup[j].points.end());
			newPatch_coarse.faces.insert(newPatch_coarse.faces.end(), coarse_subgroup[j].faces.begin(), coarse_subgroup[j].faces.end());
		}

		temp_dense_patch.push_back(newPatch_dense);
		temp_coarse_patch.push_back(newPatch_coarse);

	}
	//get final segmentation
	dense_patch = temp_dense_patch;
	coarse_patch = temp_coarse_patch;
	final_patch = coarse_patch;
}

