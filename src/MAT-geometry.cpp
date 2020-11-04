#include "MAT.h"

//EMD compuatation
signature_t MAT::getPatchSignature(Patch p1)
{
	int pa1_item_num = p1.feature_map.size();
	feature_t* pa1_f = p1.feature_value;
	float* weight1 = p1.feature_weight;
	signature_t s1 = { pa1_item_num, pa1_f, weight1 };
	return s1;
}
void MAT::setPatchEMD(int num, vector<Patch>& patches)
{
	vector<double> sort_r = radius;
	vector<double> marks(num + 1, 0);
	sort(sort_r.begin(), sort_r.end());
	float r_interval = (sort_r[sort_r.size() - 1] - sort_r[0]) / (float)num;
	marks[0] = *sort_r.begin();
	for (int i = 1; i <= num; i++)
		marks[i] = marks[i - 1] + r_interval;

	for (int i = 0; i < patches.size(); i++)
	{
		for (int j = 0; j < patches[i].points.size(); j++)
		{
			Point p = patches[i].points[j];
			//map to the closest marks
			double min_mark_dis = 99999;
			int min_mark_index = 0;
			for (int k = 0; k < marks.size(); k++)
			{
				double now_dis = abs(marks[k] - radius[pointmap[p]]);
				if (now_dis < min_mark_dis)
				{
					min_mark_dis = now_dis;
					min_mark_index = k;
				}
			}
			patches[i].feature_map[marks[min_mark_index]]++;
		}
	}
	//set feature value
	for (int i = 0; i < patches.size(); i++)
	{
		patches[i].feature_value = new feature_t[patches[i].feature_map.size()];
		map<double, int>::iterator iter;
		int count = 0;
		for (iter = patches[i].feature_map.begin(); iter != patches[i].feature_map.end(); iter++)
			*(patches[i].feature_value + count++) = iter->first;
	}
	for (int i = 0; i < patches.size(); i++)
	{
		patches[i].feature_weight = new float[patches[i].feature_map.size()];
		map<double, int>::iterator iter;
		int count = 0;
		for (iter = patches[i].feature_map.begin(); iter != patches[i].feature_map.end(); iter++)
			*(patches[i].feature_weight + count++) = iter->second / (float)patches[i].points.size();
	}
};
float MAT::getMaxEmd(vector<vector<float>> emd_values)
{
	float max_emd_value = 0;
	for (int i = 0; i < dense_patch.size(); i++)
	{
		for (int j = i + 1; j < dense_patch.size(); j++)
		{
			float emdvalue = emd_values[i][j];

			if (emdvalue > max_emd_value)
			{
				max_emd_value = emdvalue;
			}
		}
	}
	return max_emd_value;
}


//geomertric check algorithm
bool MAT::checkPatchConnect(Patch& pa1, Patch& pa2, int mode)
{
	//check by sphere intersection
	if (mode == 0)
	{
		for (int i = 0; i < pa1.points.size(); i++)
		{
			Point p1 = pa1.points[i];
			for (int j = 0; j < pa2.points.size(); j++)
			{
				Point p2 = pa2.points[j];
				double center_dis = pow(CGAL::squared_distance(p1, p2), 0.5f);
				double radius_dis = radius[pointmap[p1]] + radius[pointmap[p2]];
				if (center_dis < radius_dis)
					return true;

			}
		}
		return false;
	}
	//check by mat mesh
	if (mode == 1)
	{
		for (int i = 0; i < pa1.faces.size(); i++)
		{
			Face f1 = pa1.faces[i];
			for (int j = 0; j < pa2.faces.size(); j++)
			{
				Face f2 = pa2.faces[j];
				if (checkMATTriangleConnect(f1, f2))
				{
					return true;
				}
			}
		}
		return false;
	}
}
bool MAT::checkMATTriangleConnect(Face& f1, Face& f2)
{
	if (f1[0] == f2[0] || f1[0] == f2[1] || f1[0] == f2[2])
		return true;
	if (f1[1] == f2[0] || f1[1] == f2[1] || f1[1] == f2[2])
		return true;
	if (f1[2] == f2[0] || f1[2] == f2[1] || f1[2] == f2[2])
		return true;
	return false;
}
bool MAT::checkFaceCentroidContained(Face& f, vector<Face>& group)
{
	//check if face[ii] contained in this area
	Point centerp = CGAL::centroid(f);
	for (int i = 0; i < group.size(); i++)
	{
		Face check_face = group[i];
		for (int k = 0; k < 3; k++)
		{
			double dis = pow(CGAL::squared_distance(centerp, check_face[k]), 0.5);
			if (dis < radius[pointmap[check_face[k]]])
			{
				return true;
			}

		}
	}
	return false;
}
bool MAT::checkFacePointContained(Face& f, vector<Face>& group)
{
	//check if face[i] contained in this area
	for (int n = 0; n < 3; n++)
	{
		Point fp = f[n];
		for (int i = 0; i < group.size(); i++)
		{
			Face check_face = group[i];
			for (int k = 0; k < 3; k++)
			{
				double dis = pow(CGAL::squared_distance(fp, check_face[k]), 0.5);
				if (dis < radius[pointmap[check_face[k]]])
				{
					return true;
				}
			}
		}
	}
	return false;
}


bool MAT::checkMATtriangleStructureConnect(Face& f1, Face& f2)
{
	Point intersect_point;
	//two lines
	if (f1.is_degenerate() && f2.is_degenerate())
	{
		int count = 0;
		if (f1[0] == f2[0] || f1[0] == f2[1])
		{
			intersect_point = f1[0];
			count++;
		}
		if (f1[1] == f2[0] || f1[1] == f2[1])
		{
			intersect_point = f1[1];
			count++;
		}

		if (count >= 1 && point_share[intersect_point] == 2)
		{
			return true;
		}
		if (count >= 1 && point_share[intersect_point] > 2)
		{
			//will merge two branching lines if they form a flat angle
			double angle = compute_face_angle(f1, f2);
			if (angle > 0.95)
				return true;
			else
				return false;
		}
		return false;
	}
	//two face
	else if (!f1.is_degenerate() && !f2.is_degenerate())
	{
		vector<Point> joints = computeJointPoint(f1, f2);
		if (joints.size() < 2)return false;
		else {
			int a = pointmap[joints[0]], b = pointmap[joints[1]];
			if (edge_share[a][b] > 2)
				return false;
			else
				return true;
		}
	}
	else
		return false;
}
bool MAT::checkManifold(vector<Face>& faces)
{
	for (int i = 0; i < faces.size(); i++)
	{
		Point p1 = faces[i][0], p2 = faces[i][1], p3 = faces[i][2];
		Edge e1(p1, p2), e2(p2, p3), e3(p1, p3);
		int e1count = 0, e2count = 0, e3count = 0;
		for (int j = 0; j < faces.size(); j++)
		{
			if (j == i)continue;
			Point pa = faces[j][0], pb = faces[j][1], pc = faces[j][2];
			Edge ea(pa, pb), eb(pb, pc), ec(pa, pc), ea_(pb, pa), eb_(pc, pb), ec_(pc, pa);
			if (e1 == ea || e1 == eb || e1 == ec || e1 == ea_ || e1 == eb_ || e1 == ec_)
				e1count++;
			if (e2 == ea || e2 == eb || e2 == ec || e2 == ea_ || e2 == eb_ || e2 == ec_)
				e2count++;
			if (e3 == ea || e3 == eb || e3 == ec || e3 == ea_ || e3 == eb_ || e3 == ec_)
				e3count++;
		}
		if (e1count >= 2 || e2count >= 2 || e3count >= 2)
			return false;
	}
	return true;

}
bool MAT::checkEdgeOnFace(Edge& e, Face& f)
{
	Point ep1 = e[0], ep2 = e[1], fp1 = f[0], fp2 = f[1], fp3 = f[2];
	bool p1check = false, p2check = false;
	if (ep1 == fp1 || ep1 == fp2 || ep1 == fp3)
	{
		p1check = true;
	}
	if (ep2 == fp1 || ep2 == fp2 || ep2 == fp3)
	{
		p2check = true;
	}
	if (p1check&&p2check)
		return true;
	else
		return false;
}
bool MAT::checkPatchVisibility(Patch& pa1, Patch& pa2, float visratio)
{
	int count = 0;
	float d_smp = 3.0f; //downsample for effciency
	float total = double(pa1.points.size() / d_smp * pa2.points.size() / d_smp);
	float valid_count = total * visratio;

	for (int i = 0; i < pa1.points.size(); i += d_smp)
	{
		Point p1 = pa1.points[i];

		Vector3 v1(rand() / double(RAND_MAX), rand() / double(RAND_MAX), rand() / double(RAND_MAX));
		v1 = v1 / pow(v1.squared_length(), 0.5f);
		double r_step1 = rand() / double(RAND_MAX);

		//random sampling inside the sphere
		Point np1 = p1 + v1 * (radius[pointmap[p1]] * r_step1);
		for (int j = 0; j < pa2.points.size(); j += d_smp)
		{
			Point p2 = pa2.points[j];

			Vector3 v2(rand() / double(RAND_MAX), rand() / double(RAND_MAX), rand() / double(RAND_MAX));
			v2 = v2 / pow(v2.squared_length(), 0.5f);
			double r_step2 = rand() / double(RAND_MAX);

			Point np2 = p2 + v2 *(radius[pointmap[p2]] * r_step2);
			Edge segment_query(np1, np2);
			if (segment_query.is_degenerate())
			{
				count++;
				continue;
			};
			try
			{
				//check if visible
				if (!meshTree->do_intersect(segment_query))
					count++;
			}
			catch (exception e)
			{
				// cout << "CGAL error when checking visibility" << endl;
			}
			if (count > valid_count)
			{
				return true;
			}
		}
	}

	if (double(count) / double(total) < visratio)
		return false;
	else
		return true;
}
bool MAT::checkFaceConvexwithTwoNormals(Face f1, Face f2, Vector3 n1, Vector3 n2)
{
	Point p1 = CGAL::centroid(f1);
	Point p2 = CGAL::centroid(f2);

	Vector3 checkf1 = p2 - p1;
	checkf1 = checkf1 / pow(checkf1.squared_length(), 0.5);
	if (checkf1*n1 <= 0)
		return true;
	else
		return false;

}
bool MAT::checkFaceGrowing(int tag1, int tag2, int i, int j, float growing_threshold)
{

	double radius_dis = compute_face_radius_difference(faces[i], faces[j]);
	double mat_angle_dis = compute_face_angle_degree(faces[i], faces[j]) / 180.0f;
	double slab_angle_dis = compute_face_slab_angle(faces[i], faces[j]);

	double inner_term = radius_dis + 0.05 * mat_angle_dis;
	double outer_term = slab_angle_dis;


	inner_term *= 1.0f;
	outer_term *= 1.5f;

	double dis = inner_term < outer_term ? inner_term : outer_term;

	double t = growing_threshold;

	//growing threshold adjustment

	//a thin part
	if (tag1 == tag2&&tag1 != 0)
		t = t * tag_ratio[tag1];
	//a bulky part
	else if (tag1 == tag2&&tag1 == 0)
		t = t;
	//a topological joint, stop growing
	else if (tag1 != tag2)
		t = 0;

	if (dis > t)
		return false;
	else
		return true;
}
bool MAT::checkPointCloudGrowing(int i, int j, float growing_threshold)
{

	Vector3 n1 = normals[i];
	Vector3 n2 = normals[j];

	double cos_normal = abs(n1*n2);
	double degree_angle = acos(cos_normal) / PI*180.0f;
	double mat_angle_dis = degree_angle / 180.f;

	double radius_dis = compute_face_radius_difference(faces[i], faces[j]);

	double dis = radius_dis + 0.05 * mat_angle_dis;

	if (dis > growing_threshold)
		return false;
	else
		return true;

}


//basic geometric computation
void MAT::getPointShareNum()
{
	for (int i = 0; i < faces.size(); i++)
	{
		if (faces[i].is_degenerate())
		{
			point_share[faces[i][0]]++;
			point_share[faces[i][1]]++;
		}
		else
		{
			point_share[faces[i][0]]++;
			point_share[faces[i][1]]++;
			point_share[faces[i][2]]++;
		}
	}

}
void MAT::getEdgeShareNum()
{
	edge_share = vector<vector<int>>(points.size(), vector<int>(points.size(), 0));

	for (int i = 0; i < faces.size(); i++)
	{

		if (faces[i].is_degenerate()) continue;

		Point p1 = faces[i][0], p2 = faces[i][1], p3 = faces[i][2];
		int a = pointmap[p1], b = pointmap[p2], c = pointmap[p3];

		edge_share[a][b]++;
		edge_share[b][a]++;
		edge_share[a][c]++;
		edge_share[c][a]++;
		edge_share[b][c]++;
		edge_share[c][b]++;
	}
}
double MAT::computeBoundingBox()
{
	double maxdis = 0;
	//compute bounding box
	double INF = 999999999;
	double maxx = -INF, maxy = -INF, maxz = -INF, minx = INF, miny = INF, minz = INF;
	for (int i = 0; i < points.size(); i++) {
		Point p = points[i];
		if (p[0] > maxx) maxx = p[0];
		if (p[0] < minx) minx = p[0];

		if (p[1] > maxy) maxy = p[1];
		if (p[1] < miny) miny = p[1];

		if (p[2] > maxz) maxz = p[2];
		if (p[2] < minz) minz = p[2];
	}
	double dis = pow(maxx - minx, 2) + pow(maxy - miny, 2) + pow(maxz - minz, 2);
	return pow(dis, 0.5);
}
double MAT::compute_face_mean_r(Face f)
{
	if (f.is_degenerate())
		return (radius[pointmap[f[0]]] + radius[pointmap[f[1]]]) / 2.0f;
	else
		return (radius[pointmap[f[0]]] + radius[pointmap[f[1]]] + radius[pointmap[f[2]]]) / 3.0f;
};
double MAT::compute_group_mean_r(vector<Face> group)
{
	double sum = 0;
	for (int i = 0; i < group.size(); i++)
	{
		double r = compute_face_mean_r(group[i]);
		sum += r;
	}
	return sum / (double)group.size();
}
double MAT::compute_point_patch_distance(Point& p, Patch pa)
{
	double mindis = 9999999;
	for (int i = 0; i < pa.points.size(); i++)
	{
		Point p2 = pa.points[i];
		double dis = pow(CGAL::squared_distance(p, p2), 0.5) - radius[pointmap[p2]];
		if (dis < mindis)
			mindis = dis;
	}
	return mindis;
}
double MAT::compute_patch_closest_euclidean_distance(Patch& pa1, Patch& pa2) {
	double min_dis = 9999999.9f;
	for (int i = 0; i < pa1.points.size(); i++)
	{
		Point p1 = pa1.points[i];
		for (int j = 0; j < pa2.points.size(); j++)
		{
			Point p2 = pa2.points[j];
			double now_dis = CGAL::squared_distance(p1, p2);
			if (now_dis < min_dis)
			{
				min_dis = now_dis;
			}
		}
	}
	return pow(min_dis, 0.5f);
}
double MAT::compute_face_angle(Face& f1, Face& f2)
{
	Vector3 n1;
	Vector3 n2;
	if (f1.is_degenerate())
	{
		Point p1, p2;
		if (f1[0] == f1[1])
			p1 = f1[0], p2 = f1[2];
		else if (f1[0] == f1[2])
			p1 = f1[0], p2 = f1[1];
		else
			p1 = f1[0], p2 = f1[2];

		Vector3 v(p1, p2);
		v = v / pow(v.squared_length(), 0.5);
		n1 = v;
	}
	else
		n1 = CGAL::unit_normal(f1[0], f1[1], f1[2]);

	if (f2.is_degenerate())
	{
		Point p1, p2;
		if (f2[0] == f2[1])
			p1 = f2[0], p2 = f2[2];
		else if (f2[0] == f2[2])
			p1 = f2[0], p2 = f2[1];
		else
			p1 = f2[0], p2 = f2[2];

		Vector3 v(p1, p2);
		v = v / pow(v.squared_length(), 0.5);
		n2 = v;
	}
	else
		n2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);
	//angle between line and line
	if (f1.is_degenerate() && f2.is_degenerate())
	{
		double cos_normal = abs(n1*n2);
		return cos_normal;
	}
	//line and face
	else if ((f1.is_degenerate() && !f2.is_degenerate()) || (!f1.is_degenerate() && f2.is_degenerate()))
	{
		double cos_normal = n1*n2;
		double cos_angle = pow(1 - cos_normal*cos_normal, 0.5);
		return 0;
	}
	else
	{
		double cos_normal = abs(n1*n2);
		return cos_normal;
	}
}
double MAT::compute_face_radius_difference(Face& f1, Face& f2)
{
	double r1, r2;
	if (f1.is_degenerate())
		r1 = (radius[pointmap[f1[0]]] + radius[pointmap[f1[1]]]) / 2.0f;
	else
		r1 = (radius[pointmap[f1[0]]] + radius[pointmap[f1[1]]] + radius[pointmap[f1[2]]]) / 3.0f;

	if (f2.is_degenerate())
		r2 = (radius[pointmap[f2[0]]] + radius[pointmap[f2[1]]]) / 2.0f;
	else
		r2 = (radius[pointmap[f2[0]]] + radius[pointmap[f2[1]]] + radius[pointmap[f2[2]]]) / 3.0f;

	double radius_dis = r1 > r2 ? r1 / r2 : r2 / r1;
	return radius_dis - 1;
}
vector<Point> MAT::computeJointPoint(Face& f1, Face& f2)
{
	vector<Point> joints;

	if (f1.is_degenerate() || f2.is_degenerate())
	{
		if (f1[0] == f2[0] || f1[0] == f2[1] || f1[0] == f2[2])
		{
			joints.push_back(f1[0]);
		}
		else if (f1[1] == f2[0] || f1[1] == f2[1] || f1[1] == f2[2])
		{
			joints.push_back(f1[1]);

		}
		else if (f1[2] == f2[0] || f1[2] == f2[1] || f1[2] == f2[2])
		{
			joints.push_back(f1[2]);
		}
	}

	else if (!f1.is_degenerate() && !f2.is_degenerate())
	{
		if (f1[0] == f2[0] || f1[0] == f2[1] || f1[0] == f2[2])
		{
			joints.push_back(f1[0]);
		}
		if (f1[1] == f2[0] || f1[1] == f2[1] || f1[1] == f2[2])
		{
			joints.push_back(f1[1]);
		}
		if (f1[2] == f2[0] || f1[2] == f2[1] || f1[2] == f2[2])
		{
			joints.push_back(f1[2]);
		}

	}
	return joints;
}
double MAT::compute_face_angle_degree(Face& f1, Face& f2)
{
	//output bend angle, angle=0: no bend, angle=180: all bend

	Vector3 n1;
	Vector3 n2;
	if (f1.is_degenerate())
	{
		Point p1, p2;
		if (f1[0] == f1[1])
			p1 = f1[0], p2 = f1[2];
		else if (f1[0] == f1[2])
			p1 = f1[0], p2 = f1[1];
		else
			p1 = f1[0], p2 = f1[2];

		Vector3 v(p1, p2);
		v = v / pow(v.squared_length(), 0.5);
		n1 = v;
	}
	else
		n1 = CGAL::unit_normal(f1[0], f1[1], f1[2]);

	if (f2.is_degenerate())
	{
		Point p1, p2;
		if (f2[0] == f2[1])
			p1 = f2[0], p2 = f2[2];
		else if (f2[0] == f2[2])
			p1 = f2[0], p2 = f2[1];
		else
			p1 = f2[0], p2 = f2[2];

		Vector3 v(p1, p2);
		v = v / pow(v.squared_length(), 0.5);
		n2 = v;
	}
	else
		n2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);


	//angle between line and line
	if (f1.is_degenerate() && f2.is_degenerate())
	{

		vector<Point> joints = computeJointPoint(f1, f2);
		//center
		Point c1 = CGAL::midpoint(f1[0], f1[1]);
		Point c2 = CGAL::midpoint(f2[0], f2[1]);

		//get new normal from joint to center, Vector_3 (a,b)=b-a
		Vector3 nn1(joints[0], c1), nn2(joints[0], c2);
		nn1 = nn1 / pow(nn1.squared_length(), 0.5f);
		nn2 = nn2 / pow(nn2.squared_length(), 0.5f);

		//directly compute the angle between lines
		double cos_normal = nn1*nn2;

		//bend angle
		double degree_angle = 180 - acos(cos_normal) / PI * 180;
		return degree_angle;
	}
	//angle between line and face: 90 to 180 degree
	else if ((f1.is_degenerate() && !f2.is_degenerate()) || (!f1.is_degenerate() && f2.is_degenerate()))
	{
		//abs cos angle between n1 n2, which is 0 to 90
		double cos_normal = abs(n1*n2);

		//compute the bend
		double degree_angle = 180 - acos(cos_normal) / PI*180.0f;

		return degree_angle;
	}

	//angle between face and face: 0 to 180 degree
	else
	{
		//c1 and c2
		Point c1 = CGAL::centroid(f1), c2 = CGAL::centroid(f2);
		//mid point
		Point mid = CGAL::midpoint(c1, c2);
		//joints
		vector<Point> joints = computeJointPoint(f1, f2);
		//ref vector
		Vector3 ref_vec(joints[0], mid);

		//make n1 n2 towards inner
		if (n1*ref_vec < 0)
			n1 = -n1;
		if (n2*ref_vec < 0)
			n2 = -n2;

		double cos_normal = n1*n2;
		double degree_angle = acos(cos_normal) / PI*180.0f;

		return degree_angle;
	}
}
double MAT::compute_face_radius_gradient(Face& f1, Face& f2)
{
	double r1, r2;
	Point c1, c2;
	if (f1.is_degenerate())
	{
		r1 = (radius[pointmap[f1[0]]] + radius[pointmap[f1[1]]]) / 2.0f;
		c1 = CGAL::midpoint(f1[0], f1[1]);
	}
	else
	{
		r1 = (radius[pointmap[f1[0]]] + radius[pointmap[f1[1]]] + radius[pointmap[f1[2]]]) / 3.0f;
		c1 = CGAL::centroid(f1);
	}
	if (f2.is_degenerate())
	{
		r2 = (radius[pointmap[f2[0]]] + radius[pointmap[f2[1]]]) / 2.0f;
		c2 = CGAL::midpoint(f2[0], f2[1]);
	}
	else
	{
		r2 = (radius[pointmap[f2[0]]] + radius[pointmap[f2[1]]] + radius[pointmap[f2[2]]]) / 3.0f;
		c1 = CGAL::centroid(f2);
	}

	if (r1 > r2)
	{
		double a = r2;
		r2 = r1;
		r1 = a;
	}
	//boundingbox length
	double dis = pow(CGAL::squared_distance(c1, c2), 0.5f);
	double radius_gradient = abs(r2 - r1) / (dis);

	return radius_gradient;
}

double MAT::compute_face_slab_angle(Face& f1, Face& f2)
{
	//compute two-side angles of two slabs
	if (f1.is_degenerate() && f2.is_degenerate())
	{
		Point pc, p0, p1;
		if (f1[0] == f2[0])
			pc = f1[0], p0 = f1[1], p1 = f2[1];
		if (f1[0] == f2[1])
			pc = f1[0], p0 = f1[1], p1 = f2[0];

		if (f1[1] == f2[0])
			pc = f1[1], p0 = f1[0], p1 = f2[1];
		if (f1[1] == f2[1])
			pc = f1[1], p0 = f1[0], p1 = f2[0];

		double rc = radius[pointmap[pc]], r0 = radius[pointmap[p0]], r1 = radius[pointmap[p1]];

		Vector3 v0 = p0 - pc, v1 = p1 - pc;
		double d0 = pow(v0.squared_length(), 0.5), d1 = pow(v1.squared_length(), 0.5);

		v0 = v0 / d0, v1 = v1 / d1;

		double inner_angle = acos(v0*v1);
		double ag0 = (rc - r0) / abs(rc - r0) * asin(abs(r0 - rc) / d0);
		double ag1 = (rc - r1) / abs(rc - r1) * asin(abs(r1 - rc) / d1);

		double ag_A = (inner_angle + ag0 + ag1) / PI;
		double ag_B = (2 * PI - inner_angle + ag0 + ag1) / PI;

		if (ag_A >= 1 && ag_B >= 1)
			return (ag_A + ag_B - 2) / 2.0f;

		if (ag_A < 1 && ag_B >= 1)
			return (ag_A + ag_B - 1) / 2.0f;

		if (ag_B < 1 && ag_A >= 1)
			return (ag_A + ag_B - 1) / 2.0f;

		if (ag_A < 1 && ag_B < 1)
			return (ag_A + ag_B) / 2.0f;


	}
	else if (!f1.is_degenerate() && !f2.is_degenerate())
	{
		vector<Vector3> normals = compute_face_consistent_normal(f1, f2);
		vector<Vector3> slabnormal1, slabnormal2;

		vector<double> r1(3), r2(3);
		r1[0] = radius[pointmap[f1[0]]];
		r1[1] = radius[pointmap[f1[1]]];
		r1[2] = radius[pointmap[f1[2]]];

		r2[0] = radius[pointmap[f2[0]]];
		r2[1] = radius[pointmap[f2[1]]];
		r2[2] = radius[pointmap[f2[2]]];

		vector<Face> slab1, slab2;
		bool check1 = compute_slab(f1, normals[0], r1, slab1, slabnormal1);
		bool check2 = compute_slab(f2, normals[1], r2, slab2, slabnormal2);


		if (check1 && check2)
		{
			double cos1 = slabnormal1[0] * slabnormal2[0];
			double cos2 = slabnormal1[1] * slabnormal2[1];

			double ag1 = acos(cos1) / PI;
			double ag2 = acos(cos2) / PI;
			bool convex1 = checkFaceConvexwithTwoNormals(slab1[0], slab2[0], slabnormal1[0], slabnormal2[0]);
			bool convex2 = checkFaceConvexwithTwoNormals(slab1[1], slab2[1], slabnormal1[1], slabnormal2[1]);

			return (ag1 + ag2) / 2.0f;
		}
		else
		{
			return 1.0f;
		}
	}
}
double MAT::compute_triangle_area(Face& f)
{

	double a = pow((f[0] - f[1]).squared_length(), 0.5f);
	double b = pow((f[2] - f[1]).squared_length(), 0.5f);
	double c = pow((f[2] - f[0]).squared_length(), 0.5f);
	double p = 0.5*(a + b + c);
	double S_2 = p*(p - a)*(p - b)*(p - c);
	return pow(S_2, 0.5f);
}
vector<Vector3> MAT::compute_face_consistent_normal(Face& f1, Face& f2)
{
	//give a two faces on the manifold complex, return their normals towards the same side where angle < PI
	Vector3 n1;
	Vector3 n2;

	n1 = CGAL::unit_normal(f1[0], f1[1], f1[2]);
	n2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);

	//c1 and c2
	Point c1 = CGAL::centroid(f1), c2 = CGAL::centroid(f2);
	//mid point
	Point mid = CGAL::midpoint(c1, c2);
	//joints
	vector<Point> joints = computeJointPoint(f1, f2);
	//ref vector
	Vector3 ref_vec(joints[0], mid);

	//make n1 n2 towards inner
	if (n1*ref_vec < 0)
		n1 = -n1;
	if (n2*ref_vec < 0)
		n2 = -n2;

	//if two face are flat but normals are inverse
	if (n1*n2 == -1)n1 = -n1;

	vector<Vector3> normals(2);
	normals[0] = n1, normals[1] = n2;
	return normals;

}
bool MAT::compute_distance_to_line(Vector3&  p, Vector3& v0, Vector3& v1, double& dist, Vector3& fp)
{
	//only used for compute slab
	Vector3 v0v1(v1 - v0), pv0(v0 - p), pv1(v1 - p);
	double area = fabs(pow((CGAL::cross_product(v0v1, pv0)).squared_length(), 0.5));
	if (pow(v0v1.squared_length(), 0.5) > 1e-12)
	{
		dist = area / pow(v0v1.squared_length(), 0.5);
		double t = (pv0*pv0 - pv0*pv1) / (pv0*pv0 + pv1*pv1 - 2 * pv0*pv1);
		fp = (1 - t)*v0 + t*v1;
		return true;
	}
	else
		return false;
}
bool MAT::compute_slab(Face f, Vector3 normal, vector<double> r, vector<Face>& slab, vector<Vector3>& slabnormals) {

	if (f.is_degenerate())
		return false;

	Vector3 c0(f[0][0], f[0][1], f[0][2]), c1(f[1][0], f[1][1], f[1][2]), c2(f[2][0], f[2][1], f[2][2]);
	double r0 = r[0], r1 = r[1], r2 = r[2];

	Vector3 c0c1(c1 - c0), c0c2(c2 - c0), c1c2(c2 - c1);

	double c0c1len = c0c1.squared_length(), c0c2len = c0c2.squared_length(), c1c2len = c1c2.squared_length();
	c0c1len = pow(c0c1len, 0.5), c0c2len = pow(c0c2len, 0.5), c1c2len = pow(c1c2len, 0.5);

	double dr0r1(fabs(r0 - r1)), dr0r2(fabs(r0 - r2)), dr1r2(fabs(r1 - r2));

	// some spheres are concentric and there are no triangles.
	if ((c0c1len < 1e-8) || (c0c2len < 1e-8) || (c1c2len < 1e-8))
		return false;

	//// some spheres are included in some other spheres 
	//if ((c0c1len - abs(r0 - r1) < 1e-8) || (c0c2len - abs(r0 - r2) < 1e-8) || (c1c2len - abs(r1 - r2) < 1e-8))
	//	return false;

	Vector3 norm = normal;
	norm = norm / pow(norm.squared_length(), 0.5);

	vector<Face> boundplane(2);
	vector<Vector3> boundplanenormal(2);
	// equal-radius spheres
	if ((dr0r1 < 1e-8) && (dr0r2 < 1e-8) && (dr1r2 < 1e-8))
	{
		Vector3 f1p1 = c0 + norm * r0, f1p2 = c1 + norm * r1, f1p3 = c2 + norm * r2;

		Vector3 f2p1 = c0 - norm * r0, f2p2 = c1 - norm * r1, f2p3 = c2 - norm * r2;

		Point p0(0, 0, 0);

		Face f1(p0 + f1p1, p0 + f1p2, p0 + f1p3);
		Face f2(p0 + f2p1, p0 + f2p2, p0 + f2p3);

		boundplane[0] = f1, boundplane[1] = f2;
		slab = boundplane;

		Vector3 v1 = CGAL::unit_normal(f1[0], f1[1], f1[2]), v2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);
		boundplanenormal[0] = v1, boundplanenormal[1] = -v2;

		if (v1*normal > 0)
			boundplanenormal[0] = v1, boundplanenormal[1] = -v2;
		else
			boundplanenormal[0] = -v1, boundplanenormal[1] = v2;
		slabnormals = boundplanenormal;

		return  true;

	}
	else
	{
		// two points on the tangent plane
		Vector3 apex0, apex1;

		// two spheres are equal-radius
		if (dr0r1 < 1e-8)
		{
			apex0 = (r2 * c0 - r0 * c2) / (r2 - r0);
			apex1 = (r2 * c1 - r1 * c2) / (r2 - r1);
		}
		else if (dr0r2 < 1e-8)
		{
			apex0 = (r1 * c0 - r0 * c1) / (r1 - r0);
			apex1 = (r2 * c1 - r1 * c2) / (r2 - r1);
		}
		else if (dr1r2 < 1e-8)
		{
			apex0 = (r2 * c0 - r0 * c2) / (r2 - r0);
			apex1 = (r0 * c1 - r1 * c0) / (r0 - r1);
		}
		else
		{
			apex0 = (r2 * c0 - r0 * c2) / (r2 - r0);
			apex1 = (r2 * c1 - r1 * c2) / (r2 - r1);
		}

		double distc0;
		Vector3 fp;

		compute_distance_to_line(c0, apex0, apex1, distc0, fp);

		double sangle = r0 / distc0;
		if (fabs(sangle) > 1.)
		{
			return false;
		}
		double cangle = sqrt(1. - r0*r0 / distc0 / distc0);
		Vector3 norfpc0(c0 - fp);
		norfpc0 = norfpc0 / pow(norfpc0.squared_length(), 0.5f);


		Vector3 newnorm[2];
		newnorm[0] = norm*cangle - norfpc0*sangle;
		newnorm[1] = -norm*cangle - norfpc0*sangle;


		Vector3 f1p1 = c0 + newnorm[0] * r0, f1p2 = c1 + newnorm[0] * r1, f1p3 = c2 + newnorm[0] * r2;
		Vector3 f2p1 = c0 + newnorm[1] * r0, f2p2 = c1 + newnorm[1] * r1, f2p3 = c2 + newnorm[1] * r2;

		Point p0(0, 0, 0);

		Face f1(p0 + f1p1, p0 + f1p2, p0 + f1p3);
		Face f2(p0 + f2p1, p0 + f2p2, p0 + f2p3);

		boundplane[0] = f1, boundplane[1] = f2;
		slab = boundplane;

		if (CGAL::collinear(f1[0], f1[1], f1[2]) || CGAL::collinear(f2[0], f2[1], f2[2]))
			return false;

		Vector3 v1 = CGAL::unit_normal(f1[0], f1[1], f1[2]), v2 = CGAL::unit_normal(f2[0], f2[1], f2[2]);

		if (v1*normal > 0)
			boundplanenormal[0] = v1, boundplanenormal[1] = -v2;
		else
			boundplanenormal[0] = -v1, boundplanenormal[1] = v2;

		slabnormals = boundplanenormal;

	}

	return true;

}
Vector3 MAT::compute_face_normal(Face& f1)
{
	Vector3 n1;
	Vector3 n2;
	if (f1.is_degenerate() || CGAL::collinear(f1[0], f1[1], f1[2]))
	{
		Point p1, p2;
		if (f1[0] == f1[1])
			p1 = f1[0], p2 = f1[2];
		else if (f1[0] == f1[2])
			p1 = f1[0], p2 = f1[1];
		else
			p1 = f1[0], p2 = f1[2];

		Vector3 v(p1, p2);
		v = v / pow(v.squared_length(), 0.5);
		n1 = v;
	}
	else
		n1 = CGAL::unit_normal(f1[0], f1[1], f1[2]);

	return n1;
}
int MAT::getLargestFaceIndexOfPatch(Patch& pa1)
{
	double max_r1 = 0.0f;
	Face max_face1;
	for (int i = 0; i < pa1.faces.size(); i++)
	{
		Face f1 = pa1.faces[i];
		double now_r1 = compute_face_mean_r(f1);
		if (now_r1 > max_r1)
		{
			max_r1 = now_r1;
			max_face1 = f1;
		}
	}
	for (int i = 0; i < faces.size(); i++)
	{
		if (faces[i] == max_face1)
			return i;
	}
}