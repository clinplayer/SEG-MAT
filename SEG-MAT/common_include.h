#pragma once
#define CGAL_EIGEN3_ENABLED
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/properties_Surface_mesh.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <list>
#include <cmath>
#include <fstream>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include "emd/emd.h"

using namespace std;

#define PI 3.141592653589
typedef CGAL::Simple_cartesian<double>                        Kernel;
typedef Kernel::Point_3                                       Point;
typedef Kernel::Triangle_3                                    Face;
typedef Kernel::Segment_3                                     Edge;
typedef Kernel::Ray_3                                         Ray;
typedef Kernel::Vector_3									  Vector3;
typedef CGAL::Surface_mesh<Point>                             Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor          vertex_descriptor;
typedef CGAL::Mean_curvature_flow_skeletonization<Mesh>       Skeletonization;
typedef Skeletonization::Skeleton                             Skeleton;
typedef Skeleton::vertex_descriptor                           Skeleton_vertex;
typedef Skeleton::edge_descriptor                             Skeleton_edge;
typedef Mesh::Vertex_index							          Vertexi;
typedef Mesh::Face_index							          Facei;
typedef CGAL::Triangulation_data_structure_3<>                Tds;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Search_traits_3<K> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree NNTree;



