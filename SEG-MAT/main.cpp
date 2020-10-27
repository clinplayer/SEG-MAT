#define _CRT_SECURE_NO_DEPRECATE
#include <iostream>
#include "Decomposer.h"
#include "argparser.hpp"

vector<vector<float>> colors;

void segOneModel(string modeloff, string matpath, string smatpath, string outputpath, float growing_threshold, float min_region, int prim_check)
{
	int ni1 = modeloff.find_last_of("\\");
	int ni2 = modeloff.find_last_of(".");
	string name = modeloff.substr(ni1 + 1, ni2 - ni1 - 1);

	Decomposer solver;
	Mesh mesh = solver.readMesh(modeloff);

	if (mesh.number_of_faces() == 0) {
		cout << "input surface has no faces!" << endl;
		return;
	}

	MAT smat(smatpath, mesh);
	if (smat.points.size() == 0) {
		cout << "structure MAT has no points! structural segmentation is skipped." << endl;
	}

	MAT mat(matpath, mesh);
	if (mat.points.size() == 0) {
		cout << "base MAT has no points!" << endl;
		return;
	}
	cout << "intialzation done, start segmenting ..." << endl;
	solver.decompose3Dshape(mat, smat, mesh, growing_threshold, min_region);

	cout << "transferring to surface (graph-cut) ..." << endl;
	solver.transfer_MAT_mesh(mat, mesh, 0.3f);
	
	solver.saveColoredMesh(mat, mesh, colors, outputpath + name + "_seg.off");
	cout << "segmentation result output finished." << endl;

	if (prim_check)
	{
		cout << "computing primitive-based representation ... " << endl;
		solver.primitiveAbstraction(mat, mesh, colors, outputpath + name + "_prim.off");
		cout << "primitives output finished. " << endl;
	}
}


int main(int argc, char* argv[])
{

	ap::parser p(argc, argv);
	p.add("-m", "--mesh", "path to the surface mesh (.off)", ap::mode::REQUIRED);
	p.add("-b", "--bmat", "path to the base MAT (.ma)", ap::mode::REQUIRED);
	p.add("-s", "--smat", "path to the structure MAT (.ma)", ap::mode::REQUIRED);
	p.add("-g", "--grow", "(optional) growing threshold, default 0.015");   
	p.add("-n", "--min", "(optional) minimal region, default 0.002");    
	p.add("-p", "--prim", "(optional) whether to compute the primitive representation (0:no, 1:yes)");

	auto args = p.parse();

	if (!args.parsed_successfully()) {
		std::cerr << "Unsuccessful parse, please check SEG-MAT -h for help \n";
		return -1;
	}

	auto mesh_path = args["-m"];
	auto mat_path = args["-b"];
	auto smat_path = args["-s"];
	auto grow =	args["-g"].empty() ? "0.015" : args["-g"];
	auto min_r = args["-n"].empty() ? "0.002" : args["-n"];
	auto prim = args["-p"].empty() ? "0" : args["-p"];

	string output_path = "";
	const size_t last_slash_idx = mesh_path.rfind('\\');
	if (std::string::npos != last_slash_idx)
	{
		output_path = mesh_path.substr(0, last_slash_idx)+'\\';
	}

	float growing_threshold = atof(grow.c_str());
	float min_region = atof(min_r.c_str());
	int prim_check = atoi(prim.c_str());

	colors = {
		{ 166, 206, 227 },
		{ 31,120,180 },
		{ 178,223,138 },
		{ 51,160,44 },
		{ 251,154,153 },
		{ 227,26,28 },
		{ 253,191,111 },
		{ 255,127,0 },
		{ 166,216,84 },
		{ 202,178,214 },
		{ 106,61,154 },
		{ 255,255,153 },
		{ 177,89,40 },
		{ 102,194,165 },
		{ 252,141,98 },
		{ 141,160,203 },
		{ 231,138,195 },
		{ 166,216,84 },
		{ 255,217,47 } };
	
	segOneModel(mesh_path, mat_path, smat_path, output_path, growing_threshold, min_region, prim_check);

	return 0;

}