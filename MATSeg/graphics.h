#pragma once
#include "common_include.h"
#include "MAT.h"
#include <GL/glut.h>




vector<vector<float>> getcolors(int num)
{
	//vector<vector<float>> colors;
	//while (num--)
	//{
	//	vector<float> vf;
	//	vf.push_back(rand() / double(RAND_MAX));
	//	vf.push_back(rand() / double(RAND_MAX));
	//	vf.push_back(rand() / double(RAND_MAX));
	//	vf.push_back(1.00000f);
	//	colors.push_back(vf);
	//}


	colors =
	{ { 166, 206, 227 },
	{ 31,120,180 },
	{ 178,223,138 },
	{ 51,160,44 },
	{ 251,154,153 },
	{ 227,26,28 },
	{ 253,191,111 },
	{ 255,127,0 },
	{ 166,216,84 },
		//{ 202,178,214 },
	{ 106,61,154 },
	{ 255,255,153 },
	{ 177,89,40 },
	{ 102,194,165 },
	{ 252,141,98 },
	{ 141,160,203 },
	{ 231,138,195 },
	{ 166,216,84 },
	{ 255,217,47 } };


	//for (int i = 0; i < 18; i++)
	//{
	//	cout << "newmtl material_" << i << endl;
	//	cout << "Ka 0.1 0.1 0.1" << endl;
	//	cout << "Kd ";
	//	cout << colors[i][0] / 255.0f << " " << colors[i][1] / 255.0f << " " << colors[i][2] / 255.0f << endl;
	//	cout << "Ks 0.4 0.4 0.4" << endl;
	//	cout << "Tr 0.000000" << endl;
	//	cout << "illum 2" << endl;
	//	cout << "Ns 10.000000" << endl;
	//}

	return colors;
}
void light(double x, double y, double z)
{
	//GLfloat ambientLight[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	////// 设置光照模型，将ambientLight所指定的RGBA强度值应用到环境光
	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight);
	GLfloat position[] = { 1000, 1000,cz + 10,1.0f };
	GLfloat position2[] = { 1000 ,1000,cz - 10 ,1.0f };

	GLfloat ambient[] = { 0.1f,0.1f,0.1f,1.0f };
	GLfloat diffuse[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat specular[] = { 0.0f, 0.0f, 0.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

	glLightfv(GL_LIGHT1, GL_POSITION, position2);
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular);



	glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
	glEnable(GL_LIGHTING);
}


void drawMAT()
{


	GLfloat meshcolor[4] = { 0.5f,0.5f,0.5f,1.0f };
	//GLfloat meshcolor[4] = { 0,0,0,1.0f };

	//GLfloat diffuse[4] = { colors[10][0],colors[10][1],colors[10][2],colors[10][3] };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, meshcolor);
	glMaterialfv(GL_BACK, GL_DIFFUSE, meshcolor);

	glLineWidth(2.0f);
	//glEnable(GL_POINT_SMOOTH);
	//glEnable(GL_LINE_SMOOTH);
	//glHint(GL_POINT_SMOOTH_HINT, GL_NICEST); // Make round points, not square points  
	//glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);  // Antialias the lines  
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//绘制mat的网格
	double f = 0.0001f;
	glBegin(GL_LINES);

	for (int i = 0; i < g_mat.faces.size(); i++)
	{
		//画不画线
		if (g_mat.faces[i].is_degenerate())continue;
		Point p1 = g_mat.faces[i].vertex(0);
		Point p2 = g_mat.faces[i].vertex(1);
		Point p3 = g_mat.faces[i].vertex(2);
		glVertex3d(p1.x()+f, p1.y()+f, p1.z()+f);
		glVertex3d(p2.x() + f, p2.y() + f, p2.z() + f);
		glVertex3d(p1.x() + f, p1.y() + f, p1.z() + f);
		glVertex3d(p3.x() + f, p3.y() + f, p3.z() + f);
		glVertex3d(p2.x() + f, p2.y() + f, p2.z() + f);
		glVertex3d(p3.x() + f, p3.y() + f, p3.z() + f);
		f = -f;
		glVertex3d(p1.x() + f, p1.y() + f, p1.z() + f);
		glVertex3d(p2.x() + f, p2.y() + f, p2.z() + f);
		glVertex3d(p1.x() + f, p1.y() + f, p1.z() + f);
		glVertex3d(p3.x() + f, p3.y() + f, p3.z() + f);
		glVertex3d(p2.x() + f, p2.y() + f, p2.z() + f);
		glVertex3d(p3.x() + f, p3.y() + f, p3.z() + f);
	}
	glEnd();
	
	//绘制单一颜色的mat面片
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < g_mat.faces.size(); i++)
	{
	/*	GLfloat meshcolor2[4] = { 135.0/255, 206.0/255.0, 180.0/255.0, 1.0f };
		glMaterialfv(GL_FRONT, GL_DIFFUSE, meshcolor2);
		glMaterialfv(GL_BACK, GL_DIFFUSE, meshcolor2);
*/
		if (g_mat.faces[i].is_degenerate())
		{
			GLfloat meshcolor2[4] = { 0.0f,0.0f,0.0f,1.0f };
			glMaterialfv(GL_FRONT, GL_DIFFUSE, meshcolor2);
			glMaterialfv(GL_BACK, GL_DIFFUSE, meshcolor2);
		}
		else
		{
			GLfloat meshcolor2[4] = { 0.5f,0.5f,1.0f,1.0f };
			glMaterialfv(GL_FRONT, GL_DIFFUSE, meshcolor2);
			glMaterialfv(GL_BACK, GL_DIFFUSE, meshcolor2);
		}

		Point p1 = g_mat.faces[i].vertex(0);
		Point p2 = g_mat.faces[i].vertex(1);
		Point p3 = g_mat.faces[i].vertex(2);
		glVertex3d(p1.x(), p1.y(), p1.z());
		glVertex3d(p2.x(), p2.y(), p2.z());
		glVertex3d(p3.x(), p3.y(), p3.z());

		glPushMatrix();
		Point p = CGAL::centroid(g_mat.faces[i]);
		glTranslated(p.x(),p.y(),p.z());
		//if (p.x() == 0)continue;
		double r3 = (g_mat.radius[g_mat.pointmap[p1]]+ g_mat.radius[g_mat.pointmap[p2]]+ g_mat.radius[g_mat.pointmap[p3]])*0.12f;
		//glutSolidSphere(r3, 20, 20);
		glPopMatrix();
	}
	glEnd();



	

}
void drawmesh()
{
	Mesh::Face_range::iterator  fbegin, fend;
	Mesh::Face_range f_range = g_mesh.faces();


	GLfloat meshcolor[4] = {0.5f,0.5f,0.5f,0.4f };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, meshcolor);
	glMaterialfv(GL_BACK, GL_DIFFUSE, meshcolor);
	glLineWidth(3);
	glDepthMask(GL_FALSE);
	BOOST_FOREACH(Facei f_index, g_mesh.faces()) {
		glBegin(GL_TRIANGLES);
		BOOST_FOREACH(Vertexi v_index, vertices_around_face(g_mesh.halfedge(f_index), g_mesh)) {
			Point p = g_mesh.point(v_index);
			glVertex3d(p.x(), p.y(), p.z());

		}
	glEnd();
	}		
	glDepthMask(GL_TRUE);
}
void drawshpere()
{

	for (int i = 0; i < g_mat.final_patch.size(); i++)
	{
		Patch pa = g_mat.final_patch[i];
		GLfloat diffuse[4] = { colors[i][0],colors[i][1],colors[i][2],1.0f };
		for (int j = 0; j < pa.points.size(); j++)
		{
			Point p = pa.points[j];
			glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
			glMaterialfv(GL_BACK, GL_DIFFUSE, diffuse);
			glPushMatrix();
			glTranslated(p.x(), p.y(), p.z());
			double r = g_mat.radius[g_mat.pointmap[p]];
			glutSolidSphere(r / 1.0f, 20, 20);
			glPopMatrix();
		}
	}
}

double a1 = 0, a2 = 0;//sphere parameter angle
double dis = 10;
double step = (3.14159f * 2.0f) / (144.0f);
void display(void)
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0, 1.0f);
	
	glPushMatrix();
	

	drawmesh();
	drawshpere();
	drawMAT();
	glPopMatrix();


	glFlush();
	glutSwapBuffers();
	//cout << dis <<" "<< a1<<" "<< a2 << endl;
}

void KeyBoards(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'w':
		dis -= 1;
		break;
	case 'a':
		//a2 += step;
		break;
	case 's':
		dis += 1;
		break;
	case 'd':
		//a2 -= step;
		break;
	case 'p':
		break;

	}

	glLoadIdentity();
	double newx = dis*sin(a1)*cos(a2);
	double newy = dis*sin(a1)*sin(a2);
	double newz = dis*cos(a1);
	gluLookAt(newx, newy, newz, g_mat.centroid[0], g_mat.centroid[1], g_mat.centroid[2], 0, 1, 0);
	light(newx, newy, newz);
	glutPostRedisplay();

}
//mouse click
int lx = 0, ly = 0, nx = 0, ny = 0;
void myMouse(int button, int state, int x, int y)
{

	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			lx = x;
			ly = y;

		}

		else if (state == GLUT_UP)
		{
		}
		break;
	}
}
//mouse move to adjust the view
void motion(int x, int y)
{

	nx = x;
	ny = y;
	a1 += (double)(nx - lx) / 6.0*step;
	a2 += (double)(ny - ly) / 6.0*step;
	double newx = dis*sin(a1)*cos(a2);
	double newy = dis*sin(a1)*sin(a2);
	double newz = dis*cos(a1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(newx, newy, newz, g_mat.centroid[0], g_mat.centroid[1], g_mat.centroid[2], 0, 1, 0);
	glutPostRedisplay();
	lx = x;
	ly = y;
}
void visualize(Mesh mesh, MAT mat, MAT smat) {

	g_mesh = mesh, g_mat = mat, g_smat = smat;
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE); 
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(1500, 1500);
	glutCreateWindow("SEGMAT  w/s-zoom; mouse-rotate");

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);

	glBlendFunc(GL_ONE, GL_ONE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, 1, 0.1, 5000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, dis, 0, 0, 0 , 0, 1, 0);
	light(0,0,0);
	glutKeyboardFunc(&KeyBoards);
	glutMouseFunc(&myMouse);
	glutMotionFunc(&motion);
	glutDisplayFunc(&display);
	glutMainLoop();
}