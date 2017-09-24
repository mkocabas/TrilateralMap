#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/SbPlane.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoFont.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoTranslation.h>

#include <cmath>
#include <stdio.h>
#include <clocale>
#include <locale>
#include <codecvt>
#include <iostream>
#include <fstream>
#include <time.h>
//#include <thread>
#include "geodesic_algorithm_exact.h"

#include "Mesh.h"
#include "Painter.h"
#include "Dijkstra.h"
#include "ExactGeodesic.h"
#include "Trilateral.h"


using namespace std;

void showMesh(char ** argv);
void exactGeodesicPath(char ** argv);
void exactGeodesicPathTriple(char ** argv);
void exactClass(char ** argv);
void exactDistances(char ** argv);

void TrilateralTest(char ** argv);
void TrilateralPlain();
void ExtractTest(char ** argv);

int firstElement(std::pair<int, double> &p){
	return p.first;
}

std::vector<int> getIntVector(std::vector<std::pair<int, double>> *p){
	std::vector<int> pts;
	std::transform(p->begin(), p->end(), std::back_inserter(pts), firstElement);
	return pts;
}

int main(int, char ** argv){
	//exactGeodesicPath(argv);
	//exactClass(argv);
	//exactDistances(argv);
	//TrilateralTest(argv);
	//TrilateralPlain();
	ExtractTest(argv);
	//showMesh(argv);
	return 0;
}

void showMesh(char ** argv){
	HWND window = SoWin::init(argv[0]);
	SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
	SoSeparator * root = new SoSeparator;
	root->ref();
	Mesh* mesh = new Mesh();
	Painter* painter = new Painter();
	mesh->loadOff("Data/giraffe.off");

	root->addChild(painter->getShapeSep(mesh, Painter::white));
	viewer->setSize(SbVec2s(1280, 960));
	viewer->setSceneGraph(root);
	viewer->show();
	viewer->setBackgroundColor(SbColor(255, 255, 255));
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();
}

void exactGeodesicPath(char ** argv){
	HWND window = SoWin::init(argv[0]);
	SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
	SoSeparator * root = new SoSeparator;
	root->ref();
	Mesh* _mesh = new Mesh();
	Painter* painter = new Painter();
	

	std::vector<double> vs;
	std::vector<int> ts;
	_mesh->loadOff("Data/horse1.off", &vs, &ts);

	unsigned source_vertex_index = 1304;
	unsigned target_vertex_index = 2206;

	Dijkstra *d = new Dijkstra(_mesh);

	clock_t tClock1 = clock();
	root->addChild(painter->drawLineSet(_mesh, d->computeSinglePathInMesh(source_vertex_index, target_vertex_index), Painter::green));
	std::cout << "Execution time for Dijkstra: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;

	///////////////////////////////////////////////////////////////////////////////
	std::vector<double> points;
	std::vector<unsigned> faces;

	bool success = geodesic::convert_from_my_mesh(vs, ts, points, faces);
	if (!success)
	{
		std::cout << "something is wrong with the input file" << std::endl;
	}

	geodesic::Mesh mesh;
	mesh.initialize_mesh_data(points, faces);		//create internal mesh data structure including edges

	geodesic::GeodesicAlgorithmExact algorithm(&mesh);	//create exact algorithm for the mesh
	
	std::vector<std::array<double, 3>> path_as_vector;

	geodesic::SurfacePoint source(&mesh.vertices()[source_vertex_index]);		//create source 
	std::vector<geodesic::SurfacePoint> all_sources(1, source);					//in general, there could be multiple sources, but now we have only one


	clock_t tClock2 = clock();
	bool selection = true;
	if (selection)	//target vertex specified, compute single path
	{
		geodesic::SurfacePoint target(&mesh.vertices()[target_vertex_index]);		//create source 

		std::vector<geodesic::SurfacePoint> path;	//geodesic path is a sequence of SurfacePoints

		bool const lazy_people_flag = false;		//there are two ways to do exactly the same
		if (lazy_people_flag)
		{
			algorithm.geodesic(source, target, path); //find a single source-target path
		}
		else		//doing the same thing explicitly for educational reasons
		{
			double const distance_limit = geodesic::GEODESIC_INF;			// no limit for propagation
			std::vector<geodesic::SurfacePoint> stop_points(1, target);	//stop propagation when the target is covered
			algorithm.propagate(all_sources, distance_limit, &stop_points);	//"propagate(all_sources)" is also fine, but take more time because covers the whole mesh

			algorithm.trace_back(target, path);		//trace back a single path 
		}

		print_info_about_path(path);
		
		for (unsigned i = 0; i<path.size(); ++i)
		{
			array<double, 3> a;
			geodesic::SurfacePoint& s = path[i];
			std::cout << s.x() << "\t" << s.y() << "\t" << s.z() << std::endl;
			a[0] = s.x();
			a[1] = s.y();
			a[2] = s.z();

			path_as_vector.push_back(a);
		}
	}
	else		//target vertex is not specified, print distances to all vertices
	{
		algorithm.propagate(all_sources);	//cover the whole mesh

		for (unsigned i = 0; i<mesh.vertices().size(); ++i)
		{
			geodesic::SurfacePoint p(&mesh.vertices()[i]);

			double distance;
			unsigned best_source = algorithm.best_source(p, distance);		//for a given surface point, find closets source and distance to this source

			std::cout << distance << " ";		//print geodesic distance for every vertex
		}
		std::cout << std::endl;
	}

	std::cout << "Execution time for Exact: " << (double)(clock() - tClock2) / CLOCKS_PER_SEC << endl;
	///////////////////////////////////////////////////////////////////////////////
	
	root->addChild(painter->getShapeSep(_mesh, Painter::grey));

	root->addChild(painter->drawLineSet(path_as_vector, Painter::blue));
	

	viewer->setSize(SbVec2s(1280, 960));
	viewer->setSceneGraph(root);
	viewer->show();
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();
}

void exactClass(char ** argv){
	HWND window = SoWin::init(argv[0]);
	SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
	SoSeparator * root = new SoSeparator;
	root->ref();
	Mesh* _mesh = new Mesh();
	Painter* painter = new Painter();
	char* filename = "Data/centaur.off";
	_mesh->loadOff(filename);

	Dijkstra *d = new Dijkstra(_mesh);

	// Data/horse2.off
	//int a = 6071, b = 6710, c = 192;	// Sol karin
	//int a = 7070, b = 344, c = 1085;	// Sirt
	//int a = 8539, b = 8093, c = 7919;	// Boyun
	//int a = 7731, b = 6461, c = 5978;	// Alt karin

	// Data/centaur.off
	//int a = 2331, b = 2169, c = 154;	// Sol karin
	//int a = 4794, b = 3946, c = 5632;	// Govde
	int a = 57, b = 2431, c = 4008;	// Sag karin
	//int a = 2674, b = 2130, c = 2251;	// Alt karin

	// Data/person1.off
	//int a = 9694, b = 8981, c = 9470;	// Gogus
	//int a = 10072, b = 9245, c = 9275;	//Kol
	//int a = 10287, b = 10269, c = 7254; // Govde
	//int a = 12244, b = 10381, c = 9358; // Kafa
	//int a = 12244, b = 8372, c = 8720;  // Kafa

	/*root->addChild(painter->drawLineSet(_mesh, d->computeSinglePathInMesh(a, b), Painter::green));
	root->addChild(painter->drawLineSet(_mesh, d->computeSinglePathInMesh(b, c), Painter::green));
	root->addChild(painter->drawLineSet(_mesh, d->computeSinglePathInMesh(a, c), Painter::green));

	root->addChild(painter->getMultiplePointSep(_mesh, d->computeSinglePathInMesh(a, b), Painter::blue));
	root->addChild(painter->getMultiplePointSep(_mesh, d->computeSinglePathInMesh(b, c), Painter::blue));
	root->addChild(painter->getMultiplePointSep(_mesh, d->computeSinglePathInMesh(a, c), Painter::blue));*/

	///////////////////////////////////////////////////////////////////////////////
	ExactGeodesic *g = new ExactGeodesic(filename);
	std::vector<array<double, 3>> path1;
	std::vector<array<double, 3>> path2; 
	std::vector<array<double, 3>> path3;
	
	clock_t tClock1 = clock();
	std::vector<int> t;
	g->computeSinglePath(a, b, &path1);
	g->computeSinglePath(b, c, &path2);
	g->computeSinglePath(a, c, &path3);

	std::cout << "Execution time for Geodesic: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;




	//12143 - 12149 - 12150 - 12151 - 12152 - 12153 -

	/*root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12143]->v1i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12143]->v2i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12150]->v1i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12150]->v2i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12149]->v1i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12149]->v2i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12151]->v1i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12151]->v2i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12153]->v1i, Painter::red));
	root->addChild(painter->get1PointSep(_mesh, _mesh->edges[12153]->v2i, Painter::red));*/
	//root->addChild(painter->drawLineSet(path1, Painter::blue));
	//root->addChild(painter->drawLineSet(path2, Painter::blue));
	//root->addChild(painter->drawLineSet(path3, Painter::blue));

	/*for (int f = 0; f < g->t.size(); f+=2)
	{
		root->addChild(painter->get1PointSep(_mesh, g->t[f], Painter::purple));
		root->addChild(painter->get1PointSep(_mesh, g->t[f+1], Painter::green));
		//root->addChild(painter->get1PointSep(_mesh, g->t[f+2], Painter::purple));
	}*/
	
	
	
	/*root->addChild(painter->getMultiplePointSep(path1, Painter::orange));
	root->addChild(painter->getMultiplePointSep(path2, Painter::orange));
	root->addChild(painter->getMultiplePointSep(path3, Painter::orange));*/
	///////////////////////////////////////////////////////////////////////////////

	_mesh = new Mesh();
	_mesh->loadMesh(&g->vs, &g->ts);

	root->addChild(painter->getMultiplePointSep(_mesh, g->t, Painter::blue));

	root->addChild(painter->getShapeSep(_mesh, Painter::grey));
	viewer->setSize(SbVec2s(1280, 960));
	viewer->setSceneGraph(root);
	viewer->show();
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();
}

void exactDistances(char ** argv){
	
	HWND window = SoWin::init(argv[0]);
	SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
	SoSeparator * root = new SoSeparator;
	root->ref();

	//*************************************************************//
	Mesh* _mesh = new Mesh();
	Painter* painter = new Painter();
	char* filename = "Data/centaur.off";

	_mesh->loadOff(filename);

	// Data/centaur.off
	//int a = 2331, b = 2169, c = 154;	// Sol karin
	//int a = 4794, b = 3946, c = 5632;	// Govde
	int a = 57, b = 2431, c = 4008;	// Sag karin
	//int a = 2674, b = 2130, c = 2251;	// Alt karin

	ExactGeodesic *g = new ExactGeodesic(filename);

	clock_t tClock1 = clock();

	std::vector<std::pair<int, double>> dist;
	g->computeSinglePath(a, b, &dist);
	std::vector<std::pair<int, double>> dist1;
	g->computeSinglePath(b, c, &dist1);
	std::vector<std::pair<int, double>> dist2;
	g->computeSinglePath(c, a, &dist2);

	g->print_distance_info(&dist);
	g->print_distance_info(&dist1);
	g->print_distance_info(&dist2);

	std::cout << "Execution time for Geodesic: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;

	_mesh = new Mesh();
	_mesh->loadMesh(&g->vs, &g->ts);

	std::vector<int> pts = getIntVector(&dist);
	root->addChild(painter->drawLineSet(_mesh, pts, Painter::blue));

	std::vector<int> pts1 = getIntVector(&dist1);
	root->addChild(painter->drawLineSet(_mesh, pts1, Painter::blue));

	std::vector<int> pts2 = getIntVector(&dist2);
	root->addChild(painter->drawLineSet(_mesh, pts2, Painter::blue));
	//*************************************************************//
	root->addChild(painter->getShapeSep(_mesh, Painter::grey));
	viewer->setSize(SbVec2s(1280, 960));
	viewer->setSceneGraph(root);
	viewer->show();
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();
}

void TrilateralTest(char ** argv){
	HWND window = SoWin::init(argv[0]);
	SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
	SoSeparator * root = new SoSeparator;
	root->ref();


	Mesh* _mesh = new Mesh();
	Mesh* m = new Mesh();
	Painter* painter = new Painter();
	char* filename = "Data/dog.off";

	// Data/horse2.off
	//int a = 6071, b = 6710, c = 192;	// Sol karin
	//int a = 7070, b = 344, c = 1085;	// Sirt //failure
	//int a = 8539, b = 8093, c = 7919;	// Boyun
	//int a = 7731, b = 6461, c = 5978;	// Alt karin

	// Data/centaur.off
	//int a = 2331, b = 2169, c = 154;	// Sol karin
	//int a = 4794, b = 3946, c = 5632;	// Govde
	//int a = 57, b = 2431, c = 4008;	// Sag karin
	//int a = 2674, b = 2130, c = 2251;	// Alt karin

	// Data/person1.off
	//int a = 9694, b = 8981, c = 9470;	// Gogus
	//int a = 10072, b = 9245, c = 9275;	//Kol
	//int a = 10287, b = 10269, c = 7254; // Govde
	//int a = 12244, b = 10381, c = 9358; // Kafa
	//int a = 12244, b = 8372, c = 8720;  // Kafa
	//int a = 10201, b = 8522, c = 8527;	// Gogus center = 9449

	// Data/hand.off
	//int a = 1223, b = 729, c = 1508; // ust kismi center = 1492
	//int a = 1536, b = 486, c = 551; // ust kismi center = 1492
	//int a = 2415, b = 1930, c = 650; // ust kismi center = 1492
	//int a = 1485, b = 1496, c = 837;
	//int a = 37, b = 2390, c = 1123;
	
	// Data/dog.off
	int a = 869, b = 847, c = 440; //back

	_mesh->loadOff(filename);
	m->loadOff(filename);
	int gridSize = 6;

	/*srand(time(NULL));
	int a = 0, b = 0, c = 0;

	while (a == b && a == c & b == c){
		a = rand() % m->verts.size() - 1;
		b = rand() % m->verts.size() - 1;
		c = rand() % m->verts.size() - 1;
	}*/

	clock_t tClock1 = clock();
	Trilateral *base = new Trilateral(a, b, c, gridSize, _mesh, filename);
	base->init();
	std::cout << "\nExecution time for Trilateral: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;

	_mesh = new Mesh();
	_mesh->loadMesh(&base->geo->vs, &base->geo->ts);

	/*root->addChild(painter->drawLineSet(_mesh, getIntVector(&base->path1to2), Painter::orange));
	root->addChild(painter->drawLineSet(_mesh, getIntVector(&base->path2to3), Painter::orange));
	root->addChild(painter->drawLineSet(_mesh, getIntVector(&base->path3to1), Painter::orange));

	root->addChild(painter->getMultiplePointSep(_mesh, base->sample1to2, Painter::blue));
	root->addChild(painter->getMultiplePointSep(_mesh, base->sample2to3, Painter::purple));
	root->addChild(painter->getMultiplePointSep(_mesh, base->sample3to1, Painter::red));*/

	//root->addChild(painter->getMultiplePointSep(_mesh, base->sample3to1, Painter::red));

	for (int i = 0; i < base->inter1_dijkstra.size(); i++){
		root->addChild(painter->drawLineSet(base->geo->myMesh, base->inter1_dijkstra[i], Painter::yellow));
		root->addChild(painter->drawLineSet(base->geo->myMesh, base->inter2_dijkstra[i], Painter::green));
	}

	int bak = 5;
	for (size_t i = 0; i < base->quads[bak].insideTriangleIds.size(); i++)
	{
		root->addChild(painter->get1PointSep(base->geo->myMesh, _mesh->tris[base->quads[bak].insideTriangleIds[i]]->v1i, Painter::purple));
		root->addChild(painter->get1PointSep(base->geo->myMesh, _mesh->tris[base->quads[bak].insideTriangleIds[i]]->v2i, Painter::purple));
		root->addChild(painter->get1PointSep(base->geo->myMesh, _mesh->tris[base->quads[bak].insideTriangleIds[i]]->v3i, Painter::purple));
	}
	

	root->addChild(painter->getShapeSep(m, Painter::grey));
	viewer->setSize(SbVec2s(1280, 960));
	viewer->setSceneGraph(root);
	viewer->show();
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();
}

void TrilateralPlain(){

	std::vector<char*> f_names;
	std::vector<std::array<int, 3>> seed;

	f_names.push_back("Data/person1.off");
	f_names.push_back("Data/person6.off");

	// Data/horse2.off
	//int a = 6071, b = 6710, c = 192;	// Sol karin
	//int a = 7070, b = 344, c = 1085;	// Sirt //failure
	//int a = 8539, b = 8093, c = 7919;	// Boyun
	//int a = 7731, b = 6461, c = 5978;	// Alt karin

	// Data/centaur.off
	//int a = 2331, b = 2169, c = 154;	// Sol karin
	//int a = 4794, b = 3946, c = 5632;	// Govde
	//int a = 57, b = 2431, c = 4008;	// Sag karin
	//int a = 2674, b = 2130, c = 2251;	// Alt karin

	// Data/hand.off
	//int a = 1223, b = 729, c = 1508; // ust kismi center = 1492
	//int a = 1536, b = 486, c = 551; // ust kismi center = 1492
	//int a = 2415, b = 1930, c = 650; // ust kismi center = 1492
	//int a = 1485, b = 1496, c = 837;
	//int a = 37, b = 2390, c = 1123;

	// Data/dog.off
	//int a = 869, b = 847, c = 440; //back

	// Data/person1.off
	//int a = 9694, b = 8981, c = 9470;	// Gogus
	//int a = 10072, b = 9245, c = 9275;	//Kol
	//int a = 10287, b = 10269, c = 7254; // Govde
	//int a = 12244, b = 10381, c = 9358; // Kafa
	//int a = 12244, b = 8372, c = 8720;  // Kafa
	//int a = 10201, b = 8522, c = 8527;	// Gogus center = 9449
	int a = 10433, b = 9092, c = 9322;
	seed.push_back({ { a, b, c } });
	a = 10425, b = 9092, c = 9299;
	seed.push_back({ { a, b, c } });

	
	
	for (size_t i = 0; i < f_names.size(); i++)
	{
		Mesh* m = new Mesh();
		m->loadOff(f_names[i]);
		int gridSize = 6;
		clock_t tClock1 = clock();
		Trilateral *base = new Trilateral(seed[i][0], seed[i][1], seed[i][2], gridSize, m, f_names[i]);
		base->init();
		std::cout << "\nExecution time for Trilateral: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;
	}
}
void ExtractTest(char ** argv){
	HWND window = SoWin::init(argv[0]);
	SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
	SoSeparator * root = new SoSeparator;
	root->ref();


	Mesh* _mesh = new Mesh();
	Mesh* m = new Mesh();
	Painter* painter = new Painter();
	char* filename = "Data/person1.off";

	// Data/horse2.off
	//int a = 6071, b = 6710, c = 192;	// Sol karin
	//int a = 7070, b = 344, c = 1085;	// Sirt //failure
	//int a = 8539, b = 8093, c = 7919;	// Boyun
	//int a = 7731, b = 6461, c = 5978;	// Alt karin

	// Data/centaur.off
	//int a = 2331, b = 2169, c = 154;	// Sol karin
	//int a = 4794, b = 3946, c = 5632;	// Govde
	//int a = 57, b = 2431, c = 4008;	// Sag karin
	//int a = 2674, b = 2130, c = 2251;	// Alt karin

	// Data/person1.off
	//int a = 9694, b = 8981, c = 9470;	// Gogus
	//int a = 10072, b = 9245, c = 9275;	//Kol
	//int a = 10287, b = 10269, c = 7254; // Govde
	//int a = 12244, b = 10381, c = 9358; // Kafa
	//int a = 12244, b = 8372, c = 8720;  // Kafa
	//int a = 10201, b = 8522, c = 8527;	// Gogus center = 9449

	// Data/hand.off
	//int a = 1223, b = 729, c = 1508; // ust kismi center = 1492
	//int a = 1536, b = 486, c = 551; // ust kismi center = 1492
	//int a = 2415, b = 1930, c = 650; // ust kismi center = 1492

	// Data/dog.off
	//int a = 869, b = 847, c = 440; //back

	int a = 9245, b = 9245, c = 9275;
	_mesh->loadOff(filename);
	int gridSize = 7;

	clock_t tClock1 = clock();
	Trilateral *base = new Trilateral(a, b, c, gridSize, _mesh, filename);
	base->betterThanInit();
	std::cout << "\nExecution time for Trilateral: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;

	/*root->addChild(painter->drawLineSet(base->mesh, base->i_path3to1, Painter::orange));
	root->addChild(painter->drawLineSet(base->mesh, base->i_path1to2, Painter::orange));
	root->addChild(painter->drawLineSet(base->mesh, base->i_path2to3, Painter::orange));*/

	root->addChild(painter->getMultiplePointSep(base->geo->myMesh, base->insideVerts, Painter::red));
	root->addChild(painter->getShapeSep(base->geo->myMesh, Painter::grey));
	viewer->setSize(SbVec2s(1280, 960));
	viewer->setSceneGraph(root);
	viewer->show();
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();
}