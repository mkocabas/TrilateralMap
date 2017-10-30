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
void TrilateralPlain(char ** argv);
void ExtractTest(char ** argv);
void UniformSampling(char ** argv);
void TrilateralIsomorphicMatching(char ** argv);
struct tuple3 {
	int a, b, c;
	tuple3(int a, int b, int c) : a(a), b(b), c(c) {}
};

std::vector<tuple3> combinations3(vector<int> n) {
	std::vector<tuple3> ret;
	for (std::vector<int>::const_iterator it1 = n.begin(); it1 < n.end(); it1++) {
		for (std::vector<int>::const_iterator it2 = n.begin(); it2 < it1; it2++) {
			for (std::vector<int>::const_iterator it3 = n.begin(); it3 < it2; it3++) {
				ret.push_back(tuple3(*it1, *it2, *it3));
			}
		}
	}
	return ret;
}

int firstElement(std::pair<int, double> &p){
	return p.first;
}

struct Sample
{
	int id;
	vector<int> men;
	vector<double> menDists;
	vector<double> dists;
	Sample(int i, Dijkstra* di){
		id = i;
		dists = di->computeDistancesFrom(i);
	}
};

std::vector<int> farthestPointSampling(Mesh* m, int numSample){
	int min = 0, max = m->verts.size();
	int first = min + (rand() % (int)(max - min + 1));

	std::vector<int> sampleIds;
	std::vector<Sample> samples;


	Dijkstra* a = new Dijkstra(m);
	Sample *s = new Sample(first, a);
	std::vector<bool> visited(s->dists.size(), false);
	visited.at(first) = true;
	samples.push_back(*s);
	sampleIds.push_back(first);

	int newCandidate = std::distance(s->dists.begin(), std::max_element(s->dists.begin(), s->dists.end()));

	for (int i = 0; i < numSample - 1; i++)
	{
		Dijkstra* die = new Dijkstra(m);
		Sample *f = new Sample(newCandidate, die);
		visited.at(newCandidate) = true;
		samples.push_back(*f);
		sampleIds.push_back(newCandidate);

		for (int k = 0; k < s->dists.size(); k++)
		{
			std::vector<double> len;
			for (int j = 0; j < sampleIds.size(); j++)
				len.push_back(samples.at(j).dists.at(k));
			int minIndex = std::distance(len.begin(), std::min_element(len.begin(), len.end()));
			samples.at(minIndex).men.push_back(k);
			samples.at(minIndex).menDists.push_back(samples.at(minIndex).dists.at(k));

		}

		std::vector<double> candid;
		std::vector<int> candidId;

		for (int t = 0; t < sampleIds.size(); t++)
		{
			double c = *std::max_element(samples.at(t).menDists.begin(), samples.at(t).menDists.end());
			candid.push_back(c);
			int cId = std::distance(samples.at(t).menDists.begin(), std::max_element(samples.at(t).menDists.begin(), samples.at(t).menDists.end()));
			candidId.push_back(samples.at(t).men.at(cId));
		}

		int loc = std::distance(candid.begin(), std::max_element(candid.begin(), candid.end()));
		newCandidate = candidId.at(loc);

		for (int z = 0; z < samples.size(); z++)
		{
			samples.at(z).men.clear();
			samples.at(z).menDists.clear();
		}
	}

	return sampleIds;
}

std::vector<int> getIntVector(std::vector<std::pair<int, double>> *p){
	std::vector<int> pts;
	std::transform(p->begin(), p->end(), std::back_inserter(pts), firstElement);
	return pts;
}

int main(int, char ** argv){

	/*SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
	_set_abort_behavior(0, _WRITE_ABORT_MS*/

	int selection = atoi(argv[1]);

	switch (selection)
	{
	case 1:
		UniformSampling(argv);
		break;
	case 2:
		TrilateralPlain(argv);
		break;
	case 3:
		TrilateralIsomorphicMatching(argv);
		break;
	case 4:
		showMesh(argv);
		break;
	case 5:
		TrilateralTest(argv);
		break;
	default:
		break;
	}
	return 0;
}

void UniformSampling(char ** argv){

	char* meshFilename = argv[2];
	char* numSamplesChar = argv[3];

	Mesh* mesh = new Mesh();
	mesh->loadOff("Data/person1.off");
	Dijkstra *d = new Dijkstra(mesh);
	
	int numSamples = atoi(numSamplesChar);

	clock_t tClock1 = clock();
	std::vector<int> samples = farthestPointSampling(mesh, numSamples);
	std::cout << "\nExecution time for sampling: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;

	std::vector<tuple3> combinations = combinations3(samples);

	std::cout << combinations.size() << endl;

	std::string f = meshFilename;
	std::string fname = "" + f + ".txt";

	std::ofstream file;
	file.open(fname, std::ofstream::out);
	if (file.is_open()){
		for (size_t i = 0; i < combinations.size(); i++)
		{
			std::string out = std::to_string(combinations[i].a) + " " 
							+ std::to_string(combinations[i].b) + " " 
							+ std::to_string(combinations[i].c) + "\n";

			file << out;
		}
		file.close();
		std::cout << "Succesfully written." << endl;
	}
	else{
		cout << "Unable to open file";
	}

	HWND window = SoWin::init(argv[0]);
	SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
	SoSeparator * root = new SoSeparator;
	root->ref();
	Painter* painter = new Painter();
	root->addChild(painter->getMultiplePointSep(mesh, samples, Painter::red));
	//root->addChild(painter->getShapeSep(mesh, Painter::white));

	for (size_t i = 0; i <samples.size(); i++)
	{
		SoSeparator *africaSep = new SoSeparator;
		SoTranslation *africaTranslate = new SoTranslation;
		SoText2 *africaText = new SoText2;
		africaTranslate->translation.setValue(mesh->verts[samples[i]]->coords[0], 
			mesh->verts[samples[i]]->coords[1], 
			mesh->verts[samples[i]]->coords[2]);
		africaText->string = samples[i];
		root->addChild(africaSep);
		africaSep->addChild(africaTranslate);
		africaSep->addChild(africaText);
	}

	viewer->setSize(SbVec2s(1280, 960));
	viewer->setSceneGraph(root);
	viewer->show();
	viewer->setBackgroundColor(SbColor(0, 0, 0));
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();

}

void showMesh(char ** argv){
	HWND window = SoWin::init(argv[0]);
	SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
	SoSeparator * root = new SoSeparator;
	root->ref();
	Mesh* mesh = new Mesh();
	Painter* painter = new Painter();
	
	
	char* filename = argv[2];

	mesh->loadOff(filename);

	int a = atoi(argv[3]);
	int b = atoi(argv[4]);
	int c = atoi(argv[5]);

	root->addChild(painter->get1PointSep(mesh, a, Painter::blue));
	root->addChild(painter->get1PointSep(mesh, b, Painter::red));
	root->addChild(painter->get1PointSep(mesh, c, Painter::yellow));

	ExactGeodesic *geo = new ExactGeodesic(filename);
	std::vector<std::pair<int, double>> d;
	std::vector<std::pair<int, double>> d1;
	std::vector<std::pair<int, double>> d2;
	
	geo->computeSinglePath(a, b, &d);
	geo->computeSinglePath(a, c, &d1);
	geo->computeSinglePath(c, b, &d2);

	root->addChild(painter->drawLineSet(geo->myMesh, getIntVector(&d), Painter::blue));
	root->addChild(painter->drawLineSet(geo->myMesh, getIntVector(&d1), Painter::blue));
	root->addChild(painter->drawLineSet(geo->myMesh, getIntVector(&d2), Painter::blue));

	root->addChild(painter->getShapeSep(geo->myMesh, Painter::white));
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
	Mesh* mesh = new Mesh();
	Painter* painter = new Painter();


	char* filename = argv[2];

	mesh->loadOff(filename);

	int a = atoi(argv[3]);
	int b = atoi(argv[4]);
	int c = atoi(argv[5]);

	root->addChild(painter->get1PointSep(mesh, a, Painter::blue));
	root->addChild(painter->get1PointSep(mesh, b, Painter::red));
	root->addChild(painter->get1PointSep(mesh, c, Painter::yellow));


	Trilateral *t = new Trilateral(5, filename);
	//std::vector<std::array<double,3>> vertNorm = t->getNormalVectors();
	int f = t->initialize(a, b, c);

	//std::cout << "Vayyy: "<< f << endl;
	
	/*root->addChild(painter->getMultiplePointSep(t->geo->myMesh, t->sample1to2));
	root->addChild(painter->getMultiplePointSep(t->geo->myMesh, t->sample2to3));
	root->addChild(painter->getMultiplePointSep(t->geo->myMesh, t->sample3to1));*/

	/*root->addChild(painter->drawLineSet(t->geo->myMesh, t->i_path1to2, Painter::red));
	root->addChild(painter->drawLineSet(t->geo->myMesh, t->i_path2to3, Painter::red));
	root->addChild(painter->drawLineSet(t->geo->myMesh, t->i_path3to1, Painter::red));*/


	root->addChild(painter->getShapeSep(mesh, Painter::grey));
	viewer->setSize(SbVec2s(1280, 960));
	viewer->setSceneGraph(root);
	viewer->show();
	viewer->setBackgroundColor(SbColor(0, 0, 0));
	SoWin::show(window);
	SoWin::mainLoop();
	delete viewer;
	root->unref();
}

void TrilateralPlain(char ** argv){

	// Parse arguments
	char* meshFname = argv[2];
	int grid = atoi(argv[3]);
	int a = atoi(argv[4]);
	int b = atoi(argv[5]);
	int c = atoi(argv[6]);

	std::cout << a << " " << b << " " << c << endl;

	// Initialize
	Trilateral *t = new Trilateral(grid, meshFname);
	std::vector<array<double, 3>> vertNorms = t->getNormalVectors();
	t->vertNormals = vertNorms;
	int r = t->initialize(a, b, c);
	
}

void ExtractTest(char ** argv){

	char* filename = "Data/horse2.off";

	// Data/horse2.off
	//int a = 6071, b = 6710, c = 192;	// Sol karin
	//int a = 7070, b = 344, c = 1085;	// Sirt
	//int a = 8539, b = 8093, c = 7919;	// Boyun /////////////////////////////// flood fill
	int a = 7731, b = 6461, c = 5978;	// Alt karin

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
	//int a = 12244, b = 8372, c = 8720;  // Kafa ////////////////////////////// flood fill
	//int a = 10201, b = 8522, c = 8527;	// Gogus center = 9449

	// Data/hand.off
	//int a = 1223, b = 729, c = 1508; // ust kismi center = 1492
	//int a = 1536, b = 486, c = 551; // ust kismi center = 1492
	//int a = 2415, b = 1930, c = 650; // ust kismi center = 1492

	// Data/dog.off
	//int a = 869, b = 847, c = 440; //back ///////////////////////////////// start assertion

	int gridSize = 5;

	clock_t tClock1 = clock();

	Trilateral *base = new Trilateral(gridSize, filename);
	int out = base->initialize(a, b, c);
	
	std::cout << "\nExecution time for Trilateral: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;
}

void TrilateralIsomorphicMatching(char ** argv){
	
	// Parse arguments
	char* meshFname = argv[2];
	int grid = atoi(argv[3]);
	std::string fname = argv[4];
	int start = atoi(argv[5]);

	std::cout << meshFname << endl;
	std::cout << grid << endl;
	std::cout << fname << endl;
	std::cout << start << endl;

	/*char* meshFname = "Data/person6.off";
	int grid = 5;
	std::string fname = "Data/person6.off.txt";
	int start = 0;*/
	

	// Read combinations from file
	std::ifstream file(fname);
	std::string   line;
	std::vector<tuple3> combinations;

	while (std::getline(file, line))
	{
		std::stringstream linestream(line);
		int a, b, c;
		linestream >> a >> b >> c;
		combinations.push_back(tuple3(a, b, c));
	}
	std::cout << "Number of combinations: " << combinations.size() << endl;
	std::cout << "Start computing histograms" << endl;

	std::vector<tuple3> missedTuples;
	std::vector<tuple3> processedTuples;

	Trilateral *t = new Trilateral(grid, meshFname);

	std::vector<array<double, 3>> vertNorms = t->getNormalVectors();

	for (size_t i = start; i < combinations.size(); i++)
	{

		std::ofstream status_log_file;
		status_log_file.open("status-log.txt", std::ofstream::out);
		status_log_file << std::to_string(i);
		status_log_file.close();


		clock_t tClock1 = clock();

		Trilateral *t = new Trilateral(grid, meshFname);
		int a = combinations[i].a, b = combinations[i].b, c = combinations[i].c;
		t->vertNormals = vertNorms;
		std::cout << i << "/" << combinations.size() << " - " << a << " " << b << " " << c << endl;
		int r = t->initialize(a, b, c);

		std::cout << " time: " << (double)(clock() - tClock1) / CLOCKS_PER_SEC << endl;
		if (r < 0){
			std::cout << "Tuple missed: " << a << " " << b << " " << c << endl;
			missedTuples.push_back(tuple3(a, b, c));
		}
		else{
			processedTuples.push_back(tuple3(a, b, c));
		}
		//delete t;
	}
	
}

