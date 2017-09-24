#pragma once
#ifndef TRILATERAL_H
#define TRILATERAL_H

#include <iostream>
#include <string>
#include <vector>

#include "Mesh.h"	
#include "ExactGeodesic.h"
#include "Dijkstra.h"

struct quad
{
	int v1q, v2q, v3q, v4q;
	std::vector<int> corners;
	std::vector<int> chunk1to2, chunk2to3, chunk3to4, chunk4to1;
	std::vector<std::vector<int>> chunks;
	std::vector<int> combinedChunks;
	std::vector<int> insideTriangleIds;
	bool isThereInsideVert;
	double area = 0.0;
};

struct tri
{
	int v1q, v2q, v3q;
	std::vector<int> corners;
	std::vector<int> chunk1to2, chunk2to3, chunk3to1;
	std::vector<std::vector<int>> chunks;
	std::vector<int> combinedChunks;
	std::vector<int> insideTriangleIds;
	bool isThereInsideVert;
	double area = 0.0;
};

class Trilateral
{
public:
	Trilateral(int v1, int v2, int v3, int gridSize, Mesh* m, char* filename);
	~Trilateral();

	typedef std::vector<std::pair<int, double>> path_t;
	typedef std::vector<int> verts_t;
	typedef std::vector<array<double, 3>> normal_t;
	typedef std::vector<int> i_vect;
	typedef std::vector<double> d_vect;

	Mesh* mesh;
	ExactGeodesic* geo;
	std::string f_name;
	Dijkstra* d;
	int v1, v2, v3, gridSize;
	//verts_t insideVerts;
	path_t path1to2, path2to3, path3to1;
	verts_t i_path1to2, i_path2to3, i_path3to1;
	verts_t sample1to2, sample2to3, sample3to1;
	verts_t insideVerts;
	verts_t insideTris;

	verts_t is;
	std::vector<std::vector<int>> intersections;

	std::vector<path_t> inter1, inter2;
	std::vector<std::vector<int>> inter1_dijkstra, inter2_dijkstra;

	normal_t vertNormals, triNormals;
	std::vector<quad> quads;
	std::vector<tri> tris;

	/* Trilateral meta calc */
	void init();
	void getSampleVertices(path_t *path, verts_t *samples);
	void fillInterPaths(verts_t *sample1, verts_t *sample2, std::vector<path_t> *inter);
	void fillInterPathsDijkstra(verts_t *sample1, verts_t *sample2, std::vector<verts_t> *inter_dijkstra);
	
	void ratherThanInit();
	int betterThanInit();
	/* Area & histogram calc */
	void findIntersections();
	void findChunks();
	void combineChunks();
	void q_runFloodFill(quad q, int in);
	void t_runFloodFill(tri q, int in);
	void getTriArea();
	void getQuadArea();
	void getAreaHistogram(bool print);

	std::vector<bool> quadVisited;
	std::vector<int> inQuad;
	quad active;
	std::vector<double> quadAreas;

	std::vector<bool> triVisited;
	std::vector<int> inTri;
	tri activeTri;
	std::vector<double> triAreas;

	std::vector<int> periphery;
	/* Flood fill algo for quads */	
	int q_findInsideVert(quad q);
	int q_findInsideVert_old(quad q);
	bool q_checkIn(quad q, std::vector<std::vector<double>> ds, int v);
	bool q_stopTest(int v);
	void q_floodFill(int v);
	void q_visit(int v);

	/* Flood fill algo for tris */
	int t_findInsideVert(tri q);
	bool t_stopTest(int v);
	void t_floodFill(int v);
	void t_visit(int v);

	/* Shared functions */
	double t_findTriangleArea(std::vector<int> inTris);
	std::vector<int> verts2triIds(std::vector<int> inQuad);
	std::vector<std::vector<double>> histogram;

	/* Flood fill algo */
	void floodFill(int v);
	int findInsideVert();
	void fillPaths2Inside(verts_t *p);
	void visit(int v);
	bool stopTest(int v);
	bool checkIn(d_vect distS1, d_vect distS2, d_vect distS3, int v);
	vector<bool> visited;
	
	

	/* Vertex normal calculations */
	float angleDegreeBetweenVectors(int p1, int p2);
	void getNormalVectors();	
};
#endif
	


