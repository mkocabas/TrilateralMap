#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <fstream>
#include <string>
#include <sstream>
#include <Inventor\SbVec3f.h>

using namespace std;

struct Vertex
{
	float *coords, *normals; //3d coordinates etc
	int idx; //who am i; verts[idx]

	vector< int > vertList; //adj vvertices;
	vector< int > triList; 
	vector< int > edgeList; 
	
	Vertex(int i, float *c) : idx(i), coords(c) {};
};

struct Edge
{
	int idx; //edges[idx]
	int v1i, v2i; //endpnts
	float length;
	Edge(int id, int v1, int v2/*, float len*/) : idx(id), v1i(v1), v2i(v2)/*, length(len)*/ {};
};

struct Triangle
{
	int idx; //tris[idx]
	int v1i, v2i, v3i;
	Triangle(int id, int v1, int v2, int v3) : idx(id), v1i(v1), v2i(v2), v3i(v3) {};
};

class Mesh
{
private:
	//void addTriangle(int v1, int v2, int v3);
	//void addEdge(int v1, int v2);
	//void addVertex(float x, float y, float z);
	//bool makeVertsNeighbor(int v1i, int v2i);

public:
	vector< Vertex* > verts;
	vector< Triangle* > tris;
	vector< Edge* > edges; 
	Mesh() {} ;
	void createCube(float side);
	void loadOff(char* name);
	void loadOff(char* name, SbVec3f tr);
	void loadOff(char* name, std::vector<double> *vs, std::vector<int> *ts);
	void loadObj(char* name);
	
	void loadMesh(std::vector<double> *vs, std::vector<int> *ts);
	void loadArray(vector<array<double, 3>>, vector<array<int, 3>>);
	bool makeVertsNeighbor(int v1i, int v2i);
	void addTriangle(int v1, int v2, int v3);
	void addEdge(int v1, int v2);
	void addVertex(float x, float y, float z);
	static float computeDistanceBetweenTwoVertex(Mesh* mesh, int idFirst, int idSecond);
};
