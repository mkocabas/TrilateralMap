#pragma once
#include "Mesh.h"
#include <cmath>

void Mesh::loadOff(char* name)
{
	FILE* fPtr = fopen(name, "r");
	char str[334];

	fscanf(fPtr, "%s", str);

	int nVerts, nTris, n, i = 0;
	float x, y, z;

	fscanf(fPtr, "%d %d %d\n", &nVerts, &nTris, &n);
	while (i++ < nVerts)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addVertex(x, y, z);
	}

	while (fscanf(fPtr, "%d", &i) != EOF)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addTriangle((int)x, (int)y, (int)z);
	}

	fclose(fPtr);
}

void Mesh::loadMesh(std::vector<double> *vs, std::vector<int> *ts){
	for (int i = 0; i < vs->size(); i+=3)
		addVertex(vs->at(i), vs->at(i + 1), vs->at(i + 2));

	for (int i = 0; i < ts->size(); i += 3)
		addTriangle(ts->at(i), ts->at(i + 1), ts->at(i + 2));
}

void Mesh::loadOff(char* name, std::vector<double> *vs, std::vector<int> *ts)
{
	FILE* fPtr = fopen(name, "r");
	char str[334];

	fscanf(fPtr, "%s", str);

	int nVerts, nTris, n, i = 0;
	float x, y, z;

	fscanf(fPtr, "%d %d %d\n", &nVerts, &nTris, &n);
	while (i++ < nVerts)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addVertex(x, y, z);
		vs->push_back(x);
		vs->push_back(y);
		vs->push_back(z);
	}

	while (fscanf(fPtr, "%d", &i) != EOF)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addTriangle((int)x, (int)y, (int)z);
		ts->push_back((int)x);
		ts->push_back((int)y);
		ts->push_back((int)z);
	}

	fclose(fPtr);
}

void Mesh::loadArray(vector<array<double, 3>> vs, vector<array<int, 3>> tris)
{
	for (int i = 0; i < vs.size(); i++) addVertex(vs[i][0], vs[i][1], vs[i][2]);
	for (int i = 0; i < tris.size(); i++) addTriangle(tris[i][0], tris[i][1], tris[i][2]);
}

void Mesh::loadOff(char* name, SbVec3f tr)
{
	FILE* fPtr = fopen(name, "r");
	char str[334];

	fscanf(fPtr, "%s", str);

	int nVerts, nTris, n, i = 0;
	float x, y, z;

	fscanf(fPtr, "%d %d %d\n", &nVerts, &nTris, &n);
	while (i++ < nVerts)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addVertex(x+tr[0], y+tr[1], z+tr[2]);
	}

	while (fscanf(fPtr, "%d", &i) != EOF)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addTriangle((int)x, (int)y, (int)z);
	}

	fclose(fPtr);
}

void Mesh::loadObj(char* path){

	std::ifstream file(path);
	std::string   line;

	while (std::getline(file, line))
	{
		std::stringstream linestream(line);
		std::string data;
		float x, y, z;

		// If you have truly tab delimited data use getline() with third parameter.
		// If your data is just white space separated data
		// then the operator >> will do (it reads a space separated word into a string).
		std::getline(linestream, data, ' ');  // read up-to the first tab (discard tab).
		
		// Read the integers using the operator >>
		linestream >> x >> y >> z;

		char h = data.at(0);
		if (h == 'v'){
			//cout << "Yes" << endl;
			//cout << h << " " << x << " " << y << " " << z << endl;
			addVertex(x, y, z);
		}
		else if (h == 'f'){
			//cout << h << " " << x << " " << y << " " << z << endl;
			addTriangle((int)x, (int)y, (int)z);
		}
	}

	/*FILE * file = fopen(path, "r");

	float x, y, z;
	char lineHeader[512];
	int i = 0;
	while (fscanf(file, "%s", lineHeader) != EOF){
		if (strcmp(lineHeader, "v") == 0){
			fscanf(file, "%f %f %f\n", &x, &y, &z);
			
		}

		else if (strcmp(lineHeader, "f") == 0){
			fscanf(file, "%f %f %f\n", &x, &y, &z);
			addTriangle((int)x, (int)y, (int)z);
		}
		i++;
	} 
	fclose(file);*/
}

void Mesh::createCube(float sideLen)
{
	//coordinates
	float flbc[3] = { 0, 0, 0 }, deltaX = 0, deltaY = 0, deltaZ = 0;
	for (int v = 0; v < 8; v++)
	{
		switch (v)
		{
		case 1:
			deltaX = sideLen;
			break;
		case 2:
			deltaZ = -sideLen;
			break;
		case 3:
			deltaX = 0;
			break;
		case 4:
			deltaZ = 0;
			deltaY = sideLen;
			break;
		case 5:
			deltaX = sideLen;
			break;
		case 6:
			deltaZ = -sideLen;
			break;
		default:
			deltaX = 0;;
			break;
		}
		addVertex(flbc[0] + deltaX, flbc[1] + deltaY, flbc[2] + deltaZ);
	}

	addTriangle(0, 2, 1);
	addTriangle(0, 3, 2);

	addTriangle(1, 2, 5);
	addTriangle(2, 6, 5);

	addTriangle(2, 3, 6);
	addTriangle(3, 7, 6);

	addTriangle(3, 4, 7);
	addTriangle(3, 0, 4);

	addTriangle(4, 5, 6);
	addTriangle(4, 6, 7);

	addTriangle(0, 1, 5);
	addTriangle(0, 5, 4);
}

void Mesh::addTriangle(int v1, int v2, int v3)
{
	int idx = tris.size();
	tris.push_back(new Triangle(idx, v1, v2, v3));

	//set up structure

	verts[v1]->triList.push_back(idx);
	verts[v2]->triList.push_back(idx);
	verts[v3]->triList.push_back(idx);

	if (!makeVertsNeighbor(v1, v2))
		addEdge(v1, v2);

	if (!makeVertsNeighbor(v1, v3))
		addEdge(v1, v3);

	if (!makeVertsNeighbor(v2, v3))
		addEdge(v2, v3);
}

bool Mesh::makeVertsNeighbor(int v1i, int v2i)
{
	//returns true if v1i already neighbor w/ v2i; false o/w
	for (int i = 0; i < verts[v1i]->vertList.size(); i++)
		if (verts[v1i]->vertList[i] == v2i)
			return true;

	verts[v1i]->vertList.push_back(v2i);
	verts[v2i]->vertList.push_back(v1i);
	return false;
}

void Mesh::addVertex(float x, float y, float z)
{
	int idx = verts.size();
	float* c = new float[3];
	c[0] = x;
	c[1] = y;
	c[2] = z;

	verts.push_back(new Vertex(idx, c));
}

void Mesh::addEdge(int v1, int v2)
{
	int idx = edges.size();
	// Compute the length of edge (Euclidean Distance)
	//float length = sqrt(pow(verts[v1]->coords[0] - verts[v2]->coords[0], 2) + pow(verts[v1]->coords[1] - verts[v2]->coords[1], 2) + pow(verts[v1]->coords[2] - verts[v2]->coords[2], 2));
	
	edges.push_back(new Edge(idx, v1, v2/*, length*/));

	verts[v1]->edgeList.push_back(idx);
	verts[v2]->edgeList.push_back(idx);
}

float Mesh::computeDistanceBetweenTwoVertex(Mesh* mesh, int idFirst, int idSecond){
	return sqrt(pow(mesh->verts[idFirst]->coords[0] - mesh->verts[idSecond]->coords[0], 2) + pow(mesh->verts[idFirst]->coords[1] - mesh->verts[idSecond]->coords[1], 2) + pow(mesh->verts[idFirst]->coords[2] - mesh->verts[idSecond]->coords[2], 2));
}



/////////////////****************************************************//////////////////////////////////*********************************/////////////////////////////////////////***********************

