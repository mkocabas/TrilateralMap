#pragma once
#ifndef EXACTGEODESIC_H
#define EXACTGEODESIC_H

#include "geodesic_algorithm_exact.h"
#include "Mesh.h"

class ExactGeodesic
{
public:
	ExactGeodesic(char* filename);
	ExactGeodesic(Mesh* _mesh);
	ExactGeodesic(std::vector<double> vs, std::vector<int> ts);
	~ExactGeodesic();
	
	geodesic::Mesh mesh;
	Mesh* myMesh;
	//geodesic::GeodesicAlgorithmExact algorithm;

	std::vector<double> points;
	std::vector<unsigned> faces;
	std::vector<double> vs;
	std::vector<int> ts;

	std::vector<int> t;


	void computeDistancesFrom(int sourceVertexId, std::vector<double> *dists);
	void computeSinglePath(int sourceVertexId, int targetVertexId, std::vector<array<double, 3>> *path, bool isMeshUpdate = true);
	void computeSinglePath(int sourceVertexId, int targetVertexId, std::vector<std::pair<int, double>> *distances);
	void computeMultiplePaths(int a, int b, int c, std::vector<array<double, 3>> *path);
	void meshUpdate(std::vector<geodesic::SurfacePoint> *path);
	void meshUpdate(std::vector<geodesic::SurfacePoint> *path, std::vector<std::pair<int, double>> *distances);
	void print_distance_info(std::vector<std::pair<int, double>> *dist);
	void print_path_info(std::vector<std::pair<int, double>> *dist);
	void reinitializeMesh(std::vector<double> vs, std::vector<int> ts);
};
#endif

