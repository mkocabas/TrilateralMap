#pragma once
#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include <iostream>
#include <vector>
#include <string>
#include <list>
#include <limits>
#include <set>
#include <utility>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <time.h>
#include <queue>
#include <functional>
#include <fstream>
#include "Mesh.h"
#include "fiboheap.h"
#include <stdlib.h>
#include <assert.h>

using namespace std;

class Dijkstra
{
public:
	Dijkstra(Mesh* mesh);
	~Dijkstra();
	Mesh* mesh;
	typedef int vertex_t;
	typedef double weight_t;
	float bilateralMapLength;

	const weight_t max_weight = std::numeric_limits<double>::infinity();

	struct neighbor 
	{
		vertex_t target;
		weight_t weight;
		neighbor(vertex_t arg_target, weight_t arg_weight)
			: target(arg_target), weight(arg_weight) { }
	};

	enum QType
	{
		ARRAY, MIN_HEAP, FIBONACCI_HEAP
	};

	typedef std::vector<std::vector<neighbor>> adjacency_list_t;
	typedef std::pair<weight_t, vertex_t> weight_vertex_pair_t; // Min Heap implementation
	
	adjacency_list_t adjacencyList;
	std::vector<weight_t> min_distance;
	std::vector<vertex_t> previous;
	void computePathsFibonacciHeap(vertex_t source, const adjacency_list_t &adjacency_list, std::vector<weight_t> &min_distance, std::vector<vertex_t> &previous);
	void computePathsMinHeap(vertex_t source, const adjacency_list_t &adjacency_list, std::vector<weight_t> &min_distance, std::vector<vertex_t> &previous);
	void computePaths(vertex_t source, const adjacency_list_t &adjacency_list, std::vector<weight_t> &min_distance, std::vector<vertex_t> &previous);
	std::list<vertex_t> getShortestPathTo(vertex_t vertex, const std::vector<vertex_t> &previous);
	void computeAllPathsOfMesh(QType q, bool b);
	std::vector<vertex_t> computeSinglePathInMesh(int firstVertexId, int secondVertexId);
	float computeDistanceBetweenTwoVertex(int firstVertexId, int secondVertexId);
	std::vector<weight_t> computeDistancesFrom(int sourceVertex);
	std::vector<weight_t> computeDistancesFromTiming(int sourceVertex, Dijkstra::QType q = Dijkstra::FIBONACCI_HEAP);
	void printDistanceMatrix(std::vector<std::vector<double>> distanceMatrix);
	void saveDistanceMatrix(std::vector<std::vector<double>> distanceMatrix);
	void boostDijkstra();
};
#endif

