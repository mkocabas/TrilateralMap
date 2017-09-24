#pragma once
#include "Dijkstra.h"

Dijkstra::Dijkstra(Mesh* _mesh)
{
	this->mesh = _mesh;

	adjacency_list_t t(mesh->verts.size());
	this->adjacencyList = t; 

	//std::cout << "Dijkstra mesh size: " << mesh->verts.size() << endl;

	for (int i = 0; i < mesh->verts.size(); i++)
	{
		for (int j = 0; j < mesh->verts.at(i)->vertList.size(); j++)
		{
			//cout << mesh->verts.at(i)->vertList.size() << endl;
			int *idFirst = &mesh->verts.at(i)->vertList.at(j);
			int *idSecond = &mesh->verts.at(i)->idx;
			float len = sqrt(pow(mesh->verts[*idFirst]->coords[0] - mesh->verts[*idSecond]->coords[0], 2) + pow(mesh->verts[*idFirst]->coords[1] - mesh->verts[*idSecond]->coords[1], 2) + pow(mesh->verts[*idFirst]->coords[2] - mesh->verts[*idSecond]->coords[2], 2));
			adjacencyList[i].push_back(neighbor(*idFirst, len));
		}
	}
}


Dijkstra::~Dijkstra()
{
}


void Dijkstra::computePaths(vertex_t source,
	const adjacency_list_t &adjacency_list,
	std::vector<weight_t> &min_distance,
	std::vector<vertex_t> &previous)
{
	int n = adjacency_list.size();
	min_distance.clear();
	min_distance.resize(n, max_weight);
	min_distance[source] = 0;
	previous.clear();
	previous.resize(n, -1);
	std::set<std::pair<weight_t, vertex_t> > vertex_queue;
	vertex_queue.insert(std::make_pair(min_distance[source], source));
	while (!vertex_queue.empty())
	{
		weight_t dist = vertex_queue.begin()->first;
		vertex_t u = vertex_queue.begin()->second;
		vertex_queue.erase(vertex_queue.begin());

		const std::vector<neighbor> &neighbors = adjacency_list[u];
		for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
			neighbor_iter != neighbors.end();
			neighbor_iter++)
		{
			vertex_t v = neighbor_iter->target;
			weight_t weight = neighbor_iter->weight;
			weight_t distance_through_u = dist + weight;
			if (distance_through_u < min_distance[v]) {
				vertex_queue.erase(std::make_pair(min_distance[v], v));

				min_distance[v] = distance_through_u;
				previous[v] = u;
				vertex_queue.insert(std::make_pair(min_distance[v], v));
			}
		}
	}
}

void Dijkstra::computePathsMinHeap(vertex_t source,
	const adjacency_list_t &adjacency_list,
	std::vector<weight_t> &min_distance,
	std::vector<vertex_t> &previous)
{
	int n = adjacency_list.size();
	min_distance.clear();
	min_distance.resize(n, max_weight);
	min_distance[source] = 0;
	previous.clear();
	previous.resize(n, -1);
	std::priority_queue<weight_vertex_pair_t,
		std::vector<weight_vertex_pair_t>,
		std::greater<weight_vertex_pair_t> > vertex_queue;
	vertex_queue.push(std::make_pair(min_distance[source], source));


	while (!vertex_queue.empty())
	{
		weight_t dist = vertex_queue.top().first;
		vertex_t u = vertex_queue.top().second;
		vertex_queue.pop();
		if (dist > min_distance[u])
			continue;
		const std::vector<neighbor> &neighbors = adjacency_list[u];
		for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
			neighbor_iter != neighbors.end();
			neighbor_iter++)
		{
			vertex_t v = neighbor_iter->target;
			weight_t weight = neighbor_iter->weight;
			weight_t distance_through_u = dist + weight;
			if (distance_through_u < min_distance[v]) {
				min_distance[v] = distance_through_u;
				previous[v] = u;
				vertex_queue.push(std::make_pair(min_distance[v], v));

			}

		}
	}
}

void Dijkstra::computePathsFibonacciHeap(vertex_t source,
	const adjacency_list_t &adjacency_list,
	std::vector<weight_t> &min_distance,
	std::vector<vertex_t> &previous)
{
	int n = adjacency_list.size();
	min_distance.clear();
	min_distance.resize(n, max_weight);
 	min_distance[source] = 0;
	previous.clear();
	previous.resize(n, -1);

	FibHeap<weight_vertex_pair_t> fh;
	//FibQueue<weight_vertex_pair_t> fh;
	fh.push(std::make_pair(min_distance[source], source));

	while (!fh.empty())
	{

		weight_t dist = fh.top().first;
		vertex_t u = fh.top().second;

		fh.pop();
		if (dist > min_distance[u])
			continue;
		const std::vector<neighbor> &neighbors = adjacency_list[u];
		for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
			neighbor_iter != neighbors.end();
			neighbor_iter++)
		{
			vertex_t v = neighbor_iter->target;
			weight_t weight = neighbor_iter->weight;
			weight_t distance_through_u = dist + weight;
			if (distance_through_u < min_distance[v]) {
				min_distance[v] = distance_through_u;
				previous[v] = u;
				fh.push(std::make_pair(min_distance[v], v));
			}
		}
	}
}

std::list<Dijkstra::vertex_t> Dijkstra::getShortestPathTo(vertex_t vertex, const std::vector<vertex_t> &previous)
{
	std::list<vertex_t> path;
	for (; vertex != -1; vertex = previous[vertex])
		path.push_front(vertex);
	return path;
}

void Dijkstra::computeAllPathsOfMesh(Dijkstra::QType q, bool write){
	this->min_distance.clear();
	this->previous.clear();

	std::vector<std::vector<double>> distanceMatrix;
	cout << "Started to calculate the distance matrix" << endl;
	clock_t tStart = clock();
	for (int i = 0; i < this->mesh->verts.size(); i++)
	{
		switch (q)
		{
		case Dijkstra::ARRAY:
			computePaths(i, this->adjacencyList, this->min_distance, this->previous);
			break;
		case Dijkstra::MIN_HEAP:
			computePathsMinHeap(i, this->adjacencyList, this->min_distance, this->previous);
			break;
		case Dijkstra::FIBONACCI_HEAP:
			computePathsFibonacciHeap(i, this->adjacencyList, this->min_distance, this->previous);
			break;
		default:
			break;
		}
		distanceMatrix.push_back(min_distance);
	}
	cout << "Ended in: " << (double)(clock() - tStart) / CLOCKS_PER_SEC << " seconds." << endl;

	if (write){
		saveDistanceMatrix(distanceMatrix);
	}
}

std::vector<Dijkstra::vertex_t> Dijkstra::computeSinglePathInMesh(int firstVertexId, int secondVertexId){
	this->min_distance.clear();
	this->previous.clear();
	computePathsFibonacciHeap(firstVertexId, this->adjacencyList, this->min_distance, this->previous);
	//std::cout << "Distance from " << firstVertexId <<" to " << secondVertexId << ": " << min_distance[secondVertexId] << std::endl;
	std::list<vertex_t> path = getShortestPathTo(secondVertexId, previous);
	//std::cout << "Path : ";
	bilateralMapLength = min_distance[secondVertexId];
	//std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
	//std::cout << std::endl;

	std::vector<int> v{std::begin(path), std::end(path)};

	return v;
}

float Dijkstra::computeDistanceBetweenTwoVertex(int firstVertexId, int secondVertexId){
	this->min_distance.clear();
	this->previous.clear();
	computePathsFibonacciHeap(firstVertexId, this->adjacencyList, this->min_distance, this->previous);
	return min_distance[secondVertexId];
}

std::vector<Dijkstra::weight_t> Dijkstra::computeDistancesFrom(int sourceVertex){
	this->min_distance.clear();
	this->previous.clear();
	computePathsFibonacciHeap(sourceVertex, this->adjacencyList, this->min_distance, this->previous);
	return min_distance;
}

std::vector<Dijkstra::weight_t> Dijkstra::computeDistancesFromTiming(int sourceVertex, Dijkstra::QType q){
	this->min_distance.clear();
	this->previous.clear();

	switch (q)
	{
	case Dijkstra::ARRAY:
		computePaths(sourceVertex, this->adjacencyList, this->min_distance, this->previous);
		break;
	case Dijkstra::MIN_HEAP:
		computePathsMinHeap(sourceVertex, this->adjacencyList, this->min_distance, this->previous);
		break;
	case Dijkstra::FIBONACCI_HEAP:
		computePathsFibonacciHeap(sourceVertex, this->adjacencyList, this->min_distance, this->previous);
		break;
	default:
		break;
	}
	return min_distance;
}

void Dijkstra::printDistanceMatrix(std::vector<std::vector<double>> distanceMatrix){
	for (int i = 0; distanceMatrix.size(); i++)
	{
		cout << "(" << i << ") ->";
		for (int j = 0; j < distanceMatrix.at(i).size(); j++)
		{
			cout << distanceMatrix.at(i).at(j) << ", ";
		}
		cout << "*END*" << endl;
	}
}

void Dijkstra::saveDistanceMatrix(std::vector<std::vector<double>> distanceMatrix){
	ofstream output_file("distance_matrix.txt");
	ostream_iterator<double> output_iterator(output_file, " ");
	for (int i = 0; i < distanceMatrix.size(); i++){
		copy(distanceMatrix.at(i).begin(), distanceMatrix.at(i).end(), output_iterator);
		output_file << "" << endl;
	}
	output_file.close();
	cout << "File saved as distance_matrix.txt" << endl;
}


