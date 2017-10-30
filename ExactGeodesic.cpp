#include "ExactGeodesic.h"


ExactGeodesic::ExactGeodesic(char* filename)
{
	myMesh = new Mesh();
	
	myMesh->loadOff(filename, &vs, &ts);
	
	bool success = geodesic::convert_from_my_mesh(vs, ts, points, faces);
	if (!success)
	{
		std::cout << "something is wrong with the input file" << std::endl;
	}

	mesh.initialize_mesh_data(points, faces);
}

ExactGeodesic::ExactGeodesic(std::vector<double> _vs, std::vector<int> _ts){

	this->vs = _vs;
	this->ts = _ts;
	bool a = geodesic::convert_from_my_mesh(vs, ts, points, faces);
	if (!a)
	{
		std::cout << "something is wrong with the input file" << std::endl;
	}
	
	
	this->success = mesh.initialize_mesh_data(points, faces);
}

ExactGeodesic::~ExactGeodesic()
{
}

void ExactGeodesic::computeSinglePath(int source_vertex_index, int target_vertex_index, std::vector<array<double, 3>> *path_vector, bool isMeshUpdate){
	geodesic::GeodesicAlgorithmExact algorithm(&mesh);
	geodesic::SurfacePoint source(&mesh.vertices()[source_vertex_index]);
	geodesic::SurfacePoint target(&mesh.vertices()[target_vertex_index]);

	std::vector<geodesic::SurfacePoint> path;
	algorithm.geodesic(source, target, path);

	//print_info_about_path(path);


	for (unsigned i = 0; i < path.size(); ++i)
	{
		array<double, 3> a;
		geodesic::SurfacePoint& s = path[i];

		geodesic::MeshElementBase *b = s.base_element();
		geodesic::PointType p = b->type();
		
		a[0] = s.x();
		a[1] = s.y();
		a[2] = s.z();

		path_vector->push_back(a);

	}
	if (isMeshUpdate)
		meshUpdate(&path);
}

void ExactGeodesic::computeSinglePath(int source_vertex_index, int target_vertex_index, std::vector<std::pair<int, double>> *distances){
	geodesic::GeodesicAlgorithmExact algorithm(&mesh);
	geodesic::SurfacePoint source(&mesh.vertices()[source_vertex_index]);
	geodesic::SurfacePoint target(&mesh.vertices()[target_vertex_index]);

	std::vector<geodesic::SurfacePoint> path;
	algorithm.geodesic(source, target, path);

	//print_info_about_path(path);

	meshUpdate(&path, distances);

	myMesh = new Mesh();
	myMesh->loadMesh(&vs, &ts);
	std::vector<double> points;
	std::vector<unsigned> faces;
	geodesic::convert_from_my_mesh(vs, ts, points, faces);
	mesh.initialize_mesh_data(points, faces);
}

void ExactGeodesic::meshUpdate(std::vector<geodesic::SurfacePoint> *path){

	int temp = -1;
	for (int i = 0; i < path->size() - 1; i++){
		geodesic::SurfacePoint& v1 = path->at(i);
		geodesic::SurfacePoint& v2 = path->at(i + 1);

		geodesic::MeshElementBase *b1 = v1.base_element();
		geodesic::MeshElementBase *b2 = v2.base_element();

		int type1 = b1->type();
		int type2 = b2->type();

		if (type1 + type2 == 2){ // Both edges are on the edge

			geodesic::SimpleVector<geodesic::vertex_pointer> e1 = b1->adjacent_vertices();
			geodesic::SimpleVector<geodesic::vertex_pointer> e2 = b2->adjacent_vertices();

			int e1_v1i = e1[0]->id();
			int e1_v2i = e1[1]->id();

			int e2_v1i = e2[0]->id();
			int e2_v2i = e2[1]->id();

			// Locate each vertex for given two edges
			int sharedVertex = -1;
			int neighborVertex = -1;
			int otherVertex = -1;

			if (e1_v1i == e2_v1i) 
			{
				sharedVertex = e1_v1i;
				neighborVertex = e1_v2i;
				otherVertex = e2_v2i;
			}
			else if (e1_v2i == e2_v2i)
			{
				sharedVertex = e1_v2i;
				neighborVertex = e1_v1i;
				otherVertex = e2_v1i;
			}
			else if (e1_v1i == e2_v2i)
			{
				sharedVertex = e1_v1i;
				neighborVertex = e1_v2i;
				otherVertex = e2_v1i;
			}
			else if (e1_v2i == e2_v1i)
			{
				sharedVertex = e1_v2i;
				neighborVertex = e1_v1i;
				otherVertex = e2_v2i;
			}

			// Find the triangle Id
			int triId = -1;

			for (int i = 0; i < myMesh->verts[sharedVertex]->triList.size(); i++)
			{
				int sT = myMesh->verts[sharedVertex]->triList[i];
				for (int j = 0; j < myMesh->verts[neighborVertex]->triList.size(); j++)
				{
					int nT = myMesh->verts[neighborVertex]->triList[j];
					for (int k = 0; k < myMesh->verts[otherVertex]->triList.size(); k++)
					{
						int oT = myMesh->verts[otherVertex]->triList[k];
						if (sT == nT && nT == oT)
						{
							triId = sT;
						}
					}
				}
			}

			// Add new vertices

			/*vs.push_back(v1.x());
			vs.push_back(v1.y());
			vs.push_back(v1.z());*/
			vs.push_back(v2.x());
			vs.push_back(v2.y());
			vs.push_back(v2.z());

			int v1index = vs.size() / 3 - 2;
			int v2index = vs.size() / 3 - 1;

			t.push_back(v2index);

			// Add upper splitted triangle
			ts[triId * 3] = sharedVertex;
			ts[triId * 3 + 1] = v1index;
			ts[triId * 3 + 2] = v2index;
			
			// Add other triangles
			ts.push_back(v1index);
			ts.push_back(neighborVertex);
			ts.push_back(otherVertex);

			ts.push_back(v2index);
			ts.push_back(v1index);
			ts.push_back(otherVertex);
		}
		else if (type1 + type2 == 1)
		{
			geodesic::SimpleVector<geodesic::vertex_pointer> e;
			if (type1 == 1){
				 e = b1->adjacent_vertices();
				 int e_v1 = e[0]->id();
				 int e_v2 = e[1]->id();
				 int peak = b2->id();

				 int triId = -1;

				 for (int i = 0; i < myMesh->verts[peak]->triList.size(); i++)
				 {
					 int sT = myMesh->verts[peak]->triList[i];
					 for (int j = 0; j < myMesh->verts[e_v1]->triList.size(); j++)
					 {
						 int nT = myMesh->verts[e_v1]->triList[j];
						 for (int k = 0; k < myMesh->verts[e_v2]->triList.size(); k++)
						 {
							 int oT = myMesh->verts[e_v2]->triList[k];
							 if (sT == nT && nT == oT)
							 {
								 triId = sT;
							 }
						 }
					 }
				 }

				 /*vs.push_back(v1.x());
				 vs.push_back(v1.y());
				 vs.push_back(v1.z());*/

				 ts[triId * 3] = peak;
				 ts[triId * 3 + 1] = e_v1;
				 ts[triId * 3 + 2] = vs.size() / 3 - 1;

				 ts.push_back(peak);
				 ts.push_back(vs.size() / 3 - 1);
				 ts.push_back(e_v2);
			}
			else if (type2 == 1){
				e = b2->adjacent_vertices();
				int e_v1 = e[0]->id();
				int e_v2 = e[1]->id();
				int peak = b1->id();

				int triId = -1;

				for (int i = 0; i < myMesh->verts[peak]->triList.size(); i++)
				{
					int sT = myMesh->verts[peak]->triList[i];
					for (int j = 0; j < myMesh->verts[e_v1]->triList.size(); j++)
					{
						int nT = myMesh->verts[e_v1]->triList[j];
						for (int k = 0; k < myMesh->verts[e_v2]->triList.size(); k++)
						{
							int oT = myMesh->verts[e_v2]->triList[k];
							if (sT == nT && nT == oT)
							{
								triId = sT;
							}
						}
					}
				}


				vs.push_back(v2.x());
				vs.push_back(v2.y());
				vs.push_back(v2.z());

				t.push_back(vs.size() / 3 - 1);

				ts[triId * 3] = peak;
				ts[triId * 3 + 1] = e_v1;
				ts[triId * 3 + 2] = vs.size() / 3 - 1;

				ts.push_back(peak);
				ts.push_back(vs.size() / 3 - 1);
				ts.push_back(e_v2);
			}
		}
		else if (type1 + type2 == 0)
		{
			//std::cout << "Not on edge" << endl;
		}
		temp = b2->id();
	}
}

void ExactGeodesic::meshUpdate(std::vector<geodesic::SurfacePoint> *path, std::vector<std::pair<int, double>> *distances){

	double dist = 0.0;
	for (int i = 0; i < path->size() - 1; i++){
		geodesic::SurfacePoint& v1 = path->at(i);
		geodesic::SurfacePoint& v2 = path->at(i + 1);

		geodesic::MeshElementBase *b1 = v1.base_element();
		geodesic::MeshElementBase *b2 = v2.base_element();

		int type1 = b1->type();
		int type2 = b2->type();

		if (i == 0){
			distances->push_back(make_pair(b1->id(), dist));
		}
			

		if (type1 + type2 == 2){ // Both edges are on the edge

			geodesic::SimpleVector<geodesic::vertex_pointer> e1 = b1->adjacent_vertices();
			geodesic::SimpleVector<geodesic::vertex_pointer> e2 = b2->adjacent_vertices();

			int e1_v1i = e1[0]->id();
			int e1_v2i = e1[1]->id();

			int e2_v1i = e2[0]->id();
			int e2_v2i = e2[1]->id();

			// Locate each vertex for given two edges
			int sharedVertex = -1;
			int neighborVertex = -1;
			int otherVertex = -1;

			if (e1_v1i == e2_v1i)
			{
				sharedVertex = e1_v1i;
				neighborVertex = e1_v2i;
				otherVertex = e2_v2i;
			}
			else if (e1_v2i == e2_v2i)
			{
				sharedVertex = e1_v2i;
				neighborVertex = e1_v1i;
				otherVertex = e2_v1i;
			}
			else if (e1_v1i == e2_v2i)
			{
				sharedVertex = e1_v1i;
				neighborVertex = e1_v2i;
				otherVertex = e2_v1i;
			}
			else if (e1_v2i == e2_v1i)
			{
				sharedVertex = e1_v2i;
				neighborVertex = e1_v1i;
				otherVertex = e2_v2i;
			}

			// Find the triangle Id
			int triId = -1;

			for (int i = 0; i < myMesh->verts[sharedVertex]->triList.size(); i++)
			{
				int sT = myMesh->verts[sharedVertex]->triList[i];
				for (int j = 0; j < myMesh->verts[neighborVertex]->triList.size(); j++)
				{
					int nT = myMesh->verts[neighborVertex]->triList[j];
					for (int k = 0; k < myMesh->verts[otherVertex]->triList.size(); k++)
					{
						int oT = myMesh->verts[otherVertex]->triList[k];
						if (sT == nT && nT == oT)
						{
							triId = sT;
						}
					}
				}
			}

			// Add new vertices

			/*vs.push_back(v1.x());
			vs.push_back(v1.y());
			vs.push_back(v1.z());*/
			vs.push_back(v2.x());
			vs.push_back(v2.y());
			vs.push_back(v2.z());

			int v1index = vs.size() / 3 - 2;
			int v2index = vs.size() / 3 - 1;

			dist += v2.distance(&v1);
			distances->push_back(make_pair(v2index, dist));

			t.push_back(v2index);

			// Add upper splitted triangle
			ts[triId * 3] = sharedVertex;
			ts[triId * 3 + 1] = v1index;
			ts[triId * 3 + 2] = v2index;

			// Add other triangles
			ts.push_back(v1index);
			ts.push_back(neighborVertex);
			ts.push_back(otherVertex);

			ts.push_back(v2index);
			ts.push_back(v1index);
			ts.push_back(otherVertex);
		}
		else if (type1 + type2 == 1)
		{
			geodesic::SimpleVector<geodesic::vertex_pointer> e;
			if (type1 == 1){
				e = b1->adjacent_vertices();
				int e_v1 = e[0]->id();
				int e_v2 = e[1]->id();
				int peak = b2->id();

				dist += v2.distance(&v1);
				distances->push_back(make_pair(peak, dist));

				int triId = -1;

				for (int i = 0; i < myMesh->verts[peak]->triList.size(); i++)
				{
					int sT = myMesh->verts[peak]->triList[i];
					for (int j = 0; j < myMesh->verts[e_v1]->triList.size(); j++)
					{
						int nT = myMesh->verts[e_v1]->triList[j];
						for (int k = 0; k < myMesh->verts[e_v2]->triList.size(); k++)
						{
							int oT = myMesh->verts[e_v2]->triList[k];
							if (sT == nT && nT == oT)
							{
								triId = sT;
							}
						}
					}
				}

				/*vs.push_back(v1.x());
				vs.push_back(v1.y());
				vs.push_back(v1.z());*/

				ts[triId * 3] = peak;
				ts[triId * 3 + 1] = e_v1;
				ts[triId * 3 + 2] = vs.size() / 3 - 1;

				ts.push_back(peak);
				ts.push_back(vs.size() / 3 - 1);
				ts.push_back(e_v2);
			}
			else if (type2 == 1){
				e = b2->adjacent_vertices();
				int e_v1 = e[0]->id();
				int e_v2 = e[1]->id();
				int peak = b1->id();

				

				int triId = -1;

				for (int i = 0; i < myMesh->verts[peak]->triList.size(); i++)
				{
					int sT = myMesh->verts[peak]->triList[i];
					for (int j = 0; j < myMesh->verts[e_v1]->triList.size(); j++)
					{
						int nT = myMesh->verts[e_v1]->triList[j];
						for (int k = 0; k < myMesh->verts[e_v2]->triList.size(); k++)
						{
							int oT = myMesh->verts[e_v2]->triList[k];
							if (sT == nT && nT == oT)
							{
								triId = sT;
							}
						}
					}
				}


				vs.push_back(v2.x());
				vs.push_back(v2.y());
				vs.push_back(v2.z());

				dist += v2.distance(&v1);
				distances->push_back(make_pair(vs.size() / 3 - 1, dist));

				t.push_back(vs.size() / 3 - 1);

				ts[triId * 3] = peak;
				ts[triId * 3 + 1] = e_v1;
				ts[triId * 3 + 2] = vs.size() / 3 - 1;

				ts.push_back(peak);
				ts.push_back(vs.size() / 3 - 1);
				ts.push_back(e_v2);
			}
		}
		else if (type1 + type2 == 0)
		{
			dist += v2.distance(&v1);
			distances->push_back(make_pair(b2->id(), dist));

			//std::cout << "Not on edge" << endl;
		}
	}
}

void ExactGeodesic::computeDistancesFrom(int source_vertex_index, std::vector<double> *dists){
	geodesic::GeodesicAlgorithmExact algorithm(&mesh);
	geodesic::SurfacePoint source(&mesh.vertices()[source_vertex_index]);
	std::vector<geodesic::SurfacePoint> all_sources(1, source);

	algorithm.propagate(all_sources);

	for (unsigned i = 0; i < mesh.vertices().size(); ++i)
	{
		geodesic::SurfacePoint p(&mesh.vertices()[i]);

		double distance;
		unsigned best_source = algorithm.best_source(p, distance);		//for a given surface point, find closets source and distance to this source

		dists->push_back(distance);
	}
}

void ExactGeodesic::print_distance_info(std::vector<std::pair<int, double>> *dist){
	std::cout << "-------\n# of vertices: " << dist->size() << endl;
	for (int i = 0; i < dist->size(); i++)
	{
		std::cout << "id: " << dist->at(i).first << " distance: " << dist->at(i).second << endl;
	}
	std::cout << "-------\n";
}

void ExactGeodesic::print_path_info(std::vector<std::pair<int, double>> *dist){
//	std::cout << "# of vertices: " << dist->size() << endl;
	for (int i = 0; i < dist->size(); i++)
		std::cout << "id: " << dist->at(i).first << " ";
	std::cout << "\n";
}

void ExactGeodesic::reinitializeMesh(std::vector<double> t_vs, std::vector<int> t_ts){
	myMesh = new Mesh();
	myMesh->loadMesh(&t_vs, &t_ts);
	
	geodesic::Mesh t_mesh;
	std::vector<double> t_points;
	std::vector<unsigned> t_faces;
	geodesic::convert_from_my_mesh(t_vs, t_ts, t_points, t_faces);
	t_mesh.initialize_mesh_data(t_points, t_faces);
	this->mesh = t_mesh;
}
/*void ExactGeodesic::computeMultiplePaths(int a, int b, int c, std::vector<array<double, 3>> *paths_vector){
	geodesic::GeodesicAlgorithmExact algorithm(&mesh);

	std::vector<geodesic::SurfacePoint> sources;
	sources.push_back(geodesic::SurfacePoint(&mesh.vertices()[a]));
	sources.push_back(geodesic::SurfacePoint(&mesh.vertices()[b]));
	sources.push_back(geodesic::SurfacePoint(&mesh.vertices()[c]));

	std::vector<geodesic::SurfacePoint> destinations;
	destinations.push_back(geodesic::SurfacePoint(&mesh.vertices()[a]));
	destinations.push_back(geodesic::SurfacePoint(&mesh.vertices()[b]));
	destinations.push_back(geodesic::SurfacePoint(&mesh.vertices()[c]));

	std::vector<geodesic::SurfacePoint> path;
	algorithm.geodesic(source, target, path);

	print_info_about_path(path);

	for (unsigned i = 0; i < path.size(); ++i)
	{
		array<double, 3> a;
		geodesic::SurfacePoint& s = path[i];
		a[0] = s.x();
		a[1] = s.y();
		a[2] = s.z();

		path_vector->push_back(a);
	}
}*/

/*ExactGeodesic::ExactGeodesic(Mesh* _mesh)
{
Mesh* _mesh = new Mesh();
std::vector<double> vs;
std::vector<int> ts;

for (int i = 0; i < _mesh->verts.size(); i++){
vs.push_back(_mesh->verts[i]->coords[0]);
vs.push_back(_mesh->verts[i]->coords[1]);
vs.push_back(_mesh->verts[i]->coords[2]);
}

for (int j = 0; j < _mesh->tris.size(); j++){
ts.push_back(_mesh->tris[j]->v1i);
ts.push_back(_mesh->tris[j]->v2i);
ts.push_back(_mesh->tris[j]->v3i);
}

bool success = geodesic::convert_from_my_mesh(vs, ts, points, faces);

if (!success)
{
std::cout << "something is wrong with the input file" << std::endl;
}

mesh.initialize_mesh_data(points, faces);

//algorithm = algo;
}*/