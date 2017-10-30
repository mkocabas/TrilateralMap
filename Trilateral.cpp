#include "Trilateral.h"
#include <direct.h>
#include <time.h>
#include <random>

int firstElement_(std::pair<int, double> &p){
	return p.first;
}

std::vector<int> getIntVector_(std::vector<std::pair<int, double>> *p){
	std::vector<int> pts;
	std::transform(p->begin(), p->end(), std::back_inserter(pts), firstElement_);
	return pts;
}

bool equal(double dFirstVal, double dSecondVal)
{
	return std::fabs(dFirstVal - dSecondVal) < 1E-3;
}

void print_vector(char* header, std::vector<int> v){
	std::cout << header;
	std::ostream_iterator<int> out_it(std::cout, ", ");
	std::copy(v.begin(), v.end(), out_it);
	std::cout << "\n" << endl;
}

void print_vector(char* header, std::vector<double> v){
	std::cout << header;
	std::ostream_iterator<double> out_it(std::cout, " ");
	std::copy(v.begin(), v.end(), out_it);
	std::cout << "\n" << endl;
}

std::vector<int> slice(std::vector<int> v, int start, int end) {
	//std::cout << "start-end: " << start << "-" << end << endl;
	assert(start != end);
	assert(start >= 0 && end >= 0 && start != end);

	if (start < 0 && end < 0 && start == end){
		std::vector<int> a;
		return a;
	}
	if (start > end){
		int temp = end;
		end = start;
		start = temp;
	}
	//std::cout << "start - end: " << start << "," << end << endl;
	std::vector<int> nv;

	for (int i = 0; i <= end - start; i++){
		nv.push_back(v[start + i]);
		//std::cout << "s: " << v[start + i] << endl;
	}		
	return nv;
}

void remove_duplicates(std::vector<int>& v)
{
	std::sort(v.begin(), v.end());
	auto last = std::unique(v.begin(), v.end());
	v.erase(last, v.end());
}

Trilateral::Trilateral(int _gridSize, char* filename)
{
	mesh = new Mesh;
	mesh->loadOff(filename);
	gridSize = _gridSize;
	geo = new ExactGeodesic(filename);
	fname = filename;
	//std::string str(filename);
	meshFilename = filename;
	
	histogramFilename = "logs/histogram-" + meshFilename.substr(5) + ".txt";
}

Trilateral::~Trilateral()
{
}

int Trilateral::initialize(int a, int b, int c){

	v1 = a;
	v2 = b;
	v3 = c;
	ov1 = a;
	ov2 = b;
	ov3 = c;

	//getNormalVectors();

	/*float d12, d23, d31;
	d12 = angleDegreeBetweenVectors(a, b);
	d23 = angleDegreeBetweenVectors(b, c);
	d31 = angleDegreeBetweenVectors(c, a);

	if (d12 > 120.0 || d23 > 120.0 || d31 > 120.0)
	{
		std::cout << "Vertices aren't appropriate" << endl;
		return -11;
	}*/

	int p1 = getBoundary();

	if (p1 <= 0){
		std::cout << "Boundary edges couldn't be created." << endl;
		return -1;		
	}
		

	int p2 = extractMesh();

	if (p2 <= 0){
		std::cout << "Inner mesh couldn't be created." << endl;
		return -2;
	}
		

	int p3 = constructInnerTris();

	if (p3 <= 0){
		std::cout << "Triangles couldn't be created." << endl;
		return -3;
	}
		
	int p4 = findIntersections();

	if (p4 <= 0)
		return -4;

	int p5 = findChunks();

	if (p5 <= 0)
		return -5;

	int p6 = combineChunks();

	if (p6 <= 0)
		return -6;

	int p7 = getQuadArea();

	if (p7 <= 0)
		return -7;

	int p8 = getTriArea();

	if (p8 <= 0)
		return -8;

	int p9 = getAreaHistogram();

	if (p9 <= 0){
		std::cout << "Histogram couldn't be calculated." << endl;
		return -9;
	}
		
	int p10 = print2File();
	
	if (p10 <= 0){
		std::cout << "Couldn't be written" << endl;
		return -10;
	}
	return 1;
}

int Trilateral::getBoundary(){
	if (v1 == v2 || v2 == v3 || v3 == v1)
		return -1; // If the vertices are same, break

	// find boundary vertices
	geo->computeSinglePath(v1, v2, &path1to2);
	geo->computeSinglePath(v2, v3, &path2to3);
	geo->computeSinglePath(v3, v1, &path3to1);
	mesh = geo->myMesh;

	// reverse them to read correctly
	std::reverse(path1to2.begin(), path1to2.end());
	std::reverse(path2to3.begin(), path2to3.end());
	std::reverse(path3to1.begin(), path3to1.end());

	if (path1to2.size() < gridSize || path2to3.size() < gridSize || path3to1.size() < gridSize)
		return -2; // If the paths' size are less than grid size, break

	// Get paths as int vectors
	verts_t p1to2, p2to3, p3to1;

	p1to2 = getIntVector_(&path1to2);
	p2to3 = getIntVector_(&path2to3);
	p3to1 = getIntVector_(&path3to1);

	/*print_vector("p1to2: ", p1to2);
	print_vector("p2to3: ", p2to3);
	print_vector("p3to1: ", p3to1);*/

	auto it = std::unique(p1to2.begin(), p1to2.end());
	bool wasUnique1to2 = (it == p1to2.end());

	auto it2 = std::unique(p2to3.begin(), p2to3.end());
	bool wasUnique2to3 = (it2 == p2to3.end());

	auto it3 = std::unique(p3to1.begin(), p3to1.end());
	bool wasUnique3to1 = (it3 == p3to1.end());

	std::cout << wasUnique1to2 << endl;
	std::cout << wasUnique2to3 << endl;
	std::cout << wasUnique3to1 << endl;

	if (wasUnique1to2 == false || wasUnique1to2 == false || wasUnique1to2 == false)
		return -3;

	i_path1to2 = p1to2;
	i_path2to3 = p2to3;
	i_path3to1 = p3to1;

	periphery.insert(periphery.begin(), p1to2.begin(), p1to2.end());
	periphery.insert(periphery.begin(), p2to3.begin(), p2to3.end());
	periphery.insert(periphery.begin(), p3to1.begin(), p3to1.end());

	remove_duplicates(periphery);

	return 1;
}

int Trilateral::extractMesh(){
	verts_t p1to2, p2to3, p3to1;
	p1to2 = getIntVector_(&path1to2);
	p2to3 = getIntVector_(&path2to3);
	p3to1 = getIntVector_(&path3to1);

	vector<bool> v(mesh->verts.size(), true);
	visited = v;

	int o = findInsideVert_new();

	if (o == periphery[0] || std::find(periphery.begin(), periphery.end(), o) != periphery.end())
		return -1;

	floodFill(o);

	if (recursionIteration == recursionLimit - 1)
		return -4;

	insideVerts.insert(insideVerts.begin(), periphery.begin(), periphery.end());
	//insideVerts.insert(insideVerts.begin(), p2to3.begin(), p2to3.end());
	//insideVerts.insert(insideVerts.begin(), p3to1.begin(), p3to1.end());

	remove_duplicates(insideVerts);

	std::vector<int> mTriangles = verts2triIds(insideVerts);

	//std::cout << "Inside size: " << insideVerts.size() << endl;
	if (insideVerts.size() < 1)
		return -2;
	//std::cout << "Triangles size: " << mTriangles.size() << endl;
	if (mTriangles.size() < 1)
		return -3;

	std::vector<int> old;
	std::vector<double> newVerts;
	std::vector<int> newTriangles;

	// Construct the new mesh
	for (int i = 0; i < mTriangles.size(); i++)
	{
		int v1i = mesh->tris[mTriangles[i]]->v1i;
		int v2i = mesh->tris[mTriangles[i]]->v2i;
		int v3i = mesh->tris[mTriangles[i]]->v3i;

		int nv1, nv2, nv3;
		if (std::find(old.begin(), old.end(), v1i) != old.end()){ nv1 = std::find(old.begin(), old.end(), v1i) - old.begin(); }
		else { old.push_back(v1i); nv1 = old.size() - 1; }

		if (std::find(old.begin(), old.end(), v2i) != old.end()){ nv2 = std::find(old.begin(), old.end(), v2i) - old.begin(); }
		else { old.push_back(v2i); nv2 = old.size() - 1; }

		if (std::find(old.begin(), old.end(), v3i) != old.end()){ nv3 = std::find(old.begin(), old.end(), v3i) - old.begin(); }
		else { old.push_back(v3i); nv3 = old.size() - 1; }

		newTriangles.push_back(nv1);
		newTriangles.push_back(nv2);
		newTriangles.push_back(nv3);
	}

	for (size_t i = 0; i < old.size(); i++)
	{
		newVerts.push_back(mesh->verts[old[i]]->coords[0]);
		newVerts.push_back(mesh->verts[old[i]]->coords[1]);
		newVerts.push_back(mesh->verts[old[i]]->coords[2]);
	}

	/////////////////////////////// Check there //////////////////////////////
	mesh = new Mesh();
	mesh->loadMesh(&newVerts, &newTriangles);

	geo = new ExactGeodesic(newVerts, newTriangles);

	bool success = geo->success;
	//std::cout << success << endl;
	if (success == 0){
		//std::cout << "hey" << endl;
		return -5;
	}
		

	geo->myMesh = new Mesh();
	geo->myMesh->loadMesh(&newVerts, &newTriangles);

	v1 = std::find(old.begin(), old.end(), v1) - old.begin();
	v2 = std::find(old.begin(), old.end(), v2) - old.begin();
	v3 = std::find(old.begin(), old.end(), v3) - old.begin();

	/*std::cout << v1 << endl;
	std::cout << v2 << endl;
	std::cout << v3 << endl;

	std::cout << mesh->verts.size() << endl;
	std::cout << mesh->tris.size() << endl;*/

	for (size_t i = 0; i < p1to2.size(); i++)
		p1to2[i] = std::find(old.begin(), old.end(), p1to2[i]) - old.begin();
	for (size_t i = 0; i < p2to3.size(); i++)
		p2to3[i] = std::find(old.begin(), old.end(), p2to3[i]) - old.begin();
	for (size_t i = 0; i < p3to1.size(); i++)
		p3to1[i] = std::find(old.begin(), old.end(), p3to1[i]) - old.begin();
	for (size_t i = 0; i < periphery.size(); i++)
		periphery[i] = std::find(old.begin(), old.end(), periphery[i]) - old.begin();

	i_path1to2 = p1to2;
	i_path2to3 = p2to3;
	i_path3to1 = p3to1;

	for (size_t i = 0; i < path1to2.size(); i++){
		path1to2[i].first = p1to2[i];
		path1to2[i].second = path1to2[i].second;
	}

	for (size_t i = 0; i < path2to3.size(); i++){
		path2to3[i].first = p2to3[i];
		path2to3[i].second = path2to3[i].second;
	}

	for (size_t i = 0; i < path3to1.size(); i++){
		path3to1[i].first = p3to1[i];
		path3to1[i].second = path3to1[i].second;
	}

	return 1;
}

int Trilateral::constructInnerTris()
{
	/* Samples on edges */
	getSampleVertices(&path1to2, &sample1to2);
	getSampleVertices(&path2to3, &sample2to3);
	getSampleVertices(&path3to1, &sample3to1);

	print_vector("sample1to2: ", sample1to2);
	print_vector("sample2to3: ", sample2to3);
	print_vector("sample3to1: ", sample3to1);
	std::cout << "sample verts done" << endl;

	/* Reverse sample vectors to properly find inter paths between them */
	/*std::reverse(sample1to2.begin(), sample1to2.end());
	std::reverse(sample2to3.begin(), sample2to3.end());
	std::reverse(sample3to1.begin(), sample3to1.end());*/

	auto it = std::unique(sample1to2.begin(), sample1to2.end());
	bool wasUnique1to2 = (it == sample1to2.end());

	auto it2 = std::unique(sample2to3.begin(), sample2to3.end());
	bool wasUnique2to3 = (it2 == sample2to3.end());

	auto it3 = std::unique(sample3to1.begin(), sample3to1.end());
	bool wasUnique3to1 = (it3 == sample3to1.end());

	/*std::cout << wasUnique1to2 << endl;
	std::cout << wasUnique2to3 << endl;
	std::cout << wasUnique3to1 << endl;
	*/
	if (wasUnique1to2 == false || wasUnique1to2 == false || wasUnique1to2 == false){
		std::cout << "Samples are inappropriate" << endl;
		return -4;
	}
		

	/*if (std::adjacent_find(sample1to2.begin(), sample1to2.end(), std::not_equal_to<int>()) == sample1to2.end())
	{
		std::cout << "All elements are equal each other" << std::endl;
		return -4;
	}
	if (std::adjacent_find(sample1to2.begin(), sample1to2.end(), std::not_equal_to<int>()) == sample1to2.end())
	{
		std::cout << "All elements are equal each other" << std::endl;
		return -4;
	}
	if (std::adjacent_find(sample1to2.begin(), sample1to2.end(), std::not_equal_to<int>()) == sample1to2.end())
	{
		std::cout << "All elements are equal each other" << std::endl;
		return -4;
	}*/
	//
	/* Find inter geodesics on that mesh */
	//std::cout << "\n1st connections:\n";
	fillInterPaths(&sample1to2, &sample2to3, &inter1);
	//std::cout << "\n2nd connections:\n";
	fillInterPaths(&sample1to2, &sample3to1, &inter2);
	//std::cout << "inter done" << endl;

	/* Restore correct paths */
	//std::cout << "Restoring correct paths" << endl;
	d = new Dijkstra(geo->myMesh);
	fillInterPathsDijkstra(&sample1to2, &sample2to3, &inter1_dijkstra);
	//std::cout << "\n2nd connections:\n";
	fillInterPathsDijkstra(&sample1to2, &sample3to1, &inter2_dijkstra);
	//std::cout << "inter done" << endl;

	std::reverse(inter1_dijkstra.begin(), inter1_dijkstra.end());
	std::reverse(inter2_dijkstra.begin(), inter2_dijkstra.end());

	/* Add corners to sample vectors */
	sample1to2.insert(sample1to2.begin(), v1);
	sample1to2.push_back(v2);

	sample2to3.insert(sample2to3.begin(), v2);
	sample2to3.push_back(v3);

	sample3to1.insert(sample3to1.begin(), v3);
	sample3to1.push_back(v1);

	mesh = geo->myMesh;

	return 1;
}

int Trilateral::getAreaHistogram(){
	int idx = 0;
	double sum = 0.0;
	for (auto& n : quadAreas)
		sum += n;

	for (auto& n : triAreas)
		sum += n;

	//std::cout << "Sum: " << sum << endl;

	for (size_t i = 0; i < gridSize; i++)
	{
		std::vector<double> layer;
		layer.push_back(triAreas[i]/sum);
		for (size_t j = 0; j < gridSize - i; j++)
		{
			layer.push_back(quadAreas[idx]/sum);
			idx++;
		}
		histogram.push_back(layer);
	}
	std::vector<double> layer;
	layer.push_back(triAreas[triAreas.size() - 1]/sum);
	histogram.push_back(layer);

	std::reverse(histogram.begin(), histogram.end());

	for (size_t i = 0; i < histogram.size(); i++)
		print_vector("", histogram[i]);

	return 1;
}

int Trilateral::print2File(){

	std::cout << "OK" << endl;
	mkdir("logs");

	/*for (size_t i = 0; i < histogram.size(); i++)
		print_vector("", histogram[i]);*/

	std::string out = std::to_string(ov1) + " " +
		std::to_string(ov2) + " " +
		std::to_string(ov3) + " " +
		std::to_string(gridSize) + " ";

	// Construct the string
	for (size_t i = 0; i < histogram.size(); i++)
	{
		for (size_t j = 0; j < histogram[i].size(); j++)
		{
			if (j == histogram[histogram.size() - 1].size() - 1)
				out.append(std::to_string(histogram[i][j]));
			else
				out.append(std::to_string(histogram[i][j]) + " ");
		}

		for (size_t t = 0; t < histogram[histogram.size() - 1].size() - histogram[i].size(); t++)
		{
			if (t == histogram[histogram.size() - 1].size() - 1)
				out.append("0.0");
			else
				out.append("0.0 ");
		}
	}
	
	// Write to file
	std::ofstream file;
	file.open(histogramFilename, std::ofstream::out | std::ofstream::app);

	if (file.is_open())
	{
		file.seekp(0, ios::end);
		size_t size = file.tellp();
		if (size == 0)
		{
			file << out;
		}
		else
		{
			file << "\n" + out;
		}
		
		file.close();
		return 1;
	}
	else{
		cout << "Unable to open file";
		return -1;
	}
}

int Trilateral::combineChunks(){
	for (size_t i = 0; i < quads.size(); i++)
	{
		for (size_t j = 0; j < quads[i].chunks.size(); j++)
			quads[i].combinedChunks.insert(quads[i].combinedChunks.begin(), quads[i].chunks[j].begin(), quads[i].chunks[j].end());
		remove_duplicates(quads[i].combinedChunks);
	}
	

	for (size_t i = 0; i < tris.size(); i++)
	{
		for (size_t j = 0; j < tris[i].chunks.size(); j++)
			tris[i].combinedChunks.insert(tris[i].combinedChunks.begin(), tris[i].chunks[j].begin(), tris[i].chunks[j].end());
		remove_duplicates(tris[i].combinedChunks);
	}

	return 1;
}

int Trilateral::getQuadArea(){
	for (size_t i = 0; i < quads.size(); i++)
	{
		quad q = quads[i];
		int in = q_findInsideVert(q);
		//std::cout << "YAS" << endl;
		/*if (in > -1)
			ins.push_back(in);*/
		inQuad.insert(inQuad.begin(), q.combinedChunks.begin(), q.combinedChunks.end());
		std::vector<int> tris;
		if (in > -1){
			quads[i].isThereInsideVert = true;
			q_runFloodFill(q, in);
		}			
		else {
			quads[i].isThereInsideVert = false;
		}
		//std::cout << "in size: " << inQuad.size() - q.combinedChunks.size() << endl;
		if (inQuad.size() > 0){
			tris = verts2triIds(inQuad);
			quads[i].insideTriangleIds = tris;
			quads[i].area = t_findTriangleArea(tris);
		}
		//std::cout << "Area: " << quads[i].area << endl;
		quadAreas.push_back(quads[i].area);
		inQuad.clear();
	}

	return 1;
}

void Trilateral::q_runFloodFill(quad q, int in){
	vector<bool> v(mesh->verts.size(), true);
	quadVisited = v;
	active = q;
	q_floodFill(in);
}

std::vector<int> Trilateral::verts2triIds(std::vector<int> v){
	std::vector<int> tris;
	for (size_t i = 0; i < mesh->tris.size(); i++)
	{
		int v1i = mesh->tris[i]->v1i;
		int v2i = mesh->tris[i]->v2i;
		int v3i = mesh->tris[i]->v3i;

		/*bool p1 = std::find(periphery.begin(), periphery.end(), v1i) != periphery.end();
		bool p2 = std::find(periphery.begin(), periphery.end(), v2i) != periphery.end();
		bool p3 = std::find(periphery.begin(), periphery.end(), v3i) != periphery.end();

		int p_condition = p1 + p2 + p3;

		if (p_condition == 2)
		{
			tris.push_back(mesh->tris[i]->idx);
		}*/

		bool e1 = std::find(v.begin(), v.end(), v1i) != v.end();
		bool e2 = std::find(v.begin(), v.end(), v2i) != v.end();
		bool e3 = std::find(v.begin(), v.end(), v3i) != v.end();

		int sum = e1 + e2 + e3;

		if (sum == 3){
			tris.push_back(mesh->tris[i]->idx);
		}
	}
	return tris;
}

double Trilateral::t_findTriangleArea(std::vector<int> inTris){
	double areaSum = 0.0;
	for (size_t i = 0; i < inTris.size(); i++)
	{
		int v1i = mesh->tris[inTris[i]]->v1i;
		int v2i = mesh->tris[inTris[i]]->v2i;
		int v3i = mesh->tris[inTris[i]]->v3i;
		
		double l1 = sqrt(pow(mesh->verts[v1i]->coords[0] - mesh->verts[v2i]->coords[0], 2) + pow(mesh->verts[v1i]->coords[1] - mesh->verts[v2i]->coords[1], 2) + pow(mesh->verts[v1i]->coords[2] - mesh->verts[v2i]->coords[2], 2));
		double l2 = sqrt(pow(mesh->verts[v2i]->coords[0] - mesh->verts[v3i]->coords[0], 2) + pow(mesh->verts[v2i]->coords[1] - mesh->verts[v3i]->coords[1], 2) + pow(mesh->verts[v2i]->coords[2] - mesh->verts[v3i]->coords[2], 2));
		double l3 = sqrt(pow(mesh->verts[v1i]->coords[0] - mesh->verts[v3i]->coords[0], 2) + pow(mesh->verts[v1i]->coords[1] - mesh->verts[v3i]->coords[1], 2) + pow(mesh->verts[v1i]->coords[2] - mesh->verts[v3i]->coords[2], 2));
		double u = (l1 + l2 + l3) / 2;
		areaSum += u;
	}
	return areaSum;
}

bool Trilateral::q_stopTest(int v){
	return (std::find(active.combinedChunks.begin(), active.combinedChunks.end(), v) != active.combinedChunks.end());
}

int Trilateral::q_findInsideVert(quad q){
	Dijkstra* d = new Dijkstra(geo->myMesh);
	verts_t p1 = d->computeSinglePathInMesh(q.v1q, q.v3q);
	verts_t p2 = d->computeSinglePathInMesh(q.v2q, q.v4q);
	verts_t p3 = d->computeSinglePathInMesh(q.chunk1to2[int(q.chunk1to2.size()/2)], q.chunk3to4[q.chunk3to4.size()/2]);
	verts_t p4 = d->computeSinglePathInMesh(q.chunk2to3[int(q.chunk2to3.size() / 2)], q.chunk4to1[q.chunk4to1.size() / 2]);

	p1.insert(p1.end(), p2.begin(), p2.end());
	p1.insert(p1.end(), p3.begin(), p3.end());
	p1.insert(p1.end(), p4.begin(), p4.end());
	int diff = -1;

	for (size_t i = 0; i < p1.size(); i++){
		if (std::find(q.combinedChunks.begin(), q.combinedChunks.end(), p1[i]) != q.combinedChunks.end()) { int c = 0; }
		else { diff = p1[i]; }
	}
	return diff;
}

int Trilateral::q_findInsideVert_old(quad q){
	Dijkstra *f = new Dijkstra(mesh);

	std::vector<std::vector<double>> ds;
	for (size_t i = 0; i < q.corners.size(); i++)
	{
		ds.push_back(f->computeDistancesFrom(q.corners[i]));
	}

	int cent = -1;
	for (int i = 0; i < insideVerts.size(); i++){
		if (q_checkIn(q, ds, i))
			cent = i;
	}
	return cent;
}

bool Trilateral::q_checkIn(quad q, std::vector<std::vector<double>> ds, int v){
	double d1to2 = ds[0][q.v2q];
	double d2to3 = ds[1][q.v3q];
	double d3to4 = ds[2][q.v4q];
	double d4to1 = ds[3][q.v1q];

	double s1 = ds[0][v];
	double s2 = ds[1][v];
	double s3 = ds[2][v];
	double s4 = ds[3][v];

	int sum = 0;

	if (s1 < d1to2 && s1 < d2to3 && s1 < d3to4 && s1 < d4to1) sum++;
	if (s2 < d1to2 && s2 < d2to3 && s2 < d3to4 && s2 < d4to1) sum++;
	if (s3 < d1to2 && s3 < d2to3 && s3 < d3to4 && s3 < d4to1) sum++;
	if (s4 < d1to2 && s4 < d2to3 && s4 < d3to4 && s4 < d4to1) sum++;

	if (sum == 4) { return true; }
	else { return false; }
}

void Trilateral::q_floodFill(int v){
	//std::cout << "bool: " << quadVisited[v] << t_stopTest(v)  << endl;
	if (quadVisited[v] && !q_stopTest(v)){
		q_visit(v);
		for (int i = 0; i < mesh->verts[v]->vertList.size(); i++)
			q_floodFill(mesh->verts[v]->vertList[i]);
	}
}

void Trilateral::q_visit(int v){
	//std::cout << "best" << endl;
	quadVisited[v] = false;
	inQuad.push_back(v);
}

int Trilateral::getTriArea(){
	for (size_t i = 0; i < tris.size(); i++)
	{
		tri q = tris[i];
		int in = t_findInsideVert(q);

		//std::cout << "in" << in << endl;

		inTri.insert(inTri.begin(), q.combinedChunks.begin(), q.combinedChunks.end());
		std::vector<int> triangles;
		if (in > -1){
			tris[i].isThereInsideVert = true;
			t_runFloodFill(q, in);
		}
		else {
			tris[i].isThereInsideVert = false;
		}
		//std::cout << "in size: " << inTri.size() - q.combinedChunks.size() << endl;
		if (inTri.size() > 0){
			triangles = verts2triIds(inTri);
			tris[i].insideTriangleIds = triangles;
			tris[i].area = t_findTriangleArea(triangles);
		}
		//std::cout << "Area: " << tris[i].area << endl;
		triAreas.push_back(tris[i].area);
		inTri.clear();
	}

	return 1;
}

void Trilateral::t_runFloodFill(tri q, int in){
	vector<bool> v(mesh->verts.size(), true);
	triVisited = v;
	activeTri = q;
	t_floodFill(in);
}

bool Trilateral::t_stopTest(int v){
	return (std::find(activeTri.combinedChunks.begin(), activeTri.combinedChunks.end(), v) != activeTri.combinedChunks.end());
}

int Trilateral::t_findInsideVert(tri q){
	Dijkstra* d = new Dijkstra(geo->myMesh);
	verts_t p1 = d->computeSinglePathInMesh(q.v1q, q.chunk2to3[int(q.chunk2to3.size() / 2)]);
	verts_t p2 = d->computeSinglePathInMesh(q.v2q, q.chunk3to1[int(q.chunk3to1.size() / 2)]);
	verts_t p3 = d->computeSinglePathInMesh(q.v3q, q.chunk1to2[int(q.chunk1to2.size() / 2)]);

	p1.insert(p1.end(), p2.begin(), p2.end());
	p1.insert(p1.end(), p3.begin(), p3.end());

	int diff = -1;

	for (size_t i = 0; i < p1.size(); i++){
		if (std::find(q.combinedChunks.begin(), q.combinedChunks.end(), p1[i]) != q.combinedChunks.end()) { int c = 0; }
		else { diff = p1[i]; }
	}
	return diff;
}

void Trilateral::t_floodFill(int v){
	//std::cout << "bool: " << quadVisited[v] << t_stopTest(v)  << endl;
	if (triVisited[v] && !t_stopTest(v)){
		t_visit(v);
		for (int i = 0; i < mesh->verts[v]->vertList.size(); i++)
			t_floodFill(mesh->verts[v]->vertList[i]);
	}
}

void Trilateral::t_visit(int v){
	//std::cout << "best" << endl;
	triVisited[v] = false;
	inTri.push_back(v);
}

int Trilateral::findIntersections(){
	intersections.push_back(sample2to3);
	//std::cout << "i2 size: " << inter2_dijkstra.size() << endl;
	//std::cout << "i1 size: " << inter1_dijkstra.size() << endl;

	for (int i = 0; i < inter2_dijkstra.size(); i++)
	{
		std::vector<int> layer;
		std::vector<int> tempPath2 = inter2_dijkstra[i];
		for (int j = 0; j < inter1_dijkstra.size(); j++)
		{
			std::vector<int> tempPath1 = inter1_dijkstra[j];

			std::sort(tempPath1.begin(), tempPath1.end());
			std::sort(tempPath2.begin(), tempPath2.end());

			std::vector<int> v_intersection;

			std::set_intersection(tempPath1.begin(), tempPath1.end(),
				tempPath2.begin(), tempPath2.end(),
				std::back_inserter(v_intersection));
			//assert(v_intersection.size() < 2);

			if (v_intersection.size() >= 2)
				return -1;
			//std::cout << v_intersection.size() << endl;//
			if (v_intersection.size() > 0){
				/*std::cout << "int: ";
				std::ostream_iterator<int> out_it(std::cout, ", ");
				std::copy(v_intersection.begin(), v_intersection.end(), out_it);
				std::cout << endl;*/
				layer.push_back(v_intersection[0]);
			}
			v_intersection.clear();
		}
		layer.push_back(sample3to1[i + 1]);
		intersections.push_back(layer);
		layer.clear();
		if (i == inter2_dijkstra.size() - 1){
			layer.push_back(v1);
			intersections.push_back(layer);
		}
	}

	return 1;
}

int Trilateral::findChunks(){
	/* Find chunks */
	inter1_dijkstra.push_back(getIntVector_(&path3to1));

	inter2_dijkstra.insert(inter2_dijkstra.begin(), getIntVector_(&path2to3));

	/*std::cout << "inter 1 size: " << inter1_dijkstra.size() << endl;
	std::cout << "inter 2 size: " << inter2_dijkstra.size() << endl;*/
	intersections.pop_back();
	//std::cout << "intersection size: " << intersections.size() << endl;
	// Walk over inter paths and slice them
	for (size_t i = 0; i < intersections.size() - 1; i++)
	{
		//std::cout << "buyuk resim: " << i << endl;
		std::vector<int> tempLayer = intersections[i];
		std::vector<int> upTempLayer = intersections[i + 1];

		std::vector<int> int2first = inter2_dijkstra[i];
		std::vector<int> int2second = inter2_dijkstra[i + 1];

		for (size_t j = 0; j < tempLayer.size() - 1; j++)
		{
			//std::cout << "tebaa: " << j << endl;
			size_t s = tempLayer.size();
			//std::cout << "temp layer size: " << s << endl;
			quad q;
			tri tr;

			if (j == 0){
				tr.v1q = tempLayer[0];
				tr.v2q = tempLayer[1];
				tr.v3q = upTempLayer[0];

				tr.corners.push_back(tr.v1q);
				tr.corners.push_back(tr.v2q);
				tr.corners.push_back(tr.v3q);
				int i_v1tr, i_v2tr, i_v3tr;

				std::vector<int> int1first = inter1_dijkstra[i];

				//print_vector("temp layer:", tempLayer);
				//print_vector("Uptemp layer:", upTempLayer);
				//print_vector("int2first: ", int2first);
				//print_vector("int2second: ", int2second);
				//print_vector("int1first: ", int1first);
				//print_vector("tr: ", tr.corners);


				i_v1tr = std::find(int2first.begin(), int2first.end(), tr.v1q) - int2first.begin();
				i_v2tr = std::find(int2first.begin(), int2first.end(), tr.v2q) - int2first.begin();
				tr.chunk1to2 = slice(int2first, i_v1tr, i_v2tr);
				if (tr.chunk1to2.size() == 0)
					return -1;
				//print_vector("slice: ", tr.chunk1to2);

				i_v2tr = std::find(int1first.begin(), int1first.end(), tr.v2q) - int1first.begin();
				i_v3tr = std::find(int1first.begin(), int1first.end(), tr.v3q) - int1first.begin();
				tr.chunk2to3 = slice(int1first, i_v2tr, i_v3tr);
				if (tr.chunk2to3.size() == 0)
					return -1;
				//print_vector("slice: ", tr.chunk2to3);

				i_v3tr = std::find(i_path1to2.begin(), i_path1to2.end(), tr.v3q) - i_path1to2.begin();
				i_v1tr = std::find(i_path1to2.begin(), i_path1to2.end(), tr.v1q) - i_path1to2.begin();
				tr.chunk3to1 = slice(i_path1to2, i_v3tr, i_v1tr);
				if (tr.chunk3to1.size() == 0)
					return -1;
				//print_vector("slice: ", tr.chunk3to1);

				tr.chunks.push_back(tr.chunk1to2);
				tr.chunks.push_back(tr.chunk2to3);
				tr.chunks.push_back(tr.chunk3to1);

				tris.push_back(tr);


			}
			else {
				std::vector<int> int1first = inter1_dijkstra[i + j - 1];
				std::vector<int> int1second = inter1_dijkstra[i + j];

				q.v1q = tempLayer[j];
				q.v2q = tempLayer[j + 1];
				q.v3q = upTempLayer[j];
				q.v4q = upTempLayer[j - 1];
				//std::cout << "<" << j << "," << j + 1 << "><" << j << "," << j - 1 << ">" << endl;
				//print_vector("q corners: ", q.corners);
				q.corners.push_back(q.v1q);
				q.corners.push_back(q.v2q);
				q.corners.push_back(q.v3q);
				q.corners.push_back(q.v4q);

				int i_v1q, i_v2q, i_v3q, i_v4q;
				/*std::cout << "---A new kind of kick:---" << endl;
				print_vector("temp layer:", tempLayer);
				print_vector("Uptemp layer:", upTempLayer);
				print_vector("int2first: ", int2first);
				print_vector("int2second: ", int2second);
				print_vector("int1first: ", int1first);
				print_vector("int1second: ", int1second);
				print_vector("q: ", q.corners);*/
				//print_vector("slice: ", q.chunk1to2);

				i_v1q = std::find(int2first.begin(), int2first.end(), q.v1q) - int2first.begin();
				i_v2q = std::find(int2first.begin(), int2first.end(), q.v2q) - int2first.begin();
				q.chunk1to2 = slice(int2first, i_v1q, i_v2q);
				if (q.chunk1to2.size() == 0)
					return -1;
				//print_vector("slice: ", q.chunk1to2);

				i_v2q = std::find(int1second.begin(), int1second.end(), q.v2q) - int1second.begin();
				i_v3q = std::find(int1second.begin(), int1second.end(), q.v3q) - int1second.begin();
				q.chunk2to3 = slice(int1second, i_v2q, i_v3q);
				if (q.chunk2to3.size() == 0)
					return -1;
				//print_vector("slice: ", q.chunk2to3);

				i_v3q = std::find(int2second.begin(), int2second.end(), q.v3q) - int2second.begin();
				i_v4q = std::find(int2second.begin(), int2second.end(), q.v4q) - int2second.begin();
				q.chunk3to4 = slice(int2second, i_v3q, i_v4q);
				if (q.chunk3to4.size() == 0)
					return -1;
				//print_vector("slice: ", q.chunk3to4);

				i_v4q = std::find(int1first.begin(), int1first.end(), q.v4q) - int1first.begin();
				i_v1q = std::find(int1first.begin(), int1first.end(), q.v1q) - int1first.begin();
				q.chunk4to1 = slice(int1first, i_v4q, i_v1q);
				if (q.chunk4to1.size() == 0)
					return -1;
				//print_vector("slice: ", q.chunk4to1);

				q.chunks.push_back(q.chunk1to2);
				q.chunks.push_back(q.chunk2to3);
				q.chunks.push_back(q.chunk3to4);
				q.chunks.push_back(q.chunk4to1);

				/*std::cout << "i 1 and 2: " << i_v1q << "," << i_v2q << endl;
				print_vector("temp layer:", tempLayer);
				print_vector("int2first: ", int2first);
				std::cout << "q 1 and 2: " << q.v1q << "," << q.v2q << endl;
				print_vector("slice: ", q.chunk1to2);*/

				/*for each (auto &var in q.chunks){
				assert(var.size() > 0);
				std::cout << var.size() << endl;
				}*/

				quads.push_back(q);
			}

		}
	}
	tri tr;
	tr.v1q = intersections[intersections.size() - 1].at(0);
	tr.v2q = intersections[intersections.size() - 1][1];;
	tr.v3q = i_path1to2[0];

	std::vector<int> int2first = inter2_dijkstra[inter2_dijkstra.size() - 1];
	std::vector<int> int1first = i_path3to1;

	tr.corners.push_back(tr.v1q);
	tr.corners.push_back(tr.v2q);
	tr.corners.push_back(tr.v3q);

	//print_vector("tr: ", tr.corners);
	int i_v1tr, i_v2tr, i_v3tr;

	i_v1tr = std::find(int2first.begin(), int2first.end(), tr.v1q) - int2first.begin();
	i_v2tr = std::find(int2first.begin(), int2first.end(), tr.v2q) - int2first.begin();
	tr.chunk1to2 = slice(int2first, i_v1tr, i_v2tr);
	if (tr.chunk1to2.size() == 0)
		return -1;
	//print_vector("slice: ", tr.chunk1to2);

	i_v2tr = std::find(int1first.begin(), int1first.end(), tr.v2q) - int1first.begin();
	i_v3tr = std::find(int1first.begin(), int1first.end(), tr.v3q) - int1first.begin();
	tr.chunk2to3 = slice(int1first, i_v2tr, i_v3tr);
	if (tr.chunk2to3.size() == 0)
		return -1;
	//print_vector("slice: ", tr.chunk2to3);

	i_v3tr = std::find(i_path1to2.begin(), i_path1to2.end(), tr.v3q) - i_path1to2.begin();
	i_v1tr = std::find(i_path1to2.begin(), i_path1to2.end(), tr.v1q) - i_path1to2.begin();
	tr.chunk3to1 = slice(i_path1to2, i_v3tr, i_v1tr);
	if (tr.chunk3to1.size() == 0)
		return -1;
	//print_vector("slice: ", tr.chunk3to1);

	tr.chunks.push_back(tr.chunk1to2);
	tr.chunks.push_back(tr.chunk2to3);
	tr.chunks.push_back(tr.chunk3to1);

	tris.push_back(tr);

	//std::cout << "Tris size: " << tris.size() << endl;
	//std::cout << "Quads size: " << quads.size() << endl;

	return 1;
}

void Trilateral::fillInterPathsDijkstra(Trilateral::verts_t *sample1, Trilateral::verts_t *sample2, std::vector<std::vector<int>> *inter_dijkstra){
	for (int i = 0; i < gridSize; i++)
		inter_dijkstra->push_back(d->computeSinglePathInMesh(sample1->at(i), sample2->at(gridSize - 1 - i)));
}

void Trilateral::fillInterPaths(Trilateral::verts_t *sample1, Trilateral::verts_t *sample2, std::vector<Trilateral::path_t> *inter){
	path_t path;
	for (int i = 0; i < gridSize; i++)
	{
		geo->computeSinglePath(sample1->at(i), sample2->at(gridSize- 1 - i), &path);
		/*std::cout << "Path between: " << sample1->at(i) << "-" << sample2->at(gridSize - 1 - i) << endl;
		geo->print_path_info(&path);*/
		inter->push_back(path);
		path.clear();
	}
}

void Trilateral::getSampleVertices(Trilateral::path_t *path, Trilateral::verts_t *samples){
	double length = path->at(0).second;
	double step = length / (gridSize + 1);
	for (double i = 1.0; i < gridSize + 1.0; i += 1.0){
		double len = double(step * (i));
		std::vector<double> diff;

		for (int j = 0; j < path->size(); j++)
			diff.push_back(abs(path->at(j).second - len));

		auto result = std::min_element(std::begin(diff), std::end(diff));
		samples->push_back(path->at(std::distance(std::begin(diff), result)).first);
		//std::reverse(samples->begin(), samples->end());
	}
}

void Trilateral::floodFill(int v){
	if (visited[v] && !stopTest(v) && recursionIteration < recursionLimit){
		visit(v);
		recursionIteration++;
		for (int i = 0; i < mesh->verts[v]->vertList.size(); i++)
			floodFill(mesh->verts[v]->vertList[i]);
	}
}

bool Trilateral::stopTest(int v){
	return std::find(periphery.begin(), periphery.end(), v) != periphery.end();
}

void Trilateral::visit(int v){
	visited[v] = false;
	insideVerts.push_back(v);
}

std::vector<array<double, 3>> Trilateral::getNormalVectors(){
	for (int i = 0; i < mesh->tris.size(); i++)
	{
		int p1 = mesh->tris[i]->v1i;
		int p2 = mesh->tris[i]->v2i;
		int p3 = mesh->tris[i]->v3i;

		float p1x = mesh->verts[p1]->coords[0];
		float p1y = mesh->verts[p1]->coords[1];
		float p1z = mesh->verts[p1]->coords[2];

		float p2x = mesh->verts[p2]->coords[0];
		float p2y = mesh->verts[p2]->coords[1];
		float p2z = mesh->verts[p2]->coords[2];

		float p3x = mesh->verts[p3]->coords[0];
		float p3y = mesh->verts[p3]->coords[1];
		float p3z = mesh->verts[p3]->coords[2];

		float Ux = p2x - p1x;
		float Uy = p2y - p1y;
		float Uz = p2z - p1z;

		float Vx = p3x - p1x;
		float Vy = p3y - p1y;
		float Vz = p3z - p1z;

		float Nx = (Uy*Vz) - (Uz*Vy);
		float Ny = (Uz*Vx) - (Ux*Vz);
		float Nz = (Ux*Vy) - (Uy*Vx);

		array<double, 3> tN;
		tN[0] = Nx;
		tN[1] = Ny;
		tN[2] = Nz;

		triNormals.push_back(tN);
	}

	for (int j = 0; j < mesh->verts.size(); j++)
	{
		int p = mesh->verts[j]->idx;
		vector<int> n = mesh->verts[j]->triList;

		float Sx = 0, Sy = 0, Sz = 0;

		for (int k = 0; k < n.size(); k++)
		{
			Sx += triNormals[n[k]].at(0);
			Sy += triNormals[n[k]].at(1);
			Sz += triNormals[n[k]].at(2);
		}

		float c = float(n.size());
		Sx = Sx / c;
		Sy = Sy / c;
		Sz = Sz / c;

		array<double, 3> vN;
		vN[0] = Sx;
		vN[1] = Sy;
		vN[2] = Sz;

		vertNormals.push_back(vN);
	}
	return vertNormals;
}

float Trilateral::angleDegreeBetweenVectors(int p1, int p2){
	float p1x = vertNormals[p1][0];
	float p1y = vertNormals[p1][1];
	float p1z = vertNormals[p1][2];

	float p2x = vertNormals[p2][0];
	float p2y = vertNormals[p2][1];
	float p2z = vertNormals[p2][2];

	//std::cout << "P1 norm vector: <" << p1x << ">,<" << p1y << ">,<" << p1z << ">" << endl;
	//std::cout << "P2 norm vector: <" << p2x << ">,<" << p2y << ">,<" << p2z << ">" << endl;

	float nom = (p1x*p2x) + (p1y*p2y) + (p1z*p2z);
	float denom = sqrt(p1x*p1x + p1y*p1y + p1z*p1z)*sqrt(p2x*p2x + p2y*p2y + p2z*p2z);
	float ang = acos(nom / denom);
	//std::cout << "Angle between " << p1 << " and " << p2 << ": " << ang*(180.0 / 3.14) << " degrees" << endl;
	return ang*(180.0 / 3.14);
}

bool Trilateral::checkIn(std::vector<double> distS1, std::vector<double> distS2, std::vector<double> distS3, int v){
	double d1to2 = distS1[v2];
	double d1to3 = distS1[v3];
	double d2to3 = distS2[v3];

	double s1 = distS1[v];
	double s2 = distS2[v];
	double s3 = distS3[v];
	int sum = 0;

	if ((s1 < d1to2 || equal(s1, d1to2)) && (s1 < d1to3 || equal(s1, d1to3))) sum++;
	if ((s2 < d1to2 || equal(s2, d1to2)) && (s2 < d2to3 || equal(s2, d2to3))) sum++;
	if ((s3 < d1to3 || equal(s3, d1to3)) && (s3 < d2to3 || equal(s3, d2to3))) sum++;

	if (sum == 3) { return true; }
	else { return false; }
}

int Trilateral::findInsideVert(){
	/*
	Dijkstra *f = new Dijkstra(mesh);
	
	int mk = periphery[0];

	while (std::find(periphery.begin(), periphery.end(), mk) != periphery.end())
	{
		std::mt19937 rng;
		rng.seed(std::random_device()());
		std::uniform_int_distribution<std::mt19937::result_type> idx(0, i_path2to3.size() - 1);

		std::cout << "takildi " << idx(rng) << endl;
		std::vector<int> d1 = f->computeSinglePathInMesh(v1, periphery[periphery.size()/2]);
		mk = d1[int(d1.size() / 2)];
	}
	return mk;
	*/
	Dijkstra *f = new Dijkstra(mesh);

	int mk = periphery[0];

	while (std::find(periphery.begin(), periphery.end(), mk) != periphery.end())
	{
		std::cout << "takildi" << endl;
		std::vector<int> d1 = f->computeSinglePathInMesh(v1, periphery[int(periphery.size() / 2)]);
		mk = d1[int(d1.size() / 2)];
	}
	return mk;
}

int Trilateral::findInsideVert_new(){
	
	Dijkstra *f = new Dijkstra(mesh);

	int mk = periphery[0];

	for (size_t i = 1; i < i_path2to3.size() - 1; i++)
	{
		std::vector<int> d1 = f->computeSinglePathInMesh(v1, i_path2to3[i]);
		
		for (size_t j = 0; j < d1.size(); j++)
		{
			//std::cout << d1[j] << endl;
			if (!(std::find(periphery.begin(), periphery.end(), d1[j]) != periphery.end())){
				mk = d1[j];
				//std::cout << "Hey: " << mk << endl;
				break;
			}
		}
		if (mk != periphery[0])
			break;
	}

	if (mk == periphery[0]){
		for (size_t i = 1; i < i_path1to2.size() - 1; i++)
		{
			std::vector<int> d1 = f->computeSinglePathInMesh(v3, i_path1to2[i]);
			for (size_t j = 0; j < d1.size(); j++)
			{
				if (!(std::find(periphery.begin(), periphery.end(), d1[j]) != periphery.end())){
					mk = d1[j];
					//std::cout << "Hey: " << mk << endl;
					break;
				}
			}
			if (mk != periphery[0])
				break;
		}
	}

	if (mk == periphery[0]){
		for (size_t i = 1; i < i_path3to1.size() - 1; i++)
		{
			std::vector<int> d1 = f->computeSinglePathInMesh(v2, i_path3to1[i]);
			for (size_t j = 0; j < d1.size(); j++)
			{
				if (!(std::find(periphery.begin(), periphery.end(), d1[j]) != periphery.end())){
					mk = d1[j];
					//std::cout << "Hey: " << mk << endl;
					break;
				}
			}
			if (mk != periphery[0])
				break;
		}
	}
	return mk;
}

void Trilateral::fillPaths2Inside(verts_t *p){
	for (int i = 0; i < p->size(); i++)
		insideVerts.push_back(p->at(i));
}

void Trilateral::init()
{
	/* Outer Triangle */
	geo->computeSinglePath(v1, v2, &path1to2);
	geo->computeSinglePath(v2, v3, &path2to3);
	geo->computeSinglePath(v3, v1, &path3to1);

	std::reverse(path1to2.begin(), path1to2.end());
	std::reverse(path2to3.begin(), path2to3.end());
	std::reverse(path3to1.begin(), path3to1.end());

	print_vector("path1to2: ", getIntVector_(&path1to2));
	print_vector("path2to3: ", getIntVector_(&path2to3));
	print_vector("path3to1: ", getIntVector_(&path3to1));
	std::cout << "paths done" << endl;

	assert(path1to2.size() > gridSize);
	assert(path2to3.size() > gridSize);
	assert(path3to1.size() > gridSize);

	/* Samples on edges */
	getSampleVertices(&path1to2, &sample1to2);
	getSampleVertices(&path2to3, &sample2to3);
	getSampleVertices(&path3to1, &sample3to1);

	print_vector("sample1to2: ", sample1to2);
	print_vector("sample2to3: ", sample2to3);
	print_vector("sample3to1: ", sample3to1);
	std::cout << "sample verts done" << endl;
	/* Find an inside vertex */

	/* Find the insideMesh tris */
	i_path1to2 = getIntVector_(&path1to2);
	i_path2to3 = getIntVector_(&path2to3);
	i_path3to1 = getIntVector_(&path3to1);

	periphery.insert(periphery.begin(), i_path1to2.begin(), i_path1to2.end());
	periphery.insert(periphery.begin(), i_path2to3.begin(), i_path2to3.end());
	periphery.insert(periphery.begin(), i_path3to1.begin(), i_path3to1.end());


	mesh = new Mesh();
	mesh = geo->myMesh;

	vector<bool> v(mesh->verts.size(), true);
	visited = v;

	fillPaths2Inside(&i_path1to2);
	fillPaths2Inside(&i_path2to3);
	fillPaths2Inside(&i_path3to1);

	int o = findInsideVert();
	std::cout << "Flood fill vertex: " << o << endl;
	floodFill(o);
	//floodFill(1492);

	remove_duplicates(insideVerts);

	std::cout << "Flood fill done" << endl;

	/* Reverse sample vectors to properly find inter paths between them */

	std::reverse(sample1to2.begin(), sample1to2.end());
	std::reverse(sample2to3.begin(), sample2to3.end());
	std::reverse(sample3to1.begin(), sample3to1.end());

	/* Find inter geodesics on that mesh */
	std::cout << "\n1st connections:\n";
	fillInterPaths(&sample1to2, &sample2to3, &inter1);
	std::cout << "\n2nd connections:\n";
	fillInterPaths(&sample1to2, &sample3to1, &inter2);
	std::cout << "inter done" << endl;

	/* Restore correct paths */
	std::cout << "Restoring correct paths" << endl;
	d = new Dijkstra(geo->myMesh);
	fillInterPathsDijkstra(&sample1to2, &sample2to3, &inter1_dijkstra);
	std::cout << "\n2nd connections:\n";
	fillInterPathsDijkstra(&sample1to2, &sample3to1, &inter2_dijkstra);
	std::cout << "inter done" << endl;

	std::reverse(inter1_dijkstra.begin(), inter1_dijkstra.end());
	std::reverse(inter2_dijkstra.begin(), inter2_dijkstra.end());

	/* Add corners to sample vectors */
	sample1to2.insert(sample1to2.begin(), v1);
	sample1to2.push_back(v2);

	sample2to3.insert(sample2to3.begin(), v2);
	sample2to3.push_back(v3);

	sample3to1.insert(sample3to1.begin(), v3);
	sample3to1.push_back(v1);

	mesh = geo->myMesh;
	findIntersections();
	findChunks();
	combineChunks();

	getQuadArea();
	getTriArea();
	getAreaHistogram();

	std::cout << "finished" << endl;
	/*std::cout << "Intersections size: " << intersections.size() << endl;
	std::cout << "Intersections 0 size: " << intersections[0].size() << endl;
	std::cout << "Intersections 1 size: " << intersections[1].size() << endl;
	std::cout << "Intersections 2 size: " << intersections[2].size() << endl;*/

}

/*int Trilateral::findInsideVert(){
Dijkstra *f = new Dijkstra(mesh);
std::vector<double> d1 = f->computeDistancesFrom(v1);
std::vector<double> d2 = f->computeDistancesFrom(v2);
std::vector<double> d3 = f->computeDistancesFrom(v3);

int cent = -1;
for (int i = 0; i < mesh->verts.size(); i++){
if (checkIn(d1, d2, d3, i))
cent = i;
}
assert(cent > 0);
return cent;
}*/
/*
insideVerts.push_back(mesh->tris[i]->v1i);
insideVerts.push_back(mesh->tris[i]->v2i);
insideVerts.push_back(mesh->tris[i]->v3i);

inside_vs.push_back(mesh->verts[v1i]->coords[0]);
inside_vs.push_back(mesh->verts[v1i]->coords[1]);
inside_vs.push_back(mesh->verts[v1i]->coords[2]);

inside_vs.push_back(mesh->verts[v2i]->coords[0]);
inside_vs.push_back(mesh->verts[v2i]->coords[1]);
inside_vs.push_back(mesh->verts[v2i]->coords[2]);

inside_vs.push_back(mesh->verts[v3i]->coords[0]);
inside_vs.push_back(mesh->verts[v3i]->coords[1]);
inside_vs.push_back(mesh->verts[v3i]->coords[2]);

inside_ts.push_back(idx);
inside_ts.push_back(idx + 1);
inside_ts.push_back(idx + 2);

idx += 3;
*/
/*void Trilateral::ratherThanInit(){
Dijkstra* d = new Dijkstra(mesh);
verts_t p1to2, p2to3, p3to1;
p1to2 = d->computeSinglePathInMesh(v1, v2);
p2to3 = d->computeSinglePathInMesh(v2, v3);
p3to1 = d->computeSinglePathInMesh(v3, v1);

i_path1to2 = p1to2;
i_path2to3 = p2to3;
i_path3to1 = p3to1;

periphery.insert(periphery.begin(), p1to2.begin(), p1to2.end());
periphery.insert(periphery.begin(), p2to3.begin(), p2to3.end());
periphery.insert(periphery.begin(), p3to1.begin(), p3to1.end());

vector<bool> v(mesh->verts.size(), true);
visited = v;

int o = findInsideVert();
std::cout << "Flood fill vertex: " << o << endl;
floodFill(o);

insideVerts.insert(insideVerts.begin(), p1to2.begin(), p1to2.end());
insideVerts.insert(insideVerts.begin(), p2to3.begin(), p2to3.end());
insideVerts.insert(insideVerts.begin(), p3to1.begin(), p3to1.end());

remove_duplicates(insideVerts);

std::vector<int> mTriangles = verts2triIds(insideVerts);

std::cout << "Inside size: " << insideVerts.size() << endl;
std::cout << "Triangles size: " << mTriangles.size() << endl;

std::vector<int> old;
std::vector<double> newVerts;
std::vector<int> newTriangles;
for (int i = 0; i < mTriangles.size(); i++)
{
int v1i = mesh->tris[mTriangles[i]]->v1i;
int v2i = mesh->tris[mTriangles[i]]->v2i;
int v3i = mesh->tris[mTriangles[i]]->v3i;

int nv1, nv2, nv3;
if (std::find(old.begin(), old.end(), v1i) != old.end()){ nv1 = std::find(old.begin(), old.end(), v1i) - old.begin(); }
else { old.push_back(v1i); nv1 = old.size() - 1; }

if (std::find(old.begin(), old.end(), v2i) != old.end()){ nv2 = std::find(old.begin(), old.end(), v2i) - old.begin(); }
else { old.push_back(v2i); nv2 = old.size() - 1;  }

if (std::find(old.begin(), old.end(), v3i) != old.end()){ nv3 = std::find(old.begin(), old.end(), v3i) - old.begin(); }
else { old.push_back(v3i); nv3 = old.size() - 1; }

newTriangles.push_back(nv1);
newTriangles.push_back(nv2);
newTriangles.push_back(nv3);
}

for (size_t i = 0; i < old.size(); i++)
{
newVerts.push_back(mesh->verts[old[i]]->coords[0]);
newVerts.push_back(mesh->verts[old[i]]->coords[1]);
newVerts.push_back(mesh->verts[old[i]]->coords[2]);
}

/*mesh = new Mesh();
mesh->loadMesh(&newVerts, &newTriangles);
}
*/