#include "Painter.h"

SoSeparator* Painter::getShapeSep(Mesh* mesh, Color _c)
{
	SoSeparator* res = new SoSeparator();

	//transformation
	//not needed

	//color
	SoMaterial* mat = new SoMaterial();
	//mat->diffuseColor.setValue(0.8, 0.8, 0.8);
	mat->diffuseColor.setValue(getColor(_c));
	res->addChild(mat);

	SoShapeHints* hints = new SoShapeHints;
	hints->creaseAngle = 3.14;
	res->addChild(hints); //Gouraud shading

	//shape
	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->verts.size(); c++)
		coords->point.set1Value(c, mesh->verts[c]->coords[0], mesh->verts[c]->coords[1], mesh->verts[c]->coords[2]);
	SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
	for (int c = 0; c < mesh->tris.size(); c++)
	{
		faceSet->coordIndex.set1Value(c * 4, mesh->tris[c]->v1i);
		faceSet->coordIndex.set1Value(c * 4 + 1, mesh->tris[c]->v2i);
		faceSet->coordIndex.set1Value(c * 4 + 2, mesh->tris[c]->v3i);
		faceSet->coordIndex.set1Value(c * 4 + 3, -1);
	}
	res->addChild(coords);
	res->addChild(faceSet);

	return res;
}

SoSeparator* Painter::drawSphereOnMesh(int origin, Mesh* mesh, Color _c, int radius){
	SoSeparator* sphereSep = new SoSeparator();
	SoSphere* sphere = new SoSphere();
	sphere->radius = radius;

	SoTranslation* tr = new SoTranslation();
	SoSFVec3f* vec = new SoSFVec3f();
	vec->setValue(mesh->verts[origin]->coords[0], mesh->verts[origin]->coords[1], mesh->verts[origin]->coords[2]);
	tr->translation = *vec;

	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(getColor(_c));

	sphereSep->addChild(tr);
	sphereSep->addChild(mat);
	sphereSep->addChild(sphere);

	return sphereSep;
}
SoSeparator* Painter::drawSingleLine(Mesh* mesh, int firstPoint, int secondPoint){
	SoSeparator* line = new SoSeparator();
	SoLineSet* l = new SoLineSet();
	SoMaterial* mat = new SoMaterial();
	SoDrawStyle* sty = new SoDrawStyle(); sty->lineWidth = 2.0f;
	SoVertexProperty* vprop = new SoVertexProperty();
	vprop->vertex.set1Value(0, mesh->verts[firstPoint]->coords[0], mesh->verts[firstPoint]->coords[1], mesh->verts[firstPoint]->coords[2]);
	vprop->vertex.set1Value(1, mesh->verts[secondPoint]->coords[0], mesh->verts[secondPoint]->coords[1], mesh->verts[secondPoint]->coords[2]);
	mat->diffuseColor.setValue(0, 0, 1);
	l->vertexProperty = vprop;
	line->addChild(mat);
	line->addChild(sty);
	line->addChild(l);
	return line;
}
SoSeparator* Painter::drawLineSet(Mesh* mesh, std::vector<int> listOfVertices, Color _color){
	SoVertexProperty* vprop = new SoVertexProperty();
	SoDrawStyle* sty = new SoDrawStyle(); sty->lineWidth = 4.0f;
	for (int i = 0; i < listOfVertices.size(); i++)
	{
		int v = listOfVertices.at(i);
		vprop->vertex.set1Value(i, mesh->verts[v]->coords[0], mesh->verts[v]->coords[1], mesh->verts[v]->coords[2]);
	}

	/* Draw Red Line */
	SoSeparator* line = new SoSeparator();
	SoLineSet* l = new SoLineSet();
	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(getColor(_color));
	l->vertexProperty = vprop;
	line->addChild(mat);
	line->addChild(sty);
	line->addChild(l);

	return line;
}

SoSeparator* Painter::drawLineSet(vector<array<double, 3>> in, Color _color){
	SoVertexProperty* vprop = new SoVertexProperty();
	SoDrawStyle* sty = new SoDrawStyle(); sty->lineWidth = 4.0f;
	for (int i = 0; i < in.size(); i++)
	{
		array<double,3> v = in.at(i);
		vprop->vertex.set1Value(i, v[0], v[1], v[2]);
	}

	/* Draw Red Line */
	SoSeparator* line = new SoSeparator();
	SoLineSet* l = new SoLineSet();
	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(getColor(_color));
	l->vertexProperty = vprop;
	line->addChild(mat);
	line->addChild(sty);
	line->addChild(l);

	return line;
}

SoSeparator* Painter::drawLineSet(array<array<double, 3>, 2> in, Color _color){
	SoVertexProperty* vprop = new SoVertexProperty();
	SoDrawStyle* sty = new SoDrawStyle(); sty->lineWidth = 4.0f;
	for (int i = 0; i < in.size(); i++)
	{
		array<double, 3> v = in.at(i);
		vprop->vertex.set1Value(i, v[0], v[1], v[2]);
	}

	/* Draw Red Line */
	SoSeparator* line = new SoSeparator();
	SoLineSet* l = new SoLineSet();
	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(getColor(_color));
	l->vertexProperty = vprop;
	line->addChild(mat);
	line->addChild(sty);
	line->addChild(l);

	return line;
}

SoSeparator* Painter::drawSphere(SbVec3f _tr, Color _c, int radius){
	SoSeparator* sphereSep = new SoSeparator();
	SoSphere* sphere = new SoSphere();
	sphere->radius = radius;

	SoTranslation* tr = new SoTranslation();
	tr->translation = _tr;

	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(getColor(_c));


	sphereSep->addChild(tr);
	sphereSep->addChild(mat);
	sphereSep->addChild(sphere);

	return sphereSep;
}

SoSeparator* Painter::get1PointSep(Mesh* mesh, int pnt, Color _color)
{
	//renders only 1 pnt in blue, w/ drawWhat = 1 for spectral coords, = 2 for spatial coords, = 5 for coord written here

	SoSeparator* pntSep = new SoSeparator;
	SoMaterial* mat = new SoMaterial;

	mat->diffuseColor.setValue(getColor(_color)); //red

	pntSep->addChild(mat);
	SoDrawStyle* style = new SoDrawStyle;
	style->pointSize = 10.0f;
	pntSep->addChild(style);

	SoVertexProperty* vp = new SoVertexProperty;
	vp->vertex.set1Value(0, mesh->verts[pnt]->coords[0], mesh->verts[pnt]->coords[1], mesh->verts[pnt]->coords[2]);
	SoPointSet* pSet = new SoPointSet;
	pSet->numPoints = 1;
	pSet->vertexProperty = vp;
	pntSep->addChild(pSet);

	return pntSep;
}


SoSeparator* Painter::get1PointSep(SbVec3d p, Color _color){
	SoSeparator* pntSep = new SoSeparator;
	SoMaterial* mat = new SoMaterial;

	mat->diffuseColor.setValue(getColor(_color)); //red

	pntSep->addChild(mat);
	SoDrawStyle* style = new SoDrawStyle;
	style->pointSize = 10.0f;
	pntSep->addChild(style);

	SoVertexProperty* vp = new SoVertexProperty;
	vp->vertex.set1Value(0, p[0], p[1], p[2]);
	SoPointSet* pSet = new SoPointSet;
	pSet->numPoints = 1;
	pSet->vertexProperty = vp;
	pntSep->addChild(pSet);

	return pntSep;
}

SoSeparator* Painter::getMultiplePointSep(Mesh* mesh, vector<int> p, Color _color){
	SoSeparator* pntSep = new SoSeparator;
	SoMaterial* mat = new SoMaterial;

	mat->diffuseColor.setValue(getColor(_color)); //red

	pntSep->addChild(mat);
	SoDrawStyle* style = new SoDrawStyle;
	style->pointSize = 10.0f;
	pntSep->addChild(style);

	SoVertexProperty* vp = new SoVertexProperty;
	for (int i = 0; i < p.size(); i++)
	{
		vp->vertex.set1Value(i, mesh->verts[p.at(i)]->coords[0], mesh->verts[p.at(i)]->coords[1], mesh->verts[p.at(i)]->coords[2]);
	}
	
	SoPointSet* pSet = new SoPointSet;
	pSet->numPoints = p.size();
	pSet->vertexProperty = vp;
	pntSep->addChild(pSet);

	return pntSep;
}

SoSeparator* Painter::drawTriangle(){
	SoSeparator* tri = new SoSeparator;
	SoMaterial* mat = new SoMaterial;

	mat->diffuseColor.setValue(getColor(Painter::blue)); //red

	tri->addChild(mat);
	/*SoDrawStyle* style = new SoDrawStyle;
	style->pointSize = 10.0f;
	tri->addChild(style);*/

	SoVertexProperty* vp = new SoVertexProperty;
	for (int i = 0; i < 3; i++)
	{
		vp->vertex.set1Value(i, i+10,i+10,i+10);
	}

	SoPointSet* pSet = new SoPointSet;
	pSet->numPoints = 3;
	pSet->vertexProperty = vp;
	tri->addChild(pSet);

	return tri;
}

SoSeparator* Painter::getMultiplePointSep(vector<array<double,3>> p, Color _color){
	SoSeparator* pntSep = new SoSeparator;
	SoMaterial* mat = new SoMaterial;

	mat->diffuseColor.setValue(getColor(_color)); //red

	pntSep->addChild(mat);
	SoDrawStyle* style = new SoDrawStyle;
	style->pointSize = 10.0f;
	pntSep->addChild(style);

	SoVertexProperty* vp = new SoVertexProperty;
	for (int i = 0; i < p.size(); i++)
	{
		vp->vertex.set1Value(i, p.at(i)[0], p.at(i)[1], p.at(i)[2]);
	}

	SoPointSet* pSet = new SoPointSet;
	pSet->numPoints = p.size();
	pSet->vertexProperty = vp;
	pntSep->addChild(pSet);

	return pntSep;
}


SoSeparator* Painter::drawHistogram(Mesh* mesh, std::vector<int> triIds, Color c){
	SoSeparator* res = new SoSeparator();

	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(getColor(c));
	res->addChild(mat);

	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->verts.size(); c++)
		coords->point.set1Value(c, mesh->verts[c]->coords[0], mesh->verts[c]->coords[1], mesh->verts[c]->coords[2]);

	SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
	for (int c = 0; c < triIds.size(); c++)
	{
		faceSet->coordIndex.set1Value(c * 4, mesh->tris[triIds.at(c)]->v1i);
		faceSet->coordIndex.set1Value(c * 4 + 1, mesh->tris[triIds.at(c)]->v2i);
		faceSet->coordIndex.set1Value(c * 4 + 2, mesh->tris[triIds.at(c)]->v3i);
		faceSet->coordIndex.set1Value(c * 4 + 3, -1);
	}
	res->addChild(coords);
	res->addChild(faceSet);

	return res;
}

SoSeparator* Painter::drawBoundingBox(SbBox3f bbox){

	SoSeparator* root = new SoSeparator();
	SbVec3f maxBB = bbox.getMax();
	SbVec3f minBB = bbox.getMin();
	SbVec3f origin = bbox.getCenter();

	SoCube* cube = new SoCube;
	
	cube->width = maxBB[0] - minBB[0];
	cube->height = maxBB[1] - minBB[1];
	cube->depth = maxBB[2] - minBB[2];
	
	SoTranslation* tr = new SoTranslation();
	tr->translation = origin;

	root->addChild(tr);
	root->addChild(cube);
	
	return root;
}

SoSeparator* Painter::getCombination(SoWinExaminerViewer * viewer, Mesh* mesh1, Mesh* mesh2, int category){
	SoSeparator* base = new SoSeparator();
	SoSeparator* m1 = new SoSeparator();
	m1->addChild(getShapeSep(mesh1));
	m1->ref();

	SoGetBoundingBoxAction bboxAction(viewer->getViewportRegion());
	bboxAction.apply(m1);
	SbBox3f bbox = bboxAction.getBoundingBox();

	SbVec3f maxBB = bbox.getMax();
	SbVec3f minBB = bbox.getMin();

	float width = maxBB[0] - minBB[0];
	/*std::cout << "WIDTH: " << width << endl;
	std::cout << "HEIGHT: " << maxBB[1] - minBB[1] << endl;
	std::cout << "DEPTH: " << maxBB[2] - minBB[2] << endl;*/

	SoSeparator* m2 = new SoSeparator();	
	m2->addChild(getShapeSep(mesh2));

	base->addChild(m1);
	base->addChild(m2);
	return base;
	m1->unref();
}




/* stuff below are from my old projects; should run fine and be useful in your development

if (drawThickEdges) //draw thick edges (may be useful in geodesic path drawing)
	{
		SoSeparator* thickEdgeSep = new SoSeparator;
		//material
		SoMaterial* ma = new SoMaterial;
		ma->diffuseColor.set1Value(0, 0.0f, 0.0f, 1.0f);
		thickEdgeSep->addChild(ma);
		SoDrawStyle* sty = new SoDrawStyle;	sty->lineWidth = 5.0f;	thickEdgeSep->addChild(sty);

		//shape
		SoIndexedLineSet* ils = new SoIndexedLineSet;
		SoCoordinate3* co = new SoCoordinate3;

		//assumes no edge in sedges is removed
		for (unsigned int se = 0; se < mesh->sedges.size(); se++)
		{
			SbVec3f end1 = mesh->verts[ mesh->sedges[se]->v1i ]->coords + SbVec3f(deltaX, 0.0f, 0.0f),
					end2 = mesh->verts[ mesh->sedges[se]->v2i ]->coords + SbVec3f(deltaX, 0.0f, 0.0f);
			co->point.set1Value(2*se, end1);
			co->point.set1Value(2*se + 1, end2);
		}

		for (unsigned int ci = 0; ci < mesh->sedges.size(); ci++)
		{
			ils->coordIndex.set1Value(3*ci, 2*ci);	ils->coordIndex.set1Value(3*ci + 1, 2*ci + 1);
			ils->coordIndex.set1Value(3*ci + 2, -1); //end this edge with -1
		}
		thickEdgeSep->addChild(co);	thickEdgeSep->addChild(ils);
		obj->sep->addChild(thickEdgeSep);
	}
	
	
SoSeparator* Painter::get1PointSep(ScreenObject* obj, int pnt, int drawWhat, float deltaX, float deltaY, float scale)
{
	//renders only 1 pnt in blue, w/ drawWhat = 1 for spectral coords, = 2 for spatial coords, = 5 for coord written here

	Mesh* mesh = obj->getMesh();

	SoSeparator* pntSep = new SoSeparator;
	//material
	SoMaterial* mat = new SoMaterial;
	if (mesh->targetMesh)
		mat->diffuseColor.setValue(SbColor(1.0f, 0.0f, 0.0f)); //red
	else
		mat->diffuseColor.setValue(SbColor(0.0f, 1.0f, 0.0f)); //green
if (pnt == 594) mat->diffuseColor.setValue(SbColor(1.0f, 0.0f, 1.0f)); //magenta
//if (pnt == 6916) mat->diffuseColor.setValue(SbColor(0.0f, 1.0f, 1.0f));

	pntSep->addChild(mat);
	SoDrawStyle* style = new SoDrawStyle;
	style->pointSize = 17.0f;
	pntSep->addChild(style);
	
	//shape
	SoVertexProperty* vp = new SoVertexProperty;
	if (drawWhat == 2)
		vp->vertex.set1Value(0, scale*mesh->verts[pnt]->coords[0]+deltaX, scale*mesh->verts[pnt]->coords[1]+deltaY, scale*mesh->verts[pnt]->coords[2]);
	else if (drawWhat == 5)
		vp->vertex.set1Value(0, 0.016721f, -0.000984876f, 0.0f);
	else
		vp->vertex.set1Value(0, scale*mesh->verts[pnt]->spectralK[0]+deltaX, scale*mesh->verts[pnt]->spectralK[1]+deltaX, scale*mesh->verts[pnt]->spectralK[2]+deltaX);
	SoPointSet* pSet = new SoPointSet;
	pSet->numPoints = 1;
	pSet->vertexProperty = vp;
	pntSep->addChild(pSet);

//cout << pnt << " ------> " << mesh->verts[pnt]->matchIdx << endl;
	return pntSep;
}

SoSeparator* Painter::getSpheresSep(Mesh* mesh, float deltaX, float deltaY, float scale)
{
	//returns a set of spheres to highlight each mesh.samples[i]

	SoSeparator* spheresSep = new SoSeparator();

	float radius = 50.0f;

	for (int i = 0; i < (int) mesh->samples.size(); i++)
	{
		//1 sphere for this sample
		SoSeparator* sphere1Sep = new SoSeparator;

		//transformation
		SoTransform* tra = new SoTransform();
		tra->translation.setValue(scale*mesh->verts[ mesh->samples[i] ]->coords[0]+deltaX, scale*mesh->verts[ mesh->samples[i] ]->coords[1]+deltaY, scale*mesh->verts[ mesh->samples[i] ]->coords[2]);
		sphere1Sep->addChild(tra);

		//material
		SoMaterial* ma = new SoMaterial;
		if (i == 0)
			ma->diffuseColor.setValue(SbColor(0.0f, 0.0f, 0.7f));
		else if (i == 1)
			ma->diffuseColor.setValue(SbColor(0.0f, 0.0f, 0.0f));
		else if (i == 2)
			ma->diffuseColor.setValue(SbColor(0.0f, 0.7f, 0.0f));
		else if (i == 3)
			ma->diffuseColor.setValue(SbColor(0.7f, 0.0f, 0.7f));
		else if (i == 4)
			ma->diffuseColor.setValue(SbColor(0.7f, 0.7f, 0.0f));
		else
			ma->diffuseColor.setValue(SbColor(0.7f, 0.0f, 0.0f));

		sphere1Sep->addChild(ma);

		//shape
		SoSphere* sph1 = new SoSphere();
		sph1->radius = radius;
		sphere1Sep->addChild(sph1); //whose position is decided by the translation applied above

		spheresSep->addChild(sphere1Sep);
	}
	
	return spheresSep;
}
*/
