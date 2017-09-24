#pragma once
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/Win/SoWin.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/nodes/SoCube.h>
#include <cmath>
#include <stdio.h>

#include "Mesh.h"
#include "Dijkstra.h"


class Painter
{
public:
	enum Color
	{
		red, green, blue, yellow, purple, orange, grey, white
	};
	SoSeparator* getShapeSep(Mesh* mesh, Color _c = grey);
	SoSeparator* drawSingleLine(Mesh* mesh, int firstPoint, int secondPoint);
	//SoSeparator* drawSingleSurfaceLine(Isocurve::point p1, Isocurve::point p2);
	SoSeparator* drawLineSet(vector<array<double,3>>, Color);
	SoSeparator* drawLineSet(array<array<double, 3>, 2>, Color);
	SoSeparator* drawLineSet(Mesh* mesh, std::vector<int> listOfVertices, Color _color = Painter::blue);
	SoSeparator* drawSphereOnMesh(int origin, Mesh* mesh, Color _color = Painter::blue, int radius = 1);
	SoSeparator* drawSphere(SbVec3f _tr, Color _color = Painter::red, int radius = 1);
	SoSeparator* get1PointSep(Mesh* mesh, int pnt, Color _color = Painter::orange);
	SoSeparator* get1PointSep(SbVec3d p, Color _color = Painter::orange);
	SoSeparator* getMultiplePointSep(Mesh* mesh, vector<int> p, Color _color = Painter::blue);
	SoSeparator* getMultiplePointSep(vector<array<double, 3>> p, Color _color = Painter::blue);
	SoSeparator* drawTriangle();
	SoSeparator* drawHistogram(Mesh* mesh, std::vector<int> triIds, Color c = Painter::orange);
	SoSeparator* drawBoundingBox(SbBox3f bbox);
	//SoSeparator* drawBoundingBox(SbBox3f bbox);
	/*	cat 1: far away bbox
		cat 2: partially under
		cat 3: completely under
		cat 4: chair left/right
	*/
	SoSeparator* getCombination(SoWinExaminerViewer * viewer, Mesh* mesh1, Mesh* m2, int category);

	SbColor getColor(Color c){
		switch (c)
		{
		case Painter::red:
			return SbColor(0.8f, 0.0f, 0.0f);
			break;
		case Painter::green:
			return SbColor(0.0f, 0.8f, 0.0f);
			break;
		case Painter::blue:
			return SbColor(0.0f, 0.0f, 0.8f);
			break;
		case Painter::yellow:
			return SbColor(0.0f, 0.8f, 0.8f);
			break;
		case Painter::purple:
			return SbColor(0.8f, 0.0f, 0.8f);
			break;
		case Painter::orange:
			return SbColor(0.8f, 0.8f, 0.0f);
			break;
		case Painter::grey:
			return SbColor(0.8f, 0.8f, 0.8f);
			break;
		case Painter::white:
			return SbColor(1.0f, 1.0f, 1.0f);
			break;
		default:
			break;
		}
	}
};
