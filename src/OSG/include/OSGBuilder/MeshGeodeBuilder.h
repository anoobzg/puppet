#pragma once
#include "OSGBuilderExport.h"
#include <osg\Geode>
#include "Mesh.h"

using namespace LauncaGeometry;
namespace OSGBuilder
{
	enum MeshGeodeType
	{
		MGT_TRIANGLE,
		MGT_POINTS
	};

	unsigned MGT_POSITION = 1;
	unsigned MGT_NORMAL = 2;
	unsigned MGT_COLOR = 4;

	class OSG_BUILDER_API MeshGeodeBuilder
	{
	public:
		static osg::Geometry* Build(Mesh& mesh, MeshGeodeType geode_type, int attribute);
		static osg::Vec4Array* BuildColorArray(Mesh& mesh);
	private:
		static osg::Geometry* BuildPoint(Mesh& mesh, int attribute);
		static osg::Geometry* BuildTriangles(Mesh& mesh, int attribute);
	};
}