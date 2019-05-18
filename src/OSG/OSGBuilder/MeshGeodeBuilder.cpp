#include <OSGBuilder/MeshGeodeBuilder.h>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

using namespace OSGWrapper;
namespace OSGBuilder
{
	osg::Geometry* MeshGeodeBuilder::Build(Mesh& mesh, MeshGeodeType geode_type, int attribute)
	{
		if (geode_type == MGT_POINTS)
			return BuildPoint(mesh, attribute);
		else if (geode_type == MGT_TRIANGLE)
			return BuildTriangles(mesh, attribute);
		return 0;
	}

	osg::Vec4Array* MeshGeodeBuilder::BuildColorArray(Mesh& mesh)
	{
		float* color = new float[4 * mesh.vertex_number];
		for (unsigned i = 0; i < mesh.vertex_number; ++i)
		{
			float* c = color + 4 * i;
			unsigned char* cc = mesh.vertex_color + 3 * i;

			*c++ = (float)(*cc++)/255.0f; *c++ = (float)(*cc++) / 255.0f; *c++ = (float)(*cc++) / 255.0f; *c++ = 1.0f;
		}

		osg::Vec4Array* color_array = (osg::Vec4Array*)ArrayCreator::CreateVec4Array(mesh.vertex_number, color);
		delete[]color;
		return color_array;
	}

	osg::Geometry* MeshGeodeBuilder::BuildPoint(Mesh& mesh, int attribute)
	{
		if (mesh.vertex_number == 0) return 0;

		osg::Array* coord_array = 0;
		if(attribute & MGT_POSITION) coord_array = ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_position);
		osg::Array* normal_array = 0;
		if(attribute & MGT_NORMAL) normal_array = ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_normal);
		osg::Array* color_array = 0;
		if (attribute & MGT_COLOR) color_array = BuildColorArray(mesh);

		osg::PrimitiveSet* primitive_set = new osg::DrawArrays(GL_POINTS, 0, mesh.vertex_number);
		osg::Geometry* geom = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array, normal_array, color_array);

		geom->setCullingActive(false);
		return geom;
	}

	osg::Geometry* MeshGeodeBuilder::BuildTriangles(Mesh& mesh, int attribute)
	{
		if (mesh.triangle_number == 0 || mesh.vertex_number == 0) return 0;

		osg::Array* coord_array = 0;
		if (attribute & MGT_POSITION) coord_array = ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_position);
		osg::Array* normal_array = 0;
		if (attribute & MGT_NORMAL) normal_array = ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_normal);
		osg::Array* color_array = 0;
		if (attribute & MGT_COLOR) color_array = BuildColorArray(mesh);

		osg::PrimitiveSet* primitive_set = ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::TRIANGLES, 3 * mesh.triangle_number, mesh.triangle_index);
		osg::Geometry* geom = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array, normal_array, color_array);

		geom->setCullingActive(false);
		return geom;
	}
}