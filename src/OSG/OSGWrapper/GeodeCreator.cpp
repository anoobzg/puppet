#include <osgWrapper/GeodeCreator.h>
#include <osgWrapper/ArrayCreator.h>
#include <osgWrapper/GeometryCreator.h>

namespace OSGWrapper
{
	osg::Geode* GeodeCreator::CreateLineSegments(float* boundary_position, unsigned* boundary_segment_size, unsigned boundary_size)
	{
		osg::Geode* geode = CreateGeode();
		float* visitor = boundary_position;
		for(unsigned i = 0; i < boundary_size; ++i)
		{
			unsigned psize = boundary_segment_size[i];
			osg::Array* vertex_array = ArrayCreator::CreateVec3Array(psize, visitor);
			osg::PrimitiveSet* primitive_set = new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, psize);
			osg::Geometry* geom = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, vertex_array, 0, 0, 0);
			geode->addDrawable(geom);
			visitor += 3 * psize;
		}

		return geode;
	}

	osg::Geode* GeodeCreator::CreateQuad()
	{
		osg::Geode* geode = new osg::Geode();
		float vertex_coord[12] = {
			-1.0f, 1.0f, 0.0f,
			-1.0f, -1.0f, 0.0f,
			1.0f, -1.0f, 0.0f,
			1.0f, 1.0f, 0.0f
		};
		float tex_coord[8] = {
			0.0f, 1.0f,
			0.0f, 0.0f, 
			1.0f, 0.0f,
			1.0f, 1.0f
		};

		osg::Array* vertex_array = ArrayCreator::CreateVec3Array(4, vertex_coord);
		osg::Array* texcoord_array = ArrayCreator::CreateVec2Array(4, tex_coord);
		osg::PrimitiveSet* primitive_set = new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4);
		osg::Geometry* geometry = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, vertex_array, texcoord_array, 0, 0);;
		geode->addDrawable(geometry);
		geode->setCullingActive(false);
		return geode;
	}

	osg::Geode* GeodeCreator::CreateGeode()
	{
		osg::Geode* geode = new osg::Geode();
		geode->setCullingActive(false);
		return geode;
	}

	osg::Geode* GeodeCreator::CreateGeode(unsigned vertex_number, float* vertex_position, float* vertex_normal, float* vertex_color, unsigned triangle_number, unsigned* triangle_index)
	{
		osg::Geode* geode = new osg::Geode();
		osg::Array* vertex_array = ArrayCreator::CreateVec3Array(vertex_number, vertex_position);
		osg::Array* normal_array = ArrayCreator::CreateVec3Array(vertex_number, vertex_normal);
		osg::Array* color_array = ArrayCreator::CreateVec4Array(vertex_number, vertex_color);
		osg::PrimitiveSet* primitive_set = ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::TRIANGLES, triangle_number, triangle_index);
		osg::Geometry* geometry = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, vertex_array, normal_array, color_array, 0);;
		geode->addDrawable(geometry);
		geode->setCullingActive(false);
		return geode;
	}

	osg::Geode* GeodeCreator::CreatePoint(unsigned vertex_number, float* vertex_position, float* vertex_normal, float* vertex_color)
	{
		osg::Geode* geode = new osg::Geode();
		osg::Array* vertex_array = ArrayCreator::CreateVec3Array(vertex_number, vertex_position);
		osg::Array* normal_array = ArrayCreator::CreateVec3Array(vertex_number, vertex_normal);
		osg::Array* color_array = ArrayCreator::CreateVec4Array(vertex_number, vertex_color);
		osg::PrimitiveSet* primitive_set = new osg::DrawArrays(GL_POINTS, 0, vertex_number);
		osg::Geometry* geometry = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, vertex_array, normal_array, color_array, 0);;
		geode->addDrawable(geometry);
		geode->setCullingActive(false);
		return geode;
	}

	osg::Geode* GeodeCreator::CreateIndexAttributeGeode(osg::PrimitiveSet* primitive, osg::Array* attribute0, osg::Array* attribute1, osg::Array* attribute2, osg::Array* attribute3, osg::Array* attribute4)
	{
		osg::Geode* geode = new osg::Geode();
		osg::Geometry* geometry = GeometryCreator::CreateIndexAttributeGeometry(primitive, attribute0, attribute1, attribute2, attribute3, attribute4);
		geode->addDrawable(geometry);
		geometry->setCullingActive(false);
		geode->setCullingActive(false);
		return geode;
	}
}