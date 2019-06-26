#include <osgWrapper/UtilCreator.h>
#include <osgWrapper/GeometryCreator.h>
#include <osgWrapper/ArrayCreator.h>

namespace OSGWrapper
{
	osg::Geometry* UtilCreator::CreateGrid(const osg::Vec3f& bmin, const osg::Vec3& bmax, const osg::Vec3i& size)
	{
		osg::Vec3Array* coord_array = new osg::Vec3Array();
		int nx = size.x(); int ny = size.y(); int nz = size.z();
		int total = (nx + 1) * (ny + 1) * (nz + 1);
		coord_array->resize(total);
		osg::Vec3f fsize = bmax - bmin;
		float dx = fsize.x() / (float)nx;
		float dy = fsize.y() / (float)ny;
		float dz = fsize.z() / (float)nz;
		int index = 0;
		for (int i = 0; i <= nx; ++i)
		{
			float x = (float)i * dx;
			for (int j = 0; j <= ny; ++j)
			{
				float y = (float)j * dy;
				for (int k = 0; k <= nz; ++k)
				{
					float z = (float)k * dz;
					coord_array->at(index) = bmin + osg::Vec3f(x, y, z);
					++index;
				}
			}
		}
		osg::DrawElementsUInt* draw_array = new osg::DrawElementsUInt(GL_LINES);
		int total_element = (nx + 1) * (ny + 1) + (ny + 1) * (nz + 1) + (nx + 1) * (nz + 1);
		draw_array->resize((size_t)(2 * total_element));
		index = 0;

		int offset = nz;
		for (int i = 0; i <= nx; ++i)  //z
		{
			for (int j = 0; j <= ny; ++j)
			{
				int u = i * (ny + 1) * (nz + 1) + j * (nz + 1);
				draw_array->at(index) = u;
				++index;
				draw_array->at(index) = u + offset;
				++index;
			}
		}

		offset = nx * (ny + 1) * (nz + 1);
		for (int i = 0; i <= ny; ++i)  //x
		{
			for (int j = 0; j <= nz; ++j)
			{
				int u = i * (nz + 1) + j;
				draw_array->at(index) = u;
				++index;
				draw_array->at(index) = u + offset;
				++index;
			}
		}

		offset = ny * (nz + 1);
		for (int i = 0; i <= nx; ++i)  //y
		{
			for (int j = 0; j <= nz; ++j)
			{
				int u = i * (ny + 1) * (nz + 1) + j;
				draw_array->at(index) = u;
				++index;
				draw_array->at(index) = u + offset;
				++index;
			}
		}
		return OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array);
	}
}