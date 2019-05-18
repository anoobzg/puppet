#include <OSGBuilder\GeometryCache.h>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

using namespace OSGWrapper;
namespace OSGBuilder
{
	std::auto_ptr<GeometryCache> GeometryCache::m_cache = std::auto_ptr<GeometryCache>();
	osg::Geometry* GeometryCache::GetOrCreateBall()
	{
		if (!m_ball.valid())
		{
			unsigned slice = 10;
			unsigned stacks = 10;
			float r = 1.0f;
			unsigned vertCount = (slice - 1)*stacks + 2;
			float* m_vertex = new float[3 * vertCount];
			float* m_normal = new float[3 * vertCount];

			float sliceDepth = 2.0f * r / float(slice);
			float angleDepth = 2.0f * 3.1415926f / float(stacks);

			m_vertex[0] = 0.0f; m_vertex[1] = r; m_vertex[2] = 0.0f;
			m_normal[0] = 0.0f; m_normal[1] = 1.0f; m_normal[2] = 0.0f;
			for (unsigned i = 0; i < (slice - 1); i++)
			{
				float valueY = cos(3.1415926f / slice*(i + 1))*r;
				//float valueY = r - sliceDepth*float(i+1);
				float subR = sqrt(r*r - valueY*valueY);
				for (unsigned j = 0; j < stacks; j++)
				{
					float angle = angleDepth*j;
					float valueZ = sin(angle)*subR;
					float valueX = cos(angle)*subR;

					unsigned index = 1 + i*stacks + j;
					m_vertex[3 * index] = valueX; m_vertex[3 * index + 1] = valueY; m_vertex[3 * index + 2] = valueZ;
					m_normal[3 * index] = valueX; m_normal[3 * index + 1] = valueY; m_normal[3 * index + 2] = valueZ;
					float len = sqrtf(valueX * valueX + valueY*valueY + valueZ*valueZ);
					if (len > 0.0f)
					{
						m_normal[3 * index] /= len;
						m_normal[3 * index + 1] /= len;
						m_normal[3 * index + 2] /= len;
					}
				}
			}
			m_vertex[3 * (vertCount - 1)] = 0.0f; m_vertex[3 * (vertCount - 1) + 1] = -r; m_vertex[3 * (vertCount - 1) + 2] = 0.0f;
			m_normal[3 * (vertCount - 1)] = 0.0f; m_normal[3 * (vertCount - 1) + 1] = -1.0f; m_normal[3 * (vertCount - 1) + 2] = 0.0f;

			unsigned indexCount = 2 * stacks + (slice - 2)*stacks * 2;
			unsigned* m_index = new unsigned[3 * indexCount];

			unsigned indexOffset = 0;
			for (unsigned j = 0; j < stacks; j++)
			{
				unsigned topSlice = 0;
				unsigned botSlice = 1;
				unsigned topIndex = 0;
				unsigned botBeginIndex = 1 + (botSlice - 1)*stacks;
				unsigned botEndIndex = botBeginIndex + stacks - 1;
				unsigned botRightIndex = botBeginIndex + j;
				unsigned botLeftIndex = j + 2 > stacks ? botBeginIndex : botRightIndex + 1;

				unsigned index = indexOffset + j;
				m_index[3 * index] = topIndex; m_index[3 * index + 1] = botLeftIndex; m_index[3 * index + 2] = botRightIndex;
			}
			indexOffset += stacks;
			for (unsigned i = 1; i < (slice - 1); i++)
			{
				unsigned topSlice = i;
				unsigned botSlice = i + 1;
				unsigned topBeginIndex = 1 + (topSlice - 1)*stacks;
				unsigned topEndIndex = topBeginIndex + stacks - 1;
				unsigned botBeginIndex = 1 + (botSlice - 1)*stacks;
				unsigned botEndIndex = botBeginIndex + stacks - 1;
				for (unsigned j = 0; j < stacks; j++)
				{
					unsigned topRightIndex = topBeginIndex + j;
					unsigned topLeftIndex = j + 2 > stacks ? topBeginIndex : topRightIndex + 1;
					unsigned botRightIndex = botBeginIndex + j;
					unsigned botLeftIndex = j + 2 > stacks ? botBeginIndex : botRightIndex + 1;

					unsigned index = indexOffset + j * 2;
					m_index[3 * index] = topRightIndex; m_index[3 * index + 1] = topLeftIndex; m_index[3 * index + 2] = botRightIndex;
					index = indexOffset + j * 2 + 1;
					m_index[3 * index] = topLeftIndex; m_index[3 * index + 1] = botLeftIndex; m_index[3 * index + 2] = botRightIndex;
				}
				indexOffset += stacks * 2;
			}
			for (unsigned j = 0; j < stacks; j++)
			{
				unsigned topSlice = slice - 1;
				unsigned botSlice = slice;
				unsigned topBeginIndex = 1 + (topSlice - 1)*stacks;
				unsigned topEndIndex = topBeginIndex + stacks - 1;
				unsigned topRightIndex = topBeginIndex + j;
				unsigned topLeftIndex = j + 2 > stacks ? topBeginIndex : topRightIndex + 1;
				unsigned botIndex = vertCount - 1;

				unsigned index = indexOffset + j;
				m_index[3 * index] = topRightIndex; m_index[3 * index + 1] = topLeftIndex; m_index[3 * index + 2] = botIndex;
			}

			osg::Vec3Array* coord_array = (osg::Vec3Array*)OSGWrapper::ArrayCreator::CreateVec3Array(vertCount, m_vertex);
			osg::Vec3Array* normal_array = (osg::Vec3Array*)OSGWrapper::ArrayCreator::CreateVec3Array(vertCount, m_normal);
			osg::DrawElementsUInt* primitive_set = (osg::DrawElementsUInt*)OSGWrapper::ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::TRIANGLES, 3 * indexCount, m_index);

			delete[] m_vertex;
			delete[] m_normal;
			delete[] m_index;

			m_ball = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array, normal_array);
			m_ball->setCullingActive(false);
		}

		return m_ball;
	}

	GeometryCache::GeometryCache()
	{

	}

	GeometryCache::~GeometryCache()
	{

	}

	GeometryCache& GeometryCache::Instance()
	{
		if (!m_cache.get())
			m_cache.reset(new GeometryCache());

		return *m_cache.get();
	}

	osg::Geometry* GeometryCache::Get(CacheGeometryType type)
	{
		switch (type)
		{
		case CGT_Ball:
			return GetOrCreateBall();
		default:
			return 0;
		}

		return 0;
	}
}