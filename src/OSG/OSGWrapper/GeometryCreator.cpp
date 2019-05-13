#include <osgWrapper/GeometryCreator.h>
#include <osgWrapper/ArrayCreator.h>

namespace OSGWrapper
{

osg::Geometry* GeometryCreator::CreateIndexAttributeGeometry(osg::PrimitiveSet* primitive, osg::Array* attribute0, osg::Array* attribute1,
		osg::Array* attribute2, osg::Array* attribute3, osg::Array* attribute4)
{
	osg::Geometry* geom = new osg::Geometry();

	if(attribute0)
		geom->setVertexAttribArray(0, attribute0, osg::Array::BIND_PER_VERTEX);
	if(attribute1)
		geom->setVertexAttribArray(1, attribute1, osg::Array::BIND_PER_VERTEX);
	if(attribute2)
		geom->setVertexAttribArray(2, attribute2, osg::Array::BIND_PER_VERTEX);
	if(attribute3)
		geom->setVertexAttribArray(3, attribute3, osg::Array::BIND_PER_VERTEX);
	if(attribute4)
		geom->setVertexAttribArray(4, attribute4, osg::Array::BIND_PER_VERTEX);

	geom->addPrimitiveSet(primitive);
	geom->setUseVertexBufferObjects(true);
	geom->setUseDisplayList(false);
	geom->setCullingActive(false);

	return geom;
}

osg::Geometry* GeometryCreator::CreatePointCloud(std::vector<osg::Vec3f>& cloud, std::vector<osg::Vec3f>& normals)
{
	if (cloud.size() == 0)
		return 0;

	osg::Array* coord_array = ArrayCreator::CreateVec3Array(cloud.size(), (float*)&cloud[0]);
	osg::Array* normal_array = 0;
	if (normals.size() == cloud.size()) normal_array = ArrayCreator::CreateVec3Array(cloud.size(), (float*)&normals[0]);

	osg::Geometry* geom = new osg::Geometry();
	geom->setVertexAttribArray(0, coord_array, osg::Array::BIND_PER_VERTEX);
	if (normal_array) geom->setVertexAttribArray(1, normal_array, osg::Array::BIND_PER_VERTEX);

	osg::PrimitiveSet* primtive_set = new osg::DrawArrays(GL_POINTS, 0, cloud.size());
	geom->addPrimitiveSet(primtive_set);
	geom->setUseDisplayList(false);
	geom->setUseVertexBufferObjects(true);
	geom->setCullingActive(false);

	return geom;	
}

osg::Geometry* GeometryCreator::CreatePointCloud(std::vector<float>& cloud, std::vector<float>& normals)
{
	if (cloud.size() == 0 || cloud.size() != normals.size())
		return 0;

	size_t size = cloud.size() / 3;
	osg::Array* coord_array = ArrayCreator::CreateVec3Array(size, (float*)&cloud[0]);
	osg::Array* normal_array = ArrayCreator::CreateVec3Array(size, (float*)&normals[0]);
	osg::PrimitiveSet* primtive_set = new osg::DrawArrays(GL_POINTS, 0, size);

	return CreateIndexAttributeGeometry(primtive_set, coord_array, normal_array);
}

osg::Geometry* GeometryCreator::CreatePointCloud(float* cloud, float* normals, int count)
{
	if (cloud == 0 || count <= 0 || normals == 0)
		return 0;

	size_t size = (size_t)count;
	osg::Array* coord_array = ArrayCreator::CreateVec3Array(size, cloud);
	osg::Array* normal_array = ArrayCreator::CreateVec3Array(size, normals);
	osg::PrimitiveSet* primtive_set = new osg::DrawArrays(GL_POINTS, 0, size);

	return CreateIndexAttributeGeometry(primtive_set, coord_array, normal_array);
}

osg::Geometry* GeometryCreator::CreateLines(std::vector<osg::Vec3f>& lines)
{
	if (lines.size() <= 1)
		return 0;

	osg::Array* coord_array = ArrayCreator::CreateVec3Array(lines.size(), (float*)&lines[0]);

	osg::Geometry* geom = new osg::Geometry();
	geom->setVertexAttribArray(0, coord_array, osg::Array::BIND_PER_VERTEX);

	osg::PrimitiveSet* primtive_set = new osg::DrawArrays(GL_LINES, 0, lines.size());
	geom->addPrimitiveSet(primtive_set);
	geom->setUseDisplayList(false);
	geom->setUseVertexBufferObjects(true);
	geom->setCullingActive(false);

	return geom;
}

osg::Geometry* GeometryCreator::CreateLines(std::vector<osg::Vec3f>& points1, std::vector<osg::Vec3f>& points2)
{
	if (points1.size() == 0 || points2.size() == 0)
		return 0;
	if (points1.size() != points2.size())
		return 0;

	std::vector<osg::Vec3f> lines(2 * points1.size());
	for (size_t i = 0; i < points1.size(); ++i)
	{
		lines[2 * i] = points1[i];
		lines[2 * i + 1] = points2[i];
	}
	//memcpy(&lines[0], &points1[0], sizeof(float) * 3 * points1.size());
	//memcpy(&lines[points1.size()], &points2[0], sizeof(float) * 3 * points2.size());

	return CreateLines(lines);
}

}