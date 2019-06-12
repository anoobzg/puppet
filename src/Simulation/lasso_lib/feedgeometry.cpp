#include "FeedGeometry.h"
#include <osgWrapper/FeedbackProgram.h>
#include <osg\GLExtensions>

class FeedGeomtryCallback : public osg::NodeCallback
{
public:
	FeedGeomtryCallback(FeedGeometry* geometry)
		:m_geometry(geometry)
	{

	}

	void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		if (m_geometry)
			m_geometry->Exchange();
	}

protected:
	FeedGeometry* m_geometry;
};

osg::BoundingBox FeedGeometry::computeBoundingBox() const
{
	return osg::BoundingBox();
}

FeedGeometry::FeedGeometry(osg::Vec3Array* coord_array, osg::FloatArray* flag_array)
{
	m_in_array = flag_array;
	m_in_array->setVertexBufferObject(new osg::VertexBufferObject());
	m_coord_array = coord_array;

	setVertexAttribArray(0, m_coord_array, osg::Array::BIND_PER_VERTEX);
	setVertexAttribArray(1, m_in_array, osg::Array::BIND_PER_VERTEX);

	addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, m_coord_array->size()));
	setUseVertexBufferObjects(true);
	setUseDisplayList(false);
	setCullingActive(false);

	//setUpdateCallback(new FeedGeomtryCallback(this));
}

FeedGeometry::~FeedGeometry()
{

}

void FeedGeometry::Exchange()
{
	//osg::ref_ptr<osg::FloatArray> tmp = m_in_array;
	//m_in_array = m_out_array;
	//m_out_array = tmp;

	//setVertexAttribArray(1, m_in_array, osg::Array::BIND_PER_VERTEX);
}

void FeedGeometry::drawImplementation(osg::RenderInfo& renderInfo) const
{
	unsigned int contextID = renderInfo.getState()->getContextID();

	osg::GLExtensions* buffer_ext = renderInfo.getState()->get<osg::GLExtensions>();
	GLuint ubuff = m_in_array->getOrCreateGLBufferObject(contextID)->getGLObjectID();

	renderInfo.getState()->checkGLErrors("hello1");
	glEnable(GL_RASTERIZER_DISCARD);
	if (buffer_ext)
	{
		buffer_ext->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, ubuff);
		buffer_ext->glBeginTransformFeedback(GL_POINTS);
	}
	renderInfo.getState()->checkGLErrors("hello2");
	osg::Geometry::drawImplementation(renderInfo);

	renderInfo.getState()->checkGLErrors("hello3");
	if (buffer_ext)
	{
		buffer_ext->glEndTransformFeedback();
		buffer_ext->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, 0);
	}
	renderInfo.getState()->checkGLErrors("hello4");

	glDisable(GL_RASTERIZER_DISCARD);
}