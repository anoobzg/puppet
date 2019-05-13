#include <osgWrapper/EraseGeometry.h>
#include <osgWrapper/FeedbackProgram.h>

namespace OSGWrapper
{
	EraseGeometry::EraseGeometry(osg::Array* attribute_array, osg::Array* feedback_array, osg::Array* render_array)
	{
		m_attribute_array = attribute_array;
		m_feedback_array = feedback_array;
		m_render_array = render_array;
	}

	EraseGeometry::~EraseGeometry()
	{
	}

	void EraseGeometry::Exchange()
	{
		osg::ref_ptr<osg::Array> tmp = m_attribute_array;
		m_attribute_array = m_feedback_array;
		m_feedback_array = tmp;

		setVertexAttribArray(3, m_attribute_array, osg::Array::BIND_PER_VERTEX);
	}

	void EraseGeometry::SetArray(osg::Array* coord, osg::Array* normal, osg::Array* color, osg::PrimitiveSet* primitive_set)
	{
		setVertexAttribArray(0, coord, osg::Array::BIND_PER_VERTEX);
		//setVertexAttribArray(1, normal, osg::Array::BIND_PER_VERTEX);
		//setVertexAttribArray(2, color, osg::Array::BIND_PER_VERTEX);

		addPrimitiveSet(primitive_set);
		setUseVertexBufferObjects(true);
		setUseDisplayList(false);
	}

	void EraseGeometry::drawImplementation(osg::RenderInfo& renderInfo) const
	{
		//unsigned int contextID = renderInfo.getState()->getContextID();
		//
		//osg::Extensions * buffer_ext = osg::GLBufferObject::getExtensions(contextID, true);
		//FeedbackExtensions* feedback_ext = FeedbackExtensions::instance(contextID, true);
		//GLuint ubuff = m_feedback_array->getOrCreateGLBufferObject(contextID)->getGLObjectID();
		//GLuint fbuff = m_render_array->getOrCreateGLBufferObject(contextID)->getGLObjectID();
		//if(buffer_ext)
		//{
		//	buffer_ext->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, ubuff);
		//	buffer_ext->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 1, fbuff);
		//}
		//if(feedback_ext)
		//	feedback_ext->glBeginTransformFeedback(GL_POINTS);
		//
		//osg::Geometry::drawImplementation(renderInfo);
		//
		//if(feedback_ext)
		//	feedback_ext->glEndTransformFeedback();
		//if(buffer_ext)
		//{
		//	buffer_ext->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 1, 0);
		//	buffer_ext->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, 0);
		//}
	}
}