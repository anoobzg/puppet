#include <osgWrapper/ColorIndexPicker.h>
#include <osgWrapper/ProgramManager.h>
#include <osgWrapper/StateDeclare.h>

#include <Windows.h>
#include <osg\RenderInfo>
#include <osg\GLExtensions>

#include <iostream>


namespace OSGWrapper
{

class FinalDrawCallback : public osg::Camera::DrawCallback
{
public:
	FinalDrawCallback(osg::FrameBufferObject* fbo)
		:m_framebuffer_object(fbo)
	{
	}

	virtual void operator()(osg::RenderInfo& render_info) const
	{
		unsigned contextID = render_info.getContextID();
		//osg::FBOExtensions* fboe = osg::FBOExtensions::instance(contextID, true);
		//if (fboe->isSupported() && m_framebuffer_object.valid())
		//{
		//	m_framebuffer_object->apply(*render_info.getState());
		//	GLenum pixel_format = GL_RGBA;
		//	unsigned char color[4] = { 0 };
		//
		//	glReadPixels(686, 600, 1, 1, pixel_format, GL_UNSIGNED_BYTE, (GLvoid*)color);
		//
		//	std::cout<<"thread id : "<<::GetCurrentThreadId()<<std::endl;
		//	fboe->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
		//}
	}

protected:
	osg::ref_ptr<osg::FrameBufferObject> m_framebuffer_object;
};

class PreDrawCallback : public osg::Camera::DrawCallback
{
public:
	PreDrawCallback(osg::FrameBufferObject* fbo)
		:m_framebuffer_object(fbo)
	{
	}

	virtual void operator()(osg::RenderInfo& render_info) const
	{
		unsigned contextID = render_info.getContextID();
		osg::GLExtensions* extension = osg::GLExtensions::Get(contextID, true);
		if (extension->isFrameBufferObjectSupported && m_framebuffer_object.valid())
			m_framebuffer_object->apply(*render_info.getState());
	}

protected:
	osg::ref_ptr<osg::FrameBufferObject> m_framebuffer_object;
};

class PostDrawCallback : public osg::Camera::DrawCallback
{
public:
	PostDrawCallback(){}

	virtual void operator()(osg::RenderInfo& render_info) const
	{
		unsigned contextID = render_info.getContextID();
		osg::GLExtensions* extension = osg::GLExtensions::Get(contextID, true);
		if (extension->isFrameBufferObjectSupported && extension->glBindFramebuffer)
			extension->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
	}
};

ColorIndexPicker::ColorIndexPicker(osg::Camera* camera, int width, int height)
	:m_camera(camera)
{
	getOrCreateStateSet()->setMode(GL_BLEND, state_off);
	getOrCreateStateSet()->setAttributeAndModes(ProgramManager::Instance().Get("multipick430"), state_on);

	m_framebuffer_object = new osg::FrameBufferObject();

	m_color_texture = new osg::Texture2D();
	m_color_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
	m_color_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
	m_color_texture->setInternalFormat(GL_RGBA);
	m_color_texture->setTextureSize(width, height);

	m_depth_texture = new osg::Texture2D();
	m_depth_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
	m_depth_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
	m_depth_texture->setInternalFormat(GL_DEPTH_COMPONENT);
	m_depth_texture->setTextureSize(width, height);

	m_framebuffer_object->setAttachment(osg::Camera::DEPTH_BUFFER,
		osg::FrameBufferAttachment(m_depth_texture));
	m_framebuffer_object->setAttachment(osg::Camera::COLOR_BUFFER0, 
		osg::FrameBufferAttachment(m_color_texture));

	setReferenceFrame(osg::Transform::RELATIVE_RF);
	setRenderOrder(osg::Camera::POST_RENDER, 0);
	setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	setDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	setReadBuffer(GL_COLOR_ATTACHMENT0_EXT);

	setPreDrawCallback(new PreDrawCallback(m_framebuffer_object));
	setPostDrawCallback(new PostDrawCallback());

	//setFinalDrawCallback(new FinalDrawCallback(m_framebuffer_object));
}

ColorIndexPicker::~ColorIndexPicker()
{
}

unsigned ColorIndexPicker::Pick(int x, int y)
{
	osg::ref_ptr<osg::GraphicsContext> gc = m_camera->getGraphicsContext();
	if(gc.valid())
	{
		unsigned contextID = gc->getState()->getContextID();
		//osg::FBOExtensions* fboe = osg::FBOExtensions::instance(contextID, true);
		//if (fboe->isSupported() && fboe->glBindFramebuffer)
		//{
		//
		//	m_framebuffer_object->apply(*gc->getState());
		//	GLenum pixel_format = GL_RGBA;
		//	unsigned char color[4] = { 0 };
		//
		//	glReadPixels(x, y, 1, 1, pixel_format, GL_UNSIGNED_BYTE, (GLvoid*)color);
		//	unsigned id = Char2ID(color);
		//
		//	fboe->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
		//	return id;
		//}
	}
	return 0;
}

void ColorIndexPicker::Pick(int x, int y, unsigned char* color)
{
	osg::ref_ptr<osg::GraphicsContext> gc = m_camera->getGraphicsContext();
	if(gc.valid())
	{
		//unsigned contextID = gc->getState()->getContextID();
		//osg::FBOExtensions* fboe = osg::FBOExtensions::instance(contextID, true);
		//if (fboe->isSupported() && fboe->glBindFramebuffer)
		//{
		//
		//	m_framebuffer_object->apply(*gc->getState());
		//	GLenum pixel_format = GL_RGBA;
		//
		//	glReadPixels(x, y, 1, 1, pixel_format, GL_UNSIGNED_BYTE, (GLvoid*)color);
		//	unsigned id = Char2ID(color);
		//
		//	fboe->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
		//}
	}
}


unsigned ColorIndexPicker::Char2ID(unsigned char* d)
{
	return ((unsigned)(*d))*(1 << 24) + ((unsigned)(*(d + 1)))*(1 << 16) + ((unsigned)(*(d + 2)))*(1 << 8) + (unsigned)(*(d + 3));
}

void ColorIndexPicker::Char2ID(unsigned char* d, unsigned& mesh_id, unsigned& primitive_id)
{
	mesh_id = (unsigned)(*d);
	primitive_id = ((unsigned)(*(d + 1)))*(1 << 16) + ((unsigned)(*(d + 2)))*(1 << 8) + (unsigned)(*(d + 3));
}

void ColorIndexPicker::Clear()
{
	removeChildren(0, getNumChildren());
}

void ColorIndexPicker::SetNode(osg::Node* node)
{
	addChild(node);
}

void ColorIndexPicker::Pick(int x, int y, unsigned& mesh_id, unsigned& primitive_id)
{
	osg::ref_ptr<osg::GraphicsContext> gc = m_camera->getGraphicsContext();
	if(gc.valid())
	{
		unsigned contextID = gc->getState()->getContextID();
		osg::GLExtensions* extension = osg::GLExtensions::Get(contextID, true);
		if (extension->isFrameBufferObjectSupported && extension->glBindFramebuffer)
		{
		
			m_framebuffer_object->apply(*gc->getState());
			GLenum pixel_format = GL_RGBA;
			unsigned char color[4] = { 0 };
		
			glReadPixels(x, y, 1, 1, pixel_format, GL_UNSIGNED_BYTE, (GLvoid*)color);
			Char2ID(color, mesh_id, primitive_id);
		
			extension->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
		
			std::cout<<"pick mesh id "<<mesh_id<<" primitive id "<<primitive_id<<std::endl;
		}
	}
}

osg::Texture2D* ColorIndexPicker::GetColorTexture()
{
	return m_color_texture;
}

osg::Texture2D* ColorIndexPicker::GetDepthTexture()
{
	return m_depth_texture;
}

}