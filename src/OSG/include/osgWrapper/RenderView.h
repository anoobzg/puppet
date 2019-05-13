#ifndef RENDER_VIEW_H
#define RENDER_VIEW_H
#include <osgViewer/View>
#include <osgGA/GUIEventHandler>
#include <osgWrapper/RenderScene.h>

namespace OSGWrapper 
{
template<class T>
class GUIHandlerProxy : public osgGA::GUIEventHandler
{
public:
	GUIHandlerProxy(T& t):m_t(t) {}
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return m_t.handle(ea, aa);
	}
protected:
	T& m_t;
};

class OSG_EXPORT RenderView : public osgViewer::View
{
public:
	RenderView();
	~RenderView();

	void SetCurrentScene(RenderScene* scene, bool save_preview_scene = false);
	bool RollbackScene();
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void SetBackgroundColor(const osg::Vec4& color);
protected:
	void AddGlobalUniform(osg::Uniform* uniform);
protected:
	bool handleResizeEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool handleFrameEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool handleKeyEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool handleMouseEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool handleDoubleClickEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	unsigned m_frame_id;
	osg::ref_ptr<RenderScene> m_current_scene;
	osg::ref_ptr<RenderScene> m_preview_scene;

	osg::ref_ptr<osg::ClearNode> m_root;

	bool m_suppress;

	int m_width;
	int m_height;
};

}
#endif // RENDER_VIEW_H