#ifndef WRAPPER_RENDER_SCENE
#define WRAPPER_RENDER_SCENE
#include <osgWrapper/StateDeclare.h>
#include <osg/Camera>
#include <osgWrapper/ProgramManager.h>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>

namespace OSGWrapper
{
	class RenderView;
	class OSG_EXPORT RenderScene : public osg::Camera
	{
	public:
		RenderScene();
		~RenderScene();

		void SetViewMatrix(const osg::Matrixf& matrix);
		void SetProjectionMatrix(const osg::Matrixf& matrix);
		void SetNearFar(float near, float far);
		void SetFovy(float fovy);
		void SetSize(unsigned width, unsigned height);
		float GetFovy();
		unsigned GetWidth();
		unsigned GetHeight();
		unsigned GetWidth() const;
		unsigned GetHeight() const;

		virtual bool OnFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		virtual bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		virtual bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		virtual bool OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
		virtual bool OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

		void CopyCameraMatrix(const RenderScene& scene);

		void ClearScene();

		virtual void OnEnter();
		virtual void OnEnterOut();

		void AttachRenderView(RenderView* view);

		void GetRay(float x, float y, osg::Vec3f& eye, osg::Vec3f& center);
	protected:
		void UpdateProjectionMatrix();
		void UpdateViewportMatrix();
	private:
		unsigned m_width;
		unsigned m_height;

		float m_fovy;
		float m_near;
		float m_far;

		osg::ref_ptr<osg::Uniform> m_view_matrix_uniform;
		osg::ref_ptr<osg::Uniform> m_projection_matrix_uniform;
		osg::ref_ptr<osg::Uniform> m_viewport_matrix_uniform;

		osg::ref_ptr<osg::Uniform> m_width_uniform;
		osg::ref_ptr<osg::Uniform> m_height_uniform;
	protected:
		RenderView* m_render_view;
	};

#define MOUSE_MOVE(event) (event.getEventType()==osgGA::GUIEventAdapter::MOVE)
#define LEFT_MOUSE_PUSH(event) (event.getEventType()==osgGA::GUIEventAdapter::PUSH && event.getButtonMask()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
#define LEFT_MOUSE_DRAG(event) (event.getEventType()==osgGA::GUIEventAdapter::DRAG && event.getButtonMask()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
#define LEFT_MOUSE_RELEASE(event) (event.getEventType()==osgGA::GUIEventAdapter::RELEASE && (event.getButton()&osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
#define RIGHT_MOUSE_DRAG(event) (event.getEventType()==osgGA::GUIEventAdapter::DRAG && event.getButtonMask()==osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
#define RIGHT_MOUSE_PUSH(event) (event.getEventType()==osgGA::GUIEventAdapter::PUSH && event.getButtonMask()==osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)

using namespace osgGA;
#define KEY_DOWN(event, key) (event.getEventType()==osgGA::GUIEventAdapter::KEYDOWN && event.getKey()==key)

#define LEFT_MOUSE_DOUBLE_CLICK(event) (event.getEventType()==osgGA::GUIEventAdapter::DOUBLECLICK && event.getButton()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
}

#endif // WRAPPER_RENDER_SCENE