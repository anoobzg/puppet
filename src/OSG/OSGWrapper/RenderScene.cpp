#include <osgWrapper\RenderScene.h>

namespace OSGWrapper
{
	RenderScene::RenderScene()
		:m_width(1920), m_height(1080), m_fovy(30.0f), m_near(1.0f), m_far(100.0f), m_render_view(0)
	{
		m_view_matrix_uniform = new osg::Uniform("view_matrix", osg::Matrixf::identity());
		m_projection_matrix_uniform = new osg::Uniform("projection_matrix", osg::Matrixf::identity());
		m_viewport_matrix_uniform = new osg::Uniform("viewport_matrix", osg::Matrixf::identity());
		getOrCreateStateSet()->addUniform(m_view_matrix_uniform, state_on);
		getOrCreateStateSet()->addUniform(m_projection_matrix_uniform, state_on);
		getOrCreateStateSet()->addUniform(m_viewport_matrix_uniform, state_on);

		m_width_uniform = new osg::Uniform("viewport_width", (float)m_width);
		m_height_uniform = new osg::Uniform("viewport_height", (float)m_height);
		getOrCreateStateSet()->addUniform(m_width_uniform, state_on);
		getOrCreateStateSet()->addUniform(m_height_uniform, state_on);
		SetViewMatrix(osg::Matrixf::identity());
		UpdateProjectionMatrix();
		UpdateViewportMatrix();
	}

	RenderScene::~RenderScene()
	{
		m_render_view = 0;
	}

	void RenderScene::SetViewMatrix(const osg::Matrixf& matrix)
	{
		setViewMatrix(matrix);
		m_view_matrix_uniform->set(matrix);
	}

	void RenderScene::SetProjectionMatrix(const osg::Matrixf& matrix)
	{
		setProjectionMatrix(matrix);
		m_projection_matrix_uniform->set(matrix);
	}

	void RenderScene::SetNearFar(float near, float far)
	{
		m_near = near;
		m_far = far;
		UpdateProjectionMatrix();
	}

	void RenderScene::SetFovy(float fovy)
	{
		m_fovy = fovy;
		UpdateProjectionMatrix();
	}

	void RenderScene::SetSize(unsigned width, unsigned height)
	{
		m_width = width;
		m_height = height;
		UpdateProjectionMatrix();

		m_width_uniform->set(float(m_width));
		m_height_uniform->set(float(m_height));
	}

	float RenderScene::GetFovy()
	{
		return m_fovy;
	}

	unsigned RenderScene::GetWidth()
	{
		return m_width;
	}

	unsigned RenderScene::GetHeight()
	{
		return m_height;
	}

	unsigned RenderScene::GetWidth() const
	{
		return m_width;
	}

	unsigned RenderScene::GetHeight() const
	{
		return m_height;
	}

	void RenderScene::UpdateProjectionMatrix()
	{
		osg::Matrixf m = osg::Matrixf::perspective((double)m_fovy, (double)m_width/(double)m_height, (double)m_near, (double)m_far);
		setProjectionMatrix(m);
		m_projection_matrix_uniform->set(m);
	}

	void RenderScene::UpdateViewportMatrix()
	{
		osg::Matrixf m = osg::Matrixf::translate(1.0f, 1.0f, 1.0f) * osg::Matrixf::scale(0.5f * (float)m_width, 0.5f * m_height, 0.5f);
		m_viewport_matrix_uniform->set(m);
	}

	bool RenderScene::OnFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	bool RenderScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	bool RenderScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	bool RenderScene::OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	bool RenderScene::OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		return false;
	}

	void RenderScene::CopyCameraMatrix(const RenderScene& scene)
	{
		SetViewMatrix(scene.getViewMatrix());

		double fovy;
		double near;
		double far;
		double ratio;
		scene.getProjectionMatrixAsPerspective(fovy, ratio, near,far);

		const osg::Viewport* view_port = scene.getViewport();
		SetNearFar((float)near, (float)far);
		SetFovy(fovy);
		if(view_port) SetSize(view_port->width(), view_port->height());
	}

	void RenderScene::ClearScene()
	{
		removeChildren(0, getNumChildren());
	}

	void RenderScene::OnEnter()
	{

	}

	void RenderScene::OnEnterOut()
	{
	}

	void RenderScene::AttachRenderView(RenderView* view)
	{
		m_render_view = view;
	}

	void RenderScene::GetRay(float x, float y, osg::Vec3f& eye_out, osg::Vec3f& center_out)
	{
		osg::Vec3f eye, center, up;

		osg::Vec4f screen_point(x, y, 0.0f, 1.0f);
		osg::Matrixf view_matrix = getViewMatrix();
		view_matrix.getLookAt(eye, center, up);
		osg::Matrixf projection_matrix = getProjectionMatrix();
		osg::Matrixf viewport_matrix = getViewport()->computeWindowMatrix();

		osg::Matrixf m = view_matrix * projection_matrix * viewport_matrix;
		osg::Vec4f world_point = screen_point * osg::Matrixf::inverse(m);

		if (world_point.w() != 0.0f)
		{
			world_point.x() = world_point.x() / world_point.w();
			world_point.y() = world_point.y() / world_point.w();
			world_point.z() = world_point.z() / world_point.w();
		}

		eye_out = eye;
		center_out = osg::Vec3f(world_point.x(), world_point.y(), world_point.z());
	}
}