#include <osgWrapper/Manipulator.h>
#include <osgWrapper/ManipulableNode.h>
#include <osgWrapper/OSGMathUtil.h>

namespace OSGWrapper
{

Manipulator::Manipulator(ManipulableNode& node)
	:m_manipulable_node(node), m_rotate_enable(true), m_translate_enable(true), m_scale_enable(true)
	,m_scale(1.0f), m_use_center(false)
{
}

Manipulator::~Manipulator()
{
}

void Manipulator::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Camera& camera)
{
	osgGA::GUIEventAdapter::EventType event_type = ea.getEventType(); 

	if (event_type == osgGA::GUIEventAdapter::RELEASE && ea.getButtonMask() == 0)
	{
		m_saved_event = 0;
		m_operation_mode = 0;
	}
	else if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH)
	{
		m_operation_mode = 0;
		m_saved_event = (osgGA::GUIEventAdapter*)(&ea);
		if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
		{
			m_operation_mode = 1;
			return;
		}
		else if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
		{
			m_operation_mode = 2;
			return;
		}
		else if (ea.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
		{
			m_operation_mode = 3;
			return;
		}
		return;
	}
	else if (event_type == osgGA::GUIEventAdapter::DRAG)
	{
		float x = ea.getX();
		float y = ea.getY();
		float x0 = x;
		float y0 = y;

		if(m_saved_event.get())
		{
			x0 = m_saved_event->getX();
			y0 = m_saved_event->getY();
		}

		m_saved_event = (osgGA::GUIEventAdapter*)(&ea);
		if (m_operation_mode == 1 && m_rotate_enable)
		{
			float width = (float)camera.getViewport()->width();
			float height = (float)camera.getViewport()->height();

			osg::Vec2f window_center = osg::Vec2f(width / 2.0f, height / 2.0f );;
			osg::Vec3f p0 = OSGMathUtil::Get3DPoint(osg::Vec2f(x0, y0), window_center, width, height, false);
			osg::Vec3f p = OSGMathUtil::Get3DPoint(osg::Vec2f(x, y), window_center, width, height, false);

			float angle = OSGMathUtil::Angle(p0, p);

			if (angle == 0.0f)
				return;

			osg::Vec3f axis = p0 ^ p;
			axis.normalize();

			Rotate(angle, axis, camera);
			return;
		}
		else if (m_operation_mode == 2 && m_translate_enable)
		{
			Translate( x - x0, y - y0, camera);
			return;
		}
		else if (m_operation_mode == 3 && m_scale_enable)
		{
			float height = (float)camera.getViewport()->height();
			float fScaleFactor = 1.0f + (y - y0) / height;
			Scale(fScaleFactor, camera);
			return;
		}
	}
	else if (event_type == osgGA::GUIEventAdapter::SCROLL)
	{
		if (m_operation_mode == 0 && m_scale_enable)
		{
			osgGA::GUIEventAdapter::ScrollingMotion sm = ea.getScrollingMotion();
			float delta = 0;
			if (sm == osgGA::GUIEventAdapter::SCROLL_UP)
				delta = 0.05;
			else if (sm == osgGA::GUIEventAdapter::SCROLL_DOWN)
				delta = -0.05;
			if (delta == 0)
				return;

			float delta_y = ea.getScrollingDeltaY();
			float delta_x = ea.getScrollingDeltaX();

			float fScale_factor = 1.0f + delta;

			Scale(fScale_factor, camera);

			return;
		}
	}
	return;
}

void Manipulator::EnableRotate(bool enable)
{
	m_rotate_enable = enable;
}

void Manipulator::EnableScale(bool enable)
{
	m_scale_enable = enable;
}

void Manipulator::EnableTranslate(bool enable)
{
	m_translate_enable = enable;
}

void Manipulator::Translate(float dx, float dy, osg::Camera& camera)
{
	const osg::BoundingSphere& sphere = m_manipulable_node.getBound();
	osg::Vec3d center = sphere.center();

	osg::Vec3f eye, up, camera_center;
	osg::Matrixf view_matrix = camera.getViewMatrix();
	view_matrix.getLookAt(eye, camera_center, up);
	osg::Vec3d center_eye = center - eye;
	osg::Vec3d dir = camera_center - eye;
	dir.normalize();
	double d = std::abs(center_eye * dir);

	float fovy, aspect, near_plane, far_plane;
	osg::Matrixf projection_matrix = camera.getProjectionMatrix();
	projection_matrix.getPerspective(fovy, aspect, near_plane, far_plane);

	float height = 2.0f * near_plane * tan(fovy * 3.1415926f / 180.0f / 2.0f);
	float width = aspect * height;
	float ratio = d / near_plane;

	float x = width * dx / (float)camera.getViewport()->width();
	float y = height * dy / (float)camera.getViewport()->height();
	osg::Vec3f vector(x * ratio, y * ratio, 0.0f);
	float len = vector.length();
	vector.normalize();
	
	osg::Vec3f delta = camera.getViewMatrix().getRotate().inverse() * vector;
	m_translate_vector += delta * len;

	delta.normalize();

	const osg::Matrixf& m = m_manipulable_node.GetParentToScene();
	delta = m.getRotate().inverse() * delta;
	m_manipulable_node.SetMatrix(m_manipulable_node.GetLocalMatrix() * osg::Matrixf::translate(delta * len));
	//UpdateMatrix();
}

void Manipulator::Rotate(float angle, const osg::Vec3f& axis, osg::Camera& camera)
{
	osg::Quat new_rotate;

	osg::Vec3f camera_space_axis = camera.getViewMatrix().getRotate().inverse() * axis;
	//new_rotate.makeRotate(angle, camera_space_axis);
	//m_rotate_quat = m_rotate_quat * new_rotate;

	const osg::Matrixf& m = m_manipulable_node.GetParentToScene();
	osg::Vec3f delta = m.getRotate().inverse() * camera_space_axis;
	new_rotate.makeRotate(angle, delta);

	osg::BoundingSphere sphere = m_manipulable_node.getBound();

	osg::Vec3 center = sphere.center();
	if (m_use_center) center = m_operation_center;

	m_manipulable_node.SetMatrix(m_manipulable_node.GetLocalMatrix() * osg::Matrixf::translate(-center) * 
									osg::Matrixf::rotate(new_rotate) * osg::Matrixf::translate(center));
	//UpdateMatrix();
}

void Manipulator::Scale(float scale, osg::Camera& camera)
{
	float max_scale = 3.0f;
	float min_scale = 0.5f;
	m_scale *= scale;
	m_scale = m_scale > max_scale ? max_scale : ( m_scale < min_scale ? min_scale : m_scale);

	osg::BoundingSphere sphere = m_manipulable_node.getBound();

	osg::Vec3 center = sphere.center();
	if (m_use_center) center = m_operation_center;

	m_manipulable_node.SetMatrix(m_manipulable_node.GetLocalMatrix() * osg::Matrixf::translate(-center) *
										osg::Matrixf::scale(scale, scale, scale) * osg::Matrixf::translate(center));
	//UpdateMatrix();
}

void Manipulator::UpdateMatrix()
{
	osg::Matrixf matrix = osg::Matrixf::translate(-m_operation_center) * osg::Matrixf::rotate(m_rotate_quat)
		* osg::Matrixf::scale(osg::Vec3f(m_scale, m_scale, m_scale)) * osg::Matrixf::translate(m_translate_vector) * osg::Matrixf::translate(m_operation_center);

	m_manipulable_node.SetMatrix(matrix);
}

void Manipulator::Reset(bool reset_node)
{
	m_scale = 1.0f;
	m_translate_vector = osg::Vec3(0.0f, 0.0f, 0.0f);
	m_rotate_quat = osg::Quat();

	m_operation_center = m_manipulable_node.GetChildBounding().center();

	if(reset_node) m_manipulable_node.Reset();
}

void Manipulator::UpdateCenter()
{
	m_operation_center = m_manipulable_node.GetChildBounding().center();
}

void Manipulator::UseCenter(bool use, const osg::Vec3f& center)
{
	m_use_center = use;
	m_operation_center = center;
}
}