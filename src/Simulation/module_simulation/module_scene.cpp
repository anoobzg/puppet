#include "module_scene.h"
#include <osgWrapper\RenderView.h>

#include <osg/LineWidth>
#include <osg/Depth>

#include "surface.h"
#include "collider.h"


#include <iostream>
ModuleScene::ModuleScene(Surface& surface, RenderView* view)
	:m_surface(surface), m_hove_ball(0), m_selected_ball(0), m_proper_point_radius(1.0f)
{
	m_picker = new ColorIndexPicker(view->getCamera(), GetWidth(), GetHeight());
	m_collider.reset(new Collider(m_surface, *m_picker));

	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new AttributeUtilNode();
	m_render_node->SetRenderProgram("curvature430_ex");
	m_manipulable_node->AddChild(m_render_node);
	m_manipulable_node->AddChild(m_picker);
	m_polygon_mode = new osg::PolygonMode();
	m_render_node->SetAttribute(m_polygon_mode);

	m_control_node = new AttributeUtilNode();
	m_control_node->SetRenderProgram("ball430");
	m_manipulable_node->AddChild(m_control_node);

	m_control_points_geode = new osg::Geode();
	m_control_points_geode->setCullingActive(false);
	m_control_node->AddChild(m_control_points_geode);

	m_control_path_node = new AttributeUtilNode();
	m_control_path_node->SetRenderProgram("purecolor430");
	m_control_path_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f)));
	m_control_path_node->SetAttribute(new osg::LineWidth(2.0f));
	m_control_path_node->SetAttribute(new osg::Depth(osg::Depth::ALWAYS));
	m_manipulable_node->AddChild(m_control_path_node);

	m_control_points_path_geode = new osg::Geode();
	m_control_points_path_geode->setCullingActive(false);
	m_control_path_node->AddChild(m_control_points_path_geode);

	m_curvature_geode = new CurvatureGeode();
	m_render_node->AddChild(m_curvature_geode);
	m_picker->SetNode(m_curvature_geode);

	m_surface.SetSurfaceTopoCallback(this);
	m_proper_point_radius = m_surface.GetProperRadius();

	ResetCamera();
}

ModuleScene::~ModuleScene()
{
	m_collider.reset();
}

void ModuleScene::ResetCamera()
{
	const osg::BoundingSphere& sphere = getBound();
	osg::Vec3f center = sphere.center();
	float radius = 0.5f * sphere.radius();

	float fovy = GetFovy();
	float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(0.0f, 0.0f, -1.0f) * len;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	SetNearFar(0.1f, len + 15.1f * radius);
}

bool ModuleScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool handled = false;
	if (MOUSE_MOVE(ea))
		return Hover(ea, aa);
	if (LEFT_MOUSE_PUSH(ea))
		handled = CheckCollide(ea, aa);
	else if (LEFT_MOUSE_DRAG(ea))
		handled = Drag(ea, aa);
	else if (RIGHT_MOUSE_PUSH(ea))
		handled = Delete(ea, aa);
	if (!handled) m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool ModuleScene::Delete(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	float ray_position[3] = { 0.0f };
	float ray_direction[3] = { 1.0f, 0.0f, 0.0f };
	ScreenRay(ea.getX(), ea.getY(), ray_position, ray_direction);

	unsigned control_point_handle = 0;
	bool result = m_collider->QueryControlPoint(ray_position, ray_direction, control_point_handle);
	if (result)
		m_surface.DeleteControlPoint(control_point_handle);

	return result;
}

bool ModuleScene::Drag(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (!m_selected_ball) return false;

	unsigned control_point_handle = m_selected_ball->Handle();

	float ray_position[3] = { 0.0f };
	float ray_direction[3] = { 1.0f, 0.0f, 0.0f };
	ScreenRay(ea.getX(), ea.getY(), ray_position, ray_direction);

	float vertex[3];
	unsigned vertex_handle = 0;

	bool result = m_collider->QueryVertex(ray_position, ray_direction, ea.getX(), ea.getY(), 0, vertex_handle, vertex);
	if (result)
		m_surface.ModifyControlPoint(control_point_handle, vertex_handle, vertex);
	return true;
}

bool ModuleScene::CheckCollide(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	HoverControlPoint(-1);
	if (m_selected_ball) m_selected_ball->Unselect();
	m_selected_ball = 0;

	unsigned control_point_handle = 0;

	float ray_position[3] = { 0.0f };
	float ray_direction[3] = { 1.0f, 0.0f, 0.0f };
	ScreenRay(ea.getX(), ea.getY(), ray_position, ray_direction);

	if (m_collider->QueryControlPoint(ray_position, ray_direction, control_point_handle))
	{
		SelectControlPoint(control_point_handle);
	}
	else
	{
		SelectControlPoint(-1);
	}

	return false;
}

bool ModuleScene::Hover(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	unsigned control_point_handle = 0;

	float ray_position[3] = { 0.0f };
	float ray_direction[3] = { 1.0f, 0.0f, 0.0f };
	ScreenRay(ea.getX(), ea.getY(), ray_position, ray_direction);

	if (m_collider->QueryControlPoint(ray_position, ray_direction, control_point_handle))
	{
		HoverControlPoint(control_point_handle);
	}
	else
	{
		HoverControlPoint(-1);
	}

	return true;
}

void ModuleScene::SelectControlPoint(unsigned control_point_handle)
{
	std::map<unsigned, osg::ref_ptr<ControlBall>>::iterator it = m_control_balls.find(control_point_handle);
	if (it != m_control_balls.end())
	{
		m_selected_ball = (*it).second;
		if (m_selected_ball) m_selected_ball->Select();
	}
}

void ModuleScene::HoverControlPoint(unsigned control_point_handle)
{
	std::map<unsigned, osg::ref_ptr<ControlBall>>::iterator it = m_control_balls.find(control_point_handle);
	if (m_hove_ball) m_hove_ball->Unhover();
	m_hove_ball = 0;

	if (it != m_control_balls.end())
	{
		m_hove_ball = (*it).second;
		if (m_hove_ball) m_hove_ball->Hover();
	}
}

bool ModuleScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, GUIEventAdapter::KEY_P))
	{
		static bool line_mode = false;
		line_mode = !line_mode;

		m_polygon_mode->setMode(osg::PolygonMode::FRONT_AND_BACK, line_mode ? osg::PolygonMode::LINE : osg::PolygonMode::FILL);
	}
	else if (KEY_DOWN(ea, GUIEventAdapter::KEY_G))
	{
		m_surface.ShowGuassCurvature();
	}
	else if (KEY_DOWN(ea, GUIEventAdapter::KEY_M))
	{
		m_surface.ShowMeanCurvature();
	}
	return true;
}

bool ModuleScene::OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	return true;
}

bool ModuleScene::OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (LEFT_MOUSE_DOUBLE_CLICK(ea))
	{
		unsigned vertex_handle = -1;
		float vertex[3];
		float ray_position[3] = { 0.0f };
		float ray_direction[3] = { 1.0f, 0.0f, 0.0f };
		ScreenRay(ea.getX(), ea.getY(), ray_position, ray_direction);
		bool result = m_collider->QueryVertex(ray_position, ray_direction, ea.getX(), ea.getY(), 0, vertex_handle, vertex);
		if (result) m_surface.TryAddControlPoint(vertex_handle, vertex);
	}
	return true;
}

void ModuleScene::ScreenRay(float x, float y, float* ray_position, float* ray_direction)
{
	osg::Vec3f eye, center;
	GetRay(x, y, eye, center);

	osg::Matrixf model_matrix = m_manipulable_node->GetLocalMatrix();
	model_matrix = osg::Matrixf::inverse(model_matrix);

	osg::Vec3f local_eye = eye * model_matrix;
	osg::Vec3f local_center = center * model_matrix;

	ray_position[0] = local_eye.x(); ray_position[1] = local_eye.y(); ray_position[2] = local_eye.z();
	osg::Vec3f eye_to_center = local_center - local_eye;
	eye_to_center.normalize();

	ray_direction[0] = eye_to_center.x(); ray_direction[1] = eye_to_center.y(); ray_direction[2] = eye_to_center.z();
}

void ModuleScene::ShowSurface(Mesh& mesh)
{
	m_curvature_geode->Reload(mesh);
}

void ModuleScene::SurfaceCurvatureChanged(unsigned verrtex_number, float* curvature)
{
	m_curvature_geode->UpdateCurvature(verrtex_number, curvature);
}

void ModuleScene::ControlPointAdded(ControlPoint& control_point)
{
	unsigned handle = control_point.m_handle;

	ControlBall* geometry = new ControlBall(handle);
	geometry->SetPosition(control_point.m_xyz.x, control_point.m_xyz.y, control_point.m_xyz.z);
	m_control_balls.insert(std::pair<unsigned, osg::ref_ptr<ControlBall>>(handle, geometry));
	m_control_points_geode->addChild(geometry);
}

void ModuleScene::ControlPointDeleted(ControlPoint& control_point)
{
	unsigned handle = control_point.m_handle;

	std::map<unsigned, osg::ref_ptr<ControlBall>>::iterator it = m_control_balls.find(handle);
	if (it != m_control_balls.end())
	{
		if (m_hove_ball == (*it).second)
			m_hove_ball = nullptr;
		if (m_selected_ball == (*it).second)
			m_selected_ball = nullptr;

		m_control_points_geode->removeChild((*it).second);
		m_control_balls.erase(it);

		std::cout << "Remove " << handle << std::endl;
	}
}

void ModuleScene::ControlPointModified(ControlPoint& control_point)
{
	unsigned handle = control_point.m_handle;

	std::map<unsigned, osg::ref_ptr<ControlBall>>::iterator it = m_control_balls.find(handle);
	if (it != m_control_balls.end())
	{
		(*it).second->SetPosition(control_point.m_xyz.x, control_point.m_xyz.y, control_point.m_xyz.z);
		std::cout << "Modify " << handle << std::endl;
	}
}

void ModuleScene::PathAdded(Path& path)
{

}

void ModuleScene::PathRemoved(Path& path)
{

}