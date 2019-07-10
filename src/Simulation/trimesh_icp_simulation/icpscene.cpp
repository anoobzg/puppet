#include "icpscene.h"
#include "stepicptask.h"
#include "resettask.h"
#include "mappingtask.h"
#include <osg/Point>

ICPScene::ICPScene(trimesh::CameraData& data, trimesh::TriMesh& source, trimesh::TriMesh& target)
	:m_data(data)
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_source_node = new ICPNode(source);
	m_target_node = new ICPNode(target);
	m_source_node->SetColor(osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
	m_target_node->SetColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
	m_source_node->SetAttribute(new osg::Point(2.0f));
	m_target_node->SetAttribute(new osg::Point(2.0f));

	m_manipulable_node->AddChild(m_target_node);
	m_manipulable_node->AddChild(m_source_node);

	m_screen_graph = new ScreenGraph();
	m_manipulable_node->AddChild(m_screen_graph);

	m_space_lines = new OSGWrapper::AttributeUtilNode();
	m_space_lines->SetRenderProgram("purecolor430");
	m_space_lines->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)));
	m_manipulable_node->AddChild(m_space_lines);

	osg::BoundingSphere sphere;
	sphere.set(osg::Vec3f(0.0f, 0.0f, 0.0f), 400.0f);
	m_axis_node = new OSGWrapper::AttributeUtilNode();
	m_axis_node->setComputeBoundingSphereCallback(new osg::Node::ComputeBoundingSphereCallback());
	m_axis_node->SetRenderProgram("purecolor430");
	m_axis_node->AddChild(OSGWrapper::AxisCreator::Create(sphere));
	m_manipulable_node->AddChild(m_axis_node);
	UpdateCamera();
}

ICPScene::~ICPScene()
{

}

void ICPScene::UpdateCamera()
{
	const osg::BoundingSphere& sphere = m_manipulable_node->getBound();

	osg::Vec3f center = sphere.center();
	float radius = 0.8f * sphere.radius();

	float fovy = GetFovy();
	float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(0.0f, 0.0f, -1.0f) * len;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	SetNearFar(0.1f, len + 10.0f * radius);
}

bool ICPScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool ICPScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (m_task) return false;

	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_I))
	{
		StepICPTask* t = new StepICPTask(m_data, m_target_node, m_source_node, m_screen_graph);
		t->SetAttributeNode(m_space_lines);
		m_task = t;
		return true;
	}

	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_F))
	{
		StepICPTask* t = new StepICPTask(m_data, m_target_node, m_source_node, m_screen_graph);
		t->SetAttributeNode(m_space_lines);
		t->SetUseFast(true);
		m_task = t;
		return true;
	}

	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_R))
	{
		ResetICPTask* t = new ResetICPTask(m_source_node, m_screen_graph);
		t->SetAttributeNode(m_space_lines);
		m_task = t;
		return true;
	}

	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_O))
	{
		trimesh::ProjectionICP icp(m_data.m_fx, m_data.m_fy, m_data.m_cx, m_data.m_cy);
		icp.SetSource(&m_source_node->GetMesh());
		icp.SetTarget(&m_target_node->GetMesh());
		float error = icp.Do(m_source_node->GetMesh().global);
		return true;
	}

	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_M))
	{
		MappingTask* t = new MappingTask(m_data, m_target_node->GetMesh(), m_source_node->GetMesh());
		t->SetAttributeNode(m_space_lines);
		t->SetSelf(false);
		m_task = t;
		return true;
	}
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_S))
	{
		MappingTask* t = new MappingTask(m_data, m_target_node->GetMesh(), m_source_node->GetMesh());
		t->SetAttributeNode(m_space_lines);
		t->SetSelf(true);
		m_task = t;
		return true;
	}
	return true;
}

bool ICPScene::OnFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (m_task)
	{
		if (!m_task->Execute())
			m_task = NULL;
	}
	return true;
}
