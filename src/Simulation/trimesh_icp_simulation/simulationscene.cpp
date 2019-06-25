#include "simulationscene.h"
#include "patchnode.h"
#include <osgWrapper\MatrixAnimation.h>

SimulationScene::SimulationScene()
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_manipulable_node->AddChild(m_render_node);

	m_animation_scheduler = new OSGWrapper::AnimationScheduler();
	setUpdateCallback(m_animation_scheduler);
}

SimulationScene::~SimulationScene()
{

}

void SimulationScene::ShowOneFrame(osg::Geometry* geometry, const osg::Matrixf& matrix)
{
	float time = 0.0f;
	if (m_render_view) time = (float)m_render_view->getFrameStamp()->getSimulationTime();
	PatchNode* node = new PatchNode();
	node->AddChild(geometry);
	node->UpdateMatrix(matrix);
	node->SetTime(time);

	//m_manipulable_node->SetMatrix(osg::Matrixf::inverse(matrix));

	render_lock.Acquire();
	m_render_node->AddChild(node);

	if(m_render_node->getNumChildren() == 1)
		UpdateCamera();

	m_animation_scheduler->Clear();
	OSGWrapper::MatrixAnimation* animation = new OSGWrapper::MatrixAnimation(*m_manipulable_node);
	animation->SetMatrix(osg::Matrixf::inverse(matrix));
	m_animation_scheduler->StartAnimation(animation, (double)time);
	render_lock.Release();
}

void SimulationScene::UpdateCamera()
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

bool SimulationScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}