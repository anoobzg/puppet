#include "keyframetracerscene.h"
#include "keyframenode.h"

KeyFrameTracerScene::KeyFrameTracerScene()
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_keyframe_node = new OSGWrapper::AttributeUtilNode();
	m_current_node = new OSGWrapper::AttributeUtilNode();
	m_keyframe_node->SetRenderProgram("keyframe");
	m_current_node->SetRenderProgram("keyframe");
	m_manipulable_node->AddChild(m_keyframe_node);
	m_manipulable_node->AddChild(m_current_node);

	m_animation_scheduler = new OSGWrapper::AnimationScheduler();
	setUpdateCallback(m_animation_scheduler);
}

KeyFrameTracerScene::~KeyFrameTracerScene()
{

}

void KeyFrameTracerScene::UpdateCamera()
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

bool KeyFrameTracerScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

void KeyFrameTracerScene::Locate(osg::Geometry* geometry, const osg::Matrixf& matrix)
{
	render_lock.Acquire();

	if (geometry)
	{
		KeyFrameNode* node = new KeyFrameNode();
		node->AddChild(geometry);
		node->SetDefaultID();
		node->SetKeyMatrix(matrix);
		m_current_node->AddChild(node);
	}

	render_lock.Release();
}

void KeyFrameTracerScene::AddKeyFrame(osg::Geometry* geometry, const osg::Matrixf& matrix)
{
	render_lock.Acquire();

	if (geometry)
	{
		KeyFrameNode* node = new KeyFrameNode();
		node->AddChild(geometry);
		node->GenerateBoundingBox();
		node->SetKeyMatrix(matrix);
		m_keyframe_node->AddChild(node);
		if (m_keyframe_node->getNumChildren() == 1)
			UpdateCamera();
	}

	//m_manipulable_node->SetMatrix(osg::Matrixf::inverse(matrix));
	render_lock.Release();
}