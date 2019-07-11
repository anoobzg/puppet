#include "simulationscene.h"
#include <osgWrapper\MatrixAnimation.h>
#include <osg\Point>

SimulationScene::SimulationScene()
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_manipulable_node->AddChild(m_render_node);
	m_base_node = new OSGWrapper::AttributeUtilNode();
	m_base_node->SetRenderProgram("phong430");
	m_base_node->SetAttribute(new osg::Point(1.0f));
	m_manipulable_node->AddChild(m_base_node);
	m_current_frame = new OSGWrapper::AttributeUtilNode();
	m_current_frame->SetRenderProgram("distancephong");
	m_align_matrix = new osg::Uniform("align_matrix", osg::Matrixf::identity());
	m_current_frame->AddUniform(m_align_matrix);
	m_current_frame->SetAttribute(new osg::Point(8.0f));
	m_manipulable_node->AddChild(m_current_frame);

	m_patch_node = new OSGWrapper::AttributeUtilNode();
	m_patch_node->SetRenderProgram("esscanning");
	//m_patch_node->SetAttribute(new osg::Point(5.0f));
	m_manipulable_node->AddChild(m_patch_node);

	m_line = new OSGWrapper::ScreenLineText("D:\\Data\\Fonts\\1\\xingshu.ttf", 7);
	addChild(m_line);
	m_line->SetText(L"1234567");
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

	if (m_current_node) m_current_node->SetCurrent(false);

	m_current_node = new PatchNode();
	m_current_node->AddChild(geometry);
	m_current_node->UpdateMatrix(matrix);
	m_current_node->SetTime(time);
	m_current_node->SetCurrent(true);
	//m_manipulable_node->SetMatrix(osg::Matrixf::inverse(matrix));

	render_lock.Acquire();
	m_render_node->AddChild(m_current_node);

	if(m_render_node->getNumChildren() == 1)
		UpdateCamera();

	m_animation_scheduler->Clear();
	OSGWrapper::MatrixAnimation* animation = new OSGWrapper::MatrixAnimation(*m_manipulable_node);
	animation->SetMatrix(osg::Matrixf::inverse(matrix));
	m_animation_scheduler->StartAnimation(animation, (double)time);
	render_lock.Release();
}

void SimulationScene::UpdateCountText(int count)
{
	wchar_t chars[64];
	wsprintfW(chars, L"%d", count);

	std::wstring text(chars);
	m_line->SetText(text);
}

void SimulationScene::UpdateMatrix(const osg::Matrixf& matrix, bool use_animation)
{
	render_lock.Acquire();
	m_animation_scheduler->Clear();
	if (use_animation)
	{
		float time = 0.0f;
		if (m_render_view) time = (float)m_render_view->getFrameStamp()->getSimulationTime();
		OSGWrapper::MatrixAnimation* animation = new OSGWrapper::MatrixAnimation(*m_manipulable_node);
		animation->SetMatrix(osg::Matrixf::inverse(matrix));
		m_animation_scheduler->StartAnimation(animation, (double)time);
	}
	else
		m_manipulable_node->SetMatrix(osg::Matrixf::inverse(matrix));

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

void SimulationScene::AddPatch(osg::Geometry* geometry, bool update_camera)
{
	m_patch_node->AddChild(geometry);
}

void SimulationScene::UpdateCam()
{
	UpdateCamera();
}

void SimulationScene::Lock()
{
	render_lock.Acquire();
}

void SimulationScene::Unlock()
{
	render_lock.Release();
}

float SimulationScene::GetTime()
{
	float t = 0.0f;
	if (m_render_view)
	{
		t = (float)m_render_view->getFrameStamp()->getSimulationTime();
	}
	return t;
}

void SimulationScene::ShowCurrentFrame(osg::Geometry* geometry, const osg::Matrixf& matrix)
{
	if (!geometry) return;

	bool first_frame = m_current_frame->getNumChildren() == 0;
	render_lock.Acquire();
	m_current_frame->RemoveAll();
	m_current_frame->AddChild(geometry);
	if (first_frame)
		UpdateCamera();

	m_align_matrix->set(matrix);
	m_animation_scheduler->Clear();
	OSGWrapper::MatrixAnimation* animation = new OSGWrapper::MatrixAnimation(*m_manipulable_node);
	animation->SetMatrix(osg::Matrixf::inverse(matrix));
	float time = 0.0f;
	if (m_render_view) time = (float)m_render_view->getFrameStamp()->getSimulationTime();
	m_animation_scheduler->StartAnimation(animation, (double)time);
	render_lock.Release();
}

void SimulationScene::AppendNewPoints(osg::Geometry* geometry)
{
	if (!geometry) return;
	//render_lock.Acquire();
	//if(index == (int)m_base_node->getNumChildren())
		m_base_node->AddChild(geometry);
	//else
	//	m_base_node->replaceChild(m_base_node->getChild(index), geometry);
	//render_lock.Release();
}