#include "octreescene.h"
#include "octreework.h"
#include "boundingbox.h"

OctreeScene::OctreeScene(OctreeWork& work)
	:m_work(work)
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_render_node->SetRenderProgram("purecolor430");
	m_render_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)));
	m_manipulable_node->AddChild(m_render_node);

	m_point_node = new OSGWrapper::AttributeUtilNode();
	m_point_node->AddUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f)));
	m_octree_node = new OctreeNode();
	m_render_node->AddChild(m_point_node);
	m_render_node->AddChild(m_octree_node);

	m_work.SetRenderScene(this);

	SetBounding();
	UpdateCamera();
}

OctreeScene::~OctreeScene()
{

}

bool OctreeScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool OctreeScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Left))
	{
		m_work.Move(-1);
		return true;
	}

	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Right))
	{
		m_work.Move(1);
		return true;
	}

	return true;
}

void OctreeScene::AddPoint(osg::Node* point)
{
	m_point_node->AddChild(point);
}

void OctreeScene::RemovePoint(osg::Node* point)
{
	m_point_node->RemoveChild(point);
}

void OctreeScene::SetBounding()
{
	BoundingBox* box = new BoundingBox();
	box->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)));
	box->UpdateBoundingBox(osg::Vec3f(-500.0f, -500.0f, -500.0f), osg::Vec3f(500.0f, 500.0f, 500.0f));
	m_render_node->AddChild(box);
}

void OctreeScene::UpdateCamera()
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