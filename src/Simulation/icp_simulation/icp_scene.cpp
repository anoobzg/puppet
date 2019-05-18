#include "icp_scene.h"
#include <osg\Geode>
#include <osg\LineWidth>
#include <osg\Point>
#include "MeshGeodeBuilder.h"

ICPScene::ICPScene(Mesh& stable_mesh, Mesh& patch_mesh, GemTransform& tranform)
	:m_stable_mesh(stable_mesh), m_patch_mesh(patch_mesh)
{
	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);
	m_float_node = new ManipulableNode();
	m_float_node->UseModelUniform();

	m_stable_render_node = new AttributeUtilNode();
	m_stable_render_node->SetRenderProgram("pointphong430");
	m_stable_render_node->AddUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f)));
	m_stable_render_node->SetAttribute(new osg::Point(2.0f));

	m_patch_render_node = new AttributeUtilNode();
	m_patch_render_node->SetRenderProgram("pointphong430");
	m_patch_render_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f)));
	m_patch_render_node->SetAttribute(new osg::Point(2.0f));

	m_manipulable_node->AddChild(m_stable_render_node);
	m_manipulable_node->AddChild(m_float_node);
	m_float_node->AddChild(m_patch_render_node);

	m_stable_geode = new osg::Geode();
	m_stable_geode->setCullingActive(false);
	m_patch_geode = new osg::Geode();
	m_patch_geode->setCullingActive(false);

	m_stable_render_node->AddChild(m_stable_geode);
	m_patch_render_node->AddChild(m_patch_geode);

	for (unsigned i = 0; i < 4; ++i)
	{
		for (unsigned j = 0; j < 4; ++j)
		{
			m_patch_matrix(i, j) = tranform.m_matrix[4*j+i];
		}
	}

	m_service.reset(new ICPService(m_stable_mesh, m_patch_mesh, tranform));
	Setup();
	ResetCamera();
}

ICPScene::~ICPScene()
{

}

void ICPScene::ResetCamera()
{
	const osg::BoundingSphere& sphere = getBound();
	osg::Vec3f center = sphere.center();
	float radius = 0.8f * sphere.radius();

	float fovy = GetFovy();
	float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(0.0f, 0.0f, -1.0f) * len;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	SetNearFar(len - 3.1f * radius, len + 3.1f * radius);
}

void ICPScene::Setup()
{
	m_stable_geode->removeDrawables(0, m_stable_geode->getNumDrawables());
	m_patch_geode->removeDrawables(0, m_patch_geode->getNumDrawables());

	m_stable_geode->addDrawable((osg::Drawable*)OSGBuilder::MeshGeodeBuilder::Build(m_stable_mesh, OSGBuilder::MGT_TRIANGLE, OSGBuilder::MGT_POSITION | OSGBuilder::MGT_NORMAL));
	m_patch_geode->addDrawable((osg::Drawable*)OSGBuilder::MeshGeodeBuilder::Build(m_patch_mesh, OSGBuilder::MGT_TRIANGLE, OSGBuilder::MGT_POSITION | OSGBuilder::MGT_NORMAL));
	
	Update(m_patch_matrix);
}

bool ICPScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_R))
		Reset();
	else if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Space))
		RunICP();
	return true;
}

bool ICPScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

void ICPScene::Update(const osg::Matrixf& matrix)
{
	m_float_node->SetMatrix(matrix);
}

void ICPScene::RunICP()
{
	GemTransform result;
	m_service->Do(result);

	osg::Matrixf m = osg::Matrixf::identity();
	for (unsigned i = 0; i < 4; ++i)
	{
		for (unsigned j = 0; j < 4; ++j)
		{
			m(i, j) = result.m_matrix[4 * j + i];
		}
	}

	Update(m);
}

void ICPScene::Reset()
{
	Update(m_patch_matrix);
}
