#include "con_seg_scene.h"
#include <osg\Geode>
#include <osg\LineWidth>
#include <osg\Point>
#include "MeshGeodeBuilder.h"
#include <osg\Geometry>

ConSegScene::ConSegScene(Mesh& mesh, SegModule& module)
	:m_mesh(mesh), m_module(module)
{
	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new AttributeUtilNode();
	m_render_node->SetRenderProgram("seg430");
	m_uniform = new osg::Uniform("branch", (unsigned)0);
	m_render_node->AddUniform(m_uniform);
	m_manipulable_node->AddChild(m_render_node);

	m_geode = new osg::Geode();
	m_render_node->AddChild(m_geode);

	Setup();
	ResetCamera();
}

ConSegScene::~ConSegScene()
{

}

void ConSegScene::ResetCamera()
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

void ConSegScene::Setup()
{
	m_geode->removeDrawables(0, m_geode->getNumDrawables());
	osg::Geometry* geom = OSGBuilder::MeshGeodeBuilder::Build(m_mesh, OSGBuilder::MGT_TRIANGLE, OSGBuilder::MGT_POSITION | OSGBuilder::MGT_NORMAL );
	geom->setCullingActive(false);

	m_color_array = new osg::FloatArray(m_mesh.vertex_number);
	geom->setVertexAttribArray(2, m_color_array, osg::Array::BIND_PER_VERTEX);

	m_geode->addDrawable(geom);
}

bool ConSegScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Space))
		UpdateOneStep();
	return true;
}

bool ConSegScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

void ConSegScene::UpdateOneStep()
{
	unsigned branch = 0;
	m_module.Step((float*)&m_color_array->at(0), branch);

	m_color_array->dirty();
	m_uniform->set(branch);
}

