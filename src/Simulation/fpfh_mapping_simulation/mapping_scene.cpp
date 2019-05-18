#include "mapping_scene.h"
#include <osg\Geode>
#include <osg\LineWidth>
#include <osg\Point>

#include "feature_object_traits.h"
#include "mapping.h"
MappingScene::MappingScene(FeatureObject& fobject1, FeatureObject& fobject2)
	:m_fobject1(fobject1), m_fobject2(fobject2)
{
	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new AttributeUtilNode();
	m_render_node->SetRenderProgram("pointphong430");
	m_render_node->AddUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f)));
	m_render_node->SetAttribute(new osg::Point(2.0f));

	m_mapping_node = new AttributeUtilNode();
	m_mapping_node->SetRenderProgram("purecolor430");
	m_mapping_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f)));
	m_mapping_node->SetAttribute(new osg::LineWidth(1.0f));
	m_mapping_node->SetAttribute(new osg::Point(10.0f));

	m_manipulable_node->AddChild(m_render_node);
	m_manipulable_node->AddChild(m_mapping_node);

	m_geode1 = new osg::Geode();
	m_geode1->setCullingActive(false);
	m_geode2 = new osg::Geode();
	m_geode2->setCullingActive(false);
	m_mapping_geode = new osg::Geode();
	m_boundary_geode = new osg::Geode();

	m_render_node->AddChild(m_geode1);
	m_render_node->AddChild(m_geode2);
	m_mapping_node->AddChild(m_mapping_geode);
	m_mapping_node->AddChild(m_boundary_geode);

	Setup();
	ResetCamera();
}

MappingScene::~MappingScene()
{

}

void MappingScene::ResetCamera()
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

void MappingScene::Setup()
{
	m_geode1->removeDrawables(0, m_geode1->getNumDrawables());
	m_geode2->removeDrawables(0, m_geode2->getNumDrawables());
	m_mapping_geode->removeDrawables(0, m_mapping_geode->getNumDrawables());

	m_geode1->addDrawable(FeatureObjectTraits::CreateObjectPointCloud(m_fobject1, true));
	m_geode2->addDrawable(FeatureObjectTraits::CreateObjectPointCloud(m_fobject2, true));
}

bool MappingScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_M)
		Map();
	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_S)
		StepMap();
	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_L)
		LoweMap();
	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_B)
		ShowBoundary();
	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_I)
		ShowISSKeypoints();
	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_K)
		ISSKeyPointMap();
	return true;
}

bool MappingScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

void MappingScene::Map()
{
	std::vector<unsigned> indices1;
	std::vector<unsigned> indices2;
	Mapping::Do(m_fobject1, m_fobject2, indices1, indices2);
	GenerateMapping(indices1, indices2);
}

void MappingScene::LoweMap()
{
	std::vector<unsigned> indices1;
	std::vector<unsigned> indices2;
	Mapping::DoLowe(m_fobject1, m_fobject2, indices1, indices2);
	GenerateMapping(indices1, indices2);
}

void MappingScene::ISSKeyPointMap()
{
	std::vector<unsigned> indices1;
	std::vector<unsigned> indices2;
	Mapping::DoISS(m_fobject1, m_fobject2, indices1, indices2);
	GenerateMapping(indices1, indices2);
}

void MappingScene::StepMap()
{
	std::vector<unsigned> indices1;
	std::vector<unsigned> indices2;
	Mapping::DoStep(m_fobject1, m_fobject2, indices1, indices2);
	GenerateMapping(indices1, indices2);
}

void MappingScene::ShowBoundary()
{
	std::vector<osg::Vec3f> points;
	FeatureObjectTraits::GetBoundary(m_fobject1, m_fobject2, points);
	ShowPoints(points);
}

void MappingScene::ShowPoints(std::vector<osg::Vec3f>& points)
{
	m_boundary_geode->removeDrawables(0, m_boundary_geode->getNumDrawables());
	std::vector<osg::Vec3f> normals;
	osg::Geometry* points_geom = GeometryCreator::CreatePointCloud(points, normals);
	m_boundary_geode->addDrawable(points_geom);
}

void MappingScene::ShowISSKeypoints()
{
	std::vector<osg::Vec3f> points;
	FeatureObjectTraits::GetISSKeypoints(m_fobject1, m_fobject2, points);
	ShowPoints(points);
}

void MappingScene::GenerateMapping(std::vector<unsigned>& indices1, std::vector<unsigned>& indices2)
{
	m_mapping_geode->removeDrawables(0, m_mapping_geode->getNumDrawables());

	std::vector<osg::Vec3f> points1;
	std::vector<osg::Vec3f> points2;
	FeatureObjectTraits::GetPoints(m_fobject1, indices1, points1);
	FeatureObjectTraits::GetPoints(m_fobject2, indices2, points2);

	osg::Geometry* geom = GeometryCreator::CreateLines(points1, points2);
	m_mapping_geode->addDrawable(geom);
}