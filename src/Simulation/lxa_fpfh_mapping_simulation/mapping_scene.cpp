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

	m_trans_node = new ManipulableNode();
	m_trans_node->UseModelUniform();

	m_render_node_1 = new AttributeUtilNode();
	m_render_node_1->SetRenderProgram("pointphong430");
	m_render_node_1->AddUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f)));
	m_render_node_1->SetAttribute(new osg::Point(2.0f));

	m_render_node_2 = new AttributeUtilNode();
	m_render_node_2->SetRenderProgram("pointphong430");
	m_render_node_2->AddUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f)));
	m_render_node_2->SetAttribute(new osg::Point(2.0f));

	m_mapping_node = new AttributeUtilNode();
	m_mapping_node->SetRenderProgram("purecolor430");
	m_mapping_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f)));
	m_mapping_node->SetAttribute(new osg::LineWidth(1.0f));
	m_mapping_node->SetAttribute(new osg::Point(10.0f));

	m_matrix = osg::Matrixf::translate(10.0f, 00.0f, 0.0f);
	m_trans_node->SetMatrix(m_matrix);
	m_manipulable_node->AddChild(m_trans_node);
	m_manipulable_node->AddChild(m_render_node_1);
	m_trans_node->AddChild(m_render_node_2);
	m_manipulable_node->AddChild(m_mapping_node);

	m_geode1 = new osg::Geode();
	m_geode1->setCullingActive(false);
	m_geode2 = new osg::Geode();
	m_geode2->setCullingActive(false);
	m_mapping_geode = new osg::Geode();

	m_render_node_1->AddChild(m_geode1);
	m_render_node_2->AddChild(m_geode2);
	m_mapping_node->AddChild(m_mapping_geode);

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

	m_geode1->addDrawable(FeatureObjectTraits::CreateObjectPointCloud(m_fobject1));
	m_geode2->addDrawable(FeatureObjectTraits::CreateObjectPointCloud(m_fobject2));
}

bool MappingScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_T))
		Test();
	else if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_M))
		Mapping();
	return true;
}

bool MappingScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

#include "cloudFeature.h"
#include "feature_object.h"
void MappingScene::Test()
{
	CloudFeature::cloudFeature cloudfeature;
	pcl::CorrespondencesPtr corres_set(new pcl::Correspondences());
	float radii_search2fpfh = 2.0f;
	cloudfeature.setFpfh_search_radius(radii_search2fpfh);
	float radii_search2curva = 1.5f;
	cloudfeature.setCurvature_search_radius(radii_search2curva);
	int num_search = 2;
	cloudfeature.setFpfh_search_num(num_search);
	cloudfeature.setCurvature_search_num(num_search);

	bool if_distance_measure = true;
	cloudfeature.setIfdistance_measure(if_distance_measure);

	float correValueOrdistance = if_distance_measure ? 10.0f : 0.999f;
	cloudfeature.setDistance_tol(correValueOrdistance);
	cloudfeature.setCorrelation_tol(correValueOrdistance);

	float curvadiff_tol = 0.0015f;
	cloudfeature.setCurvature_tol(curvadiff_tol);

	float shapeindex_tol = 0.01f;
	cloudfeature.setShapeIndex_tol(shapeindex_tol);

	//cloudfeature.compareClouds_fpfhAndcurature(frame_normal_cloud1, frame_normal_cloud2, *corres_set);
	//cloudfeature.compareClouds_curaturecolor(frame_normal_cloud1, frame_normal_cloud2, *corres_set);
	//cloudfeature.compareClouds_curvadness(frame_normal_cloud1, frame_normal_cloud2, *corres_set);

	cloudfeature.compareClouds_fpfhAndcuratureAndshapeIndex(m_fobject1.m_cloud, m_fobject2.m_cloud, *corres_set);
}

void MappingScene::Mapping()
{
	std::vector<unsigned> indices1;
	std::vector<unsigned> indices2;
	Mapping::Do(m_fobject1, m_fobject2, indices1, indices2);
	GenerateMapping(indices1, indices2);
}

void MappingScene::GenerateMapping(std::vector<unsigned>& indices1, std::vector<unsigned>& indices2)
{
	m_mapping_geode->removeDrawables(0, m_mapping_geode->getNumDrawables());

	std::vector<osg::Vec3f> points1;
	std::vector<osg::Vec3f> points2;
	FeatureObjectTraits::GetPoints(m_fobject1, osg::Matrixf::identity(), indices1, points1);
	FeatureObjectTraits::GetPoints(m_fobject2, m_matrix, indices2, points2);

	osg::Geometry* geom = GeometryCreator::CreateLines(points1, points2);
	m_mapping_geode->addDrawable(geom);
}
