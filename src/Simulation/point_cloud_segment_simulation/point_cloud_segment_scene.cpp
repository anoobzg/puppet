#include "point_cloud_segment_scene.h"
#include "point_source.h"
#include "point_source_traits.h"

#include <osg\Geode>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>
RenderScene* CreateRenderScene(int argc, char* argv[])
{
	if (argc < 2) return 0;

	std::auto_ptr<PointSource> point_source(new PointSource());
	if (!point_source->Load(argv[1]))
		return 0;

	return new PointCloudSegmentScene(point_source.release());
}

PointCloudSegmentScene::PointCloudSegmentScene(PointSource* point_source)
	:m_point_source(point_source)
{
	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new AttributeUtilNode();
	m_render_node->SetRenderProgram("point_segment430");
	m_manipulable_node->AddChild(m_render_node);

	m_geode = new osg::Geode();
	m_geode->setCullingActive(false);
	m_render_node->AddChild(m_geode);

	ShowBase();
	ResetCamera();
}

PointCloudSegmentScene::~PointCloudSegmentScene()
{

}

void PointCloudSegmentScene::ShowBase()
{
	m_geode->removeDrawables(0, m_geode->getNumDrawables());

	if (!m_geometry.valid())
		m_geometry = CreateBase();

	m_geode->addDrawable(m_geometry);
}

void PointCloudSegmentScene::ResetCamera()
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
	SetNearFar(0.1f, len + 10.0f * radius);
}

bool PointCloudSegmentScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool PointCloudSegmentScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, GUIEventAdapter::KEY_R))
	{
		RegionGrowSegment();
	}
	return true;
}

#include "region_grow_segment.h"
void PointCloudSegmentScene::RegionGrowSegment()
{
	std::vector<float> flags;
	RegionGrowSegment::Do(*m_point_source, flags);

	Update(flags);
}

osg::Geometry* PointCloudSegmentScene::Create(std::vector<osg::Vec3f>& points, std::vector<osg::Vec3f>& normals, std::vector<float>& flags)
{
	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreatePointCloud(points, normals);
	if (geometry && flags.size() > 0)
	{
		geometry->setCullingActive(false);
		osg::Array* flag_array = ArrayCreator::CreateFloatArray(flags.size(), &flags[0]);
		geometry->setVertexAttribArray(2, flag_array, osg::Array::BIND_PER_VERTEX);
	}

	return geometry;
}

osg::Geometry* PointCloudSegmentScene::CreateBase()
{
	std::vector<osg::Vec3f> points;
	std::vector<osg::Vec3f> normals;
	std::vector<float> flags;
	PointSourceTraits::Trait(*m_point_source, points, normals, flags);

	return Create(points, normals, flags);
}

void PointCloudSegmentScene::Update(std::vector<float>& flags)
{
	if (flags.size() == 0) return;

	osg::Array* flag_array = ArrayCreator::CreateFloatArray(flags.size(), &flags[0]);
	m_geometry->setVertexAttribArray(2, flag_array, osg::Array::BIND_PER_VERTEX);
}