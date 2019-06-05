#include "scene.h"
#include <osgWrapper/GeometryCreator.h>

Scene::Scene(const LauncaGeometry::Mesh& mesh)
{
	SetupGeometry(mesh);

	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_render_node->SetRenderProgram("pointphong430");
	m_manipulable_node->AddChild(m_render_node);

	m_render_node->AddChild(m_geode);

	ResetCamera();
}

Scene::~Scene()
{

}

void Scene::SetupGeometry(const LauncaGeometry::Mesh& mesh)
{
	m_geode = new osg::Geode();
	m_geode->setCullingActive(false);
#if 0
	m_coord_array = (osg::Vec3Array*)OSGWrapper::ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_position);
	m_normal_array = (osg::Vec3Array*)OSGWrapper::ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_normal);
	m_draw_array = new osg::DrawArrays(GL_POINTS, 0, m_coord_array->size());
	m_geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(m_draw_array, m_coord_array, m_normal_array);
	m_geode->addDrawable(m_geometry);
#else
	unsigned N = 100;
	unsigned M = 1;
	int sN = sqrt(N);
	for (unsigned i = 0; i < N; i += M)
	{
		unsigned vertex_num = mesh.vertex_number * M;
		osg::Vec3Array* coord_array = new osg::Vec3Array(vertex_num);
		osg::Vec3Array* normal_array = new osg::Vec3Array(vertex_num);

		for (unsigned j = 0; j < M; ++j)
		{
			int len = i * M + j;
			float* p = mesh.vertex_position;
			float* n = mesh.vertex_normal;
			unsigned start_index = j * mesh.vertex_number;
			osg::Vec3f offset;
			int x = len / sN;
			int y = len % sN;
			offset = osg::Vec3f(x * 3.0f, y * 3.0f, 0.0f);
			for (unsigned k = 0; k < mesh.vertex_number; ++k)
			{
				osg::Vec3f& ov = coord_array->at(start_index + k);
				osg::Vec3f& on = normal_array->at(start_index + k);
				ov.x() = *p++; ov.y() = *p++; ov.z() = *p++;
				ov += offset;
				on.x() = *n++; on.y() = *n++; on.z() = *n++;
			}
		}
		osg::DrawArrays* draw_array = new osg::DrawArrays(GL_POINTS, 0, coord_array->size());
		osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array, normal_array);
		m_geode->addDrawable(geometry);
	}
#endif

}

void Scene::ResetCamera()
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

bool Scene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool Scene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	return false;
}