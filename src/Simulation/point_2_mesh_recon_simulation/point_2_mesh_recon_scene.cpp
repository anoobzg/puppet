#include "point_2_mesh_recon_scene.h"

#include <osg\Geode>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

#include <osgWrapper\SimMain_Ex.h>
#include <osgWrapper\RenderService.h>

#include "MeshLoader.h"
#include "MeshScaler.h"
#include "PointCloudLoader.h"

#include "MeshGeodeBuilder.h"
#include "NormalCaculator.h"
#include "MeshVertexTraits.h"
#include "MathUtil.h"

#include <osg\Point>
int RunRenderScene(int argc, char* argv[])
{
	if (argc < 4) return 0;

	std::string mesh_file(argv[1]);
	std::string mesh_mc_file(argv[2]);
	std::string cloud_file(argv[3]);

	std::auto_ptr<Mesh> mesh(MeshLoader::LoadFromFileName(mesh_file.c_str()));
	std::auto_ptr<Mesh> mc_mesh(MeshLoader::LoadFromFileName(mesh_mc_file.c_str()));
	std::auto_ptr<PointCloud> cloud(PointCloudLoader::LoadFromFileName(cloud_file.c_str()));

	if (!mesh.get() || !cloud.get() || !mc_mesh.get()) return 0;
	MeshScaler::Scale(1000.0f, 1000.0f, 1000.0f, *mesh);
	MeshScaler::Scale(1000.0f, 1000.0f, 1000.0f, *mc_mesh);

	RenderView *view = (RenderView*)RenderService::Instance().getView(0);
	RenderService::Instance().setKeyEventSetsDone(0);
	osg::ref_ptr<Point2MeshReconSegmentScene> scene(new Point2MeshReconSegmentScene(view, *mesh, *mc_mesh, *cloud));
	return RunScene(scene);
}

Point2MeshReconSegmentScene::Point2MeshReconSegmentScene(RenderView* view, Mesh& mesh, Mesh& mc_mesh, PointCloud& cloud)
	:m_mesh(mesh), m_cloud(cloud), m_mc_mesh(mc_mesh), m_plane_start_index(-1)
	, m_mc_visual(true), m_poisson_visual(false)
{
	m_picker = new ColorIndexPicker(view->getCamera(), GetWidth(), GetHeight());

	removeChildren(0, getNumChildren());
	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);

	m_clip_mani_node = new ManipulableNode();
	m_clip_mani_node->UseModelUniform();
	addChild(m_clip_mani_node);

	//m_manipulator->EnableScale(false);
	addChild(m_manipulable_node);
	m_surface_node = new ManipulableNode();
	m_surface_node->UseModelUniform();
	addChild(m_surface_node);

	m_use_clip = new osg::Uniform("use_clip", 0.0f);
	m_center = new osg::Uniform("center", osg::Vec3f(0.0f, 0.0f, 0.0f));
	m_mesh_render_node = new AttributeUtilNode();
	m_mesh_render_node->SetRenderProgram("double_phong430");
	m_mesh_render_node->AddUniform(m_use_clip);
	m_mesh_render_node->AddUniform(m_center);
	m_mesh_render_node->SetAttribute(new osg::Point(3.0f));
	m_manipulable_node->AddChild(m_mesh_render_node);
	m_clip_plane_node = new AttributeUtilNode();
	m_clip_plane_node->SetRenderProgram("pointphong430");
	m_clip_plane_node->AddUniform(new osg::Uniform("color", osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));
	//m_clip_mani_node->AddChild(m_clip_plane_node);
	//m_manipulable_node->AddChild(m_clip_plane_node);
	m_manipulable_node->AddChild(m_picker);

	m_mesh_geometry = OSGBuilder::MeshGeodeBuilder::Build(m_mesh, OSGBuilder::MGT_TRIANGLE, OSGBuilder::MGT_POSITION | OSGBuilder::MGT_NORMAL | OSGBuilder::MGT_COLOR);
	m_mesh_geometry->setCullingActive(false);
	m_mesh_geometry->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));

	m_mc_geometry = OSGBuilder::MeshGeodeBuilder::Build(m_mc_mesh, OSGBuilder::MGT_TRIANGLE, OSGBuilder::MGT_POSITION | OSGBuilder::MGT_NORMAL | OSGBuilder::MGT_COLOR);
	m_mc_geometry->setCullingActive(false);
	m_mc_geometry->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));

	m_point_geometry = CreatePoint();
	m_point_geometry->setCullingActive(false);
	m_point_geometry->getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));

	m_mesh_render_node->AddChild(m_mesh_geometry);
	//m_mesh_render_node->AddChild(m_mc_geometry);
	m_picker->SetNode(m_mesh_geometry);
	ResetCamera();
}

Point2MeshReconSegmentScene::~Point2MeshReconSegmentScene()
{

}

void Point2MeshReconSegmentScene::ResetCamera()
{
	const osg::BoundingSphere& sphere = m_manipulable_node->getBound();
	osg::Vec3f center = sphere.center();
	float radius = 0.8f * sphere.radius();

	m_center->set(center);
	//AddClipPlane(center);

	float fovy = GetFovy();
	float len = 0.8f * radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(1.0f, 0.0f, 0.0f) * len ;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	SetNearFar(1000.0f, len + 10.0f * radius);
}

bool Point2MeshReconSegmentScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool handled = false;

	//if (LEFT_MOUSE_PUSH(ea))
	//	handled = CheckCollide(ea, aa);
	//else if (LEFT_MOUSE_DRAG(ea))
	//	handled = Drag(ea, aa);
	//else if (LEFT_MOUSE_RELEASE(ea))
	//	handled = Release(ea, aa);
	if (!handled) m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool Point2MeshReconSegmentScene::OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (!LEFT_MOUSE_DOUBLE_CLICK(ea))
		return false;

	unsigned mesh_id = 0;
	unsigned primitive_id = 0;
	m_picker->Pick(ea.getX(), ea.getY(), mesh_id, primitive_id);

	if (primitive_id > 0 && primitive_id <= m_mesh.triangle_number)
	{
		unsigned index = primitive_id - 1;
		unsigned* tri = m_mesh.triangle_index + 3 * index;
		
		unsigned vertex_index = *tri;
		float* vertex = m_mesh.vertex_position + 3 * vertex_index;

		osg::Vec3f center = osg::Vec3f(*vertex, *(vertex+1), *(vertex+2));
		const osg::Matrixf& m = m_manipulable_node->GetGlobalMatrix();
		center = center * m;

		m_manipulator->UseCenter(true, center);
		m_mesh_render_node->AddChild(m_mc_geometry);
		m_mesh_render_node->AddChild(m_point_geometry);

		float fovy = GetFovy();
		float len = 1000.0f;
		osg::Vec3f eye = center + osg::Vec3f(1.0f, 0.0f, 0.0f) * len;
		osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
		osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);
		
		SetViewMatrix(view_matrix);
		SetNearFar(900.0f, 2200.0f);

		m_poisson_visual = true;
		m_mc_visual = true;
		return true;
	}

	return true;
}

bool Point2MeshReconSegmentScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_C))
		ToggleClip();
	else if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Escape))
		ExitMode();
	else if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_M))
		ToggleMC();
	else if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_P))
		TogglePoisson();

	return true;
}

void Point2MeshReconSegmentScene::ToggleClip()
{
	float use_clip = 1.0f;
	m_use_clip->get(use_clip);

	use_clip = 1.0f - use_clip;
	m_use_clip->set(use_clip);
}

void Point2MeshReconSegmentScene::UpdateClipPlane(unsigned start_index, unsigned end_index)
{
	if (start_index == end_index)
		return;

	float* start_vertex = m_mesh.vertex_position + 3 * start_index;
	float* end_vertex = m_mesh.vertex_position + 3 * end_index;
	float* start_normal = m_mesh.vertex_normal + 3 * start_index;
	float* end_normal = m_mesh.vertex_normal + 3 * end_index;

	m_plane_position[0] = start_vertex[0]; m_plane_position[1] = start_vertex[1]; m_plane_position[2] = start_vertex[2];

	osg::Vec3 v1 = osg::Vec3(*start_vertex, *(start_vertex + 1), *(start_vertex + 2));
	osg::Vec3 v2 = osg::Vec3(*end_vertex, *(end_vertex + 1), *(end_vertex + 2));
	osg::Vec3 nv1 = osg::Vec3(*start_normal, *(start_normal + 1), *(start_normal + 2));
	osg::Vec3 nv2 = osg::Vec3(*end_normal, *(end_normal + 1), *(end_normal + 2));
	osg::Vec3 nv3 = v2 - v1; nv3.normalize();

	osg::Vec3 n = nv1 ^ nv3;
	n.normalize();
	m_plane_normal[0] = n.x(); m_plane_normal[1] = n.y(); m_plane_normal[2] = n.z();

	osg::Vec3 n1 = v2 - v1; n1.normalize();
	osg::Vec3 n2 = n1 ^ n; n2.normalize();
	osg::Vec3 n3 = -n1; 
	osg::Vec3 n4 = -n2;

	float len = 1000.0f;
	osg::Vec3Array* coord_array = new osg::Vec3Array();
	coord_array->push_back(v1 + n1 * len);
	coord_array->push_back(v1 + n2 * len);
	coord_array->push_back(v1 + n3 * len);
	coord_array->push_back(v1 + n4 * len);
	osg::Vec3Array* normal_array = new osg::Vec3Array();
	normal_array->push_back(n);
	normal_array->push_back(n);
	normal_array->push_back(n);
	normal_array->push_back(n);

	osg::DrawElementsUInt* primitive_set = new osg::DrawElementsUInt(GL_QUADS);
	primitive_set->push_back(0); primitive_set->push_back(1); primitive_set->push_back(2); primitive_set->push_back(3);

	osg::Geometry* plane = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array, normal_array);
	plane->setCullingActive(false);

	m_clip_plane_node->RemoveAll();
	m_clip_plane_node->AddChild(plane);
}

void Point2MeshReconSegmentScene::AddClipPlane(const osg::Vec3f& center)
{
	m_clip_plane_node->RemoveAll();

	osg::Vec3 n = osg::Vec3f(0.0f, 1.0f, 0.0f);
	n.normalize();
	m_plane_normal[0] = n.x(); m_plane_normal[1] = n.y(); m_plane_normal[2] = n.z();
	m_plane_position[0] = center.x(); m_plane_position[1] = center.y(); m_plane_position[2] = center.z();

	osg::Vec3 n1 = osg::Vec3(1.0f, 0.0f, 0.0f);
	osg::Vec3 n2 = osg::Vec3(0.0f, 0.0f, 1.0f);
	osg::Vec3 n3 = -n1;
	osg::Vec3 n4 = -n2;

	float len = 10000.0f;
	osg::Vec3Array* coord_array = new osg::Vec3Array();
	coord_array->push_back(center + n1 * len);
	coord_array->push_back(center + n2 * len);
	coord_array->push_back(center + n3 * len);
	coord_array->push_back(center + n4 * len);
	osg::Vec3Array* normal_array = new osg::Vec3Array();
	normal_array->push_back(n);
	normal_array->push_back(n);
	normal_array->push_back(n);
	normal_array->push_back(n);

	osg::DrawElementsUInt* primitive_set = new osg::DrawElementsUInt(GL_QUADS);
	primitive_set->push_back(0); primitive_set->push_back(1); primitive_set->push_back(2); primitive_set->push_back(3);

	osg::Geometry* plane = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array, normal_array);
	plane->setCullingActive(false);


	m_clip_plane_node->AddChild(plane);
}

void Point2MeshReconSegmentScene::RemoveClipPlane()
{
	m_clip_plane_node->RemoveAll();
}

bool Point2MeshReconSegmentScene::CheckCollide(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (m_plane_start_index != -1)
		return false;

	unsigned mesh_id = 0;
	unsigned primitive_id = 0;
	m_picker->Pick(ea.getX(), ea.getY(), mesh_id, primitive_id);

	if (primitive_id > 0 && primitive_id <= m_mesh.triangle_number)
	{
		unsigned index = primitive_id - 1;
		unsigned* tri = m_mesh.triangle_index + 3 * index;
		m_plane_start_index = *tri;
		return true;
	}
	return false;
}

bool Point2MeshReconSegmentScene::Drag(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (m_plane_start_index == -1)
		return false;

	unsigned mesh_id = 0;
	unsigned primitive_id = 0;
	m_picker->Pick(ea.getX(), ea.getY(), mesh_id, primitive_id);

	if (primitive_id > 0 && primitive_id <= m_mesh.triangle_number)
	{
		unsigned index = primitive_id - 1;
		unsigned* tri = m_mesh.triangle_index + 3 * index;
		unsigned end_index = *tri;

		UpdateClipPlane(m_plane_start_index, end_index);
		return true;
	}

	return true;
}

bool Point2MeshReconSegmentScene::Release(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (m_plane_start_index == -1)
		return false;

	m_plane_start_index = -1;
	RemoveClipPlane();
	return true;
}

void Point2MeshReconSegmentScene::ExitMode()
{
	ResetCamera();

	m_manipulator->UseCenter(false);
	m_mesh_render_node->RemoveChild(m_mc_geometry);
	m_mesh_render_node->RemoveChild(m_point_geometry);
	m_mesh_render_node->AddChild(m_mesh_geometry);

	m_mc_visual = true;
	m_poisson_visual = false;
}

osg::Geometry* Point2MeshReconSegmentScene::CreatePoint()
{
	osg::Array* coord_array = ArrayCreator::CreateVec3Array(m_cloud.vertex_number, m_cloud.vertex_position);
	osg::Array* normal_array = ArrayCreator::CreateVec3Array(m_cloud.vertex_number, m_cloud.vertex_normal);

	float* color = new float[4 * m_cloud.vertex_number];
	for (unsigned i = 0; i < m_cloud.vertex_number; ++i)
	{
		float* c = color + 4 * i;
		unsigned char* cc = m_cloud.vertex_color + 3 * i;

		*c++ = (float)(*cc++) / 255.0f;
		*c++ = (float)(*cc++) / 255.0f;
		*c++ = (float)(*cc++) / 255.0f;
		*c++ = 1.0f;
	}
	osg::Array* color_array = ArrayCreator::CreateVec4Array(m_cloud.vertex_number, color);
	delete[]color;
	osg::DrawArrays* primitive_set = new osg::DrawArrays(GL_POINTS, 0, m_cloud.vertex_number);

	osg::Geometry* point = GeometryCreator::CreateIndexAttributeGeometry(primitive_set, coord_array, normal_array, color_array);
	return point;
}

void Point2MeshReconSegmentScene::ToggleMC()
{
	m_mc_visual = !m_mc_visual;

	m_mesh_render_node->RemoveChild(m_mesh_geometry);
	if(m_mc_visual)  m_mesh_render_node->AddChild(m_mesh_geometry);
}

void Point2MeshReconSegmentScene::TogglePoisson()
{
	m_poisson_visual = !m_poisson_visual;

	m_mesh_render_node->RemoveChild(m_mc_geometry);
	if (m_poisson_visual)  m_mesh_render_node->AddChild(m_mc_geometry);
}