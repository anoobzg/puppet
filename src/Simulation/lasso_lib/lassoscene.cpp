#include "lassoscene.h"
#include <osgWrapper/GeometryCreator.h>
#include <osgWrapper/ArrayCreator.h>

LassoScene::LassoScene()
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new OSGWrapper::AttributeUtilNode();
	m_render_node->SetRenderProgram("lasso");
	//m_render_node->SetMode(GL_BLEND, state_on);
	m_manipulable_node->AddChild(m_render_node);

	m_lasso_node = new LassoNode();
	m_manipulable_node->AddChild(m_lasso_node);
}

LassoScene::~LassoScene()
{

}

void LassoScene::Set(osg::Vec3Array* coord_array, osg::Vec3Array* normal_array)
{
	if (!coord_array || !normal_array || coord_array->size() == 0 || normal_array->size() == 0)
		return;

	size_t size = coord_array->size();
	m_flag_array = new osg::FloatArray();
	m_flag_array->resize(size, 0.0f);
	m_coord_array = coord_array;
	m_normal_array = normal_array;
	m_draw_array = new osg::DrawArrays(GL_POINTS, 0, size);
	m_geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(m_draw_array, m_coord_array, m_normal_array, m_flag_array);

	m_render_node->AddChild(m_geometry);
	m_lasso_node->Set(coord_array, m_flag_array);

	ResetCamera();
}

void LassoScene::ResetCamera()
{
	const osg::BoundingSphere& sphere = m_render_node->getBound();
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

bool LassoScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (m_lasso_node->OnMouse(ea, aa))
		return true;

	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool LassoScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_A))
		m_lasso_node->SelectAll();
	return false;
}

bool LassoScene::OnFrame(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_lasso_node->OnFrame();
	return true;
}