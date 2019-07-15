#include "tscene.h"

TScene::TScene()
{
	m_manipulable_node = new OSGWrapper::ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new OSGWrapper::Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_frame = new FrameNode();
	m_manipulable_node->AddChild(m_frame);
}

TScene::~TScene()
{

}

void TScene::OnFrameLocated(int effect_num, const std::vector<trimesh::point3>& vertexes,
	const std::vector<trimesh::point3>& normals, const std::vector<trimesh::point3>& colors,
	const trimesh::xform& xf, bool lost)
{
	bool first = m_frame->First();
	osg::Matrixf matrix = osg::Matrixf::identity();
	Convert(matrix, xf);

	if (!lost)
	{
		m_frame->UpdateGlobalMatrix(matrix);
		FrameGeometry* geometry = m_frame->GetFreeGeometry();
		geometry->Update(effect_num, vertexes, normals);
	}

	render_lock.Acquire();
	m_frame->Exchange(!lost);
	if (first) UpdateCamera();
	render_lock.Release();
}

void TScene::Convert(osg::Matrixf& matrix, const trimesh::xform& xf)
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			matrix(i, j) = xf(j, i);
}

void TScene::UpdateCamera()
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

bool TScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}