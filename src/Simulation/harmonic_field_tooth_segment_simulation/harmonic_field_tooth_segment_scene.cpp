#include "harmonic_field_tooth_segment_scene.h"
#include <osgWrapper\RenderView.h>

#include <osg\Geode>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

#include "mesh_segmentor.h"
#include <osgWrapper/MatrixAnimation.h>
#include "scaleanimation.h"
HarmonicFieldToothSegmentScene::HarmonicFieldToothSegmentScene(MeshSegmentor& segmentor, RenderView* view, const std::string& file_name)
	:m_mesh_segmentor(segmentor)
{
	m_picker = new ColorIndexPicker(view->getCamera(), GetWidth(), GetHeight());
	m_collider.reset(new Collider(m_mesh_segmentor, *m_picker));

	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new AttributeUtilNode();
	m_render_node->SetRenderProgram("harmonic_phong430");
	m_manipulable_node->AddChild(m_render_node);
	m_manipulable_node->AddChild(m_picker);

	m_control_node = new AttributeUtilNode();
	m_control_node->SetRenderProgram("harmonic_ball430");
	m_manipulable_node->AddChild(m_control_node);

	m_harmonic_geode = new HarmonicGeode();
	m_render_node->AddChild(m_harmonic_geode);
	m_picker->SetNode(m_harmonic_geode);

	m_mesh_segmentor.SetCallback(this);
	m_mesh_segmentor.SetGroupNode(m_control_node);

	ResetCamera();

	if (!file_name.empty()) SegFromFile(file_name.c_str());

	m_scheduler = new OSGWrapper::AnimationScheduler();
	m_manipulable_node->setUpdateCallback(m_scheduler);
}

HarmonicFieldToothSegmentScene::~HarmonicFieldToothSegmentScene()
{
	m_collider.reset();
}

void HarmonicFieldToothSegmentScene::ResetCamera()
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

bool HarmonicFieldToothSegmentScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool handled = false;
	if (LEFT_MOUSE_PUSH(ea))
		handled = CheckCollide(ea, aa);
	else if (RIGHT_MOUSE_PUSH(ea))
		handled = Delete(ea, aa);
	if (!handled) m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool HarmonicFieldToothSegmentScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Escape))
		m_mesh_segmentor.SelectNoneGroup();
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_P))
		m_mesh_segmentor.ImportPlane("E:\\Sim\\mesh_segment\\info.txt");
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_C))
		m_mesh_segmentor.CalculateHarmonic();
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_W))
		m_mesh_segmentor.Write();
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_R))
		m_mesh_segmentor.RandomWalk();
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Z))
		m_mesh_segmentor.WriteControlPoint();
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Q))
		m_mesh_segmentor.SegOnePatch();
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_A))
		SegFromPatchesFile("../ToothSegmentSimulation/groupsInfo.bin");
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_Y))
		SegFromFile_R("info2.bin");
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_T))
		m_mesh_segmentor.WritePatchControlPoint();
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_S))
		ScaleCenter(ea);
	return true;
}

void HarmonicFieldToothSegmentScene::ScaleCenter(const osgGA::GUIEventAdapter& ea)
{
	unsigned vertex_handle = -1;
	m_collider->QueryVertex(ea.getX(), ea.getY(), 0, vertex_handle);
	if (vertex_handle >= 0 && vertex_handle != -1)
	{
		osg::Vec3f p = m_collider->GetVertex(vertex_handle);
		osg::Vec3f center = m_manipulable_node->getBound().center();
		osg::Matrix m = m_manipulable_node->GetLocalMatrix();
		osg::Vec3f pp = p * m;
		osg::Vec3f cc = center * m;
		osg::Matrix d = osg::Matrix::translate(pp - cc);
		osg::Matrix rd = osg::Matrix::translate(cc - pp);


		float time = 0.0f;
		if (m_render_view) time = (float)m_render_view->getFrameStamp()->getSimulationTime();
		ScaleAnimation* ani = new ScaleAnimation(*m_manipulable_node);

		osg::BoundingSphere sphere = m_manipulable_node->getBound();

		osg::Vec3 shpere_center = sphere.center();


		osg::Matrix local_matrix = osg::Matrix::translate(-pp)/* * osg::Matrix::scale(1.4f, 1.4f, 1.4f) * osg::Matrix::translate(pp)*/;
		osg::Matrix final_matrix = local_matrix * m;

		osg::Vec3f ocenter = pp;
		final_matrix = m * osg::Matrixf::translate(-ocenter) *
			osg::Matrixf::scale(1.4f, 1.4f, 1.4f) * osg::Matrixf::translate(ocenter);

		//ani->SetMatrix(final_matrix);
		ani->SetCenter(pp);
		m_scheduler->Clear();
		m_scheduler->StartAnimation(ani, (double)time);
		//m_manipulable_node->SetMatrix(final_matrix);
		std::cout << "Scale" << std::endl;
	}
}

void HarmonicFieldToothSegmentScene::ShowMesh(Mesh& mesh)
{
	m_harmonic_geode->Reload(mesh);
}

void HarmonicFieldToothSegmentScene::ScreenRay(float x, float y, float* ray_position, float* ray_direction)
{
	osg::Vec3f eye, center;
	GetRay(x, y, eye, center);

	osg::Matrixf model_matrix = m_manipulable_node->GetLocalMatrix();
	model_matrix = osg::Matrixf::inverse(model_matrix);

	osg::Vec3f local_eye = eye * model_matrix;
	osg::Vec3f local_center = center * model_matrix;

	ray_position[0] = local_eye.x(); ray_position[1] = local_eye.y(); ray_position[2] = local_eye.z();
	osg::Vec3f eye_to_center = local_center - local_eye;
	eye_to_center.normalize();

	ray_direction[0] = eye_to_center.x(); ray_direction[1] = eye_to_center.y(); ray_direction[2] = eye_to_center.z();
}

bool HarmonicFieldToothSegmentScene::Delete(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	unsigned vertex_handle = -1;
	m_collider->QueryVertex(ea.getX(), ea.getY(), 0, vertex_handle);

	float ray_position[3] = { 0.0f };
	float ray_direction[3] = { 1.0f, 0.0f, 0.0f };
	ScreenRay(ea.getX(), ea.getY(), ray_position, ray_direction);

	unsigned control_point_handle = 0;
	bool result = m_mesh_segmentor.TryDeleteControlPoint(ray_position, ray_direction, vertex_handle);

	return result;
}

bool HarmonicFieldToothSegmentScene::OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (LEFT_MOUSE_DOUBLE_CLICK(ea))
	{
		unsigned vertex_handle = -1;
		bool result = m_collider->QueryVertex(ea.getX(), ea.getY(), 0, vertex_handle);
		if (result) m_mesh_segmentor.TryAddControlPoint(vertex_handle);

		return result;
	}
	return false;
}

bool HarmonicFieldToothSegmentScene::CheckCollide(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	unsigned vertex_handle = -1;
	m_collider->QueryVertex(ea.getX(), ea.getY(), 0, vertex_handle);

	float ray_position[3] = { 0.0f };
	float ray_direction[3] = { 1.0f, 0.0f, 0.0f };
	ScreenRay(ea.getX(), ea.getY(), ray_position, ray_direction);

	bool result = m_mesh_segmentor.TrySelectControlPoint(ray_position, ray_direction, vertex_handle);
	return result;
}

void HarmonicFieldToothSegmentScene::UpdateHarmonic(unsigned vertex_number, float* harmonic)
{
	m_harmonic_geode->UpdateHarmonic(vertex_number, harmonic);
}

#include <fstream>
void HarmonicFieldToothSegmentScene::SegFromFile(const char* file)
{
	std::fstream in;
	in.open(file, std::ios::in | std::ios::binary);
	if (in.is_open())
	{
		std::vector<unsigned> base;
		std::vector<unsigned> neg;
		std::vector<unsigned> pos;
		unsigned base_number = 0;
		in.read((char*)&base_number, sizeof(unsigned));
		if (base_number > 0)
		{
			base.resize(base_number);
			for (unsigned i = 0; i < base_number; ++i)
			{
				in.read((char*)&base[i], sizeof(unsigned));

				m_mesh_segmentor.TryAddControlPoint(base[i]);
			}
		}
		unsigned neg_number = 0;
		in.read((char*)&neg_number, sizeof(unsigned));
		if (neg_number > 0)
		{
			neg.resize(neg_number);
			for (unsigned i = 0; i < neg_number; ++i)
			{
				in.read((char*)&neg[i], sizeof(unsigned));

				m_mesh_segmentor.TryAddControlPoint(neg[i]);
			}
		}
		unsigned pos_number = 0;
		in.read((char*)&pos_number, sizeof(unsigned));
		if (pos_number > 0)
		{
			pos.resize(pos_number);
			for (unsigned i = 0; i < pos_number; ++i)
			{
				in.read((char*)&pos[i], sizeof(unsigned));

				m_mesh_segmentor.TryAddControlPoint(pos[i]);
			}
		}

		if (base_number > 0 && (pos_number > 0 || neg_number > 0))
		{
			m_mesh_segmentor.AutoSeg(base, pos, neg);
		}
	}
	in.close();
}

void HarmonicFieldToothSegmentScene::SegFromPatchesFile(const char* file)
{
	std::fstream in;
	in.open(file, std::ios::in | std::ios::binary);

	std::vector<std::set<unsigned>> groups;
	std::set<unsigned> tempSet;
	std::set<unsigned> boundarySet;

	while (in)
	{
		int tempInt;
		in.read(reinterpret_cast<char*>(&tempInt), sizeof(int));

		if (tempInt != -1)
		{
			tempSet.insert(tempInt);
		}
		else
		{
			if (boundarySet.size() == 0)
			{
				boundarySet = tempSet;
			}

			else
			{
				groups.push_back(tempSet);
			}

			tempSet.clear();
		}
	}

	m_mesh_segmentor.SegMeshByPatches(groups, boundarySet);
}

void HarmonicFieldToothSegmentScene::SegFromFile_R(const char* file)
{
	std::fstream in;
	in.open(file, std::ios::in | std::ios::binary);

	std::vector<std::vector<unsigned>> teeth_seeds;
	std::vector<unsigned> gum_seeds;

	if (in.is_open())
	{
		unsigned gum_seeds_num = 0;
		in.read((char*)&gum_seeds_num, sizeof(unsigned));
		if (gum_seeds_num > 0)
		{
			gum_seeds.resize(gum_seeds_num, 0);
			for(unsigned i = 0; i < gum_seeds_num; ++i)
				in.read((char*)&gum_seeds[i], sizeof(unsigned));
		}
		unsigned teeth_group_num = 0;
		in.read((char*)&teeth_group_num, sizeof(unsigned));
		if (teeth_group_num > 0)
		{
			teeth_seeds.resize(teeth_group_num);
			for (unsigned i = 0; i < teeth_group_num; ++i)
			{
				std::vector<unsigned>& teeth_group = teeth_seeds[i];
				unsigned num = 0;
				in.read((char*)&num, sizeof(unsigned));
				if (num > 0)
				{
					teeth_group.resize(num);
					for (unsigned j = 0; j < num; ++j)
					{
						in.read((char*)&teeth_group[j], sizeof(unsigned));
					}
				}
			}
		}
	}
	in.close();

	m_mesh_segmentor.SegMeshParallPatches(teeth_seeds, gum_seeds);
}
