#include "mesh_segmentor.h"

#include <fstream>
MeshSegmentor::MeshSegmentor(Mesh& mesh)
	:m_mesh(mesh), m_callback(0), m_current_group(0), m_harmonic(0)
{
	m_harmonic = new float[m_mesh.vertex_number];

	m_caculator.reset(new HarmonicCaculator(m_mesh));
}

MeshSegmentor::~MeshSegmentor()
{
	m_caculator.reset();
}

void MeshSegmentor::SetCallback(MeshSegmentorCallback* callback)
{
	m_callback = callback;
	if (m_callback) m_callback->ShowMesh(m_mesh);
}

void MeshSegmentor::SetGroupNode(AttributeUtilNode* node)
{
	m_group_attribute_node = node;
}

bool MeshSegmentor::TryDeleteControlPoint(float* ray_position, float* ray_direction, unsigned vertex_handle)
{
	osg::ref_ptr<ControlBall> ball = Test(ray_position, ray_direction, vertex_handle);
	if (ball.valid())
	{
		osg::ref_ptr<ControlGroupGeode> group = ball->m_parent;
		if (group == m_current_group)
			m_current_group = 0;
		group->removeChild(ball);
		if (group->getNumChildren() == 0)
			m_group_attribute_node->RemoveChild(group);
	}

	return false;
}

bool MeshSegmentor::TryAddControlPoint(unsigned vertex_handle)
{
	if (!m_current_group.valid())
	{
		m_current_group = new ControlGroupGeode();
		m_group_attribute_node->AddChild(m_current_group);
		SelectControlGroup(m_current_group);
	}

	float* v = m_mesh.vertex_position + 3 * vertex_handle;
	ControlBall* ball = new ControlBall(vertex_handle, *v, *(v+1), *(v+2));
	m_current_group->addChild(ball);

	if (m_current_group == m_base_group)
		ball->SetColor(osg::Vec4f(0.0f, 0.4f, 0.0f, 1.0f));
	else
		ball->SetColor(Handle2Color(m_current_group->m_group_id));
	ball->m_parent = m_current_group;

	SelectControlPoint(ball);
	return false;
}

bool MeshSegmentor::TrySelectControlPoint(float* ray_position, float* ray_direction, unsigned vertex_handle)
{
	osg::ref_ptr<ControlBall> ball = Test(ray_position, ray_direction, vertex_handle);
	if (ball.valid())
	{
		SelectControlGroup(ball->m_parent);
	}

	return false;
}

void MeshSegmentor::SelectControlPoint(ControlBall* control_ball)
{
	if (m_current_ball.valid()) m_current_ball->Unselect();
	m_current_ball = control_ball;
	if (m_current_ball.valid()) m_current_ball->Select();
}

void MeshSegmentor::SelectControlGroup(ControlGroupGeode* group)
{
	if (m_current_group.valid()) m_current_group->Unselect();
	m_current_group = group;
	if (m_current_group.valid()) m_current_group->Select();
}

osg::Vec4 MeshSegmentor::Handle2Color(unsigned handle)
{
	//static osg::Vec4 color[8] = 
	//{
	//	osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f),
	//	osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f),
	//	osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f),
	//	osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f),
	//	osg::Vec4(1.0f, 0.5f, 0.5f, 1.0f),
	//	osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f),
	//	osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f),
	//	osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f),
	//};
	//return color[handle % 8];

	static osg::Vec4 color[2] = 
	{
		osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f),
		osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)
	};
	return color[handle % 2];
}

ControlBall* MeshSegmentor::Test(float* ray_position, float* ray_direction, unsigned vertex_handle)
{
	ControlBall* b = 0;
	float len = FLT_MAX;
	for (unsigned i = 0; i < m_group_attribute_node->getNumChildren(); ++i)
	{
		ControlGroupGeode* group = (ControlGroupGeode*)(m_group_attribute_node->getChild(i));
		if(group)
		{
			for (unsigned j = 0; j < group->getNumChildren(); ++j)
			{
				ControlBall* ball = (ControlBall*)group->getChild(j);
				if (ball)
				{
					osg::Vec3f point(ball->m_x, ball->m_y, ball->m_z);

					osg::Vec3f local_eye = osg::Vec3f(ray_position[0], ray_position[1], ray_position[2]);
					osg::Vec3f eye_to_point = point - local_eye;
					osg::Vec3f eye_to_center = osg::Vec3f(ray_direction[0], ray_direction[1], ray_direction[2]);

					float dot = eye_to_center * eye_to_point;
					if (dot <= 0.0f) continue;

					float slen = eye_to_point.length2();
					if (slen < dot * dot)
						continue;

					float d = sqrtf(slen - dot * dot);
					if (0.5f * d > ball->m_radius)
						continue;

					if (len > eye_to_point.length())
					{
						b = ball;
						len = eye_to_point.length();
					}
				}
			}
		}
	}

	if (b && vertex_handle != -1)
	{
		float* vp = m_mesh.vertex_position + 3 * vertex_handle;
		osg::Vec3f v = osg::Vec3f(*vp, *(vp+1), *(vp+2));
		osg::Vec3f local_eye = osg::Vec3f(ray_position[0], ray_position[1], ray_position[2]);
		float l = (v - local_eye).length();
		if (l < len - 0.1f)
			b = 0;
	}
	return b;
}

void MeshSegmentor::SelectNoneGroup()
{
	SelectControlGroup(0);
}

void MeshSegmentor::ImportPlane(const std::string& file)
{
	std::fstream in;
	in.open(file.c_str(), std::ios::in);
	if (in.is_open())
	{
		m_current_group = new ControlGroupGeode();
		m_group_attribute_node->AddChild(m_current_group);
		//SelectControlGroup(m_current_group);

		osg::Vec3f position;
		osg::Vec3f direction;

		in >> direction[0];
		in >> direction[1];
		in >> direction[2];
		in >> position[0];
		in >> position[1];
		in >> position[2];

		float l = 0.01f;
		for (unsigned i = 0; i < m_mesh.vertex_number; ++i)
		{
			float* v = m_mesh.vertex_position + 3 * i;
			osg::Vec3f p(*v, *(v + 1), *(v + 2));
			osg::Vec3f pp = p - position;
			if (std::abs(pp * direction) < l)
			{
				ControlBall* ball = new ControlBall(i, *v, *(v + 1), *(v + 2));
				m_current_group->addChild(ball);
				ball->SetColor(osg::Vec4f(0.0f, 0.4f, 0.0f, 1.0f));
				ball->m_parent = m_current_group;
			}
		}

		m_base_group = m_current_group;

		m_current_group = 0;
	}
}

void MeshSegmentor::CalculateHarmonic()
{
	std::vector<unsigned> base;
	std::vector<unsigned> pos;
	std::vector<unsigned> neg;

	if (m_base_group.valid())
	{
		for (unsigned j = 0; j < m_base_group->getNumChildren(); ++j)
		{
			ControlBall* ball = (ControlBall*)m_base_group->getChild(j);
			if (ball)
			{
				base.push_back(ball->m_vertex_handle);
			}
		}
	}
	for (unsigned i = 0; i < m_group_attribute_node->getNumChildren(); ++i)
	{
		ControlGroupGeode* group = (ControlGroupGeode*)(m_group_attribute_node->getChild(i));
		if (group && group != m_base_group)
		{
			for (unsigned j = 0; j < group->getNumChildren(); ++j)
			{
				ControlBall* ball = (ControlBall*)group->getChild(j);
				if (ball)
				{
					unsigned index = ball->m_vertex_handle;
					if (group->m_group_id % 2)
					{
						neg.push_back(index);
					}
					else
					{
						pos.push_back(index);
					}
				}
			}
		}
	}

	m_caculator->Do(base, neg, pos, m_harmonic);
	if (m_callback) m_callback->UpdateHarmonic(m_mesh.vertex_number, m_harmonic);
}

void MeshSegmentor::Write()
{
	std::vector<unsigned> base;
	std::vector<unsigned> pos;
	std::vector<unsigned> neg;

	if (m_base_group.valid())
	{
		for (unsigned j = 0; j < m_base_group->getNumChildren(); ++j)
		{
			ControlBall* ball = (ControlBall*)m_base_group->getChild(j);
			if (ball)
			{
				base.push_back(ball->m_vertex_handle);
			}
		}
	}
	for (unsigned i = 0; i < m_group_attribute_node->getNumChildren(); ++i)
	{
		ControlGroupGeode* group = (ControlGroupGeode*)(m_group_attribute_node->getChild(i));
		if (group && group != m_base_group)
		{
			for (unsigned j = 0; j < group->getNumChildren(); ++j)
			{
				ControlBall* ball = (ControlBall*)group->getChild(j);
				if (ball)
				{
					unsigned index = ball->m_vertex_handle;
					if (group->m_group_id % 2)
					{
						neg.push_back(index);
					}
					else
					{
						pos.push_back(index);
					}
				}
			}
		}
	}

	m_caculator->Write(base, neg, pos);
	if (m_callback) m_callback->UpdateHarmonic(m_mesh.vertex_number, m_harmonic);
}

void MeshSegmentor::RandomWalk()
{
	if (m_group_attribute_node->getNumChildren() == 0)
		return;

	std::vector<std::vector<unsigned>> seeds;
	seeds.resize(m_group_attribute_node->getNumChildren());
	for (unsigned i = 0; i < m_group_attribute_node->getNumChildren(); ++i)
	{
		ControlGroupGeode* group = (ControlGroupGeode*)(m_group_attribute_node->getChild(i));
		std::vector<unsigned>& group_seeds = seeds[i];
		if (group)
		{
			for (unsigned j = 0; j < group->getNumChildren(); ++j)
			{
				ControlBall* ball = (ControlBall*)group->getChild(j);
				if (ball)
				{
					unsigned index = ball->m_vertex_handle;
					group_seeds.push_back(index);
				}
			}
		}
	}

	m_caculator->DoRandomWalk(seeds, m_harmonic);
	if (m_callback) m_callback->UpdateHarmonic(m_mesh.vertex_number, m_harmonic);
}

void MeshSegmentor::AutoSeg(std::vector<unsigned> base, std::vector<unsigned> pos, std::vector<unsigned> neg)
{
	m_caculator->Do(base, neg, pos, m_harmonic);

	if (m_callback) m_callback->UpdateHarmonic(m_mesh.vertex_number, m_harmonic);
}

void MeshSegmentor::WriteControlPoint()
{
	std::vector<unsigned> base;
	std::vector<unsigned> pos;
	std::vector<unsigned> neg;

	if (m_base_group.valid())
	{
		for (unsigned j = 0; j < m_base_group->getNumChildren(); ++j)
		{
			ControlBall* ball = (ControlBall*)m_base_group->getChild(j);
			if (ball)
			{
				base.push_back(ball->m_vertex_handle);
			}
		}
	}
	for (unsigned i = 0; i < m_group_attribute_node->getNumChildren(); ++i)
	{
		ControlGroupGeode* group = (ControlGroupGeode*)(m_group_attribute_node->getChild(i));
		if (group && group != m_base_group)
		{
			for (unsigned j = 0; j < group->getNumChildren(); ++j)
			{
				ControlBall* ball = (ControlBall*)group->getChild(j);
				if (ball)
				{
					unsigned index = ball->m_vertex_handle;
					if (group->m_group_id % 2)
					{
						neg.push_back(index);
					}
					else
					{
						pos.push_back(index);
					}
				}
			}
		}
	}

	unsigned base_number = base.size();
	unsigned neg_number = neg.size();
	unsigned pos_number = pos.size();

	std::fstream out;
	out.open("E:\\Sim\\mesh_segment\\harmonic_control_point", std::ios::out | std::ios::binary);
	if (out.is_open())
	{
		auto f = [&out](const std::vector<unsigned>& points) {
			unsigned size = (unsigned)points.size();
			out.write((char*)&size, sizeof(unsigned));
			for(unsigned i = 0; i < size; ++i)
				out.write((char*)&points[i], sizeof(unsigned));
		};
		f(base); f(neg); f(pos);
	}
	out.close();
}

void MeshSegmentor::WritePatchControlPoint()
{
	std::vector<unsigned> gum;
	std::vector<std::vector<unsigned>> teeth;

	if (m_base_group.valid())
	{
		for (unsigned j = 0; j < m_base_group->getNumChildren(); ++j)
		{
			ControlBall* ball = (ControlBall*)m_base_group->getChild(j);
			if (ball)
			{
				gum.push_back(ball->m_vertex_handle);
			}
		}
	}
	for (unsigned i = 0; i < m_group_attribute_node->getNumChildren(); ++i)
	{
		ControlGroupGeode* group = (ControlGroupGeode*)(m_group_attribute_node->getChild(i));
		if (group && group != m_base_group)
		{
			std::vector<unsigned> tooth;
			for (unsigned j = 0; j < group->getNumChildren(); ++j)
			{
				ControlBall* ball = (ControlBall*)group->getChild(j);
				if (ball)
				{
					unsigned index = ball->m_vertex_handle;
					tooth.push_back(index);
				}
			}

			teeth.push_back(tooth);
		}
	}

	std::fstream out;
	out.open("E:\\Sim\\mesh_segment\\harmonic_patch_control_point", std::ios::out | std::ios::binary);
	if (out.is_open())
	{
		unsigned gum_seeds_num = (unsigned)gum.size();
		out.write((const char*)&gum_seeds_num, sizeof(unsigned));
		for (unsigned i = 0; i < gum_seeds_num; ++i)
			out.write((const char*)&gum[i], sizeof(unsigned));

		unsigned teeth_group_num = (unsigned)teeth.size();
		out.write((const char*)&teeth_group_num, sizeof(unsigned));

		for (unsigned i = 0; i < teeth_group_num; ++i)
		{
			std::vector<unsigned>& teeth_group = teeth[i];
			unsigned num = (unsigned)teeth_group.size();
			out.write((const char*)&num, sizeof(unsigned));
			for (unsigned j = 0; j < num; ++j)
			{
				out.write((const char*)&teeth_group[j], sizeof(unsigned));
			}
		}
	}
	out.close();
}

void MeshSegmentor::SegOnePatch()
{
	if (m_group_attribute_node->getNumChildren() == 0)
		return;

	std::vector<unsigned> pos;
	for (unsigned i = 0; i < m_group_attribute_node->getNumChildren(); ++i)
	{
		ControlGroupGeode* group = (ControlGroupGeode*)(m_group_attribute_node->getChild(i));
		if (group && group != m_base_group)
		{
			for (unsigned j = 0; j < group->getNumChildren(); ++j)
			{
				ControlBall* ball = (ControlBall*)group->getChild(j);
				if (ball)
				{
					unsigned index = ball->m_vertex_handle;
					pos.push_back(index);
				}
			}
			break;
		}
	}

	m_caculator->DoPatch(pos, m_harmonic);
	if (m_callback) m_callback->UpdateHarmonic(m_mesh.vertex_number, m_harmonic);
}

void MeshSegmentor::SegMeshByPatches(const std::vector<std::set<unsigned>>& groups, const std::set<unsigned>& boundarySet)
{
	int* bToothArea = new int[m_mesh.vertex_number];
	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		bToothArea[i] = -1;
	}

	unsigned* boundaryArray = new unsigned[boundarySet.size()];
	auto it = boundarySet.begin();
	for (unsigned i = 0; i < boundarySet.size(); i++)
	{
		boundaryArray[i] = *(it++);
	}

	int nLoopCount = 0;
	for (const auto& vertexSet : groups)
	{
		m_caculator->DoPatch(std::vector<unsigned>(vertexSet.begin(), vertexSet.end()), m_harmonic, boundaryArray, boundarySet.size());
		// m_caculator->DoPatch(std::vector<unsigned>(vertexSet.begin(), vertexSet.end()), m_harmonic);
		for (unsigned i = 0; i < m_mesh.vertex_number; i++)
		{
			if (m_harmonic[i] > 0.6f)
			{
				if (bToothArea[i] == -1)
				{
					bToothArea[i] = nLoopCount;
				}
				else
				{
					bToothArea[i] = INT_MAX;
				}
			}
		}

		nLoopCount++;
	}

	for (unsigned i = 0; i < m_mesh.triangle_number; i++)
	{
		unsigned* vertices = m_mesh.triangle_index + 3 * i;

		std::set<unsigned> resetVertexSet;

		if (-1 < bToothArea[vertices[0]] && bToothArea[vertices[0]] < INT_MAX)
		{
			if (-1 < bToothArea[vertices[1]] && bToothArea[vertices[1]] < INT_MAX)
			{
				if (bToothArea[vertices[0]] != bToothArea[vertices[1]])
				{
					resetVertexSet.insert(vertices[0]);
					resetVertexSet.insert(vertices[1]);
				}
			}

			if (-1 < bToothArea[vertices[2]] && bToothArea[vertices[2]] < INT_MAX)
			{
				if (bToothArea[vertices[0]] != bToothArea[vertices[2]])
				{
					resetVertexSet.insert(vertices[0]);
					resetVertexSet.insert(vertices[2]);
				}
			}
		}

		if (-1 < bToothArea[vertices[1]] && bToothArea[vertices[1]] < INT_MAX)
		{
			if (-1 < bToothArea[vertices[2]] && bToothArea[vertices[2]] < INT_MAX)
			{
				if (bToothArea[vertices[1]] != bToothArea[vertices[2]])
				{
					resetVertexSet.insert(vertices[1]);
					resetVertexSet.insert(vertices[2]);
				}
			}
		}

		for (auto vertexIndex : resetVertexSet)
		{
			bToothArea[vertexIndex] = -1;
		}
	}

	for (unsigned i = 0; i < m_mesh.vertex_number; i++)
	{
		m_harmonic[i] = (bToothArea[i] == -1) ? 0.5f : (bToothArea[i] == INT_MAX) ? 0.0f : 1.0f;
	}

	delete[] boundaryArray;
	delete[] bToothArea;

	if (m_callback) m_callback->UpdateHarmonic(m_mesh.vertex_number, m_harmonic);
}

void MeshSegmentor::SegMeshParallPatches(const std::vector<std::vector<unsigned>>& teeth_seeds, const std::vector<unsigned>& gum_seeds)
{
	m_caculator->DoParallPatch(teeth_seeds, gum_seeds, m_harmonic);
	if (m_callback) m_callback->UpdateHarmonic(m_mesh.vertex_number, m_harmonic);
}
