#include "mapping_scene.h"
#include <osg\Geode>
#include <osg\LineWidth>
#include <osg\Point>
#include <osg\PolygonMode>

#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeodeCreator.h>

#include "MeshEdgeStripper.h"
#include <fstream>
#include <MeshSaver.h>
#include <iostream>

MappingScene::MappingScene(Mesh& mesh)
	:m_mesh(mesh)
{
	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new AttributeUtilNode();
	m_render_node->SetRenderProgram("pointphong430");
	m_render_node->AddUniform(new osg::Uniform("color", osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f)));
	m_render_node->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
	m_manipulable_node->AddChild(m_render_node);

	m_edge_node = new AttributeUtilNode();
	m_edge_node->SetRenderProgram("purecolor430");
	m_edge_node->AddUniform(new osg::Uniform("color", osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f)));
	m_edge_node->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL));
	m_edge_node->SetMode(GL_DEPTH_TEST, state_off);
	m_manipulable_node->AddChild(m_edge_node);

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
	float radius = 0.6f * sphere.radius();

	float fovy = GetFovy();
	float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(0.0f, 0.0f, -1.0f) * len;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	//SetNearFar(len - 3.1f * radius, len + 3.1f * radius);
	SetNearFar(0.01f, len + 3.1f * radius);
}

void MappingScene::Setup()
{
	if (!m_mesh_geode.valid())
	{
		osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(m_mesh.vertex_number, m_mesh.vertex_position);
		osg::Array* normal_array = OSGWrapper::ArrayCreator::CreateVec3Array(m_mesh.vertex_number, m_mesh.vertex_normal);

		osg::PrimitiveSet* primitive_set = OSGWrapper::ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::TRIANGLES, 3 * m_mesh.triangle_number, m_mesh.triangle_index);
		m_mesh_geode = OSGWrapper::GeodeCreator::CreateIndexAttributeGeode(primitive_set, coord_array, normal_array);
	}

	ShowPolygonMesh(m_mesh_geode);
}

bool MappingScene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_S))
		Strip();
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_C))
	{
		m_edge_node->RemoveAll();
	}
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_P))
	{
		static bool bLineMode = false;
		m_render_node->SetAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, (bLineMode) ? osg::PolygonMode::LINE : osg::PolygonMode::FILL));
		bLineMode = !bLineMode;

		return true;
	}
	if (KEY_DOWN(ea, osgGA::GUIEventAdapter::KEY_E))
	{
		MeshEdgeStripper stripper(m_mesh);
		std::vector<unsigned> edges;
		std::ifstream pathVertexInfoFile("PathInfoText.txt", std::ios::in);

		if (pathVertexInfoFile.is_open())
		{
			unsigned vertexIndex;
			while (pathVertexInfoFile >> vertexIndex)
			{
				edges.push_back(vertexIndex);
			}

			pathVertexInfoFile.close();
		}

		std::vector<unsigned> triangles;
		auto mesh = stripper.Do(edges, triangles);

		//Add color information
		/*if (mesh->vertex_color)
		{
			delete[] mesh->vertex_color;
		}

		mesh->vertex_color = new unsigned char[3 * mesh->vertex_number];
		if (!mesh->vertex_color)
		{
			std::cout << "Alloc memory failed." << std::endl;
			return false;
		}*/

		//memset(mesh->vertex_color, 0, 3 * mesh->vertex_number * sizeof(unsigned char));

		/*for (int i = 0; i < triangles.size(); i++)
		{
			unsigned vertexIndices[3] =
			{
				*(mesh->triangle_index + 3 * triangles[i]),
				*(mesh->triangle_index + 3 * triangles[i] + 1),
				*(mesh->triangle_index + 3 * triangles[i] + 2)
			};

			for (unsigned vertexIndex : vertexIndices)
			{
				*(mesh->vertex_color + 3 * vertexIndex) = 255;
				*(mesh->vertex_color + 3 * vertexIndex + 1) = 0;
				*(mesh->vertex_color + 3 * vertexIndex + 2) = 0;
			}
		}*/

		/*for (int i = 0; i < edges.size(); i++)
		{
			*(mesh->vertex_color + 3 * edges[i]) = 255;
			*(mesh->vertex_color + 3 * edges[i] + 1) = 0;
			*(mesh->vertex_color + 3 * edges[i] + 2) = 0;
		}*/

		return MeshSaver::Save("Export.ply", *mesh);
	}
	return true;
}

bool MappingScene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

void MappingScene::ShowPolygonMesh(osg::Geode* geode)
{
	m_render_node->RemoveAll();
	m_render_node->AddChild(geode);
}

void MappingScene::ShowEdgeStrip(osg::Geode* geode)
{
	m_edge_node->RemoveAll();
	m_edge_node->AddChild(geode);
}

void MappingScene::Strip()
{
	MeshEdgeStripper stripper(m_mesh);

	std::vector<unsigned> edges;
	std::ifstream pathVertexInfoFile("PathInfoText.txt", std::ios::in);

	if (pathVertexInfoFile.is_open())
	{
		unsigned vertexIndex;
		while (pathVertexInfoFile >> vertexIndex)
		{
			edges.push_back(vertexIndex);
		}

		pathVertexInfoFile.close();
	}

	std::vector<unsigned> triangles;

	std::auto_ptr<Mesh> result(stripper.Do(edges, triangles));
	if (result.get())
	{
		osg::Geode* geode = BuildStrip(*result, triangles);
		ShowEdgeStrip(geode);
	}
}

osg::Geode* MappingScene::BuildStrip(Mesh& mesh, std::vector<unsigned>& triangles)
{
	if (triangles.size() == 0)
		return 0;

	unsigned vertex_number = 3 * triangles.size();
	float* coord = new float[3 * vertex_number];
	float* tc = coord;
	for (size_t i = 0; i < triangles.size(); ++i)
	{
		unsigned* index = mesh.triangle_index + 3 * triangles[i];
		for (unsigned j = 0; j < 3; ++j)
		{
			float* vertex = mesh.vertex_position + 3 * *(index + j);
			*tc++ = *vertex++; *tc++ = *vertex++; *tc++ = *vertex++;
		}
	}
	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(vertex_number, coord);

	osg::PrimitiveSet* primitive_set = new osg::DrawArrays(GL_TRIANGLES, 0, vertex_number);
	osg::Geode* geode = OSGWrapper::GeodeCreator::CreateIndexAttributeGeode(primitive_set, coord_array);
	return geode;
}
