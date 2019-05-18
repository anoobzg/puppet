#include "characteristics_scene.h"
#include <osg\Depth>
#include <osg\LineWidth>
#include <osg\PolygonOffset>
#include <osg\Point>
#include <osgWrapper\GeometryCreator.h>
#include <osgWrapper\RenderView.h>

#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeodeCreator.h>
#include <MeshVertexTraits.h>
#include <iostream>
#include <fstream>
#include <MeshSaver.h>

float _vecDot(float vec3A[3], float vec3B[3])
{
	return vec3A[0] * vec3B[0] + vec3A[1] * vec3B[1] + vec3A[2] * vec3B[2];
}

void _vecCross(float vec3A[3], float vec3B[3], float outVec3[3])
{
	outVec3[0] = vec3A[1] * vec3B[2] - vec3A[2] * vec3B[1];
	outVec3[1] = vec3A[2] * vec3B[0] - vec3A[0] * vec3B[2];
	outVec3[2] = vec3A[0] * vec3B[1] - vec3A[1] * vec3B[0];

	float length = sqrtf(outVec3[0] * outVec3[0] + outVec3[1] * outVec3[1] + outVec3[2] * outVec3[2]);
	float rhLength = 1.0f / length;

	outVec3[0] = outVec3[0] * rhLength;
	outVec3[1] = outVec3[1] * rhLength;
	outVec3[2] = outVec3[2] * rhLength;
}

bool _exportPathInfoSTL(Mesh& mesh, std::list<unsigned>& pathPrimitives, const char* szOutputFilePath)
{
	std::ofstream outFile(szOutputFilePath, std::ios::out | std::ios::binary);
	if (!outFile.is_open())
	{
		std::cout << "Can not open outputFile:" << szOutputFilePath << std::endl;
		return false;
	}

	std::set<unsigned> pathPrimitiveSet(pathPrimitives.begin(), pathPrimitives.end());

	char stlFileHeader[80];
	outFile.write(stlFileHeader, 80);

	int nTriangleSize = mesh.triangle_number;
	outFile.write(reinterpret_cast<const char*>(&nTriangleSize), 4);

	auto _writeFloatToFile = [&](float value)
	{
		outFile.write(reinterpret_cast<const char*>(&value), sizeof(float));
	};

	char reserveBit[2];
	char colorBit[2];

	memset(reserveBit, 0, 2);
	int16_t pathColor = 16 << 10;
	int16_t defaultColor = 31 << 10 | 31 << 5 | 31;

	for (int i = 0; i < nTriangleSize; i++)
	{
		// Write normal
		_writeFloatToFile(0.0f);
		_writeFloatToFile(0.0f);
		_writeFloatToFile(0.0f);

		// Write position
		unsigned vertexIndexs[3];
		vertexIndexs[0] = *(mesh.triangle_index + 3 * i);
		vertexIndexs[1] = *(mesh.triangle_index + 3 * i + 1);
		vertexIndexs[2] = *(mesh.triangle_index + 3 * i + 2);

		float vertexPosition[3][3];
		MeshVertexTraits::GetVertexCoord(mesh, vertexIndexs[0], vertexPosition[0][0], vertexPosition[0][1], vertexPosition[0][2]);
		MeshVertexTraits::GetVertexCoord(mesh, vertexIndexs[1], vertexPosition[1][0], vertexPosition[1][1], vertexPosition[1][2]);
		MeshVertexTraits::GetVertexCoord(mesh, vertexIndexs[2], vertexPosition[2][0], vertexPosition[2][1], vertexPosition[2][2]);

		_writeFloatToFile(vertexPosition[0][0]);
		_writeFloatToFile(vertexPosition[0][1]);
		_writeFloatToFile(vertexPosition[0][2]);

		_writeFloatToFile(vertexPosition[1][0]);
		_writeFloatToFile(vertexPosition[1][1]);
		_writeFloatToFile(vertexPosition[1][2]);

		_writeFloatToFile(vertexPosition[2][0]);
		_writeFloatToFile(vertexPosition[2][1]);
		_writeFloatToFile(vertexPosition[2][2]);

		if (pathPrimitiveSet.count(i + 1) > 0)
		{
			outFile.write(reinterpret_cast<const char*>(&pathColor), 2);
		}
		else
		{
			outFile.write(reinterpret_cast<const char*>(&defaultColor), 2);
		}
	}

	outFile.flush();
	outFile.close();

	return true;
}

bool _exportClippedSTLFile(Mesh& mesh, std::list<unsigned>& selectPrimitiveIDList, const char* szOutputFilePath)
{
	std::ofstream outFile(szOutputFilePath, std::ios::out | std::ios::binary);
	if (!outFile.is_open())
	{
		std::cout << "Can not open outputFile:" << szOutputFilePath << std::endl;
		return false;
	}

	char stlFileHeader[80];
	outFile.write(stlFileHeader, 80);

	int nTriangleSize = selectPrimitiveIDList.size();
	outFile.write(reinterpret_cast<const char*>(&nTriangleSize), 4);

	auto _writeFloatToFile = [&](float value)
	{
		outFile.write(reinterpret_cast<const char*>(&value), sizeof(float));
	};

	char reserveBit[2];
	memset(reserveBit, 0, 2);

	for (auto it = selectPrimitiveIDList.begin(); it != selectPrimitiveIDList.end(); it++)
	{
		// Write normal
		_writeFloatToFile(0.0f);
		_writeFloatToFile(0.0f);
		_writeFloatToFile(0.0f);

		// Write position
		unsigned vertexIndexs[3];
		if ((*it) == 0)
		{
			std::cout << "FATAL error." << std::endl;
			return false;
		}

		vertexIndexs[0] = *(mesh.triangle_index + 3 * ((*it) - 1));
		vertexIndexs[1] = *(mesh.triangle_index + 3 * ((*it) - 1) + 1);
		vertexIndexs[2] = *(mesh.triangle_index + 3 * ((*it) - 1) + 2);

		float vertexPosition[3][3];
		MeshVertexTraits::GetVertexCoord(mesh, vertexIndexs[0], vertexPosition[0][0], vertexPosition[0][1], vertexPosition[0][2]);
		MeshVertexTraits::GetVertexCoord(mesh, vertexIndexs[1], vertexPosition[1][0], vertexPosition[1][1], vertexPosition[1][2]);
		MeshVertexTraits::GetVertexCoord(mesh, vertexIndexs[2], vertexPosition[2][0], vertexPosition[2][1], vertexPosition[2][2]);

		_writeFloatToFile(vertexPosition[0][0]);
		_writeFloatToFile(vertexPosition[0][1]);
		_writeFloatToFile(vertexPosition[0][2]);

		_writeFloatToFile(vertexPosition[1][0]);
		_writeFloatToFile(vertexPosition[1][1]);
		_writeFloatToFile(vertexPosition[1][2]);

		_writeFloatToFile(vertexPosition[2][0]);
		_writeFloatToFile(vertexPosition[2][1]);
		_writeFloatToFile(vertexPosition[2][2]);

		outFile.write(reserveBit, 2);
	}

	outFile.flush();
	outFile.close();

	return true;
}

void Scene::_calRayIntersectTri(unsigned meshId, unsigned primitiveId, int nSX, int nSY, float outUVWCoord[3], float outClickCoord[3])
{
	const float FLOAT_DEVIATION = 1e-5f;

	outUVWCoord[0] = 0.0f;
	outUVWCoord[1] = 0.0f;
	outUVWCoord[2] = 0.0f;

	outClickCoord[0] = 0.0f;
	outClickCoord[1] = 0.0f;
	outClickCoord[2] = 0.0f;

	osg::Vec3f eye, center;
	GetRay(nSX, nSY, eye, center);

	osg::Matrixf model_matrix = m_manipulable_node->GetLocalMatrix();
	model_matrix = osg::Matrixf::inverse(model_matrix);

	eye = eye * model_matrix;
	center = center * model_matrix;

	float eyeVec[3];
	eyeVec[0] = eye.x();
	eyeVec[1] = eye.y();
	eyeVec[2] = eye.z();

	float LookVec[3];
	LookVec[0] = center.x() - eye.x();
	LookVec[1] = center.y() - eye.y();
	LookVec[2] = center.z() - eye.z();

	float fLength = sqrtf(LookVec[0] * LookVec[0] + LookVec[1] * LookVec[1] + LookVec[2] * LookVec[2]);
	LookVec[0] /= fLength;
	LookVec[1] /= fLength;
	LookVec[2] /= fLength;

	float fDistance = -1.0f;

	unsigned uVertexIndex[3];
	uVertexIndex[0] = *(m_mesh.triangle_index + 3 * (primitiveId - 1));
	uVertexIndex[1] = *(m_mesh.triangle_index + 3 * (primitiveId - 1) + 1);
	uVertexIndex[2] = *(m_mesh.triangle_index + 3 * (primitiveId - 1) + 2);

	float fVertexPosition[3][3];
	MeshVertexTraits::GetVertexCoord(m_mesh, uVertexIndex[0], fVertexPosition[0][0], fVertexPosition[0][1], fVertexPosition[0][2]);
	MeshVertexTraits::GetVertexCoord(m_mesh, uVertexIndex[1], fVertexPosition[1][0], fVertexPosition[1][1], fVertexPosition[1][2]);
	MeshVertexTraits::GetVertexCoord(m_mesh, uVertexIndex[2], fVertexPosition[2][0], fVertexPosition[2][1], fVertexPosition[2][2]);

	// Get triangle plane equation
	float edgeVec1[3] = { fVertexPosition[1][0] - fVertexPosition[0][0],  fVertexPosition[1][1] - fVertexPosition[0][1] , fVertexPosition[1][2] - fVertexPosition[0][2] };
	float edgeVec2[3] = { fVertexPosition[2][0] - fVertexPosition[1][0],  fVertexPosition[2][1] - fVertexPosition[1][1] , fVertexPosition[2][2] - fVertexPosition[1][2] };

	float normalVec[3];
	_vecCross(edgeVec1, edgeVec2, normalVec);
	float planeVertex[3] = { fVertexPosition[0][0], fVertexPosition[0][1], fVertexPosition[0][2] };

	// Check ray is parallel to triangle plane
	if (fabs(_vecDot(LookVec, normalVec)) < FLOAT_DEVIATION)
	{
		std::cout << "Ray is parallel to triangle plane." << std::endl;
		return;
	}

	/*
	Calculate intersection between ray and triangle plane
	plane equation: (P - planeVertex) * normalVec = 0.0f
	ray equation: P = ray.position + t * ray.direction

	===> t = normalVec * (planeVertex - ray.position) / normalVec * ray.Direction
	*/

	float tempVec[3];
	tempVec[0] = planeVertex[0] - eyeVec[0];
	tempVec[1] = planeVertex[1] - eyeVec[1];
	tempVec[2] = planeVertex[2] - eyeVec[2];

	fDistance = _vecDot(normalVec, tempVec) / _vecDot(normalVec, LookVec);
	if (fDistance < 0.0f)
	{
		std::cout << "Inverse direction." << std::endl;
		return;
	}

	float intersectPosition[3];
	intersectPosition[0] = eyeVec[0] + LookVec[0] * fDistance;
	intersectPosition[1] = eyeVec[1] + LookVec[1] * fDistance;
	intersectPosition[2] = eyeVec[2] + LookVec[2] * fDistance;

	// Calculate intersection's triangle Barycentric Coordination
	double D =
		static_cast<double>(fVertexPosition[0][0]) * static_cast<double>(fVertexPosition[1][1]) * static_cast<double>(fVertexPosition[2][2]) +
		static_cast<double>(fVertexPosition[1][0]) * static_cast<double>(fVertexPosition[2][1]) * static_cast<double>(fVertexPosition[0][2]) +
		static_cast<double>(fVertexPosition[2][0]) * static_cast<double>(fVertexPosition[0][1]) * static_cast<double>(fVertexPosition[1][2]) -
		static_cast<double>(fVertexPosition[2][0]) * static_cast<double>(fVertexPosition[1][1]) * static_cast<double>(fVertexPosition[0][2]) -
		static_cast<double>(fVertexPosition[1][0]) * static_cast<double>(fVertexPosition[0][1]) * static_cast<double>(fVertexPosition[2][2]) -
		static_cast<double>(fVertexPosition[0][0]) * static_cast<double>(fVertexPosition[2][1]) * static_cast<double>(fVertexPosition[1][2]);

	if (fabs(D) < 1e-5)
	{
		return;
	}

	double rhD = 1.0 / D;

	double Du =
		intersectPosition[0] * static_cast<double>(fVertexPosition[1][1]) * static_cast<double>(fVertexPosition[2][2]) +
		static_cast<double>(fVertexPosition[1][0]) * static_cast<double>(fVertexPosition[2][1]) * intersectPosition[2] +
		static_cast<double>(fVertexPosition[2][0]) * intersectPosition[1] * static_cast<double>(fVertexPosition[1][2]) -
		static_cast<double>(fVertexPosition[2][0]) * static_cast<double>(fVertexPosition[1][1]) * intersectPosition[2] -
		static_cast<double>(fVertexPosition[1][0]) * intersectPosition[1] * static_cast<double>(fVertexPosition[2][2]) -
		intersectPosition[0] * static_cast<double>(fVertexPosition[2][1]) * static_cast<double>(fVertexPosition[1][2]);

	double Dv =
		static_cast<double>(fVertexPosition[0][0]) * intersectPosition[1] * static_cast<double>(fVertexPosition[2][2]) +
		intersectPosition[0] * static_cast<double>(fVertexPosition[2][1]) * static_cast<double>(fVertexPosition[0][2]) +
		static_cast<double>(fVertexPosition[2][0]) * static_cast<double>(fVertexPosition[0][1]) * intersectPosition[2] -
		static_cast<double>(fVertexPosition[2][0]) * intersectPosition[1] * static_cast<double>(fVertexPosition[0][2]) -
		intersectPosition[0] * static_cast<double>(fVertexPosition[0][1]) * static_cast<double>(fVertexPosition[2][2]) -
		static_cast<double>(fVertexPosition[0][0]) * static_cast<double>(fVertexPosition[2][1]) * intersectPosition[2];

	double Dw =
		static_cast<double>(fVertexPosition[0][0]) * static_cast<double>(fVertexPosition[1][1]) * intersectPosition[2] +
		static_cast<double>(fVertexPosition[1][0]) * intersectPosition[1] * static_cast<double>(fVertexPosition[0][2]) +
		intersectPosition[0] * static_cast<double>(fVertexPosition[0][1]) * static_cast<double>(fVertexPosition[1][2]) -
		intersectPosition[0] * static_cast<double>(fVertexPosition[1][1]) * static_cast<double>(fVertexPosition[0][2]) -
		static_cast<double>(fVertexPosition[1][0]) * static_cast<double>(fVertexPosition[0][1]) * intersectPosition[2] -
		static_cast<double>(fVertexPosition[0][0]) * intersectPosition[1] * static_cast<double>(fVertexPosition[1][2]);

	double u = Du * rhD;
	double v = Dv * rhD;
	double w = Dw * rhD;

	// Outside
	if (u < 0.0 || v < 0.0 || w < 0.0)
	{
		std::cout << "Outside point, UVW = " << u << " " << v << " " << w << std::endl;
		// return;
	}

	outUVWCoord[0] = u;
	outUVWCoord[1] = v;
	outUVWCoord[2] = w;

	outClickCoord[0] = u * fVertexPosition[0][0] + v * fVertexPosition[1][0] + w * fVertexPosition[2][0];
	outClickCoord[1] = u * fVertexPosition[0][1] + v * fVertexPosition[1][1] + w * fVertexPosition[2][1];
	outClickCoord[2] = u * fVertexPosition[0][2] + v * fVertexPosition[1][2] + w * fVertexPosition[2][2];

	std::cout << "Intersection UVW = " << outUVWCoord[0] << " " << outUVWCoord[1] << " " << outUVWCoord[2] << std::endl;

	return;
}

Scene::Scene(Mesh& mesh, RenderView* view) :m_mesh(mesh)
{
	m_surface.reset(new Surface(mesh));

	m_picker = new ColorIndexPicker(view->getCamera(), GetWidth(), GetHeight());
	m_manipulable_node = new ManipulableNode();
	m_manipulable_node->UseModelUniform();
	m_manipulator = new Manipulator(*m_manipulable_node);
	addChild(m_manipulable_node);

	m_render_node = new AttributeUtilNode();
	m_render_node->SetRenderProgram("curvature430");
	m_manipulable_node->AddChild(m_render_node);
	m_manipulable_node->AddChild(m_picker);
	m_polygon_mode = new osg::PolygonMode();
	m_render_node->SetAttribute(m_polygon_mode);

	m_pathModule.reset(new PathModule(m_manipulable_node, mesh, m_surface));

	osg::ref_ptr<osg::Geode> geode = CreateMeshGeode(m_mesh);
	m_render_node->AddChild(geode);
	m_picker->SetNode(geode);

	ResetCamera();
}

Scene::~Scene()
{

}

void Scene::OnEnter()
{
}

void Scene::OnEnterOut()
{

}

bool Scene::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool handled = false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
		return Hover(ea, aa);
	if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
		handled = CheckCollide(ea, aa);
	else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG && ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
		handled = Drag(ea, aa);

	if(!handled) m_manipulator->OnMouse(ea, aa, *this);
	return true;
}

bool Scene::Hover(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	osg::Vec3f eye, center;
	GetRay(ea.getX(), ea.getY(), eye, center);

	unsigned control_point_handle = 0;

	osg::Matrixf model_matrix = m_manipulable_node->GetLocalMatrix();
	model_matrix = osg::Matrixf::inverse(model_matrix);

	if (m_pathModule->m_surface->CheckCollideControlPoint(model_matrix, eye, center, control_point_handle))
		m_pathModule->HoverControlPoint(control_point_handle);
	else
		m_pathModule->HoverControlPoint(-1);

	return true;
}

bool Scene::Drag(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	static int nDeltaX = 0;
	static int nDeltaY = 0;
	static bool bInitDelta = false;

	if (m_pathModule->m_selected_ball)
	{
		unsigned mesh_id = 0;
		unsigned primitive_id = 0;

		if (!bInitDelta)
		{
			auto projMatrix = getProjectionMatrix();
			auto viewMatrix = getViewMatrix();
			auto viewportMatrix = getViewport()->computeWindowMatrix();
			osg::Matrixf modelMatrix = m_manipulable_node->GetLocalMatrix();
			osg::Matrixf position_matrix;
			m_pathModule->m_selected_ball->GetModelMatrix(position_matrix);

			auto viewportCoord = osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f) * position_matrix * modelMatrix * viewMatrix * projMatrix * viewportMatrix;

			int sx = viewportCoord.x() / viewportCoord.w();
			int sy = viewportCoord.y() / viewportCoord.w();

			nDeltaX = sx - ea.getX();
			nDeltaY = sy - ea.getY();

			bInitDelta = true;
			std::cout << "Init delta value : " << nDeltaX << " " << nDeltaY << std::endl;
		}

		m_picker->Pick(ea.getX() + nDeltaX, ea.getY() + nDeltaY, mesh_id, primitive_id);
		if (primitive_id > 0 && primitive_id <= m_mesh.triangle_number)
		{
			float clickUVWCoord[3];
			float clickCoord[3];
			_calRayIntersectTri(mesh_id, primitive_id, ea.getX() + nDeltaX, ea.getY() + nDeltaY, clickUVWCoord, clickCoord);
			if (clickUVWCoord[0] == 0.0f &&
				clickUVWCoord[1] == 0.0f &&
				clickUVWCoord[2] == 0.0f)
			{
				std::cout << "Calculate UVWCoord error." << std::endl;
				return false;
			}

			unsigned offset = (clickUVWCoord[0] > clickUVWCoord[1]) ? ((clickUVWCoord[0] > clickUVWCoord[2]) ? 0 : 2) : ((clickUVWCoord[1] > clickUVWCoord[2]) ? 1 : 2);
			unsigned index = *(m_mesh.triangle_index + 3 * (primitive_id - 1) + offset);

			m_pathModule->m_surface->ModifyControlPoint(m_pathModule->m_selected_ball->Handle(), index, clickCoord);

			std::cout << "drag." << std::endl;
		}
		return true;
	}

	bInitDelta = false;
	return false;
}

bool Scene::CheckCollide(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_pathModule->HoverControlPoint(-1);
	if (m_pathModule->m_selected_ball) m_pathModule->m_selected_ball->Unselect();
	m_pathModule->m_selected_ball = 0;

	osg::Vec3f eye, center;
	GetRay(ea.getX(), ea.getY(), eye, center);

	osg::Matrixf model_matrix = m_manipulable_node->GetLocalMatrix();
	model_matrix = osg::Matrixf::inverse(model_matrix);

	unsigned control_point_handle = 0;
	if (m_pathModule->m_surface->CheckCollideControlPoint(model_matrix, eye, center, control_point_handle))
	{
		std::map<unsigned, osg::ref_ptr<ControlBall>>::iterator it = m_pathModule->m_control_balls.find(control_point_handle);
		if (it != m_pathModule->m_control_balls.end())
		{
			m_pathModule->m_selected_ball = (*it).second;
			if (m_pathModule->m_selected_ball) m_pathModule->m_selected_ball->Select();
			return true;
		}
	}
	return false;
}

void Scene::GetRay(float sx, float sy, osg::Vec3f& eye_out, osg::Vec3f& center_out)
{
	osg::Vec3f eye, center, up;

	osg::Vec4f screen_point(sx, sy, 0.0f, 1.0f);
	osg::Matrixf view_matrix = getViewMatrix();
	view_matrix.getLookAt(eye, center, up);
	osg::Matrixf projection_matrix = getProjectionMatrix();
	osg::Matrixf viewport_matrix = getViewport()->computeWindowMatrix();

	osg::Matrixf m = view_matrix * projection_matrix * viewport_matrix;
	osg::Vec4f world_point = screen_point * osg::Matrixf::inverse(m);

	if(world_point.w() != 0.0f)
	{
		world_point.x() = world_point.x() / world_point.w();
		world_point.y() = world_point.y() / world_point.w();
		world_point.z() = world_point.z() / world_point.w();
	}

	eye_out = eye;
	center_out = osg::Vec3f(world_point.x(), world_point.y(), world_point.z());
}

bool Scene::OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Delete)
	{
		if (m_pathModule->m_selected_ball) m_pathModule->m_surface->DeleteControlPoint(m_pathModule->m_selected_ball->Handle());
		if (m_pathModule->m_hove_ball == m_pathModule->m_selected_ball)
			m_pathModule->m_hove_ball = nullptr;

		m_pathModule->m_selected_ball = nullptr;
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Insert)
	{
		unsigned mesh_id = 0;
		unsigned primitive_id = 0;
		m_picker->Pick(ea.getX(), ea.getY(), mesh_id, primitive_id);

		if (primitive_id > 0 && primitive_id <= m_mesh.triangle_number)
		{
			std::cout << "insert." << std::endl;

			float clickUVWCoord[3];
			float clickCoord[3];
			_calRayIntersectTri(mesh_id, primitive_id, ea.getX(), ea.getY(), clickUVWCoord, clickCoord);
			if (clickUVWCoord[0] == 0.0f &&
				clickUVWCoord[1] == 0.0f &&
				clickUVWCoord[2] == 0.0f)
			{
				std::cout << "Calculate UVWCoord error." << std::endl;
				return false;
			}

			unsigned offset = (clickUVWCoord[0] > clickUVWCoord[1]) ? ((clickUVWCoord[0] > clickUVWCoord[2]) ? 0 : 2) : ((clickUVWCoord[1] > clickUVWCoord[2]) ? 1 : 2);
			unsigned index = *(m_mesh.triangle_index + 3 * (primitive_id - 1) + offset);

			m_pathModule->m_surface->InsertControlPoint(index, clickCoord);
		}
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_P)
	{
		static bool line_mode = false;
		line_mode = !line_mode;

		m_polygon_mode->setMode(osg::PolygonMode::FRONT_AND_BACK, line_mode ? osg::PolygonMode::LINE : osg::PolygonMode::FILL);
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Space)
	{
		if (!m_pathModule->ReLoadWeight())
		{
			return false;
		}

		if (!m_pathModule->ReCreatePath())
		{
			return false;
		}
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Z)
	{
		if (!m_pathModule->ReLoadSmoothFactor())
		{
			return false;
		}

		if (!m_pathModule->ReCreatePath())
		{
			return false;
		}
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_R)
	{
		// ReCalcualte curvature
		static int nCurvatureType = 1;

		nCurvatureType = (nCurvatureType == 1) ? 0 : 1;
		m_pathModule->m_surface->SetCurvatureType(nCurvatureType);

		if (!m_pathModule->ReLoadWeight())
			return false;

		if (!m_pathModule->ReCreatePath())
			return false;

		osg::Geode* geode = CreateMeshGeode(m_mesh);
		m_render_node->RemoveAll();
		m_render_node->AddChild(geode);
		m_picker->SetNode(geode);

		std::cout << "Change curvature type: " << nCurvatureType << std::endl;
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_X)
	{
		// Clip mesh
		unsigned mesh_id = 0;
		unsigned primitive_id = 0;
		m_picker->Pick(ea.getX(), ea.getY(), mesh_id, primitive_id);

		unsigned vertexIndex[3];
		vertexIndex[0] = *(m_mesh.triangle_index + 3 * (primitive_id - 1));
		vertexIndex[1] = *(m_mesh.triangle_index + 3 * (primitive_id - 1) + 1);
		vertexIndex[2] = *(m_mesh.triangle_index + 3 * (primitive_id - 1) + 2);

		auto circleVertexList = m_surface->GetCirclePath();
		if (circleVertexList.size() == 0)
		{
			std::cout << "Circle path size == 0" << std::endl;
			return false;
		}

		unsigned* circleData = (unsigned*)malloc(sizeof(unsigned) * circleVertexList.size());
		auto it = circleVertexList.begin();
		for (int i = 0; i < circleVertexList.size(); i++)
		{
			circleData[i] = (*it);
			it++;
		}

		m_surface->ClipMesh(circleVertexList.size(), circleData, vertexIndex);
		delete circleData;

		auto uClipVertexSize = m_surface->getClipMeshPrimitiveSize();
		unsigned* selectedVertexData = (unsigned*)malloc(sizeof(unsigned) * uClipVertexSize);
		m_surface->getClipMeshPrimitiveData(selectedVertexData);

		std::set<unsigned> selectedVertexSet;
		for (unsigned i = 0; i < uClipVertexSize; i++)
		{
			selectedVertexSet.insert(selectedVertexData[i]);
		}

		delete selectedVertexData;

		std::list<unsigned> selectedPrimitiveIDList;
		for (unsigned i = 0; i < m_mesh.triangle_number; i++)
		{
			unsigned vertexIndexs[3];
			vertexIndexs[0] = *(m_mesh.triangle_index + 3 * i);
			vertexIndexs[1] = *(m_mesh.triangle_index + 3 * i + 1);
			vertexIndexs[2] = *(m_mesh.triangle_index + 3 * i + 2);
			for (int j = 0; j < 3; j++)
			{
				if (selectedVertexSet.count(vertexIndexs[j]) > 0)
				{
					selectedPrimitiveIDList.push_back(i + 1);
					break;
				}
			}
		}

		if (!_exportClippedSTLFile(m_mesh, selectedPrimitiveIDList, "ClipMeshOutput.stl"))
		{
			std::cout << "Export STL file failed." << std::endl;
			return false;
		}

		std::cout << "Clip finish." << std::endl;
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_C)
	{
		// Export stlFile with path info.
		auto pathPrimitive = m_pathModule->GetPathPrimitives();
		auto pathVertexIndices = m_pathModule->GetPathVertexIndexs();

		if (pathPrimitive.size() == 0)
		{
			std::cout << "Empty path primitive." << std::endl;
			return false;
		}

		std::ofstream pathVerticesInfoFile("PathInfoText.txt", std::ios::out);
		for (auto vertexIndex : pathVertexIndices)
		{
			pathVerticesInfoFile << vertexIndex << "\t";
		}

		pathVertexIndices.sort();
		
		int nCount = 0;
		auto it = pathVertexIndices.begin();
		for (int i = 0; i < pathVertexIndices.size() - 1; i++)
		{
			auto firstValue = *(it);
			auto secondValue = *(++it);

			if (firstValue == secondValue)
			{
				nCount++;
			}
		}

		if (nCount > 1)
		{
			std::cout << "Invalid circle index" << std::endl;
		}

		pathVerticesInfoFile.flush();
		pathVerticesInfoFile.close();


		if (!_exportPathInfoSTL(m_mesh, pathPrimitive, "PathInfoSTL.stl"))
		{
			std::cout << "Export stl with path info failed." << std::endl;
			return false;
		}
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN && ea.getKey() == osgGA::GUIEventAdapter::KEY_Q)
	{
		return MeshSaver::Save("D:\\WangJie\\TJourney\\build\\Simulation\\ExportPLY.ply", m_mesh);
	}
	return true;
}

bool Scene::OnResize(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	int w = ea.getWindowWidth();
	int h = ea.getWindowHeight();

	return true;
}

bool Scene::OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
	{
		unsigned mesh_id = 0;
		unsigned primitive_id = 0;
		m_picker->Pick(ea.getX(), ea.getY(), mesh_id, primitive_id);

		if (primitive_id > 0 && primitive_id <= m_mesh.triangle_number)
		{
			std::cout << "pick." << std::endl;

			float clickUVWCoord[3];
			float clickCoord[3];
			_calRayIntersectTri(mesh_id, primitive_id, ea.getX(), ea.getY(), clickUVWCoord, clickCoord);

			if (clickUVWCoord[0] == 0.0f &&
				clickUVWCoord[1] == 0.0f &&
				clickUVWCoord[2] == 0.0f)
			{
				std::cout << "Calculate UVWCoord error." << std::endl;
				return false;
			}

			unsigned offset = (clickUVWCoord[0] > clickUVWCoord[1]) ? ((clickUVWCoord[0] > clickUVWCoord[2]) ? 0 : 2) : ((clickUVWCoord[1] > clickUVWCoord[2]) ? 1 : 2);
			unsigned index = *(m_mesh.triangle_index + 3 * (primitive_id - 1) + offset);

			if (m_pathModule->m_bUserClickCorrect)
			{
				index = m_surface->GetProperStartIndex(index);
			}
			MeshVertexTraits::GetVertexCoord(m_mesh, index, clickCoord[0], clickCoord[1], clickCoord[2]);

			if (!m_pathModule->m_surface->AddControlPoint(index, clickCoord))
			{
				return false;
			}

			if (!m_pathModule->m_surface->AddControlPointPath())
			{
				return false;
			}
		}
	}

	return true;
}

void Scene::ResetCamera()
{
	const osg::BoundingSphere& sphere = getBound();
	osg::Vec3f center = sphere.center();
	float radius = 0.5f * sphere.radius();

	float fovy = GetFovy();
	float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
	osg::Vec3f eye = center + osg::Vec3f(0.0f, 0.0f, -1.0f) * len;
	osg::Vec3f up = osg::Vec3f(0.0f, -1.0f, 0.0f);
	osg::Matrixf view_matrix = osg::Matrixf::lookAt(eye, center, up);

	SetViewMatrix(view_matrix);
	//SetNearFar(len - 3.1f * radius, len + 3.1f * radius);
	SetNearFar(0.01f, len + 3.1f * radius);
}

osg::Geode* Scene::CreateMeshGeode(Mesh& mesh)
{
	osg::Array* coord_array = OSGWrapper::ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_position);
	osg::Array* normal_array = OSGWrapper::ArrayCreator::CreateVec3Array(mesh.vertex_number, mesh.vertex_normal);

	float* color = new float[4 * mesh.vertex_number];
	for (unsigned i = 0; i < 4 * mesh.vertex_number; ++i)
		*(color + i) = 1.0f;

	float* curvature = new float[mesh.vertex_number];
	m_pathModule->m_surface->SetupCurvatureArray(curvature);

	osg::Array* color_array = OSGWrapper::ArrayCreator::CreateVec4Array(mesh.vertex_number, color);
	osg::Array* curvature_array = OSGWrapper::ArrayCreator::CreateFloatArray(mesh.vertex_number, curvature);
	delete[] color;
	delete[] curvature;
	osg::PrimitiveSet* primitive_set = OSGWrapper::ArrayCreator::CreatePrimitiveSet(osg::PrimitiveSet::TRIANGLES, 3 * mesh.triangle_number, mesh.triangle_index);
	return OSGWrapper::GeodeCreator::CreateIndexAttributeGeode(primitive_set, coord_array, normal_array, color_array, curvature_array);
}
