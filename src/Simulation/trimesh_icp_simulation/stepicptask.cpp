#include "stepicptask.h"
#include <iostream>
#include <osgWrapper/GeometryCreator.h>

StepICPTask::StepICPTask(trimesh::CameraData& data, ICPNode* target, ICPNode* source, ScreenGraph* screen_graph)
	:m_target(target), m_source(source), m_data(data)
	,m_icp(m_data.m_fx, m_data.m_fy, m_data.m_cx, m_data.m_cy)
	,m_screen_graph(screen_graph)
{
	m_icp.SetSource(&source->GetMesh());
	m_icp.SetTarget(&target->GetMesh());
	m_icp.SetTracer(this);

	m_last_matrix = osg::Matrixf::identity();
}

StepICPTask::~StepICPTask()
{

}

bool StepICPTask::Execute()
{
	static int tick = 0;

	++tick;
	if(tick % 100 == 0)
		return m_icp.Step();
	return true;
}

void StepICPTask::OnPreStepCorrespondences(const std::vector<trimesh::PtPair>& correspondences)
{
	m_lines->RemoveAll();

	osg::Vec3Array* coord_array = new osg::Vec3Array();
	for (size_t i = 0; i < correspondences.size(); ++i)
	{
		const trimesh::PtPair& pair = correspondences.at(i);
		const trimesh::point& p1 = pair.p1;
		const trimesh::point& p2 = pair.p2;
		coord_array->push_back(osg::Vec3f(p1.x, p1.y, p1.z));
		coord_array->push_back(osg::Vec3f(p2.x, p2.y, p2.z));
	}
	osg::DrawArrays* draw_array = new osg::DrawArrays(GL_LINES, 0, coord_array->size());

	osg::Geometry* geometry = OSGWrapper::GeometryCreator::CreateIndexAttributeGeometry(draw_array, coord_array);
	m_lines->AddChild(geometry);
}

void StepICPTask::OnMatrix(const trimesh::xform& xf)
{
	osg::Matrixf m = osg::Matrixf::identity();
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			m(i, j) = xf(j, i);

	//m_source->UpdateMatrix(m);
	m_source->UpdateMatrix(m_last_matrix);
	m_last_matrix = m;
}

void StepICPTask::OnError(float error)
{
	//std::cout << "error " << error << std::endl;
	m_screen_graph->AddError(error);
}

void StepICPTask::SetAttributeNode(OSGWrapper::AttributeUtilNode* node)
{
	m_lines = node;
}