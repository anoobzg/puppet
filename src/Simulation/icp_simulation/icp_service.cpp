#include "icp_service.h"
#include <pcl\registration\icp.h>
ICPService::ICPService(Mesh& stable_mesh, Mesh& patch_mesh, GemTransform& init_transform)
{
	m_stable.reset(new Cloud());
	m_patch.reset(new Cloud());

	BuildPCLCloud(*m_stable, stable_mesh);
	BuildPCLCloud(*m_patch, patch_mesh);

	for (unsigned i = 0; i < 4; ++i)
		for (unsigned j = 0; j < 4; ++j)
			m_init_matrix(i, j) = init_transform.m_matrix[4 * i + j];

	std::cout << m_init_matrix << std::endl;
	pcl::transformPointCloud(*m_patch, *m_patch, m_init_matrix);
}

ICPService::~ICPService()
{

}

void ICPService::BuildPCLCloud(Cloud& cloud, Mesh& mesh)
{
	unsigned vertex_number = mesh.vertex_number;
	cloud.width = vertex_number;
	cloud.height = 1;

	float* p = mesh.vertex_position;
	float* n = mesh.vertex_normal;

	if (vertex_number > 0)
	{
		cloud.resize(vertex_number);

		for (unsigned i = 0; i < vertex_number; ++i)
		{
			Point& pp = cloud.at(i);

			pp.x = *p++; pp.y = *p++; pp.z = *p++;
			pp.normal_x = *n++; pp.normal_y = *n++; pp.normal_z = *n++;
		}
	}
}

void ICPService::Do(GemTransform& transform)
{
	pcl::IterativeClosestPoint<Point, Point> icp;
	icp.setInputTarget(m_stable);
	icp.setInputSource(m_patch);
	icp.setTransformationEpsilon(1e-6);
	icp.setMaxCorrespondenceDistance(400.0);

	Eigen::Matrix4f result = m_init_matrix;
	Cloud::Ptr result_cloud(new Cloud());
	icp.align(*result_cloud);
	if (icp.hasConverged())
	{
		std::cout << "ICP success." << std::endl;
		std::cout<<"Fitness score :"<<icp.getFitnessScore()<<std::endl;
		Eigen::Matrix4f m = icp.getFinalTransformation();
		result = m * result;
	}
	for (unsigned i = 0; i < 4; ++i)
		for (unsigned j = 0; j < 4; ++j)
			transform.m_matrix[4 * i + j] = result(i, j);
}
