#pragma once
#include "projectionicpmapping.h"
#include "projectionicpestimation.h"

namespace trimesh
{
	class ProjectionICP
	{
	public:
		ProjectionICP(float fx, float fy, float cx, float cy);
		~ProjectionICP();

		void SetSource(trimesh::TriMesh* source);
		void SetTarget(trimesh::TriMesh* target);

		float Do(trimesh::xform &source_form);
		float FastDo(trimesh::xform & source_form);
		float DefaultTimesDo(trimesh::xform & source_form, int times);
		float FMQuickDo(trimesh::xform & source_form, int max_times);
		void SetTracer(ProjectionICPTracer* tracer);

		//step functions
		bool Step();
		bool FastStep();
		void SetStepInitialMatrix(const trimesh::xform& xf);
		void ResetStep();
	private:
		bool StepFinalInteration(trimesh::xform &source_form);
		void StepMiddleInteration(trimesh::xform& source_form, float& error);
		bool StepInitialInteration(trimesh::xform &source_form);
		float FinalInteration(trimesh::xform &source_form);
		float MiddleInteration(trimesh::xform& source_form);
		float InitialInteration(trimesh::xform &source_form);
		float FastInteration(trimesh::xform &source_form);
		float InnerDo(trimesh::xform& source_form);
	protected:
		ProjectionICPMapping m_mapping;
		ProjectionICPEstimation m_estimation;

		int m_initial_iters;
		int m_max_iters;
		int m_termination_iter_thresh;
		int m_final_iters;

		float m_dist_thresh_mult_start;
		float m_normdot_thresh_start;
		float m_dist_thresh_mult_final;
		float m_normdot_thresh_final;

		float m_thresh_rate_of_change; 

		ProjectionICPTracer* m_tracer;
		trimesh::xform m_init_step_xf;
		int m_step; // 0 prepare, 1 init, 2 middle, 3 final
		int m_step_init_iter;
		int m_step_middle_iter;
		int m_step_final_iter;
		float m_step_dist_thresh_mult;
		float m_step_normdot_thresh;

		float m_step_min_err;
		int m_step_iter_of_min_err;

		float m_last_error;
		int m_cinteration;
	};
}