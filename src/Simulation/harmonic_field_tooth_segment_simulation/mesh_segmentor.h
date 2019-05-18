#pragma once
#include "scene_logic_interface.h"
#include <osgWrapper\AttributeUtilNode.h>
#include "control_group_geode.h"
#include "control_ball.h"
#include "harmonic_caculator.h"

using namespace OSGWrapper;
class MeshSegmentor
{
	friend class Collider;
public:
	MeshSegmentor(Mesh& mesh);
	~MeshSegmentor();

	void ImportPlane(const std::string& file);

	void SetCallback(MeshSegmentorCallback* callback);
	void SetGroupNode(AttributeUtilNode* node);

	bool TryDeleteControlPoint(float* ray_position, float* ray_direction, unsigned vertex_handle);
	bool TryAddControlPoint(unsigned vertex_handle);
	bool TrySelectControlPoint(float* ray_position, float* ray_direction, unsigned vertex_handle);

	void SelectNoneGroup();
	void CalculateHarmonic();
	void Write();
	void RandomWalk();
	void WriteControlPoint();
	void WritePatchControlPoint();

	void AutoSeg(std::vector<unsigned> base, std::vector<unsigned> pos, std::vector<unsigned> neg);

	void SegOnePatch();
	void SegMeshByPatches(const std::vector<std::set<unsigned>>& groups, const std::set<unsigned>& boundaryset);
	void SegMeshParallPatches(const std::vector<std::vector<unsigned>>& teeth_seeds, const std::vector<unsigned>& gum_seeds);

private:
	void SelectControlPoint(ControlBall* control_ball);
	void SelectControlGroup(ControlGroupGeode* group);
	osg::Vec4 Handle2Color(unsigned handle);

	ControlBall* Test(float* ray_position, float* ray_direction, unsigned vertex_handle);
private:
	Mesh& m_mesh;
	osg::ref_ptr<AttributeUtilNode> m_group_attribute_node;

	osg::ref_ptr<ControlGroupGeode> m_current_group;
	osg::ref_ptr<ControlBall> m_current_ball;
	osg::ref_ptr<ControlGroupGeode> m_base_group;

	MeshSegmentorCallback* m_callback;

	std::auto_ptr<HarmonicCaculator> m_caculator;
	float* m_harmonic;
};