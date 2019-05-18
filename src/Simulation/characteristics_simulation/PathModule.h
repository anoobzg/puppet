#pragma once

#include <osgWrapper/AttributeUtilNode.h>
#include <osgWrapper/ArrayCreator.h>
#include <osgWrapper/ManipulableNode.h>
#include <memory>

#include "control_ball.h"
#include "control_path.h"
#include "surface.h"

using namespace OSGWrapper;

class PathModule : public SurfaceTopoCallback
{
public:
	PathModule(osg::ref_ptr<ManipulableNode> m_manipulable_node, LauncaGeometry::Mesh& mesh, std::shared_ptr<Surface> surface);
	~PathModule();

	// Surface class callback
	void ControlPointModified(ControlPoint& control_point);
	void ControlPointAdded(ControlPoint& control_point);
	void ControlPointDeleted(ControlPoint& control_point);
	void PathAdded(Path& path);
	void PathRemoved(Path& path);

	void HoverControlPoint(unsigned handle);
	bool ReLoadWeight();
	bool ReLoadSmoothFactor();
	bool ReCreatePath();
	bool IsCirclePath();
	void SetNodeData();

	std::list<unsigned> GetPathPrimitives();
	std::list<unsigned> GetPathVertexIndexs();

	std::shared_ptr<Surface> m_surface;
	std::map<unsigned, osg::ref_ptr<ControlBall>> m_control_balls;
	std::map<unsigned, osg::ref_ptr<ControlPath>> m_control_paths;
	ControlBall* m_hove_ball;
	ControlBall* m_selected_ball;

	bool m_bUserClickCorrect;
	bool m_bSmoothControlPath;

	std::pair<int, int> m_nSmoothFactorPair;

private:
	void _generateBezierCurve(std::vector<Path::Point3D>& inputPath, std::list<Path::Point3D>& outputPath, Path* pPrevPath, Path* pNextPath);
	
	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<AttributeUtilNode> m_control_node;
	osg::ref_ptr<AttributeUtilNode> m_control_path_node;
	osg::ref_ptr<osg::Geode> m_control_points_geode;
	osg::ref_ptr<osg::Geode> m_control_points_path_geode;
};