#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\AttributeUtilNode.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\ColorIndexPicker.h>

using namespace OSGWrapper;

class PointSource;
class PointCloudSegmentScene : public RenderScene
{
public:
	PointCloudSegmentScene(PointSource* point_source);
	~PointCloudSegmentScene();

protected:
	void ShowBase();
	void ResetCamera();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	osg::Geometry* Create(std::vector<osg::Vec3f>& points, std::vector<osg::Vec3f>& normals, std::vector<float>& flags);
	osg::Geometry* CreateBase();
	void Update(std::vector<float>& flags);

	void RegionGrowSegment();
private:
	std::auto_ptr<PointSource> m_point_source;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<AttributeUtilNode> m_render_node;

	osg::ref_ptr<osg::Geode> m_geode;
	osg::ref_ptr<osg::Geometry> m_geometry;
};
