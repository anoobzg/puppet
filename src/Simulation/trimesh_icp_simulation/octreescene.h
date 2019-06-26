#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\AttributeUtilNode.h>
#include <osgWrapper\Manipulator.h>
#include "octreenode.h"

class OctreeWork;
class OctreeScene : public OSGWrapper::RenderScene
{
public:
	OctreeScene(OctreeWork& work);
	~OctreeScene();

	void AddPoint(osg::Node* point);
	void RemovePoint(osg::Node* point);
private:
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void SetBounding();
	void UpdateCamera();
private:
	OctreeWork& m_work;

	osg::ref_ptr<OSGWrapper::ManipulableNode> m_manipulable_node;
	osg::ref_ptr<OSGWrapper::Manipulator> m_manipulator;
	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_render_node;

	osg::ref_ptr<OSGWrapper::AttributeUtilNode> m_point_node;
	osg::ref_ptr<OctreeNode> m_octree_node;
};
