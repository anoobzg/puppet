#pragma once
#include <osgWrapper\RenderScene.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AttributeUtilNode.h>

using namespace OSGWrapper;
class FeatureObject;
class MappingScene : public RenderScene
{
public:
	MappingScene(FeatureObject& fobject1, FeatureObject& fobject2);
	~MappingScene();

protected:
	void ResetCamera();

	void Setup();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

private:
	void Test();
	void Mapping();

	void GenerateMapping(std::vector<unsigned>& indices1, std::vector<unsigned>& indices2);
private:
	FeatureObject& m_fobject1;
	FeatureObject& m_fobject2;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<ManipulableNode> m_trans_node;
	osg::ref_ptr<AttributeUtilNode> m_render_node_1;
	osg::ref_ptr<AttributeUtilNode> m_render_node_2;
	osg::ref_ptr<AttributeUtilNode> m_mapping_node;

	osg::Matrix m_matrix;
	osg::ref_ptr<osg::Geode> m_geode1;
	osg::ref_ptr<osg::Geode> m_geode2;
	osg::ref_ptr<osg::Geode> m_mapping_geode;
};