#pragma once
#include <osgWrapper\RenderScene.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\AttributeUtilNode.h>

#include "Mesh.h"
#include "GemTransform.h"

#include "icp_service.h"
using namespace OSGWrapper;
using namespace LauncaGeometry;
class ICPScene : public RenderScene
{
public:
	ICPScene(Mesh& stable_mesh, Mesh& patch_mesh, GemTransform& tranform);
	~ICPScene();

protected:
	void ResetCamera();
	void Setup();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void Update(const osg::Matrixf& matrix);
private:
	void RunICP();
	void Reset();
private:
	Mesh& m_stable_mesh;
	Mesh& m_patch_mesh;

	osg::Matrixf m_patch_matrix;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<ManipulableNode> m_float_node;
	osg::ref_ptr<AttributeUtilNode> m_stable_render_node;
	osg::ref_ptr<AttributeUtilNode> m_patch_render_node;

	osg::ref_ptr<osg::Geode> m_stable_geode;
	osg::ref_ptr<osg::Geode> m_patch_geode;

	std::auto_ptr<ICPService> m_service;
};