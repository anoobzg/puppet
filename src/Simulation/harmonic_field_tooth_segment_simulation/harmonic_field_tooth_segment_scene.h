#pragma once
#include <osgWrapper\RenderScene.h>
#include <osgWrapper\AttributeUtilNode.h>

#include <osgWrapper\ManipulableNode.h>
#include <osgWrapper\Manipulator.h>
#include <osgWrapper\ColorIndexPicker.h>

#include "control_ball.h"
#include "harmonic_geode.h"
#include "scene_logic_interface.h"
#include "collider.h"

using namespace OSGWrapper;
class MeshSegmentor;
class HarmonicFieldToothSegmentScene : public RenderScene, public MeshSegmentorCallback
{
public:
	HarmonicFieldToothSegmentScene(MeshSegmentor& segmentor, RenderView* view, const std::string& file_name);
	~HarmonicFieldToothSegmentScene();

protected:
	void ResetCamera();

	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnKey(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void ScreenRay(float x, float y, float* ray_position, float* ray_direction);
	bool Delete(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool OnDoubleClick(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	bool CheckCollide(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	void ShowMesh(Mesh& mesh);
	void UpdateHarmonic(unsigned vertex_number, float* harmonic);
	void SegFromFile(const char* file);
	void SegFromPatchesFile(const char* file);
	void SegFromFile_R(const char* file);
private:
	MeshSegmentor& m_mesh_segmentor;

	osg::ref_ptr<ManipulableNode> m_manipulable_node;
	osg::ref_ptr<Manipulator> m_manipulator;
	osg::ref_ptr<AttributeUtilNode> m_render_node;
	osg::ref_ptr<AttributeUtilNode> m_control_node;

	osg::ref_ptr<ColorIndexPicker> m_picker;
	osg::ref_ptr<HarmonicGeode> m_harmonic_geode;

	std::auto_ptr<Collider> m_collider;
};
