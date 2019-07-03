#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include "operation.h"
#include <osg\Texture2D>

class LassoNode : public OSGWrapper::AttributeUtilNode
{
public:
	LassoNode();
	~LassoNode();

	void Set(osg::Vec3Array* coord_array, osg::FloatArray* flag_array);
	bool OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	void OnFrame();

	void SelectAll();
	void DeselectAll();
	void Reserve();
	void EnterEraseMode();
	void EnterOutEraseMode();
	void Delete();
	void EnterQuadMode();
	void EnterOutQuadMode();
	void Flip();
	void EnterMultiQuadMode();
	void EnterOutMultiQuadMode();
	void EnterTriangleMode();
	void EnterOutTriangleMode();

	void SetGraphcontext(osg::GraphicsContext* context);
	void Get(std::vector<bool>& flags);
protected:
	void ReleaseOperation();
	void AddOperation(Operation* operation);
	void CreateTexture();
	void ResetImage();
protected:
	int m_frame;
	osg::ref_ptr<Operation> m_operation;
	
	osg::ref_ptr<FeedGeometry> m_feed_geometry;
	osg::ref_ptr<FeedGeometry> m_copy_geometry;
	osg::ref_ptr<osg::FloatArray> m_flag_array;
	osg::ref_ptr<osg::Texture2D> m_texture;
	osg::ref_ptr<osg::Image> m_image;

	osg::ref_ptr<osg::GraphicsContext> m_context;
};