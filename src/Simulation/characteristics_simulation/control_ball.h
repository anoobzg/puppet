#pragma once
#include <osgWrapper\GeometryCreator.h>

class ControlBall : public osg::Geometry
{
public:
	ControlBall(unsigned handle);
	~ControlBall();

	void SetPosition(float x, float y, float z);
	void GetModelMatrix(osg::Matrixf& out);

	void Hover();
	void Unhover();
	void Select();
	void Unselect();

	unsigned Handle();
private:
	osg::ref_ptr<osg::Uniform> m_matrix;
	osg::ref_ptr<osg::Uniform> m_color;

	bool m_select;

	unsigned m_handle;

	bool m_release;
};