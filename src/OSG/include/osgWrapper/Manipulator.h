#ifndef MANIPULATOR_H
#define MANIPULATOR_H
#include <osg\Referenced>
#include <osgGA\GUIActionAdapter>
#include <osgGA\GUIEventAdapter>

namespace OSGWrapper
{
class ManipulableNode;
class OSG_EXPORT Manipulator : public osg::Referenced
{
public:
	Manipulator(ManipulableNode& node);
	~Manipulator();

	void OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Camera& camera);

	void EnableRotate(bool enable);
	void EnableScale(bool enable);
	void EnableTranslate(bool enable);

	void Reset(bool reset_node = true);

	void UpdateCenter();

	void UseCenter(bool use, const osg::Vec3f& center = osg::Vec3());
private:
	void Rotate(float angle, const osg::Vec3f& axis, osg::Camera& camera);
	void Translate(float dx, float dy, osg::Camera& camera);
	void Scale(float scale, osg::Camera& camera);

	void UpdateMatrix();
private:
	ManipulableNode& m_manipulable_node;

	bool m_rotate_enable;
	bool m_scale_enable;
	bool m_translate_enable;

	osg::ref_ptr<osgGA::GUIEventAdapter> m_saved_event;
	int m_operation_mode;

	float m_scale;
	osg::Vec3f m_translate_vector;
	osg::Vec3f m_operation_center;
	osg::Quat m_rotate_quat;

	bool m_use_center;
};

}

#endif // MANIPULATOR_H