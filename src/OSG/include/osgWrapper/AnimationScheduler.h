#ifndef ANIMATION_SCHEDULER
#define ANIMATION_SCHEDULER
#include <osgGA\GUIEventAdapter>
#include <osgGA\GUIActionAdapter>
#include <list>


namespace osgViewer {
	class View;
}

namespace OSGWrapper
{

class Animation;
class OSG_EXPORT AnimationScheduler : public osg::NodeCallback
{
public:
	AnimationScheduler();
	~AnimationScheduler();

protected:
	void operator()(osg::Node* node, osg::NodeVisitor* nv);
public:
	void StartAnimation(Animation* animation);//no used
	void StartAnimation(Animation* animation, double start_time);
	void StopAnimation(Animation* animation);
	void Clear();
protected:
	typedef std::list<osg::ref_ptr<Animation> > Animations;
	Animations m_animations;
};

}
#endif // ANIMATION_SCHEDULER