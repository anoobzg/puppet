#include <osgWrapper/AnimationScheduler.h>
#include <osgWrapper/Animation.h>
#include <osgWrapper/RenderService.h>

#include <osgViewer\View>
namespace OSGWrapper
{

AnimationScheduler::AnimationScheduler()
{
}

AnimationScheduler::~AnimationScheduler()
{
}

void AnimationScheduler::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	double time = nv->getFrameStamp()->getReferenceTime();

	Animations delete_later;

	Animations::iterator it = m_animations.begin();
	for(; it != m_animations.end(); ++it)
	{
		if(!(*it)->IsPlaying())
		{
			if((*it)->ReleaseAfterStop())
				delete_later.push_back(*it);
			continue;
		}

		(*it)->At(time);
	}

	for(Animations::iterator itr = delete_later.begin(); itr != delete_later.end(); ++itr)
		m_animations.remove(*itr);
	
	NodeCallback::traverse(node,nv);
}

void AnimationScheduler::StopAnimation(Animation* animation)
{
	Animations::iterator it = std::find(m_animations.begin(), m_animations.end(), animation);
	if(it != m_animations.end())
		m_animations.erase(it);
}

void AnimationScheduler::StartAnimation(Animation* animation)
{
	if(animation)
	{
		animation->Start(RenderService::Instance().getFrameStamp()->getReferenceTime());
		m_animations.push_back(animation);
	}
}

void AnimationScheduler::Clear()
{
	m_animations.clear();
}

}