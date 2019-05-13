#ifndef ANIMATION_H
#define ANIMATION_H
#include <osg/Referenced>

namespace OSGWrapper
{

class OSG_EXPORT Animation : public osg::Referenced
{
public:
	Animation();
	~Animation();

	bool IsPlaying();
	void Start(double start_time);
	void SetAnimationLength(double length);

	void Elapse(double elapse);
	void At(double ref_time);

	virtual void OnPlay(float lambda);
	virtual bool ReleaseAfterStop();

private:
	void Perform();
protected:
	bool m_stopped;
	double m_animation_length;
	double m_start_time;
	double m_played_time;
};

}
#endif // ANIMATION_H