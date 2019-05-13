#include <osgWrapper\Animation.h>

namespace OSGWrapper
{

Animation::Animation()
	:m_stopped(false), m_played_time(0.0), m_animation_length(1.0), m_start_time(0.0)
{
}

Animation::~Animation()
{
}

bool Animation::IsPlaying()
{
	return !m_stopped;
}

void Animation::Start(double start_time)
{
	m_start_time = start_time;
	m_stopped = false;
}

void Animation::SetAnimationLength(double length)
{
	m_animation_length = length;
	if(m_animation_length < 0.0)
		m_animation_length = 1.0;
}

void Animation::Elapse(double elapse)
{
	m_played_time += elapse;

	Perform();
}

void Animation::At(double ref_time)
{
	m_played_time = ref_time - m_start_time;

	Perform();
}

void Animation::Perform()
{
	float lambda = float(m_played_time / m_animation_length);


	if(lambda > 1.0)
	{
		m_stopped = true;
		lambda = 1.0;
	}

	if(lambda < 0.0)
	{
		m_stopped = true;
		lambda = 0.0;
	}

	OnPlay(lambda);
}

void Animation::OnPlay(float lambda)
{
}

bool Animation::ReleaseAfterStop()
{
	return true;
}

}