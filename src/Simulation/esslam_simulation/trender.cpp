#include "trender.h"
#include <base\bind.h>
#include <osgWrapper\ArrayCreator.h>
#include <osgWrapper\GeometryCreator.h>

TRender::TRender()
	:base::Thread("TRender")
{
	m_scene = new TScene();
}

TRender::~TRender()
{

}

void TRender::StartRender()
{
	Start();
}

void TRender::StopRender()
{
	Stop();
}

TScene* TRender::GetScene()
{
	return m_scene;
}
