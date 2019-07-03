#include "keyframerender.h"
#include "trimeshgeometrycreator.h"
#include <base/bind.h>

KeyFrameRender::KeyFrameRender()
	:base::Thread("KeyFrameRender")
{
	m_scene = new KeyFrameTracerScene();
}

KeyFrameRender::~KeyFrameRender()
{

}

void KeyFrameRender::StartRender()
{
	Start();
}

void KeyFrameRender::StopRender()
{
	Stop();
}

KeyFrameTracerScene* KeyFrameRender::GetScene()
{
	return m_scene;
}

void KeyFrameRender::OnKeyFrame(KeyFrameData* data)
{
	task_runner()->PostTask(FROM_HERE, base::Bind(&KeyFrameRender::ProcessKeyFrame, base::Unretained(this), data));
}

void KeyFrameRender::ProcessKeyFrame(KeyFrameData* data)
{
	osg::Matrixf matrix = osg::Matrixf::identity();
	T2OConvert(matrix, data->mesh->global);
	osg::Geometry* geometry = Create(data->mesh.get());
	if (data->locate)
	{
		m_scene->Locate(geometry, matrix);
	}
	else
	{
		if (data->use_as_key_frame)
			m_scene->AddKeyFrame(geometry, matrix);
	}

	
	delete data;
}