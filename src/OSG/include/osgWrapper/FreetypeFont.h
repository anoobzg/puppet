#pragma once
#include <osg/Referenced>
#include <osg/Texture2D>

namespace OSGWrapper
{
	class OSG_EXPORT FreetypeFont : public osg::Referenced
	{
	public:
		FreetypeFont();
		virtual ~FreetypeFont();

		osg::Texture2D* GetTexture();
	};
}
