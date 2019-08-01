#pragma once
#include <osg/Texture2D>
#include <osgWrapper/UnionTexture.h>

namespace OSGWrapper
{
	class OSG_EXPORT D2TextureManager
	{
		D2TextureManager();
		~D2TextureManager();

	public:
		static D2TextureManager& Instance();
		static void Setup();
		static void Release();

		UnionTexture* Get(const std::string& name, UnionCoord*& coord, bool only_one = false);
	private:
		static D2TextureManager* m_d2_texture_manager;

		std::vector<osg::ref_ptr<UnionTexture>> m_textures;
	};
}