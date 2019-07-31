#include <osgWrapper/TextureManager.h>

namespace OSGWrapper
{
	D2TextureManager* D2TextureManager::m_d2_texture_manager = NULL;
	D2TextureManager::D2TextureManager()
	{
		m_textures.reserve(100);
	}

	D2TextureManager::~D2TextureManager()
	{

	}

	D2TextureManager& D2TextureManager::Instance()
	{
		if (!m_d2_texture_manager) m_d2_texture_manager = new D2TextureManager();
		return *m_d2_texture_manager;
	}

	void D2TextureManager::Setup()
	{

	}

	void D2TextureManager::Release()
	{
		if (m_d2_texture_manager)
		{
			delete m_d2_texture_manager;
			m_d2_texture_manager = NULL;
		}
	}

	UnionTexture* D2TextureManager::Get(const std::string& name, UnionCoord*& coord)
	{
		UnionTexture* texture = 0;
		size_t size = m_textures.size();
		for (size_t i = 0; i < size; ++i)
		{
			UnionCoord* rt_coord = m_textures.at(i)->Get(name);
			if (rt_coord)
			{
				texture = m_textures.at(i);
				coord = rt_coord;
				break;
			}
		}

		if (texture) return texture;

		UnionTexture* t = new UnionTexture();
		t->AddTexture(name, coord);
		m_textures.push_back(t);
		return t;
	}
}