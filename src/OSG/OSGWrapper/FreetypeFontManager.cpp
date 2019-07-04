#include <osgWrapper/FreetypeFontManager.h>

namespace OSGWrapper
{
	FreetypeFontManager* FreetypeFontManager::m_instance = 0;
	FreetypeFontManager& FreetypeFontManager::Instance()
	{
		if (!m_instance) m_instance = new FreetypeFontManager();
		return *m_instance;
	}

	void FreetypeFontManager::Release()
	{
		if (m_instance) delete m_instance;
	}

	FreetypeFontManager::FreetypeFontManager()
	{
	}

	FreetypeFontManager::~FreetypeFontManager()
	{
		for (FreetypeFontIter it = m_fonts.begin(); it != m_fonts.end(); ++it)
		{
			FreetypeFontFamily& font_family = (*it).second;
			for (size_t i = 0; i < font_family.size(); ++i)
			{
				delete font_family.at(i);
			}
			font_family.clear();
		}
		m_fonts.clear();
	}

	FreetypeFont* FreetypeFontManager::Get(const std::string& family_name, int size)
	{
		if (size <= 0 || size >= 100)
			return NULL;

		FreetypeFontIter it = m_fonts.find(family_name);
		if (it != m_fonts.end())
		{
			FreetypeFontFamily& font_family = (*it).second;
			for (size_t i = 0; i < font_family.size(); ++i)
			{
				if (font_family.at(i)->GetSize() == size)
					return font_family.at(i);
			}
		}

		//load
		FreetypeFont* font = Load(family_name, size);
		if (font)
		{
			FreetypeFontFamily family;
			family.push_back(font);
			m_fonts.insert(FreetypeFontPair(family_name, family));
		}

		return font;
	}

	FreetypeFont* FreetypeFontManager::Load(const std::string& family_name, int size)
	{
		std::auto_ptr<FreetypeFont> font(new FreetypeFont(size));
		if (font->Init(family_name))
			return font.release();
		return NULL;
	}
}