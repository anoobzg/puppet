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
		m_fonts.clear();
	}

	FreetypeFont* FreetypeFontManager::Get(const char* name)
	{
		return 0;
	}
}