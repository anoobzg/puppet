#pragma once
#include <osgWrapper/FreetypeFont.h>
#include <map>

namespace OSGWrapper
{

	class OSG_EXPORT FreetypeFontManager
	{
		typedef std::vector<FreetypeFont*> FreetypeFontFamily;
		typedef std::map<std::string, FreetypeFontFamily> FreetypeFonts;
		typedef FreetypeFonts::iterator FreetypeFontIter;
		typedef std::pair<std::string, FreetypeFontFamily> FreetypeFontPair;

	protected:
		FreetypeFontManager();

	public:
		~FreetypeFontManager();

		static FreetypeFontManager& Instance();
		static void Release();

		FreetypeFont* Get(const std::string& family_name, int size);
	private:
		FreetypeFont* Load(const std::string& family_name, int size);
	private:
		FreetypeFonts m_fonts;

		static FreetypeFontManager* m_instance;
	};
}