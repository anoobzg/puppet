#pragma once
#include <osgWrapper/FreetypeFont.h>
#include <map>

namespace OSGWrapper
{

	class OSG_EXPORT FreetypeFontManager
	{
		typedef std::map<std::string, osg::ref_ptr<FreetypeFont> > FreetypeFonts;
		typedef FreetypeFonts::iterator FreetypeFontIter;
		typedef std::pair<std::string, osg::ref_ptr<FreetypeFonts> > FreetypeFontPair;

	protected:
		FreetypeFontManager();

	public:
		~FreetypeFontManager();

		static FreetypeFontManager& Instance();
		static void Release();

		FreetypeFont* Get(const char* name);
	private:
	private:
		FreetypeFonts m_fonts;

		static FreetypeFontManager* m_instance;
	};
}