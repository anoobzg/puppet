#pragma once
#include <osg/Referenced>
#include <osg/Texture2D>

namespace OSGWrapper
{
	struct FreetypeGlygh
	{
		unsigned code;
		float x;
		float y;
		float wx;
		float wy;
	};

	class OSG_EXPORT FreetypeFont
	{
	public:
		FreetypeFont(int size);
		virtual ~FreetypeFont();

		int GetSize() const;
		osg::Texture2D* GetTexture();

		bool Init(const std::string& file);
		void GenTextureCoord(const std::string& text, std::vector<osg::Vec2f>& texcoord);
		void GenTextureCoord(const std::wstring& text, std::vector<osg::Vec2f>& texcoord);
	private:
		FreetypeGlygh* IsGlyghLoaded(unsigned c);
		void GenTextureCoord(const FreetypeGlygh& glygh, std::vector<osg::Vec2f>& texcoord);
	protected:
		const int m_size;
		std::string m_file;

		osg::ref_ptr<osg::Texture2D> m_texture;
		osg::ref_ptr<osg::Image> m_image;

		int m_hinting;
		float m_outline_thickness;
		int m_filtering;
		float m_height;
		float m_linegap;
		float m_ascender;
		float m_descender;
		float m_underline_thickness;
		float m_underline_position;
		int m_padding;

		std::vector<FreetypeGlygh> m_glyghs;
		int m_line;
		int m_image_width;
		int m_image_height;
	};
}
