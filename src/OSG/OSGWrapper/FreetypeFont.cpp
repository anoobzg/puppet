#include <ft2build.h>
#include FT_FREETYPE_H
#include <freetype\ftglyph.h>
#include <osgWrapper/FreetypeFont.h>
#include <assert.h>

#define HRES  64
#define HRESf 64.f
#define DPI   72

namespace OSGWrapper
{
	bool LoadFreetype(const std::string& file, int size, FT_Library& library, FT_Face& face)
	{
		FT_Error error;

		library = 0;
		face = 0;
		do {
			/* Initialize library */
			error = FT_Init_FreeType(&library);
			if (error) break;
			error = FT_New_Face(library, file.c_str(), 0, &face);
			if (error) break;

			/* Set char size */
			error = FT_Set_Char_Size(face, size << 6, size << 6, 300, 300);
			if (error) break;
		} while (false);

		if (error)
		{
			if(face) FT_Done_Face(face);
			if(library) FT_Done_FreeType(library);
			return false;
		}
		return true;
	}

	bool LoadGlygh(FreetypeGlygh& G, char c, FT_Face& face, unsigned char* data, int& line, int width, int height)
	{
		if (line >= height - 200) return false;

		FT_ULong code = (FT_ULong)c;

		FT_Glyph glyph;
		FT_Load_Glyph(face, FT_Get_Char_Index(face, c), FT_LOAD_DEFAULT);
		FT_Error error = FT_Get_Glyph(face->glyph, &glyph);
		if (error) return false;

		FT_Glyph_To_Bitmap(&glyph, ft_render_mode_normal, 0, 1);
		FT_BitmapGlyph bitmap_glyph = (FT_BitmapGlyph)glyph;
		FT_Bitmap& bitmap = bitmap_glyph->bitmap;

		float x = 0.0f;
		float y = (float)line / (float)height;
		float dx = (float)(bitmap.width - 1) / (float)width;
		float dy = (float)bitmap.rows / (float)height;
		G.x = x; G.y = y; G.wx = dx; G.wy = dy;

		unsigned char* b = bitmap.buffer;
		for (unsigned int i = 0; i < bitmap.rows; ++i, ++line)
		{
			unsigned char* ddata = data + line * width;
			for (unsigned int j = 0; j < bitmap.width; ++j)
			{
				*ddata++ = bitmap.buffer[i * bitmap.width + j];
			}
		}

		FT_Done_Glyph(glyph);
		return true;
	}

	FreetypeFont::FreetypeFont(int size)
		:m_size(size), m_line(0)
	{
		m_texture = new osg::Texture2D;
		m_texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
		m_texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
		m_texture->setInternalFormat(GL_R32F);
		m_texture->setSourceFormat(GL_RED);
		m_texture->setSourceType(GL_UNSIGNED_BYTE);
		m_texture->setResizeNonPowerOfTwoHint(false);

		m_image_width = 200;
		m_image_height = 400;
		m_image = new osg::Image();
		m_image->allocateImage(m_image_width, m_image_height, 1, GL_RED, GL_UNSIGNED_BYTE);
		unsigned char* data = m_image->data();
		memset(data, 255, m_image_width * m_image_height);
		m_texture->setImage(m_image);
	}

	FreetypeFont::~FreetypeFont()
	{

	}

	osg::Texture2D* FreetypeFont::GetTexture()
	{
		return m_texture;
	}

	int FreetypeFont::GetSize() const
	{
		return m_size;
	}

	bool FreetypeFont::Init(const std::string& file)
	{
		m_file = file;

		FT_Library library;
		FT_Face face;
		FT_Size_Metrics metrics;
		if (!LoadFreetype(file, m_size, library, face))
			return false;

		//glyphs = vector_new(sizeof(texture_glyph_t *));
		m_height = 0.0f;
		m_ascender = 0.0f;
		m_descender = 0.0f;
		m_outline_thickness = 0.0f;
		int m_hinting = 1;

		m_underline_position = face->underline_position / (float)(HRESf*HRESf) * m_size;
		m_underline_position = roundf(m_underline_position);
		if (m_underline_position > -2.0f)
			m_underline_position = -2.0f;

		m_underline_thickness = face->underline_thickness / (float)(HRESf*HRESf) * m_size;
		m_underline_thickness = roundf(m_underline_thickness);
		if (m_underline_thickness < 1.0f)
			m_underline_thickness = 1.0f;

		metrics = face->size->metrics;
		m_ascender = (metrics.ascender >> 6) / 100.0f;
		m_descender = (metrics.descender >> 6) / 100.0f;
		m_height = (metrics.height >> 6) / 100.0f;
		m_linegap = m_height - m_ascender + m_descender;

		FT_Done_Face(face);
		FT_Done_FreeType(library);
		return true;
	}

	FreetypeGlygh* FreetypeFont::IsGlyghLoaded(char c)
	{
		unsigned index = 0;
		for (size_t i = 0; i < m_glyghs.size(); ++i)
		{
			FreetypeGlygh& glygh = m_glyghs.at(i);
			if (glygh.code = index) return &glygh;
		}
		return NULL;
	}

	void FreetypeFont::GenTextureCoord(const FreetypeGlygh& glygh, std::vector<osg::Vec2f>& texcoord)
	{
		osg::Vec2f dmin = osg::Vec2f(glygh.x, glygh.y);
		float dx = glygh.wx; float dy = glygh.wy;
		texcoord.push_back(dmin);
		texcoord.push_back(dmin + osg::Vec2f(dx, 0.0f));
		texcoord.push_back(dmin + osg::Vec2f(dx, dy));
		texcoord.push_back(dmin + osg::Vec2f(0.0f, dy));
	}

	void FreetypeFont::GenTextureCoord(const std::string& text, std::vector<osg::Vec2f>& texcoord)
	{
		FT_Library library;
		FT_Face face;

		if (!LoadFreetype(m_file, m_size, library, face))
			return;

		bool need_update = false;
		for (std::string::const_iterator it = text.begin(); it != text.end(); ++it)
		{
			FreetypeGlygh* glygh = IsGlyghLoaded(*it);
			if (!glygh)
			{
				FreetypeGlygh lg;
				bool load = LoadGlygh(lg, *it, face, m_image->data(), m_line, m_image_width, m_image_height);
				if (load)
				{
					m_glyghs.push_back(lg);
					glygh = &m_glyghs.back();
					need_update = true;
				}
			}
			if (glygh) GenTextureCoord(*glygh, texcoord);
		}

		if (need_update) m_texture->dirtyTextureObject();

		FT_Done_Face(face);
		FT_Done_FreeType(library);
	}
}