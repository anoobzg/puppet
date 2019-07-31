#include <osgWrapper/UnionTexture.h>
#include <osgDB/ReadFile>

namespace OSGWrapper
{
	UnionTexture::UnionTexture()
		:m_current_x(0), m_current_y(0)
	{
		unsigned char* data = new unsigned char[4 * UnionWidth * UnionWidth];
		memset(data, 0, 4 * UnionWidth * UnionWidth);
		m_image = new osg::Image();
		m_image->setImage(UnionWidth, UnionWidth, 1, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, data, osg::Image::USE_NEW_DELETE);
		setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
		setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
		setImage(m_image);
	}

	UnionTexture::~UnionTexture()
	{

	}

	UnionCoord* UnionTexture::Get(const std::string& name)
	{
		std::map<std::string, UnionCoord>::iterator it = m_coords.find(name);
		if (it != m_coords.end())
			return &(*it).second;

		UnionCoord* coord = 0;
		AddTexture(name, coord);
		return coord;
	}

	void UnionTexture::AddTexture(const std::string& name, UnionCoord*& coord)
	{
		coord = 0;

		osg::ref_ptr<osg::Image> image = osgDB::readImageFile(name);
		if (!image) return;

		if (image->getDataType() != GL_RGBA) return;

		Add(name, image);

		std::map<std::string, UnionCoord>::iterator it = m_coords.find(name);
		if (it != m_coords.end())
			coord = &(*it).second;
	}

	void UnionTexture::Add(const std::string& name, osg::Image* sub_image)
	{
		UnionCoord tcoord;
		tcoord.dmax = osg::Vec2f(1.0f, 1.0f);
		tcoord.dmin = osg::Vec2f(0.0f, 0.0f);

		int w = sub_image->s();
		int h = sub_image->t();

		int start_x = -1;
		int start_y = -1;
		if (w + m_current_x <= UnionWidth && h <= UnionWidth)
			return; 


		if (start_x == -1 || start_y == -1)
			return;
		//copy data
		unsigned char* d = sub_image->data();
		unsigned char* dst_raw = m_image->data();
		for (int i = 0; i < h; ++i)
		{
			int dst_i = i + start_y;
			for (int j = 0; j < w; ++j)
			{
				int dst_j = j + start_x;
				unsigned char* d_ = dst_raw + 4 * (dst_i * UnionWidth + dst_j);

				*d_++ = *d++;
				*d_++ = *d++;
				*d_++ = *d++;
				*d_++ = *d++;
			}
		}

		m_coords.insert(std::pair<std::string, UnionCoord>(name, tcoord));
	}
}