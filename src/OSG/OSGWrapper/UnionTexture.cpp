#include <osgWrapper/UnionTexture.h>
#include <osgDB/ReadFile>

namespace OSGWrapper
{
	UnionTexture::UnionTexture()
		:m_chunk_size(200), m_full(false), m_current_x(0), m_current_y(0)
	{
		setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
		setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
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
		if (m_full) return;

		osg::ref_ptr<osg::Image> image = osgDB::readImageFile(name);
		if (!image) return;

		//if (image->getPixelFormat() != GL_RGBA || ima) return;

		Add(name, image);

		std::map<std::string, UnionCoord>::iterator it = m_coords.find(name);
		if (it != m_coords.end())
			coord = &(*it).second;
	}

	void UnionTexture::AllocateImage(int width, int height)
	{
		if (width <= 0 || height <= 0) return;

		unsigned char* data = new unsigned char[4 * width * height];
		memset(data, 0, 4 * width * height);
		m_image = new osg::Image();
		m_image->setImage(width, height, 1, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, data, osg::Image::USE_NEW_DELETE);
		setImage(m_image);
	}

	void UnionTexture::Add(const std::string& name, osg::Image* sub_image)
	{
		bool valid = false;
		UnionCoord tcoord;
		tcoord.dmax = osg::Vec2f(1.0f, 1.0f);
		tcoord.dmin = osg::Vec2f(0.0f, 0.0f);

		int w = sub_image->s();
		int h = sub_image->t();

		if (w > m_chunk_size || h > m_chunk_size)
		{
			valid = true;
			m_image = sub_image;
			setImage(m_image);
			m_full = true;
		}
		else
		{
			valid = true;
			int start_x = m_current_x;
			int start_y = m_current_y;
			
			float minx = (float)start_x / (float)UnionWidth;
			float miny = (float)start_y / (float)UnionWidth;
			//copy data
			if (!m_image)
			{
				AllocateImage(UnionWidth, UnionWidth);
			}

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

			float maxx = (float)(m_current_x + w) / (float)UnionWidth;
			float maxy = (float)(m_current_y + h) / (float)UnionWidth;

			m_current_x += m_chunk_size;

			if (m_current_x >= 4000 && m_current_y >= 4000)
			{
				m_full = true;
			}
			else
			{
				if (m_current_x == 4000)
				{
					m_current_y += m_chunk_size;
					m_current_x = 0;
				}
			}

			tcoord.dmin.x() = minx;
			tcoord.dmin.y() = miny;
			tcoord.dmax.x() = maxx;
			tcoord.dmax.y() = maxy;
		}

		if(valid)
			m_coords.insert(std::pair<std::string, UnionCoord>(name, tcoord));
	}

	bool UnionTexture::Full()
	{
		return m_full;
	}

	void UnionTexture::SetFull(bool full)
	{
		m_full = full;
	}
}