#pragma once
#include <osg/Texture2D>
#include <map>

namespace OSGWrapper
{
	struct UnionCoord
	{
		osg::Vec2f dmin;
		osg::Vec2f dmax;
	};

#define UnionWidth 4096

	class OSG_EXPORT UnionTexture : public osg::Texture2D
	{
	public:
		UnionTexture();
		~UnionTexture();

		UnionCoord* Get(const std::string& name);
		void AddTexture(const std::string& name, UnionCoord*& coord);
	protected:
		void Add(const std::string& name, osg::Image* sub_image);
	private:
		std::map<std::string, UnionCoord> m_coords;

		osg::ref_ptr<osg::Image> m_image;

		int m_current_x;
		int m_current_y;
	};
}