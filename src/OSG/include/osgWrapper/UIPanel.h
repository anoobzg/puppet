#pragma once
#include <osgWrapper/AttributeUtilNode.h>
#include <osgWrapper/UIItem.h>

namespace OSGWrapper
{
	class OSG_EXPORT UIPanel : public  AttributeUtilNode
	{
	public:
		UIPanel();
		~UIPanel();

		void SetGlobalUI(UIItem* item);
		void AddUI(UIItem* item);
		void RemoveUI(UIItem* item);
	private:
		osg::ref_ptr<UIItem> m_global_item;
		std::vector<osg::ref_ptr<UIItem>> m_local_items;
	};
}