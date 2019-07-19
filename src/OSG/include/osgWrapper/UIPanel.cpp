#include <osgWrapper/UIPanel.h>

namespace OSGWrapper
{
	UIPanel::UIPanel()
	{

	}

	UIPanel::~UIPanel()
	{

	}

	void UIPanel::SetGlobalUI(UIItem* item)
	{
		m_global_item = item;
	}

	void UIPanel::AddUI(UIItem* item)
	{
		if(item) m_local_items.push_back(item);
	}

	void UIPanel::RemoveUI(UIItem* item)
	{
		std::vector<osg::ref_ptr<UIItem>>::iterator it = std::find(m_local_items.begin(), m_local_items.end(), item);
		if (it != m_local_items.end())
		{
			m_local_items.erase(it);
		}
	}
}