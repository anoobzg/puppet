#include "timepanel.h"
#include <iostream>
#include "timeitem.h"

TimePanel::TimePanel()
{
	SetRenderProgram("ui_color");
}

TimePanel::~TimePanel()
{

}

void TimePanel::Load(const char* usb, const char* build, const char* locate, const char* fusion)
{
	m_usb_object.reset(Load(usb));
	m_build_object.reset(Load(build));
	m_locate_object.reset(Load(locate));
	m_fusion_object.reset(Load(fusion));

	SetupItems();
}

StampObject* TimePanel::Load(const char* file)
{
	StampObject* object = 0;
	std::fstream in(file, std::ios::in);
	if (in.is_open())
	{
		object = new StampObject(file);
		object->Load(in);
	}
	in.close();
	return object;
}

void TimePanel::SetupItems()
{
	if (!m_usb_object.get() && !m_build_object.get() && !m_locate_object.get() && !m_fusion_object.get())
		return;

	float max_time = 0.0f;
	auto expand = [&max_time](StampObject& object) {
		size_t size = object.m_times.size();
		if (size > 0)
		{
			float t = object.m_times[size - 1].end;
			if (t > max_time) max_time = t;
		}
	};
	if (m_usb_object.get()) expand(*m_usb_object);
	if (m_build_object.get()) expand(*m_build_object);
	if (m_locate_object.get()) expand(*m_locate_object);
	if (m_fusion_object.get()) expand(*m_fusion_object);

	std::cout << "max time " << max_time << std::endl;

	auto add_item = [this](int type, StampObject& object) {
		float line_start_time = 0.0f;
		float pixel_per_s = 300.0f;
		float h_start = (float)type * 11.0f;
		float height = 10.0f;
		float max_width = 1200.0f;
		osg::Vec4 color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
		if(type == 1) color = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f);
		if(type == 2) color = osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f);
		if(type == 3) color = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);

		if (object.m_times.size() > 0)
		{
			const std::vector<TimeSeg>& times = object.m_times;
			size_t size = times.size();

			for (size_t i = 0; i < 100; ++i)
			{
				const TimeSeg& seg = times.at(i);
				float d = seg.end - seg.start;
				TimeItem* item = new TimeItem(d);
				AddItem(item);

				float x = (seg.start - line_start_time) * pixel_per_s;
				float y = h_start;
				float wx = d * pixel_per_s;
				float wy = height;

				osg::Vec2 offset = osg::Vec2(x, y);
				osg::Vec2 rect_size = osg::Vec2(wx, wy);
				item->SetOffset(offset);
				item->SetSize(rect_size);
				item->SetColor(color);

				if (x >= 1000.0f)
				{
					line_start_time = seg.end;
					h_start += 100.0f;
				}
			}
		}
	};

	if (m_usb_object.get()) add_item(0, *m_usb_object);
	if (m_build_object.get()) add_item(1, *m_build_object);
	if (m_locate_object.get()) add_item(2, *m_locate_object);
	if (m_fusion_object.get()) add_item(3, *m_fusion_object);
}