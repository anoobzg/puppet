#include "Searcher.h"

Bucket::Bucket(const QColor& color)
	:m_color(color)
{
	m_pixels.reserve(1000);
}

Bucket::~Bucket()
{

}

void Bucket::Push(const QPoint& point)
{
	m_pixels.push_back(point);
}

Searcher::Searcher()
	:m_running(false), m_listener(NULL), m_image(NULL), m_p_image(NULL)
	, m_current_bucket(NULL), m_end(false)
{
	m_timer = new QTimer(this);
	connect(m_timer, SIGNAL(timeout()), this, SLOT(Step()));

	offsets.reserve(4);
	offsets.push_back(QPoint(-1, 0));
	offsets.push_back(QPoint(1, 0));
	offsets.push_back(QPoint(0, -1));
	offsets.push_back(QPoint(0, 1));
}

Searcher::~Searcher()
{
	SetEnd();
}

void Searcher::SetListener(SearcherListener* listener)
{
	m_listener = listener;
}

void Searcher::Start()
{
	if (m_running || !m_image) return;

	m_timer->start(10);
	m_running = true;
	ClearSearch();
	if (m_listener) m_listener->SearchStart();
}

void Searcher::SetEnd()
{
	m_timer->stop();
	m_running = false;
	if (m_listener) m_listener->SearchEnd();
}

void Searcher::Step()
{
	ProcessOneStep();
	if (m_listener) m_listener->SearchStep(*m_p_image);
}

void Searcher::ProcessOnePixel()
{

}

void Searcher::ProcessOneStep()
{
	m_changes.clear();
	
	if (m_current_bucket) ProcessBucket();
	else SearchBucket();

	VisualChange(m_changes);

	if (m_end)
		SetEnd();
}

void Searcher::SearchBucket()
{
	int max_search_pixels = 100;
	QColor gray = QColor(100, 100, 100);
	QColor white = QColor(255, 255, 255, 255);

	Change c;
	for (int i = 0; i < max_search_pixels; ++i)
	{
		if ((m_x == m_size.width()) && (m_y == m_size.height()))
		{
			m_end = true;
			break;
		}

		int index = m_y * m_size.width() + m_x;
		if (m_search_flags.at(index) == false)
		{
			c.color = gray;
			c.point = QPoint(m_x, m_y);

			m_search_flags.at(index) = true;
			QColor raw_color = m_image->pixelColor(c.point);
			if (raw_color.rgb() != white.rgb())
			{//new bucket
				Bucket b = Bucket(raw_color);
				m_buckets.push_back(b);
				m_current_bucket = &m_buckets.back();
				m_next_bucket_pixels.push_back(c.point);
				break;
			}

			m_changes.push_back(c);
		}

		++m_x;

		if (m_x == m_size.width())
		{
			++m_y;

			if ((m_x == m_size.width()) && (m_y == m_size.height()))
			{
				m_end = true;
				break;
			}

			m_x = 0;
		}
	}
}

void Searcher::ProcessBucket()
{
	std::vector<QPoint> temp_pixels;
	const QColor& color = m_current_bucket->m_color;
	Change change;

#define INDEX(p) (p.y() * m_size.width() + p.x())
	while (!m_next_bucket_pixels.empty())
	{
		size_t size = m_next_bucket_pixels.size();
		for (size_t i = 0; i < size; ++i)
		{
			const QPoint& p = m_next_bucket_pixels.at(i);
			QColor c = m_image->pixelColor(p);
			change.point = p;
			change.color = c;
			m_changes.push_back(change);

			for (int j = 0; j < 4; ++j)
			{
				QPoint pp = p + offsets.at(j);
				if ((pp.x() >= 0) && (pp.x() < m_size.width())
					&& (pp.y() >= 0) && (pp.y() < m_size.height()))
				{
					int index = INDEX(pp);

					QColor oc = m_image->pixelColor(pp);
					if (!m_search_flags.at(index) && (oc == color))
					{
						m_search_flags.at(index) = true;
						temp_pixels.push_back(pp);
					}
				}
			}
		}
		m_next_bucket_pixels.swap(temp_pixels);
		temp_pixels.clear();
	}

	m_current_bucket = NULL;
}

void Searcher::VisualChange(const std::vector<Change>& changes)
{
	size_t size = changes.size();
	for (size_t i = 0; i < size; ++i)
	{
		const Change& change = changes.at(i);
		m_p_image->setPixelColor(change.point, change.color);
	}
}

void Searcher::SetImage(QImage* image)
{
	if (m_p_image)
	{
		delete m_p_image;
		m_p_image = NULL;
	}

	m_image = image;
	if (m_image)
	{
		m_p_image = new QImage(m_image->width(), m_image->height(), m_image->format());
		m_p_image->fill(QColor(255, 255, 255, 255));
		m_size = m_image->size();
	}
}

void Searcher::ClearSearch()
{
	m_buckets.clear();
	m_search_flags.resize(m_size.width() * m_size.height(), false);
	m_changes.clear();
	m_x = 0;
	m_y = 0;

	m_next_bucket_pixels.clear();
	m_current_bucket = NULL;

	m_end = false;

	m_p_image->fill(QColor(255, 255, 255, 255));
	if (m_listener) m_listener->SearchStep(*m_p_image);
}