#pragma once
#include <QObject>
#include <QTimer>
#include <QImage>

class SearcherListener
{
public:
	virtual ~SearcherListener() {}
	virtual void SearchStart() = 0;
	virtual void SearchStep(const QImage& image) = 0;
	virtual void SearchEnd() = 0;
};

class Bucket
{
public:
	Bucket(const QColor& color);
	~Bucket();

	void Push(const QPoint& point);
public:
	QColor m_color;
	std::vector<QPoint> m_pixels;
};

struct Change
{
	QColor color;
	QPoint point;
};

class Searcher : public QObject
{
	Q_OBJECT
public:
	Searcher();
	~Searcher();

	void SetListener(SearcherListener* listener);
	void SetImage(QImage* image);
	void Start();
protected slots:
	void Step();

private:
	void SetEnd();
	void ClearSearch();
	void ProcessOneStep();
	void ProcessOnePixel();
	void SearchBucket();
	void ProcessBucket();
	void VisualChange(const std::vector<Change>& changes);
private:
	QTimer* m_timer;
	SearcherListener* m_listener;
	bool m_running;

	QImage* m_image;
	QImage* m_p_image;

	std::vector<QPoint> offsets;

	QSize m_size;
	std::vector<bool> m_search_flags;
	std::vector<Bucket> m_buckets;
	
	Bucket* m_current_bucket;
	std::vector<QPoint> m_next_bucket_pixels;

	std::vector<Change> m_changes;
	bool m_end;
	int m_x;
	int m_y;
};