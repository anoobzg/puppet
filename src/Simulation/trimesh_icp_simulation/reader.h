#pragma once
#include "interface.h"
#include <base\threading\thread.h>
#include "csvwriter.h"

class Reader : public base::Thread
{
public:
	Reader();
	virtual ~Reader();

	void StartRead(const ReaderParameters& parameters);
	void StopRead();
	void SetVO(VO* vo);

private:
	void Read();
	trimesh::TriMesh* LoadOneFrame();
protected:
	VO* m_vo;
	ReaderParameters m_parameters;
	int m_current_index;
	bool m_stop;

	std::unique_ptr<CSVWriter> m_writer;
};