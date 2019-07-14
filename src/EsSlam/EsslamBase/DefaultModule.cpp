#include "DefaultModule.h"
#include <iostream>

namespace esslam
{
	DefaultInput::DefaultInput()
		:m_read_tracer(NULL), m_processor(NULL)
	{

	}

	DefaultInput::~DefaultInput()
	{

	}

	void DefaultInput::SetInputTracer(IReadTracer* tracer)
	{
		m_read_tracer = tracer;
	}

	void DefaultInput::SetProcessor(IProcessor* processor)
	{
		m_processor = processor;
	}

	void DefaultInput::StartInput(const SlamParameters& parameters)
	{
		std::cout << "Start Default Input." << std::endl;
	}

	void DefaultInput::StopInput()
	{

	}

	void DefaultInput::SetImageData(HandleScanImageData* data)
	{

	}

	void DefaultInput::Release(DFrame* frame)
	{

	}

	DefaultProcessor::DefaultProcessor()
		:m_visual(NULL)
	{

	}

	DefaultProcessor::~DefaultProcessor()
	{

	}

	void DefaultProcessor::SetVisual(IVisual* visual)
	{
		m_visual = visual;
	}

	void DefaultProcessor::StartProcessor(const SlamParameters& parameters)
	{
		std::cout << "Start Default Processor." << std::endl;
	}

	void DefaultProcessor::StopProcessor()
	{

	}

	void DefaultProcessor::Build(IBuildTracer* tracer)
	{

	}

	void DefaultProcessor::Clear()
	{

	}

	void DefaultProcessor::ProcessFrame(DFrame* frame)
	{

	}

	DefaultVisual::DefaultVisual()
		:m_tracer(NULL), m_input(NULL)
	{

	}

	DefaultVisual::~DefaultVisual()
	{

	}

	void DefaultVisual::SetOSGTracer(IOSGTracer* tracer)
	{
		m_tracer = tracer;
	}

	void DefaultVisual::SetInput(IInput* input)
	{
		m_input = input;
	}

	void DefaultVisual::StartVisual(const SlamParameters& parameters)
	{
		std::cout << "Start Default Visual." << std::endl;
	}

	void DefaultVisual::StopVisual()
	{

	}

	void DefaultVisual::FrameLocated(DFrame* frame)
	{

	}
}