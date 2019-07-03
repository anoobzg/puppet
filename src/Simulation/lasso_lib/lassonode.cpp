#include "lassonode.h"
#include <iostream>
#include <osgWrapper/ProgramManager.h>

#include "selectalloperation.h"
#include "deletealloperation.h"
#include "reserveoperation.h"
#include "copyoperation.h"
#include "eraseoperation.h"
#include "deleteoperation.h"
#include "quaderaseoperation.h"
#include "flipoperation.h"
#include "triangleoperation.h"
#include <osg\GLExtensions>

LassoNode::LassoNode()
	:m_frame(0), m_context(NULL)
{
	std::vector<std::string> program_lists;
	program_lists.push_back("feed_selectall");
	program_lists.push_back("feed_deleteall");
	program_lists.push_back("feed_reserve");
	program_lists.push_back("feed_copy");
	program_lists.push_back("feed_erase");
	program_lists.push_back("feed_quaderase");
	program_lists.push_back("feed_delete");
	program_lists.push_back("feed_flip");
	program_lists.push_back("feed_multiquaderase");
	program_lists.push_back("feed_triangle");
	for (size_t i = 0; i < program_lists.size(); ++i)
	{
		osg::Program* program = OSGWrapper::ProgramManager::Instance().Get(program_lists.at(i).c_str());
		if (program)
		{
			program->addTransformFeedBackVarying(std::string("feedback_attribute"));
			program->setTransformFeedBackMode(GL_INTERLEAVED_ATTRIBS);
		}
	}

	CreateTexture();
}

LassoNode::~LassoNode()
{

}

void LassoNode::Set(osg::Vec3Array* coord_array, osg::FloatArray* flag_array)
{
	osg::FloatArray* flag_array_ex = new osg::FloatArray();
	flag_array_ex->resize(flag_array->size(), 0.0f);
	flag_array->setVertexBufferObject(new osg::VertexBufferObject());
	flag_array_ex->setVertexBufferObject(new osg::VertexBufferObject());

	m_feed_geometry = new FeedGeometry(coord_array, flag_array, flag_array_ex);
	m_copy_geometry = new FeedGeometry(coord_array, flag_array_ex, flag_array);
	//SelectAll();

	m_flag_array = flag_array_ex;
}

bool LassoNode::OnMouse(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (m_operation) return m_operation->OnMouse(ea, aa);
	return false;
}

void LassoNode::OnFrame()
{
	++m_frame;
	if (m_operation && m_operation->ReleaseOnFrame(m_frame))
	{
		bool copy = m_operation->NeedCopy();
		ReleaseOperation();
		if (false)
		{
			m_operation = new CopyOperation(m_frame, m_feed_geometry);
			AddChild(m_operation);

			std::cout << "CopyOperation on frame " << m_frame << std::endl;
		}
	}
	//std::cout << "frame number " << m_frame << std::endl;
}

void LassoNode::ReleaseOperation()
{
	RemoveAll();

	std::cout << m_operation->OperationName() << " Release on frame " << m_frame << std::endl;
	m_operation = 0;
}

void LassoNode::SelectAll()
{
	osg::ref_ptr<SelectAllOperation> opeartion = new SelectAllOperation(m_frame, m_feed_geometry);
	AddOperation(opeartion);
}

void LassoNode::DeselectAll()
{
	osg::ref_ptr<DeleteAllOperation> opeartion = new DeleteAllOperation(m_frame, m_feed_geometry);
	AddOperation(opeartion);
}

void LassoNode::Reserve()
{
	osg::ref_ptr<ReserveOperation> opeartion = new ReserveOperation(m_frame, m_feed_geometry);
	AddOperation(opeartion);
}

void LassoNode::EnterEraseMode()
{
	osg::ref_ptr<EraseOperation> operations = new EraseOperation(m_frame, m_feed_geometry);
	AddOperation(operations);
}

void LassoNode::EnterOutEraseMode()
{
	EraseOperation* operation = (EraseOperation*)m_operation.get();
	if (operation) operation->SetRelease();
}

void LassoNode::Delete()
{
	osg::ref_ptr<DeleteOperation> operations = new DeleteOperation(m_frame, m_feed_geometry);
	AddOperation(operations);
}

void LassoNode::EnterQuadMode()
{
	osg::ref_ptr<QuadEraseOperation> operations = new QuadEraseOperation(m_frame, m_feed_geometry);
	AddOperation(operations);
}

void LassoNode::EnterOutQuadMode()
{
	QuadEraseOperation* operation = (QuadEraseOperation*)m_operation.get();
	if (operation) operation->SetRelease();
}

void LassoNode::EnterMultiQuadMode()
{
	osg::ref_ptr<QuadEraseOperation> operations = new QuadEraseOperation(m_frame, m_feed_geometry);
	operations->SetRenderProgram("feed_multiquaderase");
	AddOperation(operations);
}

void LassoNode::EnterOutMultiQuadMode()
{
	QuadEraseOperation* operation = (QuadEraseOperation*)m_operation.get();
	if (operation) operation->SetRelease();
}

void LassoNode::EnterTriangleMode()
{
	osg::ref_ptr<TriangleOperation> operations = new TriangleOperation(m_frame, m_feed_geometry);
	operations->SetTexture(m_texture);
	AddOperation(operations);
}

void LassoNode::EnterOutTriangleMode()
{
	TriangleOperation* operation = (TriangleOperation*)m_operation.get();
	if (operation) operation->SetRelease();

	ResetImage();
}

void LassoNode::Flip()
{
	osg::ref_ptr<FlipOperation> operations = new FlipOperation(m_frame, m_feed_geometry);
	AddOperation(operations);
}

void LassoNode::AddOperation(Operation* operation)
{
	if (m_operation || !operation)
		return;

	m_operation = operation;
	AddChild(m_operation);
	AddChild(new CopyOperation(m_frame, m_copy_geometry));
	std::cout << operation->OperationName() << " on frame " << m_frame << std::endl;
}

void LassoNode::CreateTexture()
{
	m_texture = new osg::Texture2D;
	m_texture->setTextureSize(1920, 1080);
	m_texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
	m_texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
	m_texture->setInternalFormat(GL_R32F);
	m_texture->setSourceFormat(GL_RED);
	m_texture->setSourceType(GL_FLOAT);
	m_texture->bindToImageUnit(0, osg::Texture::READ_WRITE);
	m_texture->setResizeNonPowerOfTwoHint(false);

	m_image = new osg::Image();
	int size = 1920 * 1080;
	float* data = new float[size];
	for(int i = 0; i < size; ++i)
		*(data + i) = 0.0f;
	m_image->setImage(1920, 1080, 1, GL_R32F, GL_RED, GL_FLOAT, (unsigned char*)data, osg::Image::USE_NEW_DELETE);
	m_texture->setImage(m_image);
}

void LassoNode::ResetImage()
{
	float* data = (float*)m_image->data();
	int size = 1920 * 1080;
	for (int i = 0; i < size; ++i)
		*(data + i) = 0.0f;
	m_image->dirty();
}

void LassoNode::Get(std::vector<bool>& flags)
{
	osg::GLExtensions* extension = 0;
	unsigned int context_id = 0;
	if (m_context) context_id = m_context->getState()->getContextID();
	
	extension = osg::GLExtensions::Get(context_id, true);
	if (m_flag_array && m_flag_array->size() > 0 && extension)
	{
		size_t size = m_flag_array->size();
		flags.resize(size, false);

		GLuint ubuff = m_flag_array->getOrCreateGLBufferObject(context_id)->getGLObjectID();
		extension->glBindBuffer(GL_ARRAY_BUFFER_ARB, ubuff);

		float* data = (float*)(extension->glMapBuffer(GL_ARRAY_BUFFER_ARB, GL_READ_WRITE_ARB));
		if (data)
		{
			for (size_t i = 0; i < size; ++i)
			{
				if (*(data + i) >= 2.0f)
					flags.at(i) = true;
			}
		}

		extension->glUnmapBuffer(GL_ARRAY_BUFFER_ARB);
		extension->glBindBuffer(GL_ARRAY_BUFFER_ARB, 0);
	}
}

void LassoNode::SetGraphcontext(osg::GraphicsContext* context)
{
	m_context = context;
}
