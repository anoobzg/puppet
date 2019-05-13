#include <Windows.h>

#include <osgWrapper/FeedbackProgram.h>
#include <osg\State>

namespace OSGWrapper
{

static osg::buffered_object< osg::ref_ptr<FeedbackExtensions> > s_feedback_extensions;
FeedbackExtensions* FeedbackExtensions::instance(unsigned contextID, bool create)
{
	if(!s_feedback_extensions[contextID] && create)
		s_feedback_extensions[contextID] = new FeedbackExtensions(contextID);
	return s_feedback_extensions[contextID];
}

FeedbackExtensions::FeedbackExtensions(unsigned int contextID)
{
	convert(glBeginTransformFeedback, GetExtensionFunc("glBeginTransformFeedback", "glBeginTransformFeedbackEXT"));
	convert(glEndTransformFeedback, GetExtensionFunc("glEndTransformFeedback", "glEndTransformFeedbackEXT"));
	convert(glTransformFeedbackVaryings, GetExtensionFunc("glTransformFeedbackVaryings", "glTransformFeedbackVaryingsEXT"));
	convert(glGetTransformFeedbackVarying, GetExtensionFunc("glGetTransformFeedbackVarying", "glGetTransformFeedbackVaryingEXT"));
	convert(glBindTransformFeedback, GetExtensionFunc("glBindTransformFeedback", "glBindTransformFeedbackEXT"));
	convert(glDeleteTransformFeedbacks, GetExtensionFunc("glDeleteTransformFeedbacks", "glDeleteTransformFeedbacksEXT"));
	convert(glGenTransformFeedbacks, GetExtensionFunc("glGenTransformFeedbacks", "glGenTransformFeedbacksEXT"));
	convert(glIsTransformFeedback, GetExtensionFunc("glIsTransformFeedback", "glIsTransformFeedbackEXT"));
	convert(glPauseTransformFeedback, GetExtensionFunc("glPauseTransformFeedback", "glPauseTransformFeedbackEXT"));
	convert(glResumeTransformFeedback, GetExtensionFunc("glResumeTransformFeedback", "glResumeTransformFeedbackEXT"));
}

void* FeedbackExtensions::GetExtensionFunc(const char* func, const char* fallback_func)
{
	void* pointer = GetExtensionFunc(func);
	return (pointer != 0) ? pointer : GetExtensionFunc(fallback_func);
}

void* FeedbackExtensions::GetExtensionFunc(const char* func)
{
#ifdef WIN32
	PROC proc = ::wglGetProcAddress(func);
	void* result = 0;
	memcpy(&result, &proc, sizeof(proc));
	return result;
#else
	return 0;
#endif
}

FeedbackProgram::FeedbackProgram()
{
}

FeedbackProgram::~FeedbackProgram()
{
}

void FeedbackProgram::SetTransformFeedbackMode(GLenum e)
{
	m_feedback_mode = e;
}

GLenum FeedbackProgram::GetTransformFeedbackMode() const 
{
	return m_feedback_mode;
}

void FeedbackProgram::AddTransformFeedbackVarying(const std::string& outname)
{
	m_feedback_out.push_back(outname);
}

unsigned FeedbackProgram::GetNumTransformFeedbackVaryings() const
{
	return (unsigned)m_feedback_out.size();
}

const std::string& FeedbackProgram::GetTransformFeedbackVarying(unsigned i) const
{
	return m_feedback_out[i];
}

void FeedbackProgram::compileGLObjects(osg::State& state) const
{
	if( isFixedFunction() ) return;
	const unsigned int contextID = state.getContextID();

	for( unsigned int i=0; i < _shaderList.size(); ++i )
	{
		_shaderList[i]->compileShader( state );
	}

	const FeedbackExtensions* extension = FeedbackExtensions::instance(contextID, true);
	if(!m_feedback_out.empty() && extension && extension->glTransformFeedbackVaryings)
	{
		const osg::Program::PerContextProgram* pcp = getPCP(state);

		unsigned num_feedback = m_feedback_out.size();
		const char** varyings = new const char*[num_feedback];
		const char** varingsptr = varyings;
		for(std::vector<std::string>::const_iterator it = m_feedback_out.begin(); it != m_feedback_out.end(); ++it)
			*varingsptr++ = (*it).c_str();

		extension->glTransformFeedbackVaryings(pcp->getHandle(), num_feedback, varyings, m_feedback_mode);
		delete [] varyings;
	}

	getPCP( state )->linkProgram(state);
}

}