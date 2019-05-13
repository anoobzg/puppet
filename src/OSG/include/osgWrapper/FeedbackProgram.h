#ifndef FEEDBACK_PROGRAM
#define FEEDBACK_PROGRAM
#include <osg\Program>

#ifndef GL_INTERLEAVED_ATTRIBS 
#define GL_INTERLEAVED_ATTRIBS 0x8C8C
#endif
#ifndef GL_SEPARATE_ATTRIBS 
#define GL_SEPARATE_ATTRIBS 0x8C8D
#endif
#ifndef GL_TRANSFORM_FEEDBACK_BUFFER
#define GL_TRANSFORM_FEEDBACK_BUFFER 0x8C8E
#endif

namespace OSGWrapper
{

class OSG_EXPORT FeedbackExtensions : public osg::Referenced
{
public:
    static FeedbackExtensions* instance(unsigned contextID, bool create);
    bool isSupported() const { return m_supported; }

	void (GL_APIENTRY * glBeginTransformFeedback)(GLenum primitive_mode);
	void (GL_APIENTRY * glEndTransformFeedback)(void);
	void (GL_APIENTRY * glTransformFeedbackVaryings)(GLuint program, GLsizei count, const GLchar *const* varyings, GLenum buffer_mode);
	void (GL_APIENTRY * glGetTransformFeedbackVarying)(GLuint program, GLuint index, GLsizei buff_size, GLsizei *length, GLsizei *size, GLenum *type, GLchar *name);
	void (GL_APIENTRY * glBindTransformFeedback)(GLenum target, GLuint id);
	void (GL_APIENTRY * glDeleteTransformFeedbacks)(GLsizei n, const GLuint *ids);
	void (GL_APIENTRY * glGenTransformFeedbacks)(GLsizei n, GLuint *ids);
	GLboolean (GL_APIENTRY * glIsTransformFeedback)(GLuint id);
	void (GL_APIENTRY * glPauseTransformFeedback)(void);
	void (GL_APIENTRY * glResumeTransformFeedback)(void);

private:
	inline void* GetExtensionFunc(const char* func, const char* fallback_func);
	void* GetExtensionFunc(const char* func);

	template<typename T, typename R>
	bool convert(T& dest, R src)
	{
		memcpy(&dest, &src, sizeof(src));
		return src != 0;
	}
protected:
    FeedbackExtensions(unsigned int contextID);

    bool m_supported;
};

class OSG_EXPORT FeedbackProgram : public osg::Program
{
public:
	FeedbackProgram();
	~FeedbackProgram();

	void SetTransformFeedbackMode(GLenum e);
	GLenum GetTransformFeedbackMode() const ;
	void AddTransformFeedbackVarying(const std::string& outname);
	inline unsigned GetNumTransformFeedbackVaryings() const;
	inline const std::string& GetTransformFeedbackVarying(unsigned i) const;

    /** Compile program and associated shaders.*/
    virtual void compileGLObjects(osg::State& state) const;
protected:
	GLenum m_feedback_mode;
	std::vector<std::string> m_feedback_out;
};

}
#endif // FEEDBACK_PROGRAM