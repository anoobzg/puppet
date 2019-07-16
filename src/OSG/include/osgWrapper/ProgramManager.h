#pragma once
#ifndef _PGM_MANAGER
#define _PGM_MANAGER
#include <osgWrapper/Singleton.h>
#include <osg/Program>
#include <map>

namespace OSGWrapper
{

class OSG_EXPORT ProgramManager
{
	typedef std::map<std::string, osg::ref_ptr<osg::Program> > Programs;
	typedef Programs::iterator ProgramIter;
	typedef std::pair<std::string, osg::ref_ptr<osg::Program> > ProgramPair;

	friend class Singleton<ProgramManager>;
protected:
	ProgramManager();

public:
	~ProgramManager();

	static ProgramManager& Instance();
	static void Release();

	osg::Program* Get(const char* name);
private:
	bool Load(osg::Program* program, const char* name);
	bool LoadLocal(osg::Program* program, const char* name);
	void Insert(osg::Program* program);
	osg::Shader* OpenShader(const std::string& name);

	void MapSourceIndex();

	bool LoadFromShaderSource(osg::Program* program, const char* name);
private:
	Programs m_programs;

	bool m_use_env_config;

	static ProgramManager* m_instance;

	std::map<std::string, unsigned> ShaderSourcesIndex;
};

//typedef Singleton<ProgramManager> SPM;
//#define spm (SPM::GetRef())
//#define RELEASE_PROGRAMES SPM::Delete();
//#define INTI_PROGRAMES SPM::GetRef();

}
#endif // _PGM_MANAGER