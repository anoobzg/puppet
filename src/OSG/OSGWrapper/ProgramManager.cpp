#include <osgWrapper\ProgramManager.h>
#include <fstream>

#include <osgWrapper\FeedbackProgram.h>
struct ShaderDef
{
	const char* name;
	const char* vert_source;
	const char* geom_source;
	const char* frag_source;
};

#include "ShaderString.h"
static ShaderDef shader_sources[] = {
	#include "ProgramDef.h"
	{0, 0, 0, 0}
};

namespace OSGWrapper
{

ProgramManager* ProgramManager::m_instance = 0;
ProgramManager& ProgramManager::Instance()
{
	if(!m_instance) m_instance = new ProgramManager();
	return *m_instance;
}

void ProgramManager::Release()
{
	if(m_instance) delete m_instance;
}

ProgramManager::ProgramManager()
	:m_use_env_config(false)
{
	m_programs.clear();

	MapSourceIndex();
}

ProgramManager::~ProgramManager()
{
	m_programs.clear();
}

void ProgramManager::MapSourceIndex()
{
	unsigned i = 0;
	ShaderDef def = shader_sources[i];
	while(def.name)
	{
		ShaderSourcesIndex.insert(std::pair<std::string, unsigned>(def.name, i)); 
		def = shader_sources[++i];
	}
}

osg::Program* ProgramManager::Get(const char* name)
{
	ProgramIter iter = m_programs.find(name);
	if (iter != m_programs.end())
		return (*iter).second;

	osg::ref_ptr<osg::Program> program = 0;
	std::string program_name(name);
	std::string::size_type pos = program_name.find("feedback");
	if(pos != std::string::npos)
		program = new FeedbackProgram();
	else
		program = new osg::Program();

	bool result = Load(program, name);
	if(!result)
		result = LoadFromShaderSource(program, name);
	return result ? program : 0;
}

bool ProgramManager::Load(osg::Program* program, const char* name)
{
	if(!program)
		return false;

	std::string prefix = "shaders\\";

	const char* path = ::getenv("SIMULATION_DIR");

	if(path)
		prefix = std::string(path) + "\\" + prefix;

	static const char* extension[] = { ".vert", ".geom", ".frag", 0 };
	static osg::Shader::Type type[] = { osg::Shader::VERTEX, osg::Shader::GEOMETRY, osg::Shader::FRAGMENT };

	program->setName(name);
	char** tmp = (char**)extension;
	unsigned i = 0;
	for (char* s = *tmp; s; s = *(++tmp), ++i)
	{
		std::string file_name = prefix + name + s;
		osg::ref_ptr<osg::Shader> shader = OpenShader(file_name);
		
		if (shader.valid())
		{
			shader->setType(type[i]);
			program->addShader(shader);
		}
	}

	if (program->getNumShaders() == 0)
		return false;

	std::pair<ProgramIter, bool> result = m_programs.insert(ProgramPair(name, program));
	if (result.second == false)
		return false;

	return true;
}

osg::Shader* ProgramManager::OpenShader(const std::string& name)
{
	osg::Shader* shader = 0;
	std::ifstream in;
	std::string source;
	in.open(name.c_str(), std::ios::in);
	if(in.is_open())
	{

		char line[256];
		while(in.getline(line, 256))
		{
			source += line;
			source += "\n";
		}

		shader = new osg::Shader();
		shader->setShaderSource(source);
	}

	in.close();

	return shader;
}

bool ProgramManager::LoadFromShaderSource(osg::Program* program, const char* name)
{
	if(!program)
		return false;

	program->setName(name);
	unsigned i = 0;

	std::map<std::string, unsigned>::iterator it = ShaderSourcesIndex.find(name);
	if(it != ShaderSourcesIndex.end())
	{
		ShaderDef& def = shader_sources[it->second];
		osg::ref_ptr<osg::Shader> v_shader = new osg::Shader(osg::Shader::VERTEX);
		osg::ref_ptr<osg::Shader> g_shader = new osg::Shader(osg::Shader::GEOMETRY);
		osg::ref_ptr<osg::Shader> f_shader = new osg::Shader(osg::Shader::FRAGMENT);
		
		if(def.vert_source)
		{
			v_shader->setShaderSource(def.vert_source);
			program->addShader(v_shader);
		}
		if(def.geom_source)
		{
			g_shader->setShaderSource(def.geom_source);
			program->addShader(g_shader);
		}
		if(def.frag_source)
		{
			f_shader->setShaderSource(def.frag_source);
			program->addShader(f_shader);
		}
	}

	if (program->getNumShaders() == 0)
		return false;

	std::pair<ProgramIter, bool> result = m_programs.insert(ProgramPair(name, program));
	if (result.second == false)
		return false;

	return true;
}

}

