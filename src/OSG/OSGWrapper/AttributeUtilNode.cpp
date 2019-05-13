#include <osgWrapper/AttributeUtilNode.h>
#include <osgWrapper/StateDeclare.h>
#include <osgWrapper/ProgramManager.h>

namespace OSGWrapper
{
	AttributeUtilNode::AttributeUtilNode()
	{
	}

	AttributeUtilNode::~AttributeUtilNode()
	{
	}

	void AttributeUtilNode::SetRenderProgram(const char* name)
	{
		if(m_used_program.valid())
			getOrCreateStateSet()->removeAttribute(m_used_program);

		m_used_program = ProgramManager::Instance().Get(name);
		if(m_used_program.valid())
			getOrCreateStateSet()->setAttributeAndModes(m_used_program, state_on);
	}


	void AttributeUtilNode::RemoveAll()
	{
		removeChildren(0, getNumChildren());
	}

	void AttributeUtilNode::AddChild(osg::Node* child)
	{
		addChild(child);
	}

	void AttributeUtilNode::RemoveChild(osg::Node* child)
	{
		removeChild(child);
	}

	void AttributeUtilNode::ReplaceNode(osg::Node* old_node, osg::Node* new_node)
	{

	}

	void AttributeUtilNode::AddUniform(osg::Uniform* uniform)
	{
		getOrCreateStateSet()->addUniform(uniform, state_on);
	}

	void AttributeUtilNode::RemoveUniform(osg::Uniform* uniform)
	{
		getOrCreateStateSet()->removeUniform(uniform);
	}

	void AttributeUtilNode::SetAttribute(osg::StateAttribute* attribute)
	{
		getOrCreateStateSet()->setAttribute(attribute, state_on);
	}

	void AttributeUtilNode::SetAttributeMode(osg::StateAttribute* attribute)
	{
		getOrCreateStateSet()->setAttributeAndModes(attribute, state_on);
	}

	void AttributeUtilNode::SetTextureAttribute(unsigned unit, osg::StateAttribute* attribute)
	{
		getOrCreateStateSet()->setTextureAttributeAndModes(unit, attribute, state_on);
	}

	void AttributeUtilNode::SetMode(GLenum mode, unsigned int value)
	{
		getOrCreateStateSet()->setMode(mode, value);
	}
}