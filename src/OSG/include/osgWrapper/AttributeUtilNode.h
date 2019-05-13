#ifndef ATTRIBUTE_UTIL_NODE
#define ATTRIBUTE_UTIL_NODE

#include <osg\MatrixTransform>
#include <osg\Program>

namespace OSGWrapper
{
class OSG_EXPORT AttributeUtilNode : public osg::Group
{
public:
	AttributeUtilNode();
	~AttributeUtilNode();

	void SetRenderProgram(const char* name);
	void AddUniform(osg::Uniform* uniform);
	void RemoveUniform(osg::Uniform* uniform);
	void SetAttribute(osg::StateAttribute* attribute);
	void SetAttributeMode(osg::StateAttribute* attribute);
	void SetTextureAttribute(unsigned unit, osg::StateAttribute* attribute);
	void SetMode(GLenum mode, unsigned int value);

	void RemoveAll();
	void AddChild(osg::Node* child);
	void RemoveChild(osg::Node* child);
	void ReplaceNode(osg::Node* old_node, osg::Node* new_node);
private:
	osg::ref_ptr<osg::Program> m_used_program;
};
}

#endif // ATTRIBUTE_UTIL_NODE