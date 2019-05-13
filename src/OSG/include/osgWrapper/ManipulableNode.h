#ifndef MANIPULABLE_NODE_H
#define MANIPULABLE_NODE_H
#include <osg\MatrixTransform>

namespace OSGWrapper
{

class OSG_EXPORT ManipulableNode : public osg::MatrixTransform
{
	friend class ManipulableVisitor;
public:
	ManipulableNode();
	~ManipulableNode();

	void UseModelUniform();

	void RemoveAll();
	void AddChild(osg::Node* child, bool contains_not = false);
	void RemoveChild(osg::Node* child);
	bool ContainChild(osg::Node* child);

	void SetMatrix(const osg::Matrixf& mat);
	const osg::Matrixf& GetGlobalMatrix() const;
	const osg::Matrixf& GetLocalMatrix() const;
	const osg::Matrixf& GetParentToScene() const;

	osg::BoundingSphere GetChildBounding();

	void Reset();
protected:
	void TraverseUpdate();

	void ParentChanged(const osg::Matrixf& parent_to_scene);
private:
	osg::ref_ptr<osg::Uniform> m_model_uniform;

	osg::Matrixf m_local_matrix;
	osg::Matrixf m_global_matrix;
	osg::Matrixf m_parent_to_scene;
};
} 

#endif // MANIPULABLE_NODE_H