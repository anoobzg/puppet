#include <osgWrapper/ManipulableNode.h>
#include <osgWrapper/StateDeclare.h>
#include <osgWrapper/ProgramManager.h>

namespace OSGWrapper
{
	class ManipulableVisitor : public osg::NodeVisitor
	{
	public:
		ManipulableVisitor(const osg::Matrixf& matrix)
			:osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
		{
			m_parent_to_scene_matrix = matrix;
		}

		virtual void apply(osg::Node& node)
		{
			ManipulableNode* mn = dynamic_cast<ManipulableNode*>(&node);
			if(mn)
			{
				mn->ParentChanged(m_parent_to_scene_matrix);

				m_parent_to_scene_matrix = mn->GetGlobalMatrix();
				traverse(*mn);
				
				osg::Matrixf local_matrix = mn->GetLocalMatrix();
				m_parent_to_scene_matrix = osg::Matrixf::inverse(local_matrix) * m_parent_to_scene_matrix;
			}
		}

	private:
		osg::Matrixf m_parent_to_scene_matrix;
	};

	ManipulableNode::ManipulableNode()
	{
		m_model_uniform = new osg::Uniform("model_matrix", osg::Matrixf::identity());
	}

	ManipulableNode::~ManipulableNode()
	{
	}

	void ManipulableNode::UseModelUniform()
	{
		getOrCreateStateSet()->addUniform(m_model_uniform, state_on);
	}

	void ManipulableNode::RemoveAll()
	{
		removeChildren(0, getNumChildren());
	}

	void ManipulableNode::AddChild(osg::Node* child, bool contains_not)
	{
		if(contains_not && containsNode(child))
			return;

		addChild(child);
		ManipulableNode* update_node = (ManipulableNode*)child;
		if(update_node)
			TraverseUpdate();
	}

	void ManipulableNode::RemoveChild(osg::Node* child)
	{
		removeChild(child);
	}

	bool ManipulableNode::ContainChild(osg::Node* child)
	{
		return containsNode(child);
	}

	void ManipulableNode::SetMatrix(const osg::Matrixf& mat)
	{
		osg::Matrixf parent_to_scene = osg::Matrixf::inverse(m_local_matrix) * m_global_matrix;
		m_local_matrix = mat;
		setMatrix(mat);
		m_global_matrix = m_local_matrix * parent_to_scene;
		m_model_uniform->set(m_global_matrix);

		TraverseUpdate();
	}

	const osg::Matrixf& ManipulableNode::GetGlobalMatrix() const
	{
		return m_global_matrix;
	}

	const osg::Matrixf& ManipulableNode::GetLocalMatrix() const
	{
		return m_local_matrix;
	}

	void ManipulableNode::TraverseUpdate()
	{
		ManipulableVisitor visitor(m_global_matrix);
		visitor.traverse(*this);
	}

	void ManipulableNode::ParentChanged(const osg::Matrixf& parent_to_scene)
	{
		m_global_matrix = m_local_matrix * parent_to_scene;
		m_model_uniform->set(m_global_matrix);

		m_parent_to_scene = parent_to_scene;
	}

	osg::BoundingSphere ManipulableNode::GetChildBounding()
	{
		osg::BoundingSphere bsphere;
		if (_children.empty())
		{
			return bsphere;
		}

		for(osg::NodeList::const_iterator itr = _children.begin(); itr != _children.end(); ++itr)
			bsphere.expandBy((*itr)->getBound());
		return bsphere;
	}

	void ManipulableNode::Reset()
	{
		SetMatrix(osg::Matrixf::identity());
	}

	const osg::Matrixf& ManipulableNode::GetParentToScene() const
	{
		return m_parent_to_scene;
	}
}