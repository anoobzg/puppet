#include "OSGCube.h"
#include "osgWrapper/ArrayCreator.h"

OSGCube::OSGCube(float* center, float* fScale)
{
	osg::ref_ptr<osg::Vec3Array> coord_array = 0;

	float cube[72] =
	{
		-0.5f, -0.5f, -0.5f, // Front
		 0.5f, -0.5f, -0.5f,
		 0.5f,  0.5f, -0.5f,
		-0.5f,  0.5f, -0.5f,

		-0.5f, -0.5f, -0.5f, // Bottom
		 0.5f, -0.5f, -0.5f,
		 0.5f, -0.5f,  0.5f,
		-0.5f, -0.5f,  0.5f,

		 0.5f,  0.5f, -0.5f, // Head
		-0.5f,  0.5f, -0.5f,
		-0.5f,  0.5f,  0.5f,
		 0.5f,  0.5f,  0.5f,

		-0.5f, -0.5f, -0.5f, // Left
		-0.5f,  0.5f, -0.5f,
		-0.5f,  0.5f,  0.5f,
		-0.5f, -0.5f,  0.5f,

		 0.5f, -0.5f, -0.5f, // Right
		 0.5f,  0.5f, -0.5f,
		 0.5f,  0.5f,  0.5f,
		 0.5f, -0.5f,  0.5f,

		 -0.5f, -0.5f,  0.5f, // Back
		  0.5f, -0.5f,  0.5f,
		  0.5f,  0.5f,  0.5f,
		 -0.5f,  0.5f,  0.5f,
	};

	coord_array = (osg::Vec3Array*)OSGWrapper::ArrayCreator::CreateVec3Array(24, cube);
	
	// Scale
	for (auto& vec : *coord_array)
	{
		vec[0] *= fScale[0];
		vec[1] *= fScale[1];
		vec[2] *= fScale[2];
	}
	
	// Translate
	for (auto& vec : *coord_array)
	{
		vec[0] += center[0];
		vec[1] += center[1];
		vec[2] += center[2];
	}

	setVertexAttribArray(0, coord_array, osg::Array::BIND_PER_VERTEX);
	addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 24));

	setUseVertexBufferObjects(true);
	setUseDisplayList(false);
	setCullingActive(false);

	m_matrix = new osg::Uniform("position_matrix", osg::Matrixf::identity());
	getOrCreateStateSet()->addUniform(m_matrix);
}

void OSGCube::SetCenter(float* center)
{
	m_matrix->set(osg::Matrixf::translate(center[0], center[1], center[2]));
}

void OSGCube::SetColor(float* rgba)
{
	getOrCreateStateSet()->addUniform(new osg::Uniform("color", osg::Vec4f(rgba[0], rgba[1], rgba[2], rgba[3])));
}
