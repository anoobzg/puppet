#ifndef COLOR_INDEX_PICKER
#define COLOR_INDEX_PICKER
#include <osg\Camera>
#include <osg\Texture2D>
#include <osg\FrameBufferObject>

namespace OSGWrapper
{

class OSG_EXPORT ColorIndexPicker : public osg::Camera
{
public:
	ColorIndexPicker(osg::Camera* camera, int width, int height);
	~ColorIndexPicker();

	unsigned Pick(int x, int y);
	void Pick(int x, int y, unsigned char* color);
	void Pick(int x, int y, unsigned& mesh_id, unsigned& primitive_id);

	void Clear();
	void SetNode(osg::Node* node);

	osg::Texture2D* GetColorTexture();
	osg::Texture2D* GetDepthTexture();
private:
	unsigned Char2ID(unsigned char* c);
	void Char2ID(unsigned char* c, unsigned& mesh_id, unsigned& primitive_id);
protected:
	osg::ref_ptr<osg::Camera> m_camera;

	osg::ref_ptr<osg::Texture2D> m_depth_texture;
	osg::ref_ptr<osg::Texture2D> m_color_texture;

	osg::ref_ptr<osg::FrameBufferObject> m_framebuffer_object;
};

}
#endif // COLOR_INDEX_PICKER