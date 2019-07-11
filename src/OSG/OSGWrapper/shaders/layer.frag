#version 430 core

layout(r32f, binding = 0) uniform image2D mask;

out vec4 frag_color;

in vec2 screen_pos;
uniform vec4 color = vec4(1.0, 1.0, 0.0, 1.0);
void main()
{	
	imageStore(mask, ivec2(screen_pos), vec4(1.0, 0.0, 0.0, 0.0));
	//frag_color = color;
}

//out vec4 fragment_color;
//uniform float s = 0.5;
//uniform float h = 190.0f;
//uniform float v = 0.9;
//
//void main()
//{
//	float f = h / 60.0 - 3.0;
//	float p = v * (1.0 - s);
//	float q = v * (1.0 - f * s);
//	fragment_color = vec4(p, q, v, 1.0);
//}