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