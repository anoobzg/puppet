#version 430 core

layout(location = 0) in vec2 ver_attribute_position;
//layout(r32f, binding = 0) uniform image2D mask;

uniform float viewport_width;
uniform float viewport_height;
uniform vec2 origin = vec2(0.0, 0.0);
out vec2 screen_pos;
void main()
{
	screen_pos = ver_attribute_position;
	//imageStore(mask, ivec2(0, 0), vec4(1.0, 0.0, 0.0, 0.0));
	vec2 viewport_rect = 2.0 * (ver_attribute_position) / vec2(viewport_width, viewport_height);
	vec2 norm_screen_pos = viewport_rect + vec2(-1.0, -1.0);
    gl_Position = vec4(norm_screen_pos, 0.0, 1.0);
}