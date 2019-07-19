#version 430 core

layout(location = 0) in vec2 ver_attribute_position;
layout(location = 1) in vec2 ver_attribute_texcoord;

uniform float viewport_width;
uniform float viewport_height;
uniform vec2 size;
uniform vec2 offset;

out vec2 texcoord;

void main()
{
	vec2 viewport_rect = 2.0 * (ver_attribute_position * size + offset) / vec2(viewport_width, viewport_height);
	vec2 screen_pos = viewport_rect + vec2(-1.0, -1.0);
    gl_Position = vec4(screen_pos, 0.0, 1.0);
	texcoord = ver_attribute_texcoord;
}