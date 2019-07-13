#version 430 core

layout(location = 0) in vec2 ver_attribute_position;

void main()
{
	gl_Position = vec4(ver_attribute_position, -0.9990, 1.0);
}