#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;

out vec3 position;
out vec3 normal;
void main()
{
	position = ver_attribute_position;
	normal = ver_attribute_normal;
}