#version 430 core

layout(location = 0) in vec3 ver_attribute_position;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

void main()
{
	gl_Position = projection_matrix * view_matrix * model_matrix * vec4(ver_attribute_position, 1.0);
}