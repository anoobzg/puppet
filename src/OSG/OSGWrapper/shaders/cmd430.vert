#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec2 ver_attribute_texcoord;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

out vec2 tex_coord;
void main()
{
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position =  modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	tex_coord = ver_attribute_texcoord;
}