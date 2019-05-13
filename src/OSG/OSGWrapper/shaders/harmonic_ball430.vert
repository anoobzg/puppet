#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

uniform mat4 scale_matrix;
uniform mat4 position_matrix;
uniform vec4 ball_color;

out vec3 view_direction;
out vec3 normal;
out vec4 color;
void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position =  modelview_matrix * position_matrix * scale_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
    normal          = mat3(modelview_matrix) * ver_attribute_normal;
	color 			= ball_color;
}