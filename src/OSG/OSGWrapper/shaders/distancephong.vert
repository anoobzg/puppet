#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform mat4 align_matrix;

out vec3 view_direction;
out vec3 normal;
out vec4 color;

vec4 d2color(float d)
{
	if(d > 100.0f) return vec4(1.0, 0.0, 0.0, 1.0);
	else return vec4(0.0, 1.0, 0.0, 1.0);
}

void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix * align_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
    normal          = mat3(modelview_matrix) * ver_attribute_normal;
	color       	= d2color(ver_attribute_position.z);
}