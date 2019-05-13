#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;
layout(location = 2) in float ver_attribute_flag;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

out vec3 view_direction;
out vec3 normal;
out vec4 color;

vec4 flag2color(float value)
{
	if(value == 1.0)
		return vec4(0.0, 1.0, 0.0, 1.0);
	else if(value == 2.0)
		return vec4(1.0, 0.0, 0.0, 1.0);
	else if(value == 3.0)
		return vec4(1.0, 1.0, 0.0, 1.0);
	else if(value == 4.0)
		return vec4(1.0, 0.0, 1.0, 1.0);
	else if(value == 5.0)
		return vec4(0.0, 0.0, 1.0, 1.0);
	else if(value == 6.0)
		return vec4(0.0, 1.0, 0.5, 1.0);
	else if(value == 7.0)
		return vec4(1.0, 0.0, 0.4, 1.0);
	else if(value == 8.0)
		return vec4(0.4, 0.4, 0.7, 1.0);
	else if(value == 9.0)
		return vec4(1.0, 0.9, 0.3, 1.0);
	else if(value == 10.0)
		return vec4(0.4, 0.8, 1.0, 1.0);	
	return vec4(1.0, 1.0, 1.0, 1.0);
}

void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
    normal          = mat3(modelview_matrix) * ver_attribute_normal;
	
	color = flag2color(ver_attribute_flag);
}