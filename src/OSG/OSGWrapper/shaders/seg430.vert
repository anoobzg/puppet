#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;
layout(location = 2) in float ver_attribute;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform uint branch;

out vec3 view_direction;
out vec3 normal;
out vec4 color;

vec4 visual_curvature(float value)
{
	if(value < -1.0)
		return vec4(1.0, 0.0, 0.0, 1.0);
	else if(value > 1.0)
		return vec4(0.0, 0.0, 1.0, 1.0);
	return vec4(0.0, 1.0, 0.0f, 1.0);
}

vec4 visual_distance(float value)
{
	return vec4(0.0, abs(value)/15.0, 0.0f, 1.0);
}

vec4 visual_color(float value)
{
	if(branch == 0)
		return vec4(1.0, 1.0, 1.0, 1.0);
	else if(branch == 1)
		return visual_curvature(value);
	else if(branch == 2)
		return visual_distance(value);
	else
		return vec4(1.0, 1.0, 1.0, 1.0);
}

void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
    normal          = mat3(modelview_matrix) * ver_attribute_normal;
	
	color = visual_color(ver_attribute);
}