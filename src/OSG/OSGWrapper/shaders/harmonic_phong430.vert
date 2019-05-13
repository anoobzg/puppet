#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;
layout(location = 2) in float ver_harmonic_value;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

out vec3 view_direction;
out vec3 normal;
out vec4 color;

vec4 harmonic_value(float value)
{
	vec4 r = vec4(1.0, 0.0, 0.0, 1.0);
	vec4 g = vec4(0.0, 1.0, 0.0, 1.0);
	vec4 b = vec4(0.0, 0.0, 1.0, 1.0);
	
	//if(value > 0.5)
	//{
	//	float l = (value - 0.5)/0.5;
	//	return r * l + g * ( 1.0 - l);
	//}else
	//{
	//	float l = (0.5 - value)/0.5;
	//	return b * l + g * (1.0 - l);
	//}
	
	if(value > 0.6)
		return r;
	else if(value < 0.4)
		return b;
	else 
		return g;
}

void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
    normal          = mat3(modelview_matrix) * ver_attribute_normal;
	color 			= harmonic_value(ver_harmonic_value);
}