#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in float ver_attribute_energy_value;
layout(location = 2) in vec3 ver_attribute_normal;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

out vec3 view_direction;
out vec3 normal;
out vec4 color;

vec4 curvature_to_color(float energy)
{
	vec4 n_color = vec4(0.0, 0.0, 1.0, 1.0);
	vec4 m_color = vec4(0.0, 1.0, 0.0, 1.0);
	vec4 p_color = vec4(1.0, 0.0, 0.0, 1.0);
	
	if (energy > 1.0)
	{
		return n_color;
	}
	
	if (energy > 0.5)
	{
		energy = 2 * (energy - 0.5);
		return (1 - energy) * m_color + energy * n_color;
	}
	
	if (energy > 0.01)
	{
		energy = 2 * energy;
		return energy * m_color + (1 - energy) * p_color;
	}
	
	return p_color;
}

void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
	normal          = mat3(modelview_matrix) * ver_attribute_normal;
	
	color = curvature_to_color(ver_attribute_energy_value);
	//color = vec4(1.0, 1.0, 1.0, 1.0);
}