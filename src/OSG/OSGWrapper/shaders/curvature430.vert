#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;
layout(location = 2) in vec4 ver_attribute_color;
layout(location = 3) in float ver_attribute_curvature;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

out vec3 view_direction;
out vec3 normal;
out vec4 color;

vec4 curvature_to_color(float curvature)
{
	vec4 n_color = vec4(0.0, 0.0, 1.0, 1.0);
	vec4 p_color = vec4(1.0, 0.0, 0.0, 1.0);
	
	float max_c = 0.5;
	if(curvature <= -max_c)
		return n_color;
	else if(curvature >= max_c)
		return p_color;
	else
	{
		vec4 c1 = vec4(0.0, 1.0, 0.0, 1.0);
		vec4 c2 = n_color;
		if(curvature >= 0.0) c2 = p_color;
		
		float delta = abs(curvature) / max_c;
		return c1 * ( 1.0 - delta) + c2 * delta;
	}
}

void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
    normal          = mat3(modelview_matrix) * ver_attribute_normal;
	
	color =  curvature_to_color(ver_attribute_curvature);
}