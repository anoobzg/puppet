#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in vec3 ver_attribute_normal;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform mat4 key_matrix;
uniform float colorid;

out vec3 view_direction;
out vec3 normal;
out vec4 color;

vec4 gencolor(float id)
{
	float v = 4.0f;
	int value = int(id);
	int b = value%5;
	int g = (value/5)%5;
	int r = (value/25)%4;
	float cb = float(b)/v;
	float cg = float(g)/v;
	float cr = float(r)/v;
	return vec4(cr, cg, cb, 1.0);
}

void main( void )
{
	mat4 modelview_matrix = view_matrix * model_matrix * key_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
    gl_Position = projection_matrix *  world_position;
	
	view_direction  = normalize(vec3(-world_position));
    normal          = mat3(modelview_matrix) * ver_attribute_normal;
	if(colorid == 0.0)
		color = vec4(1.0, 1.0, 1.0, 1.0);
	else
		color = gencolor(colorid);
}