#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in float ver_attribute_flag;
layout(r32f, binding = 0) uniform image2D mask;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform mat4 viewport_matrix;

out float feedback_attribute;

void main( void )
{		
	float value = ver_attribute_flag;
	
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
	
	vec4 screen_position = viewport_matrix * projection_matrix *  world_position;
	vec2 xy = vec2(screen_position.x / screen_position.w, screen_position.y / screen_position.w);
	
	vec4 mask_color = imageLoad(mask, ivec2(xy));
	
	if(value == 0.0 && mask_color.r == 1.0)
		value = 1.0;
	feedback_attribute = value;
}