#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in float ver_attribute_flag;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform mat4 viewport_matrix;

uniform vec2 pick_xy;
uniform float pick_radius = 50.0;

out float feedback_attribute;

void main( void )
{		
	feedback_attribute = ver_attribute_flag;
	
	mat4 modelview_matrix = view_matrix * model_matrix;
	vec4 world_position = modelview_matrix * vec4(ver_attribute_position, 1.0);
	
	vec4 screen_position = viewport_matrix * projection_matrix *  world_position;
	vec2 xy = vec2(screen_position.x / screen_position.w, screen_position.y / screen_position.w);
	vec2 delta = pick_xy - xy;
	
	if(dot(delta, delta) <= pick_radius * pick_radius && feedback_attribute == 0.0)
	//if(pick_xy.x > 500 && pick_xy.y > 500 && feedback_attribute == 0.0)
		feedback_attribute = 1.0;
}