#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in float ver_attribute_flag;

out float feedback_attribute;

void main( void )
{		
	feedback_attribute = ver_attribute_flag;
	if(feedback_attribute == 0.0)
		feedback_attribute = 2.0;
	else
		feedback_attribute += 1.0;
}