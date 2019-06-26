#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in float ver_attribute_flag;

out float feedback_attribute;

void main( void )
{		
	float value = ver_attribute_flag;
	if(value == 1.0)
		value = 0.0;
	else if(value == 0.0)
		value = 1.0;
		
	feedback_attribute = value;
}