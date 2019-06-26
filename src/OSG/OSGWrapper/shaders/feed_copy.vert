#version 430 core

layout(location = 0) in vec3 ver_attribute_position;
layout(location = 1) in float ver_attribute_flag;

out float feedback_attribute;

void main( void )
{		
	feedback_attribute = ver_attribute_flag;
}