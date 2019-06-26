#version 430 core

layout(location = 0) in float ver_attribute_position;
layout(location = 1) in float ver_attribute_feedback;

out float feedback_attribute;

void main( void )
{		
	feedback_attribute = clamp(ver_attribute_feedback + 0.01, 0.0, 1.0);
}
