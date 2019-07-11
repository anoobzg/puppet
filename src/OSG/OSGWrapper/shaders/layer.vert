#version 430 core
layout(location = 0) in vec2 ver_attribute_position;

void main()
{
    gl_Position = vec4(norm_screen_pos, 0.0, 1.0);
}