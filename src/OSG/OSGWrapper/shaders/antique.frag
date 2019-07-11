#version 430 core

out vec4 fragment_color;
void main()
{
	fragment_color = vec4(125.0/255.0, 221.0/255.0, 169.0/255.0, 0.1);
}

//uniform float s = 0.5;
//uniform float h = 190.0f;
//uniform float v = 0.9;
//
//void main()
//{
//	float f = h / 60.0 - 3.0;
//	float p = v * (1.0 - s);
//	float q = v * (1.0 - f * s);
//	fragment_color = vec4(p, q, v, 1.0);
//}