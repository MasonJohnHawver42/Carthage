#version 430 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoords;

uniform vec2 offset;
uniform vec2 scale;

void main()
{
    gl_Position = vec4((aPos.x * scale.x) + offset.x, (aPos.y * scale.y) + offset.y, 0.0, 1.0); 
}  

