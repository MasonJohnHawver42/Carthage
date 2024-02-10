#version 430 core
layout (location = 0) in vec3 aPos;   // the position variable has attribute position 0
layout (location = 1) in vec3 aColor; // the color variable has attribute position 1
layout (location = 2) in vec2 aTexCord; // the color variable has attribute position 2
  
out vec3 ourColor; // output a color to the fragment shader
out vec2 TexCord; // output a color to the fragment shader

uniform mat4 MVP;
uniform vec3 Color;

void main()
{
    gl_Position = MVP * vec4(aPos, 1.0);
    ourColor = aColor; // set ourColor to the input color we got from the vertex data
    TexCord = aTexCord;
}     