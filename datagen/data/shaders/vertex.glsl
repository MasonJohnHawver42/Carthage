#version 430 core
layout (location = 0) in vec3 aPos;   // the position variable has attribute position 0
layout (location = 1) in vec3 aNormal; // the color variable has attribute position 1
layout (location = 2) in vec2 aTexCord; // the color variable has attribute position 2

out vec3 Normal;
out vec3 ourColor; // output a color to the fragment shader
out vec2 TexCord; // output a color to the fragment shader
flat out float TexSlider;

uniform mat4 M;
uniform mat4 VP;
uniform vec3 Color;

uniform float TextureSlider;


void main()
{
    gl_Position = VP * M * vec4(aPos, 1.0);
    vec3 worldNormal = normalize(mat3(transpose(inverse(M))) * aNormal);
    Normal = worldNormal;
    ourColor = Color; // set ourColor to the input color we got from the vertex data
    TexCord = aTexCord;
    TexSlider = TextureSlider;
}     