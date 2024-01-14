#version 430 core

out vec4 FragColor;

in vec3 ourColor;
in vec2 TexCord;

uniform sampler2D ourTexture;

void main()
{
    FragColor = texture(ourTexture, TexCord) * vec4(ourColor, 1.0);
}