#version 430 core

out vec4 FragColor;

in vec3 ourColor;
in vec2 TexCord;

uniform sampler2D ourTexture;
uniform float near;
uniform float far;

float linearize_depth(float depth) 
{
    float near_x = 0.01f;
    float far_x = 300.0f;
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * near_x * far_x) / (far_x + near_x - z * (far_x - near_x));	
}

void main()
{
    vec4 albedo = texture(ourTexture, TexCord);
    if(albedo.a < 0.1)
        discard;
    FragColor = albedo; // vec4(vec3(linearize_depth(gl_FragCoord.z) * .05), 1.0f);
}