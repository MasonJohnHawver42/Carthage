#version 430 core

out vec4 FragColor;

in vec3 Normal;
in vec3 ourColor;
in vec2 TexCord;
flat in float TexSlider;

uniform sampler2D ourTexture;
uniform float near;
uniform float far;
uniform float DepthSlider;

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
    {
        discard;
    }
    vec3 color = (albedo.xyz * TexSlider) + ((1 - TexSlider) * ourColor);
    vec3 normal = normalize(Normal);
    float diff = Normal.z * Normal.z * (1 + sign( Normal.z));
    vec3 diffuse = 0.2 * diff * color;
    vec3 ambient = 0.5 * color;
    FragColor = DepthSlider * vec4(vec3(linearize_depth(gl_FragCoord.z) * .2), 1.0f) + (1 - DepthSlider) * vec4(diffuse + ambient, 1.0);
    // FragColor = vec4(diffuse + ambient, 1.0);
}