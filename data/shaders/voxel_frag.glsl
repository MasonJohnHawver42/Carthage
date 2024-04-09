#version 430 core

in vec2 TexCord;
flat in vec4 ao_data;
flat in vec3 pos;

out vec4 FragColor;

// float rand1(float n) {return fract(sin(n) * 43758.5453123);}

float rand(vec2 x){
    return fract(sin(dot(x, vec2(12.9898, 78.233))) * 43758.5453);
}


void main()
{
    float ambient = ((1 - TexCord.x) * (1 - TexCord.y) * ao_data[0]) +
                    ((TexCord.x) * (1 - TexCord.y) * ao_data[1]) + 
                    ((1 - TexCord.x) * (TexCord.y) * ao_data[2]) +
                    ((TexCord.x) * (TexCord.y) * ao_data[3]);
    
    // float ambient = ((1 - TexCord.x) * (1 - TexCord.y) * ao_data[0]) +
    //             ((TexCord.x) * (1 - TexCord.y) * ao_data[1]) + 
    //             ((1 - TexCord.x) * (TexCord.y) * ao_data[2]) +
    //             ((TexCord.x) * (TexCord.y) * ao_data[3]);

    float base = rand(pos.xy);
    base = rand(vec2(base, pos.z)) * .05 + .1;

    FragColor = vec4(vec3((1 - base) * (1 - ambient)), 0.75f);
} 