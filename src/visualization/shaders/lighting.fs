#version 330

in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

out vec4 finalColor;

#define     MAX_LIGHTS              4
#define     LIGHT_DIRECTIONAL       0
#define     LIGHT_POINT             1

struct Light {
    int enabled;
    int type;
    vec3 position;
    vec3 target;
    vec4 color;
};

uniform Light lights[MAX_LIGHTS];
uniform vec4 ambient;
uniform vec3 viewPos;

uniform vec4 fogColor;
uniform float fogDensity;

void main()
{
    vec4 texelColor = texture(texture0, fragTexCoord);
    vec3 lightDot = vec3(0.0);
    vec3 normal = normalize(fragNormal);

    vec4 tint = colDiffuse * fragColor;

    for (int i = 0; i < MAX_LIGHTS; i++)
    {
        if (lights[i].enabled == 1)
        {
            vec3 light = vec3(0.0);

            if (lights[i].type == LIGHT_DIRECTIONAL)
            {
                light = -normalize(lights[i].target - lights[i].position);
            }

            if (lights[i].type == LIGHT_POINT)
            {
                light = normalize(lights[i].position - fragPosition);
            }

            float NdotL = max(dot(normal, light), 0.0);
            float halfLambert = NdotL * 0.5 + 0.5;
            lightDot += lights[i].color.rgb * halfLambert * 0.7;
        }
    }

    finalColor = texelColor * tint * vec4(lightDot, 1.0);
    finalColor += texelColor * (ambient / 10.0) * tint * 0.5;

    // Fog
    if (fogDensity > 0.0)
    {
        float dist = length(viewPos - fragPosition);
        float fogFactor = 1.0 - exp(-fogDensity * dist);
        fogFactor = clamp(fogFactor, 0.0, 1.0);
        finalColor = mix(finalColor, fogColor, fogFactor);
    }

    // Saturation boost
    float luminance = dot(finalColor.rgb, vec3(0.299, 0.587, 0.114));
    finalColor.rgb = mix(vec3(luminance), finalColor.rgb, 1.4);

    finalColor = pow(finalColor, vec4(1.0/2.2));
}
