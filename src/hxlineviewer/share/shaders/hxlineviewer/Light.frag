
/**
    Lights types

    Only for the first 4 openGL lights.
*/
uniform vec4 lights;

/*
 Fixed Function Equivalent Shader
*/
vec4 Ambient;
vec4 Diffuse;
vec4 Specular;

void
pointLight(in int light, in vec3 normal, in vec3 ecPosition3)
{
    // Compute vector from surface to light position
    vec3 VP = vec3(gl_LightSource[light].position) - ecPosition3;

    // Compute distance between surface and light position
    float d = length(VP);

    // Normalize the vector from surface to light position
    VP = normalize(VP);

    // Compute attenuation
    float attenuation = 1.0 / (gl_LightSource[light].constantAttenuation +
                               gl_LightSource[light].linearAttenuation * d +
                               gl_LightSource[light].quadraticAttenuation * d * d);

    vec3 halfVector = normalize(VP);

    float nDotVP = max(0.0, dot(normal, VP));
    float nDotHV = max(0.0, dot(normal, halfVector));

    float pf;          // power factor
    if (nDotVP == 0.0)
    {
        pf = 0.0;
    }
    else
    {
        pf = pow(nDotHV, gl_FrontMaterial.shininess);
    }
    Ambient += gl_LightSource[light].ambient * attenuation;
    Diffuse += gl_LightSource[light].diffuse * nDotVP * attenuation;
    Specular += gl_LightSource[light].specular * pf * attenuation;
}

void
spotLight(in int light, in vec3 normal, in vec3 ecPosition3)
{
    // Compute vector from surface to light position
    vec3 VP = vec3(gl_LightSource[light].position) - ecPosition3;

    // Compute distance between surface and light position
    float d = length(VP);

    // Normalize the vector from surface to light position
    VP = normalize(VP);

    // Compute attenuation
    float attenuation = 1.0 / (gl_LightSource[light].constantAttenuation +
                               gl_LightSource[light].linearAttenuation * d +
                               gl_LightSource[light].quadraticAttenuation * d * d);

    // See if point on surface is inside cone of illumination
    float spotDot = dot(-VP, normalize(gl_LightSource[light].spotDirection));

    float spotAttenuation; // spotlight attenuation factor
    if (spotDot < gl_LightSource[light].spotCosCutoff)
    {
        spotAttenuation = 0.0; // light adds no contribution
    }
    else
    {
        spotAttenuation = pow(spotDot, gl_LightSource[light].spotExponent);
    }
    // Combine the spotlight and distance attenuation.
    attenuation *= spotAttenuation;

    vec3 halfVector = normalize(VP);

    float nDotVP = max(0.0, dot(normal, VP));
    float nDotHV = max(0.0, dot(normal, halfVector));

    float pf;              // power factor
    if (nDotVP == 0.0)
    {
        pf = 0.0;
    }
    else
    {
        pf = pow(nDotHV, gl_FrontMaterial.shininess);
    }
    Ambient += gl_LightSource[light].ambient * attenuation;
    Diffuse += gl_LightSource[light].diffuse * nDotVP * attenuation;
    Specular += gl_LightSource[light].specular * pf * attenuation;
}

void
directionalLight(in int light, in vec3 normal)
{
    float nDotVP = max(0.0, dot(normal, normalize(vec3(gl_LightSource[light].position))));
    float nDotHV = max(0.0, dot(normal, vec3(gl_LightSource[light].halfVector)));

    float pf;     // power factor
    if (nDotVP == 0.0)
    {
        pf = 0.0;
    }
    else
    {
        pf = pow(nDotHV, gl_FrontMaterial.shininess);
    }
    Ambient += gl_LightSource[light].ambient;
    Diffuse += gl_LightSource[light].diffuse * nDotVP;
    Specular += gl_LightSource[light].specular * pf;
}

/**
    Computes the shaded color depending on a material color and
    the material properties, for the first for lights. 
*/
vec4
illuminate(vec4 color, vec3 position, vec3 normal)
{
    // Clear the light intensity accumulators
    Ambient = vec4(0.0);
    Diffuse = vec4(0.0);
    Specular = vec4(0.0);

    for (int i = 0; i < 4; ++i)
    {
        if (int(lights[i]) == 1)
            pointLight(i, normal, position);
        else if (int(lights[i]) == 2)
            spotLight(i, normal, position);
        else if (int(lights[i]) == 3)
            directionalLight(i, normal);
    }

    color = gl_FrontLightModelProduct.sceneColor +
            /*Ambient **/ gl_FrontMaterial.ambient * color + 
            Diffuse * gl_FrontMaterial.diffuse * color;
    color += Specular * gl_FrontMaterial.specular;

    return clamp(color, 0.0, 1.0);
}
