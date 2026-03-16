#ifndef VIEWSHEDSHADERS_H
#define VIEWSHEDSHADERS_H


const char* depthMapVertVP = R"(
    #version 330 core

    //in vec4 gl_Vertex;
    out float lightDistance;

    uniform mat4 osg_ModelViewProjectionMatrix;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 osg_ModelViewMatrix;
    uniform vec3 lightPos;
    uniform mat4 inverse_view;

    uniform float near_plane;
    uniform float far_plane;

    void depthMapVertex(inout vec4 vertex)
    {
        //IN Model space
        // vec3 worldPos = (inverse_view * osg_ModelViewMatrix * vertex).xyz;

        //in view space
        vec4 w = inverse_view * vertex;
        vec3 worldPos = w.xyz / w.w;

        lightDistance = length(worldPos - lightPos);
        lightDistance = ((1 / lightDistance) - (1 / near_plane)) / (1 / far_plane - 1 / near_plane);

        // gl_Position = osg_ModelViewProjectionMatrix * vertex;
    }
)";

const char* depthMapFragVP = R"(
    #version 330 core
    in float lightDistance;
    uniform float far_plane;

    void depthMapFragment(inout vec4 color)
    {
        // Mapping to [0, 1]
        // float depth = clamp(lightDistance, 0.0, 1.0);

        // color = vec4(vec3(depth), 1.0);
        gl_FragDepth = lightDistance;
    }
)";

const char* visibilityShaderVertVP = R"(
    #version 330 core

in vec3 osg_Normal;

    uniform mat4 osg_ModelViewProjectionMatrix;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 osg_ModelViewMatrix;

    uniform mat4 inverse_view;


    uniform vec3 lightPos;

    out vec3 worldPos;
    out vec3 normal;
    out float lightDistance;

    void visibilityVertex(inout vec4 vertex)
    {
        //in Model space
        // worldPos = (osg_ViewMatrixInverse * osg_ModelViewMatrix * vertex).xyz;

        //IN view space
        vec4 w = osg_ViewMatrixInverse * vertex;
        worldPos = w.xyz / w.w;

        lightDistance = length(worldPos - lightPos);

        normal = normalize( osg_Normal );
        // gl_Position = osg_ModelViewProjectionMatrix * vertex;
    }
)";


const char* visibilityShaderFragVP = R"(
    #version 330 core

    in vec3 worldPos;
    in vec3 normal;
    in float lightDistance;

    uniform vec3 lightPos;
    uniform vec4 visibleColor;
    uniform vec4 invisibleColor;

    uniform samplerCube shadowMap;

    uniform float near_plane;
    uniform float far_plane;
    uniform float user_area;

    float linearizeDepth(float z)
    {
        float z_n = 2.0 * z - 1.0;
        return 2.0 * near_plane * far_plane / (far_plane + near_plane - z_n * (far_plane - near_plane));
    };

    // Return 1 for shadowed, 0 visible
    bool isShadowed(vec3 lightDir)
    {
        float bias = max(0.01 * (1.0 - dot(normal, lightDir)), 0.001) * far_plane;

        float z = linearizeDepth(texture(shadowMap, lightDir).r);
        return lightDistance - bias > z;
    }

    void visibilityFragment(inout vec4 FragColor)
    {
        if (length(lightPos - worldPos) < user_area)
        {

            vec3 lightDir = normalize(lightPos - worldPos);
            float normDif = max(dot(lightDir, normal), 0.0);

            // if (normDif > 0.0 && isShadowed(-lightDir) == false)
            if (isShadowed(-lightDir) == false)
            {
                // Blends the current FragColor with visibleColor based on its alpha
                FragColor.rgb = mix(FragColor.rgb, visibleColor.rgb, visibleColor.a);
            }
            else
            {
                // Blends the current FragColor with invisibleColor based on its alpha
                FragColor.rgb = mix(FragColor.rgb, invisibleColor.rgb, invisibleColor.a);
            }
        }
    }
)";


//////////////////////////////////////////////////////////////////////////////

const char* depthMapVert = R"(
    #version 330 core

    //in vec4 gl_Vertex;
    out float lightDistance;

    uniform mat4 osg_ModelViewProjectionMatrix;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 osg_ModelViewMatrix;
    uniform vec3 lightPos;
    uniform mat4 inverse_view;

    uniform float near_plane;
    uniform float far_plane;

    void main()
    {
        // get distance between fragment and light source
        vec3 worldPos = (inverse_view * osg_ModelViewMatrix * osg_Vertex).xyz;
        lightDistance = length(worldPos - lightPos);
        lightDistance = ((1 / lightDistance) - (1 / near_plane)) / (1 / far_plane - 1 / near_plane);

        gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
    }
)";

const char* depthMapFrag = R"(
    #version 330 core
    in float lightDistance;
    uniform float far_plane;

    void main()
    {
        // Mapping to [0, 1]
        gl_FragDepth = lightDistance;
    }
)";

const char* visibilityShaderVert = R"(
    #version 330 core

    //in vec4 gl_Vertex;
    //in vec3 gl_Normal;
    //in vec2 gl_MultiTexCoord0;

    uniform mat4 osg_ModelViewProjectionMatrix;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 osg_ModelViewMatrix;

    uniform vec3 lightPos;

    out vec3 worldPos;
    out vec3 normal;
    // out vec2 texCoords;
    out float lightDistance;

    void main()
    {
        //for Model space
        worldPos = (osg_ViewMatrixInverse * osg_ModelViewMatrix * osg_Vertex).xyz;


        //for ViewSpace
        // vec4 w = osg_ViewMatrixInverse * VertexVIEW;
        // worldPos = w.xyz / w.w;

        lightDistance = length(worldPos - lightPos);

        normal = normalize( osg_Normal );
        // texCoords = osg_MultiTexCoord0.xy;
        gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
    }
)";


const char* visibilityShaderFrag = R"(
    #version 330 core
    out vec4 FragColor;

    in vec3 worldPos;
    in vec3 normal;
    in vec2 texCoords;
    in float lightDistance;

    uniform vec3 lightPos;
    uniform vec4 visibleColor;
    uniform vec4 invisibleColor;

    // uniform sampler2D baseTexture;
    uniform samplerCube shadowMap;

    uniform float near_plane;
    uniform float far_plane;
    uniform float user_area;

    float linearizeDepth(float z)
    {
        float z_n = 2.0 * z - 1.0;
        return 2.0 * near_plane * far_plane / (far_plane + near_plane - z_n * (far_plane - near_plane));
    };

    // Return 1 for shadowed, 0 visible
    bool isShadowed(vec3 lightDir)
    {
        float bias = max(0.01 * (1.0 - dot(normal, lightDir)), 0.001) * far_plane;

        float z = linearizeDepth(texture(shadowMap, lightDir).r);
        return lightDistance - bias > z;
    }

    void main()
    {
        if (length(lightPos - worldPos) < user_area)
        {

            vec3 lightDir = normalize(lightPos - worldPos);
            float normDif = max(dot(lightDir, normal), 0.0);

            if (normDif > 0.0 && isShadowed(-lightDir) == false)
            {
                FragColor.rgb *= visibleColor.rgb;
            }
            else
            {
                FragColor.rgb *= invisibleColor.rgb;
            }
        }
    }
)";



const char* vert = R"(
// vertex shader
#version 330
uniform mat4 osg_ViewMatrixInverse;
out vec3 vWorld;

void visibilityVertex(inout vec4 VertexVIEW) {
    // Transforming back to world space
    vec4 w = osg_ViewMatrixInverse * VertexVIEW;
    vWorld = w.xyz / w.w;
}
)";

const char* frag = R"(
// fragment shader
#version 330
in vec3 vWorld;
uniform vec3 lightPos;
uniform float user_area;

void visibilityFragment(inout vec4 color) {
    float d = length(vWorld - lightPos);
    if(d < user_area) {
        color = vec4(1.0, 0.0, 0.0, 1.0); // Bright Red
    }
}
)";


#endif // VIEWSHEDSHADERS_H
