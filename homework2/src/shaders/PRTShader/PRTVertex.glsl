attribute mat3 aPrecomputeLT;
attribute vec3 aVertexPosition;
attribute vec3 aNormalPosition;
attribute vec2 aTextureCoord;
attribute vec3 aIndirectLight;

uniform mat4 uModelMatrix;
uniform mat4 uViewMatrix;
uniform mat4 uProjectionMatrix;
uniform mat3 urL;
uniform mat3 ugL;
uniform mat3 ubL;

varying highp vec2 vTextureCoord;
varying highp vec3 vColor;
varying highp vec3 vNormal;

void main(void){

    vNormal = aNormalPosition;
    vTextureCoord = aTextureCoord;
    gl_Position = uProjectionMatrix * uViewMatrix * uModelMatrix  * vec4(aVertexPosition, 1.0);
    //gl_Position = uProjectionMatrix * uViewMatrix * uModelMatrix * vec4(aVertexPosition, 1.0);

    vec3 color = vec3(0.0);
    // vec3 albedo=vec3(0.5,0.5,0.5);
    // vec3 albedo=vec3(0.7,0.7,0.7);
    vec3 albedo=vec3(0.9,0.9,0.9);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            color+=aPrecomputeLT[i][j]*vec3(urL[i][j],ugL[i][j],ubL[i][j]);
        }
    }
    vColor = color*albedo+aIndirectLight*0.3;
    // vColor=color*albedo;
}