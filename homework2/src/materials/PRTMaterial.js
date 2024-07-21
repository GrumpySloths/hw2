class PRTMaterial extends Material {

    constructor(vertexShader, fragmentShader) {
        console.log("precomputeL in PRT material:", precomputeL);
        console.log("guiParams:", guiParams);
        //将维度为3*9的precomputeL拆解为urL,ugL,ubL,
        let rL = mat3.create();
        let gL = mat3.create();
        let bL = mat3.create();

        for (let i = 0; i < 9; i++){
            rL[i] = precomputeL[0][i][0];
            gL[i] = precomputeL[0][i][1];
            bL[i] = precomputeL[0][i][2];
        }
      
        console.log("rL:", rL);
        console.log("gL:", gL);
        console.log("bL:", bL);
        super({
            'urL': { type: 'updatedInRealTime', value: rL },
            'ugL': { type: 'updatedInRealTime', value: gL },
            'ubL': { type: 'updatedInRealTime', value: bL },
        }, ['aPrecomputeLT','aIndirectLight'], vertexShader, fragmentShader, null);
        // super({
        // }, ['aPrecomputeLT'], vertexShader, fragmentShader, null);
    }
}

async function buildPRTMaterial(vertexPath, fragmentPath) {
    

    let vertexShader = await getShaderString(vertexPath);
    let fragmentShader = await getShaderString(fragmentPath);

    return new PRTMaterial(vertexShader, fragmentShader);

}