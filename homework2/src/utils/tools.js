/**
 * Calculates the precomputed L matrix for rotation.
 * @param {Array} precompute_L - dim: SH_Coff*3.
 * @param {mat4} rotationMatrix - The rotation matrix .
 */
function getRotationPrecomputeL(precompute_L, rotationMatrix) {
	console.log("===================================");
	//创建一个数组临时存储最开始的precompute_L
	let precompute_L_copy = [];
	for (let i = 0; i < precompute_L.length; i++) {
		precompute_L_copy[i] = precompute_L[i].slice();
	}

	// 第1阶快速球谐旋转矩阵和对应旋转后的球谐系数计算
	let M1 = computeSquareMatrix_3by3(rotationMatrix)
	for (let i = 0; i < 3; i++){
		// 让M1 作用在precomuteL[1:4][i]上
		let tmp = [precompute_L_copy[1][i], precompute_L_copy[2][i], precompute_L_copy[3][i]];
		console.log("tmp:", tmp);
		console.log("M1:", M1);
		let result = math.multiply(M1, tmp);
		precompute_L_copy[1][i] = result.get([0]);
		precompute_L_copy[2][i] = result.get([1]);
		precompute_L_copy[3][i] = result.get([2]);
	}
	//第2阶快速球谐旋转矩阵和对应旋转后的球谐系数计算
	let M2 = computeSquareMatrix_5by5(rotationMatrix)
	for (let i = 0; i < 3; i++){
		// 让M2 作用在precomuteL[4:8][i]上
		let tmp = [precompute_L_copy[4][i], precompute_L_copy[5][i], precompute_L_copy[6][i], precompute_L_copy[7][i], precompute_L_copy[8][i]];
		let result = math.multiply(M2, tmp);
		precompute_L_copy[4][i] = result.get([0]);
		precompute_L_copy[5][i] = result.get([1]);
		precompute_L_copy[6][i] = result.get([2]);
		precompute_L_copy[7][i] = result.get([3]);
		precompute_L_copy[8][i] = result.get([4]);
	}

	let result = getMat3ValueFromRGB(precompute_L_copy);

	console.log("original precomputeL:", precompute_L);
	console.log("final transform of precomputeL:", precompute_L_copy);
	//计算precompute_L和precompute_L_copy的差值
	let diff = math.subtract(precompute_L, precompute_L_copy);
	console.log("diff:", diff);
	console.log("===================================");
	return result;
}

/**
 * Computes the square matrix SA(-1) of size 3x3.
 * @param {mat4} rotationMatrix - The rotation matrix.
 */
function computeSquareMatrix_3by3(rotationMatrix) {
	// 1、pick ni - {ni}
	let n1 = [1, 0, 0, 0]; let n2 = [0, 0, 1, 0]; let n3 = [0, 1, 0, 0];

	// 2、{P(ni)} - A  A_inverse
	let P_n1, P_n2, P_n3;
	// let A = mat3.create(), A_inverse = mat3.create();
	let A=math.identity(3), A_inverse=math.identity(3);
	

	P_n1 = SHEval(n1[0], n1[1], n1[2], 3);
	P_n2 = SHEval(n2[0], n2[1], n2[2], 3);
	P_n3 = SHEval(n3[0], n3[1], n3[2], 3);
	let P_ns = [P_n1, P_n2, P_n3];
	//取出ni投影系数的第1阶系数传递给A并求逆
	for (let i = 0; i < 3; i++){
		for (let j = 0; j < 3; j++){
			A.set([i, j], P_ns[j][i+1]);
			// A.set([i, j], P_ns[i][j+1]);
		}
	}
	// mat3.invert(A_inverse, A);
	A_inverse = math.inv(A);
	console.log(P_ns);
	console.log("computeSquareMatrix_3by3 A:", A);
	console.log("computeSquareMatrix_3by3 A_inverse:", A_inverse);

	// 3、用 R 旋转 ni - {R(ni)}
	let R_n1, R_n2, R_n3;
	//先利用循环将mat4的rotationMatrix转换为math的matrix
	rotationMatrix = mat4Matrix2mathMatrix(rotationMatrix);
	R_n1 = math.multiply(rotationMatrix, n1);
	R_n2 = math.multiply(rotationMatrix, n2);
	R_n3 = math.multiply(rotationMatrix, n3);

	console.log("computeSquareMatrix_3by3 R_n1:", R_n1);
	console.log("computeSquareMatrix_3by3 R_n2:", R_n2);
	console.log("computeSquareMatrix_3by3 R_n3:", R_n3);
	// 4、R(ni) SH投影 - S
	let P_rn1, P_rn2, P_rn3;
	P_rn1 = SHEval(R_n1.get([0]), R_n1.get([1]), R_n1.get([2]), 3);
	P_rn2 = SHEval(R_n2.get([0]), R_n2.get([1]), R_n2.get([2]), 3);
	P_rn3 = SHEval(R_n3.get([0]), R_n3.get([1]), R_n3.get([2]), 3);
	let P_rns = [P_rn1, P_rn2, P_rn3];
	let S = math.identity(3);
	for (let i = 0; i < 3; i++){
		for (let j = 0; j < 3; j++){
			S.set([i, j], P_rns[j][i+1]);
			// S.set([i, j], P_rns[i][j+1]);
		}
	}
	console.log("computeSquareMatrix_3by3 S:", S);
	console.log("computeSquareMatrix_3by3 P_rns:", P_rns);
	// 5、S*A_inverse
	let SA_inverse = math.multiply(S, A_inverse);

	console.log("computeSquareMatrix_3by3 SA_inverse:", SA_inverse);

	return SA_inverse;
}

/**
 * Computes the square matrix SA(-1) of size 5x5.
 * @param {Array} rotationMatrix - The rotation matrix.
 */
function computeSquareMatrix_5by5(rotationMatrix) {
	// 1、pick ni - {ni}
	let k = 1 / math.sqrt(2);
	let n1 = [1, 0, 0, 0]; let n2 = [0, 0, 1, 0]; let n3 = [k, k, 0, 0]; 
	let n4 = [k, 0, k, 0]; let n5 = [0, k, k, 0];

	// 2、{P(ni)} - A  A_inverse
	let P_n1, P_n2, P_n3, P_n4, P_n5,P_ns;
	let A = math.identity(5), A_inverse = math.identity(5);

	P_n1 = SHEval(n1[0], n1[1], n1[2], 3);
	P_n2 = SHEval(n2[0], n2[1], n2[2], 3);
	P_n3 = SHEval(n3[0], n3[1], n3[2], 3);
	P_n4 = SHEval(n4[0], n4[1], n4[2], 3);
	P_n5 = SHEval(n5[0], n5[1], n5[2], 3);
	P_ns = [P_n1, P_n2, P_n3, P_n4, P_n5];
	//取出ni投影系数的第2阶系数[4-8]传递给A并求逆
	for (let i = 0; i < 5; i++){
		for (let j = 0; j < 5; j++){
			A.set([i, j], P_ns[j][i+4]);
			// A.set([i, j], P_ns[i][j+4]);
		}
	}
	A_inverse=math.inv(A);
	
	console.log("computeSquareMatrix_5by5 A:", A);
	console.log("computeSquareMatrix_5by5 A_inverse:", A_inverse);
	console.log("computeSquareMatrix_5by5 P_ns:", P_ns);
	// 3、用 R 旋转 ni - {R(ni)}
	let R_n1, R_n2, R_n3, R_n4, R_n5;
	//先利用循环将mat4的rotationMatrix转换为math的matrix
	rotationMatrix = mat4Matrix2mathMatrix(rotationMatrix);
	R_n1 = math.multiply(rotationMatrix, n1);
	R_n2 = math.multiply(rotationMatrix, n2);
	R_n3 = math.multiply(rotationMatrix, n3);
	R_n4 = math.multiply(rotationMatrix, n4);
	R_n5 = math.multiply(rotationMatrix, n5);

	console.log("computeSquareMatrix_5by5 R_n1:", R_n1);
	console.log("computeSquareMatrix_5by5 R_n2:", R_n2);
	console.log("computeSquareMatrix_5by5 R_n3:", R_n3);
	console.log("computeSquareMatrix_5by5 R_n4:", R_n4);
	console.log("computeSquareMatrix_5by5 R_n5:", R_n5);

	// 4、R(ni) SH投影 - S
	let P_rn1, P_rn2, P_rn3, P_rn4, P_rn5,P_rns;
	P_rn1 = SHEval(R_n1.get([0]), R_n1.get([1]), R_n1.get([2]), 3);
	P_rn2 = SHEval(R_n2.get([0]), R_n2.get([1]), R_n2.get([2]), 3);
	P_rn3 = SHEval(R_n3.get([0]), R_n3.get([1]), R_n3.get([2]), 3);
	P_rn4 = SHEval(R_n4.get([0]), R_n4.get([1]), R_n4.get([2]), 3);
	P_rn5 = SHEval(R_n5.get([0]), R_n5.get([1]), R_n5.get([2]), 3);
	P_rns = [P_rn1, P_rn2, P_rn3, P_rn4, P_rn5];
	let S = math.identity(5);
	for (let i = 0; i < 5; i++){
		for (let j = 0; j < 5; j++){
			S.set([i, j], P_rns[j][i+4]);
			// S.set([i, j], P_rns[i][j+4]);
		}
	}

	console.log("computeSquareMatrix_5by5 S:", S);
	console.log("computeSquareMatrix_5by5 P_rns:", P_rns);
	// 5、S*A_inverse
	let SA_inverse = math.multiply(S, A_inverse);

	console.log("computeSquareMatrix_5by5 SA_inverse:", SA_inverse);

	return SA_inverse;
}

/**
 * Converts a mat4 rotation matrix to a math matrix.
 * @param {Array} rotationMatrix - The rotation matrix.
 * @returns {Object} - The math matrix.
 */
function mat4Matrix2mathMatrix(rotationMatrix) {
	let mathMatrix = [];
	for(let i = 0; i < 4; i++){
		let r = [];
		for(let j = 0; j < 4; j++){
			r.push(rotationMatrix[i*4+j]);
		}
		mathMatrix.push(r);
	}
	return math.matrix(mathMatrix)
}

/**
 * Gets the mat3 value from the RGB precomputeL matrix.
 * @param {Array} precomputeL - The precomputeL matrix.
 * @returns {Array} - The mat3 color matrix.
 */
function getMat3ValueFromRGB(precomputeL) {
    let colorMat3 = [];
    for(var i = 0; i<3; i++){
        colorMat3[i] = mat3.fromValues( precomputeL[0][i], precomputeL[1][i], precomputeL[2][i],
										precomputeL[3][i], precomputeL[4][i], precomputeL[5][i],
										precomputeL[6][i], precomputeL[7][i], precomputeL[8][i] ); 
	}
    return colorMat3;
}