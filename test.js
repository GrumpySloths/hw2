// 引入 glmatrix 库
const glMatrix = require('gl-matrix');

// 引入 math.js
const math = require('mathjs');

const matrix = math.matrix([[7, 1], [-2, 3]]) // Matrix
const array = [1, 3]
const result = math.multiply(matrix, array)
console.log("result:", result);
console.log("result[0]",result.get([0]))
const matrix_inv = math.inv(matrix);
console.log("matrix_inv:", matrix_inv);
console.log("matrix*matrix_inv:", math.multiply(matrix, matrix_inv));