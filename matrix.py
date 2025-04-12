
#
# LIBRARY FOR MATRIX MATHS
#

class Matrix:
    #get a matrix full of zeros of defined size
    def get_zeros_matrix(a,b):
        return [[0 for i in range(a)] for j in range(b)]


    #add 2 matrices of the same size
    def add_matrix(matrixA, matrixB):
        result = matrixA
        for i in range(len(matrixA)):
            for j in range(len(matrixA[i])):
                result[i][j] = matrixA[i][j] + matrixB[i][j]
        return result


    #subtract 2 matrices of the same size
    def subtract(matrixA, matrixB):
        result = matrixA
        for i in range(len(matrixA)):
            for j in range(len(matrixA[i])):
                result[i][j] = matrixA[i][j] - matrixB[i][j]
        return result


    #get the transpose of a matrix
    def transpose_matrix(matrixA):
        return [list(row) for row in zip(*matrixA)]


    #multiply 2 matrix that can be multiplied
    def multiply_matrices(matrixA, matrixB):
        return [[sum(a*b for a,b in zip(row,col)) for col in zip(*matrixB)] for row in matrixA]

    #get hadamard product of 2 matrices of the same size
    def hadamard_product(matrixA, matrixB):
        result = matrixA
        for i in range(len(matrixA)):
            for j in range(len(matrixA[i])):
                result[i][j] = matrixA[i][j] * matrixB[i][j]
        return result