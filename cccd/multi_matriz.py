x = [[1,2,3], [4,5,6]]
y = [[1,2], [3,4], [5,6]]

def comprobar_matrices(x,y):
    if len(x[0]) == len(y):
        print "Se pueden multiplicar las matrices"

def multi_matrix(x,y):
    if comprobar_matrices(x,y):
        print "pasamos"
        matrix = []
        aux = -1
        for i in x[i]:
            aux += 1
            for j in x[i][j]:
                matrix[i][aux] += (x[i][j] * y[j][i])

multi_matrix(x,y)
