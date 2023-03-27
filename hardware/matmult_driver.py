import numpy as np

HW_SIZE = 8

def matmult_driver(A, B):
  (a_xSize, a_ySize) = np.shape(A)
  
  (b_xSize, b_ySize) = np.shape(B)

  a = np.concatenate((A, np.zeros((a_ySize, HW_SIZE-a_xSize))), axis=1)
  a = np.concatenate((a, np.concatenate((np.zeros((HW_SIZE-a_xSize, a_ySize)), 
                                            np.eye(HW_SIZE-a_ySize , HW_SIZE-a_xSize)), axis=1)), axis=0)
  b = np.concatenate((B, np.zeros((b_ySize, HW_SIZE-b_xSize))), axis=1)
  b = np.concatenate((b, np.concatenate((np.zeros((HW_SIZE-b_xSize, b_ySize)), 
                                            np.eye(HW_SIZE-b_ySize , HW_SIZE-b_xSize)), axis=1)), axis=0)

  AB = spoof_mult(a, b)

  print(AB)

  return AB[:a_xSize, :a_ySize]


def spoof_mult(A, B):
    return A @ B





if __name__ == "__main__":
    A = np.array([[1, 2], [3, 4]])
    B = np.array([[5, 6], [7, 8]])

    print(matmult_driver(A, B))