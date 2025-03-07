import numpy as np

a = np.array([[1,2],[1,2],[1,2],[1,4]])

for i in a:
    dis = np.linalg.norm(i - [3,4])
    print(dis)