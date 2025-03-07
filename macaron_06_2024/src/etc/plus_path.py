import numpy as np

path = ["first_kcity_y_0728.npy", "first_kcity_y_0728_1.npy", "first_kcity_y_0728_2.npy"]

p = np.zeros((1,2))

for i in path:
    k = np.load("/home/macaron/catkin_ws/src/macaron_5/path/npy_file/path/"+i)
    p = np.concatenate((p,k))
    print(p)
    
p = np.delete(p,(0),axis=0)

np.save("/home/macaron/catkin_ws/src/macaron_5/path/npy_file/path/test",p)