# # import numpy as np
# # import os
# import math
# # # import matplotlib.pyplot as plt

# # np_load1 = np.load('./track_map1.npy')
# # np_load2 = np.load('./track_map2.npy')
# # np_load3 = np.load('./track_map3.npy')
# # arr = np.empty((1, 2))
# # for i in range(len(np_load3)-1):
# #     # print(np_load3[i])
# #     print(arr)
# #     a = math.sqrt(((np_load3[i][0] - np_load3[i+1][0])**2) + ((np_load3[i][1] - np_load3[i+1][1])**2))
# #     if a > 1:
# #         print('original i : ', i)
# #         print('here', a)
# #         new = np.array([[5,5]])
# #         np.append(arr, [[np_load3[i][0],np_load3[i][1]]], axis=0)
# #         np.append(arr, new, axis=0)
# #         np.append(arr, [[np_load3[i+1][0],np_load3[i+1][1]]], axis=0)
# #         # np_load3[i][0] = 5
# #         # np_load3[i][1] = 5
# #         i += 1
# #         print('added i : ', i)
# #     else:
# #         np.append(arr, [[np_load3[i][0],np_load3[i][1]]], axis=0)
        

# # print('===============')
# # arr = np.delete(arr, [0, 0], axis=0)
# # print(arr)
# # filename = './newfile.npy'
# # np.save(filename, arr)

# # b = math.sqrt(((np_load1[-1][0] - np_load2[0][0])**2) + ((np_load1[-1][1] - np_load2[0][1])**2))
# # # print('two dis : ', b)
# # # print(len(np_load3))
# # # print(np.concatenate((np_load1, np_load2), axis=0))
# # # print(np_load2)
# # # print(np.shape(np.concatenate((np_load1, np_load2), axis=0)))
# # # print(np.shape(np_load3))
# # # # print(np_load1)
# # print("========================")
# # # print(np_load2)

# # # waypoint = np.delete(waypoint, (0), axis=0)
# # # print(np_load3[:, 0])
# # # print(np_load3[:, 1])
# import numpy as np
# import matplotlib.pyplot as plt
# np_load3 = np.load("./track_map2.npy")
# np_load2 = np.load("./track_map2.npy")
# np_load1 = np.load("./track_map1.npy")
# temp = np.load("./temp.npy")
# # plt.plot(np_load3[:, 0], np_load3[:, 1])
# plt.plot(np_load3[:, 0], np_load3[:, 1])
# plt.show()
# print(np_load3)
# a = math.sqrt(((np_load3[-1][0] - np_load3[0][0])**2) + ((np_load3[-1][1] - np_load3[0][1])**2))
# print(a)
# print(np_load3[-1])
# print(np_load2[0])
# # import numpy as np
# # np_load1 = np.load("./track_map1.npy")
# # np_load2 = np.load("./track_map2.npy")
# # np_load3 = np.load("./track_map3.npy")

# # a = np.array([np_load1[np.shape(np_load1)[0] - 1][0], np_load1[np.shape(np_load1)[0] - 1][1]])
# # b = np.array([np_load2[0][0], np_load2[0][1]])
# # print(a/3)
# # print(b)
# # temp = np.array([[(2*a[0]+b[0])/3,(2*a[1]+b[1])/3], [(a[0]+2*b[0])/3,(a[1]+2*b[1])/3]])
# # np_load1 = np.concatenate((np_load1, temp), axis = 0)

# # np_load3 = np.concatenate((np_load1, np_load2), axis = 0)
# # np.save("./track_map3.npy", np_load3)
# # for i in range(len(np_load3)-1):
# #     # print(np_load3[i])
# #     # print(arr)
# #     a = math.sqrt(((np_load3[i][0] - np_load3[i+1][0])**2) + ((np_load3[i][1] - np_load3[i+1][1])**2))
# #     # if a > 1:
# #     #     print('original i : ', i)
# #     #     print('here', a)
# #     #     new = np.array([[5,5]])
# #     #     np.append(arr, [[np_load3[i][0],np_load3[i][1]]], axis=0)
# #     #     np.append(arr, new, axis=0)
# #     #     np.append(arr, [[np_load3[i+1][0],np_load3[i+1][1]]], axis=0)
# #     #     # np_load3[i][0] = 5
# #     #     # np_load3[i][1] = 5
# #     #     i += 1
# #     #     print('added i : ', i)
# #     print(a)

import os

file = os.listdir("./track")

print(sorted(file))