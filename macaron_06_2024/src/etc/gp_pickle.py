#!/usr/bin/env python
#-*-coding:utf-8-*-

import pickle
import os, sys
sys.path.append((os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))) + "/src/path_planning")
from global_path import GlobalPath


# name = 'kcity_bonseon_2023'
name = "manhae_06.30_obs"
GLOBAL_PATH_NAME = name + ".npy"
PATH_ROOT = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
gp_name = PATH_ROOT + GLOBAL_PATH_NAME

GB = GlobalPath(gp_name)

pickle.dump(GB, open((os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/pkl_file/" + name + '.pkl', 'wb'))
gp = pickle.load(open((os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/pkl_file/" + name + '.pkl', 'rb'))
print(gp)


# 폴더 내부에 있는 파일 모두 변환
# path = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
# file_lst = os.listdir(path)

# for file in file_lst:
#     if file.count(".") == 1:
#         name = file.split('.')[0]
        
#         GLOBAL_PATH_NAME = name + ".npy"
#         PATH_ROOT = (os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
#         gp_name = PATH_ROOT + GLOBAL_PATH_NAME

#         try:
#             GB = GlobalPath(gp_name)

#             pickle.dump(GB, open((os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/pkl_file/" + name + '.pkl', 'wb'))
#             gp = pickle.load(open((os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/pkl_file/" + name + '.pkl', 'rb'))
#             print(gp)
#         except:
#             pass
