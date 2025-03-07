#!/usr/bin/env python
#-*-coding:utf-8-*-

import sys, os

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/path_planning")

# Module import
from global_path import GlobalPath


# Parameter
# path_name = 'PJ1.npy' # 팔정도
# path_name = 'yaeseon_xy.npy' # 예선
# path_name = 'k_city_bonseon1.npy' # 본선1
# path_name = 'bonseon.npy' # 본선2
# path_name = "kcity_trial2.npy"
#### base frame 바꿔주기
path_name="snu_crosswalk.npy"

#### 변환할 좌표 넣기 (s,q)
coord = [14.8, 41.8]

class find_s:
    def __init__(self):
        GLOBAL_NPY = path_name
        PATH_ROOT=(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))+"/path/npy_file/path/"
        gp_name = PATH_ROOT + GLOBAL_NPY

        self.GB = GlobalPath(gp_name)

    def main(self):
        num = 0
        for i in coord:
            num += 1
            x_t, y_t = self.GB.sl2xy(i, 0)
            print(num),i, "의 xy 좌표는 =>>", x_t, y_t
            # s_t2, p_t2 = self.GB.xy2sl(x_t, y_t, mode = 1)
            # print(num),x_t,y_t, '의 s 좌표는 =>>', s_t2, p_t2


if __name__ == '__main__':
    F = find_s()
    F.main()
