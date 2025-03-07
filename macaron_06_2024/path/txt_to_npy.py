import numpy as np
import os

class tf_txt_to_npy():
    def __init__(self):
        self.SAVE_ROOT = (os.path.dirname(os.path.abspath(os.path.dirname(__file__))))+"/path/npy_file/"
        self.PATH_ROOT = (os.path.dirname(os.path.abspath(os.path.dirname(__file__))))+"/path/txt_file/"
        self.np_array = []

    def load_file(self, file_name):
        with open(self.PATH_ROOT + file_name) as f:
            self.np_array = []
            for line in f:
                point = []
                x,y = line.split('\t')
                point = [float(x),float(y)]
                self.np_array.append(point)
                    
    def save_numpy(self, file_name):
        np.save(self.SAVE_ROOT + file_name, arr = self.np_array)

def file_conversion():
    tf = tf_txt_to_npy()
    f_type = input('Select the file type to open (1 : global_map / 2 : path / 3 : obstacle) >> ')
    if f_type == 1:
        file_type = "global_map/"
    elif f_type == 2:
        file_type = "path/"
    else:
        file_type = "obstacle/"
    load_file_name = raw_input('Please enter the file name to convert (whitout .txt) >> ')
    save_file_name = raw_input('Please enter the file name to save (whitout .npy) >> ')
    try:
        tf.load_file(file_type + load_file_name + ".txt")
        if save_file_name == "":
            save_file_name = load_file_name
        tf.save_numpy(file_type + save_file_name)
    except:
        print("!!!!!Please re-enter the file path.")
        return "y"
    print("Conversion is complete!!!\n===============================================================")

    st = raw_input('Would you like to check the saved file? (y/n) >> ')
    try:
        if st == "y":
            data = np.load(tf.SAVE_ROOT + file_type + save_file_name + ".npy")
            print(data)
        else:
            pass
    except:
        pass

    return raw_input("Would you like to conversion another file? (y/n) >> ")

def file_open():
    tf = tf_txt_to_npy()
    try:
        f_type = input('Select the file type to open (1 : global_map / 2 : path / 3 : obstacle) >> ')
        if f_type == 1:
            file_type = "global_map/"
        elif f_type == 2:
            file_type = "path/"
        else:
            file_type = "obstacle/"
        open_file_name = raw_input('Please enter the file name to open (from npy_file folder, whitout .npy) >> ')
        data = np.load(tf.SAVE_ROOT + file_type + open_file_name + ".npy")
        print(data)
        for a in range(len(data)-1):
            if (data[a+1][1] - data[a][1] > 1.0) or (data[a+1][0] - data[a][0] > 1.0) :
                print("!!!!!!!!error!!!!!!!!")
            
        return raw_input("Would you like to open another file? (y/n) >> ")
    except:
        print("!!!!!Please re-enter the file path. ex) path/test")
        return "y"

if __name__ == '__main__':
    a = "y"
    print("===============================================================")
    if raw_input("Choose what you want to do (1 : file conversion / 2 : file open) >> ") == "1":
        while(a == "y"):
            a = file_conversion()
        print("===============================================================")
    else:
        while(a == "y"):
            a = file_open()
        print("===============================================================")