import os 
import numpy as np 

class FileReader():
    def txt_reader(self, target):
        self.storage = []
        assert os.path.splitext(target)[-1][1:]=="txt", "THE FILE IS NOT TXT FORMAT"
        with open(target,mode='r') as f:
            for line in f.readlines():
                self.storage.append([int(i) for i in line if str.isdigit(i)])

        return np.array(self.storage)

if __name__ == "__main__":
    fileReader = FileReader()
    print np.array(fileReader.txt_reader("/home/apollo/catkin_ws/src/ros_pa2/ros_pa2_extract/map.txt"))
    print np.array(fileReader.txt_reader("/home/apollo/catkin_ws/src/ros_pa2/ros_pa2_extract/map.txt")).shape