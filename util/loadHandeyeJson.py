import sys
sys.path.insert(0, '/home/shc/RoboMaster/util')
from dataLoader import dataLoader


if __name__ == '__main__':
    ld = dataLoader()
    ld.loadHandeyeJson(sys.argv[1])
