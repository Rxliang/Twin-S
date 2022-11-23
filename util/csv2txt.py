#!/usr/bin/python3

import sys

def csv2txt(src, dest):

    with open(src) as file:
        output = ""
        line = file.readline()
        while line != '':
            line_values = line.split(',')
            output += line_values[0] + ''
            for i in range(7):
                output += line_values[i+1] + ""
            # output += '\n'
            line = file.readline()

    with open(dest, mode='w') as file:
        file.writelines(output)

if __name__ == '__main__':
    csv2txt(sys.argv[1], sys.argv[2])