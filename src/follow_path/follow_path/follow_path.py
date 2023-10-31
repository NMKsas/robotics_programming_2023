#!/usr/bin/env python

import os

# Goal constants
DEFINED_PATHS_DIRECTORY = "../resource/"


def main(args=None):
    directory_entries = os.listdir(DEFINED_PATHS_DIRECTORY)

    while True:
        print("Enter M to define path manually")
        print("Enter F to read a file containing the path points")
        print("Enter Q to quit")
        choose_insertion_method = input()
        if choose_insertion_method.upper() == "M":
            print("Enter manually")
            break
        elif choose_insertion_method.upper() == "F":
            print("Enter from a file")
            break
        elif choose_insertion_method.upper() == "Q":
            return
        else:
            continue


if __name__ == '__main__':
    main()
