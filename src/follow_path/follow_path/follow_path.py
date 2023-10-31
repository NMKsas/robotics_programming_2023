#!/usr/bin/env python

import os

# Goal constants
DEFINED_PATHS_DIRECTORY = "../resource/"

QUIT = 1
INVALID_VALUE = "Invalid"
RETURN_LIST = 3


def is_return_signal(input_value):
    string_input = str(input_value)

    if string_input.upper() == "R":
        print("Returning the coordinate list...")
        return True
    return False


def is_exit_signal(input_value):
    string_input = str(input_value)

    if string_input.upper() == "Q":
        print("Quitting the program...")
        return True
    return False


def validate_coordinate(input_value):
    try:
        coord = float(input_value)
        return coord
    except ValueError:
        print("Invalid value '" + input_value + "'! Give numeric values (e.g., 2.0)")
        return INVALID_VALUE


def enter_points_manually():
    print("-----------------------------")
    print("Entering coordinates manually")
    print("-----------------------------")
    path_points_list = []
    while True:
        print()
        print("Enter Q to quit, R to return the current list")
        print("Current coordinate list: " + str(path_points_list) + "\n")

        x_coord = input("Enter x-coordinate: ")
        if is_exit_signal(x_coord):
            return QUIT
        if is_return_signal(x_coord):
            return path_points_list

        y_coord = input("Enter y-coordinate: ")
        if is_exit_signal(y_coord):
            return QUIT
        if is_return_signal(y_coord):
            return path_points_list

        # continue to validate the inputs
        validated_x = validate_coordinate(x_coord)
        validated_y = validate_coordinate(y_coord)
        if validated_x != INVALID_VALUE and validated_y != INVALID_VALUE:
            path_points_list.append((validated_x, validated_y))


def main():
    directory_entries = os.listdir(DEFINED_PATHS_DIRECTORY)

    while True:
        print("Enter M to define path manually")
        print("Enter F to read a file containing the path points")
        print("Enter Q to quit")
        choose_insertion_method = input()
        print()
        if choose_insertion_method.upper() == "M":
            list_of_coordinates = enter_points_manually()
            if list_of_coordinates == QUIT:
                return
            break
        elif choose_insertion_method.upper() == "F":
            print("Enter from a file")
            break
        elif is_exit_signal(choose_insertion_method):
            return
        else:
            continue


if __name__ == '__main__':
    main()
