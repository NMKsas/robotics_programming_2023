#!/usr/bin/env python

import os

# Goal constants
COORDINATES_DEFAULT_DIRECTORY = "../resource/"

QUIT = 1
INVALID_VALUE = "Invalid"
RETURN_LIST = 3


def create_ros2_node(list_of_coordinates):
    return None


def is_return_signal(input_value):
    string_input = str(input_value)

    if string_input.upper() == "R":
        print("Returning the coordinate list...")
        return True
    return False


def is_exit_signal(input_value):
    string_input = str(input_value)

    if string_input.upper() == "Q":
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


def parse_to_coordinate_list(file_path):
    coordinate_list = []
    f = open(file_path, "r")
    lines = f.readlines()

    line_count = 1
    for line in lines:
        line_content = line.strip()
        coordinates = line_content.split(",")

        if len(coordinates) != 2:
            break

        validated_x = validate_coordinate(coordinates[0])
        validated_y = validate_coordinate(coordinates[1])
        if validated_x == INVALID_VALUE or validated_y == INVALID_VALUE:
            print("Error in file, line " + str(line_count) + ": '" + line_content + "'.\n" +
                  "Coordinates must be numeric and in format <x_coordinate>,<y_coordinate>.")
            return QUIT
        else:
            coordinate_list.append((validated_x, validated_y))
        line_count += 1
    f.close()
    return coordinate_list


def read_from_file(file_directory):
    entries = os.listdir(file_directory)
    if len(entries) == 0:
        print("No files in default directory '" + COORDINATES_DEFAULT_DIRECTORY + "'!")
        return QUIT

    print("Files in the default directory: " + str(entries))
    file_name = input("Choose file: ")

    for entry in entries:
        if entry == file_name:
            return parse_to_coordinate_list(file_directory + file_name)
    print("File not found!")
    return QUIT


def is_list_empty(list_of_coordinates):
    if len(list_of_coordinates) == 0:
        print("The coordinate list is empty!")
        return True
    return False


def main():
    list_of_coordinates = []
    while True:
        print("Enter M to define path manually")
        print("Enter F to read a file containing the path points")
        print("Enter Q to quit")
        choose_insertion_method = input()
        print()

        if choose_insertion_method.upper() == "M":
            list_of_coordinates = enter_points_manually()
        elif choose_insertion_method.upper() == "F":
            list_of_coordinates = read_from_file(COORDINATES_DEFAULT_DIRECTORY)

        if is_exit_signal(choose_insertion_method) or list_of_coordinates == QUIT \
                or is_list_empty(list_of_coordinates):
            print("Quitting the program...")
            return
        else:
            break

    create_ros2_node(list_of_coordinates)


if __name__ == '__main__':
    main()
