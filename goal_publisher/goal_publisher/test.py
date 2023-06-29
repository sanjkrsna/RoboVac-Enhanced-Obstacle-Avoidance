# This is the code of Boustrophedon Cellular Decomposition algorithm.
# I write it in python3.6.
# For more details, please read the paper:
# Choset, H. (2000). Coverage of Known Spaces: The Boustrophedon Cellular Decomposition. Autonomous Robots (Vol. 9).
#
#
#                                               - Dechao Meng
import numpy as np
import cv2
from PIL import Image
from matplotlib import pyplot as plt
import matplotlib
from typing import Tuple, List
import random

# matplotlib.rcParams['figure.figsize'] = [30, 20]

Slice = List[Tuple[int, int]]


def calc_connectivity(slice: np.ndarray) -> Tuple[int, Slice]:
    """
    Calculates the connectivity of a slice and returns the connected area of ​​the slice.

    Args:
        slice: rows. A slice of map.

    Returns:
        The connectivity number and connectivity parts.

    Examples:
        >>> data = np.array([0,0,0,0,1,1,1,0,1,0,0,0,1,1,0,1,1,0])
        >>> print(calc_connectivity(data))
        (4, [(4, 7), (8, 9), (12, 14), (15, 17)])
    """
    connectivity = 0
    last_data = 0
    open_part = False
    connective_parts = []
    for i, data in enumerate(slice):
        if last_data == 0 and data == 1:
            open_part = True
            start_point = i
        elif last_data == 1 and data == 0 and open_part:
            open_part = False
            connectivity += 1
            end_point = i
            connective_parts.append((start_point, end_point))

        last_data = data
    return connectivity, connective_parts


def get_adjacency_matrix(parts_left: Slice, parts_right: Slice) -> np.ndarray:
    """
    Get adjacency matrix of 2 neiborhood slices.

    Args:
        slice_left: left slice
        slice_right: right slice

    Returns:
        [L, R] Adjacency matrix.
    """
    adjacency_matrix = np.zeros([len(parts_left), len(parts_right)])
    for l, lparts in enumerate(parts_left):
        for r, rparts in enumerate(parts_right):
            if min(lparts[1], rparts[1]) - max(lparts[0], rparts[0]) > 0:
                adjacency_matrix[l, r] = 1

    return adjacency_matrix


def bcd(erode_img: np.ndarray) -> Tuple[np.ndarray, int]:
    """
    Boustrophedon Cellular Decomposition

    Args:
        erode_img: [H, W], eroded map. The pixel value 0 represents obstacles and 1 for free space.

    Returns:
        [H, W], separated map. The pixel value 0 represents obstacles and others for its' cell number.
    """
    assert len(erode_img.shape) == 2, 'Map should be single channel.'
    last_connectivity = 0
    last_connectivity_parts = []
    current_cell = 1
    current_cells = []
    separate_img = np.copy(erode_img)

    for col in range(erode_img.shape[1]):
        current_slice = erode_img[:, col]
        connectivity, connective_parts = calc_connectivity(current_slice)

        if last_connectivity == 0:
            current_cells = []
            for i in range(connectivity):
                current_cells.append(current_cell)
                current_cell += 1
        elif connectivity == 0:
            current_cells = []
            continue
        else:
            adj_matrix = get_adjacency_matrix(last_connectivity_parts, connective_parts)
            new_cells = [0] * len(connective_parts)

            for i in range(adj_matrix.shape[0]):
                if np.sum(adj_matrix[i, :]) == 1:
                    new_cells[np.argwhere(adj_matrix[i, :])[0][0]] = current_cells[i]
                # If a previous part is connected to multiple parts this time, it means that IN has occurred.
                elif np.sum(adj_matrix[i, :]) > 1:
                    for idx in np.argwhere(adj_matrix[i, :]):
                        new_cells[idx[0]] = current_cell
                        current_cell = current_cell + 1

            for i in range(adj_matrix.shape[1]):
                # If a part of this time is connected to the last multiple parts, it means that OUT has occurred.
                if np.sum(adj_matrix[:, i]) > 1:
                    new_cells[i] = current_cell
                    current_cell = current_cell + 1
                # If this part of the part does not communicate with any part of the last time, it means that it happened in
                elif np.sum(adj_matrix[:, i]) == 0:
                    new_cells[i] = current_cell
                    current_cell = current_cell + 1
            current_cells = new_cells

        # Draw the partition information on the map.
        for cell, slice in zip(current_cells, connective_parts):
            # print(current_cells, connective_parts)
            separate_img[slice[0]:slice[1], col] = cell

            # print('Slice {}: connectivity from {} to {}'.format(col, last_connectivity, connectivity))
        last_connectivity = connectivity
        last_connectivity_parts = connective_parts
    return separate_img, current_cell


def display_separate_map(separate_map, cells):
    display_img = np.empty([*separate_map.shape, 3], dtype=np.uint8)
    random_colors = np.random.randint(0, 255, [cells, 3])
    for cell_id in range(1, cells):
        display_img[separate_map == cell_id, :] = cell_id
    plt.imshow(display_img)
    plt.show()

def extract_optimized_path(separate_img):
    # Step 1: Iterate over the separate image
    coordinates = []
    for row in range(separate_img.shape[0]):
        for col in range(separate_img.shape[1]):
            cell_number = separate_img[row, col]
            if cell_number != 0:  # Ignore obstacles (cell number 0)
                coordinates.append((row, col, cell_number))
    print((coordinates[0][2]))
    # Step 2: Sort coordinates based on cell number
    coordinates.sort(key=lambda x: x[2])
    # print(coordinates)

    # Step 3: Remove consecutive coordinates with the same cell number
    optimized_path = [coordinates[0]]
   
    grouped_dict = {}

    # Iterate over the input list
    for item in coordinates:
        key = item[2]  # Get the third element as the key
        value = item[:2]  # Get the first two elements as the value
        if key in grouped_dict:
            grouped_dict[key].append(value)  # Append the value to the existing key
        else:
            grouped_dict[key] = [value]  # Create a new key-value pair in the dictionary

    # Print the grouped dictionary
    print(grouped_dict.keys())
        # for i in range1, len(coordinates)):
        #     if coordinates[i][2] != optimized_path[-1][2]:
        #         optimized_path.append(coordinates[i-1])
        #         optimized_path.append(coordinates[i])

    # Step 4: Extract only the (row, col) coordinates
    optimized_path = [(row, col) for row, col, _ in optimized_path]

    return optimized_path


if __name__ == '__main__':
    img = np.array(Image.open('/home/ros2/ros2_ws/src/my_bot/maps/house_map.pgm'))
    # print(get_adjacency_matrix([(138, 140), (310, 313), (337, 338)], [(138, 140), (310, 312), (337, 338)]))
    # kernel = np.ones(12)
    # erode_img = 1 - cv2.erode(img, kernel) / 255
    print(img.shape)

    # if len(img.shape) > 2:
    #     img = img[:, :, 0]

    erode_img = img / np.max(img)
    separate_img, cells = bcd(img)
    print(bcd(img))
    plt.imshow(separate_img, cmap='gray')
    plt.show()


    print('Total cells: {}'.format(cells))
    # display_separate_map(separate_img, cells)



    a = np.array([0,0,1,0,0,1,1,1,0])
    b = np.array([0,1,1,1,1,1,1,1,0])
    _,aa = calc_connectivity(a)
    _,bb = calc_connectivity(b)
    # a =a>5
    # print(a)
    # print(calc_connectivity(a))
    # print(calc_connectivity(b))
    # display_separate_map(bcd(np.array([a,b]))[0],bcd(np.array([a,b]))[1])
    print(extract_optimized_path(bcd(img)[0]))
    # print(get_adjacency_matrix(aa,bb))


