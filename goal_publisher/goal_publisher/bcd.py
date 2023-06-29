import numpy as np
import cv2
from PIL import Image
from matplotlib import pyplot as plt
import matplotlib
from typing import Tuple, List
import random
import math

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
    # c, con = calc_connectivity(erode_img[:,213])
    # print(c,con)
    for col in range(erode_img.shape[1]):
        current_slice = erode_img[:, col]
        connectivity, connective_parts = calc_connectivity(current_slice)
        #print(connective_parts, col)
        if last_connectivity == 0:
            current_cells = []
            for i in range(connectivity):
                current_cells.append(current_cell)
                current_cell += 1
            #print(f'if -> {col}, {current_cell}, {current_cells}')
        elif connectivity == 0:
            #print(f'elif -> {col}')
            current_cells = []
            continue
        else:
            #print(f'else - >{last_connectivity_parts}, {col}')
            adj_matrix = get_adjacency_matrix(last_connectivity_parts, connective_parts)
            print(adj_matrix)
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
            print(current_cells, connective_parts)
            separate_img[slice[0]:slice[1], col] = cell
            print(cell)
            #print('Slice {}: connectivity from {} to {}'.format(col, last_connectivity, connectivity))
        last_connectivity = connectivity
        last_connectivity_parts = connective_parts
    return separate_img, current_cell


def display_separate_map(separate_map, cells):
    display_img = np.empty([*separate_map.shape, 3], dtype=np.uint8)
    random_colors = np.random.randint(0, 255, [cells, 3])
    for cell_id in range(1, cells):
        display_img[separate_map == cell_id, :] = random_colors[cell_id, :]
    plt.imshow(display_img)
    plt.show()
