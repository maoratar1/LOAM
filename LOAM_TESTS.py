import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import pandas as pd

from LOAM_MAOR import *

gray = [0.5, 0.5, 0.5]

pc0_toy_points = np.array([[10., 0., 0.],
                           [9., 0., 0.],
                           [8., 0., 0.],
                           [7., 0., 0.],
                           [6., 0., 0.],
                           [5., 0., 0.],
                           [4., 0., 0.],
                           [3., 0., 0.],
                           [2., 0., 0.],
                           [1., 0., 0.],
                           [0., 1., 0.],
                           [0., 2., 0.],
                           [0., 3., 0.],
                           [0., 4., 0.],
                           [0., 5., 0.],
                           [0., 6., 0.],
                           [0., 7., 0.],
                           [0., 8., 0.],
                           [0., 9., 0.],
                           [0., 10., 0.],
                           [10., 0., 1.],
                           [9., 0., 1.],
                           [8., 0., 1.],
                           [7., 0., 1.],
                           [6., 0., 1.],
                           [5., 0., 1.],
                           [4., 0., 1.],
                           [3., 0., 1.],
                           [2., 0., 1.],
                           [1., 0., 1.],
                           [0., 1., 1.],
                           [0., 2., 1.],
                           [0., 3., 1.],
                           [0., 4., 1.],
                           [0., 5., 1.],
                           [0., 6., 1.],
                           [0., 7., 1.],
                           [0., 8., 1.],
                           [0., 9., 1.],
                           [0., 10., 1.],
                           [10., 0., 2.],
                           [9., 0., 2.],
                           [8., 0., 2.],
                           [7., 0., 2.],
                           [6., 0., 2.],
                           [5., 0., 2.],
                           [4., 0., 2.],
                           [3., 0., 2.],
                           [2., 0., 2.],
                           [1., 0., 2.],
                           [0., 1., 2.],
                           [0., 2., 2.],
                           [0., 3., 2.],
                           [0., 4., 2.],
                           [0., 5., 2.],
                           [0., 6., 2.],
                           [0., 7., 2.],
                           [0., 8., 2.],
                           [0., 9., 2.],
                           [0., 10., 2.],
                           [10., 0., 3.],
                           [9., 0., 3.],
                           [8., 0., 3.],
                           [7., 0., 3.],
                           [6., 0., 3.],
                           [5., 0., 3.],
                           [4., 0., 3.],
                           [3., 0., 3.],
                           [2., 0., 3.],
                           [1., 0., 3.],
                           [0., 1., 3.],
                           [0., 2., 3.],
                           [0., 3., 3.],
                           [0., 4., 3.],
                           [0., 5., 3.],
                           [0., 6., 3.],
                           [0., 7., 3.],
                           [0., 8., 3.],
                           [0., 9., 3.],
                           [0., 10., 3.],
                           [10., 0., 4.],
                           [9., 0., 4.],
                           [8., 0., 4.],
                           [7., 0., 4.],
                           [6., 0., 4.],
                           [5., 0., 4.],
                           [4., 0., 4.],
                           [3., 0., 4],
                           [2., 0., 4],
                           [1., 0., 4],
                           [0., 1., 4],
                           [0., 2., 4],
                           [0., 3., 4],
                           [0., 4., 4.],
                           [0., 5., 4.],
                           [0., 6., 4.],
                           [0., 7., 4.],
                           [0., 8., 4.],
                           [0., 9., 4.],
                           [0., 10., 4.],
                           [10., 0., 5.],
                           [9., 0., 5.],
                           [8., 0., 5.],
                           [7., 0., 5.],
                           [6., 0., 5.],
                           [5., 0., 5],
                           [4., 0., 5],
                           [3., 0., 5.],
                           [2., 0., 5.],
                           [1., 0., 5.],
                           [0., 1., 5.],
                           [0., 2., 5.],
                           [0., 3., 5.],
                           [0., 4., 5.],
                           [0., 5., 5.],
                           [0., 6., 5.],
                           [0., 7., 5.],
                           [0., 8., 5.],
                           [0., 9., 5.],
                           [0., 10., 5.]
                           ])

custom_colors = np.array([
        [0, 1, 0],  # Green
        [1, 1, 0],  # Yellow
        [0, 1, 1],  # Cyan
        [1, 0, 1],  # Magenta
        [0.5, 0, 0],  # Dark Red
        [0, 0.5, 0],  # Dark Green
        [0, 0, 0.5],  # Dark Blue
        [0.5, 0.5, 0],  # Olive
        [0, 0.5, 0.5],  # Teal
        [0.5, 0, 0.5],  # Purple
        [0.9, 0.1, 0.1],  # Light Red
        [0.1, 0.9, 0.1],  # Light Green
        [0.1, 0.1, 0.9],  # Light Blue
        [0.8, 0.6, 0],  # Orange
        [0.6, 0, 0.8],  # Violet
        [0.8, 0, 0.6],  # Pink
        [0, 0.8, 0.6],  # Aqua
        [0.6, 0.8, 0],  # Lime
        [0.4, 0.4, 0.4],  # Grey
        [0.2, 0.2, 0.2],  # Dark Grey
        [0.2, 0.4, 0.6],  # Muted Blue
        [0.6, 0.2, 0.4],  # Muted Purple
        [0.4, 0.6, 0.2],  # Muted Green
        [1, 0.5, 0],  # Bright Orange
        [0.5, 1, 0],  # Bright Lime
        [0, 1, 0.5],  # Bright Aqua
        [0.5, 0, 1],  # Bright Purple
        [1, 0, 0.5],  # Bright Pink
        [0, 0.5, 1],  # Bright Blue
        [1, 1, 0.5],  # Light Yellow
        [0.5, 1, 1],  # Light Cyan
        [1, 0.5, 1],  # Light Magenta
        [0.5, 0.5, 1],  # Soft Blue
        [0.5, 1, 0.5],  # Soft Green
        [1, 0.5, 0.5],  # Soft Red
        [1, 0.75, 0],  # Amber
        [0, 1, 0.75],  # Mint
        [0.75, 0, 1],  # Lavender
        [1, 0, 0.75],  # Rose
        [0.75, 1, 0],  # Chartreuse
        [0, 0.75, 1],  # Sky Blue
        [1, 0.4, 0.2],  # Coral
        [0.2, 1, 0.4],  # Spring Green
        [0.4, 0.2, 1],  # Periwinkle
        [1, 0.2, 0.4],  # Raspberry
        [0.4, 1, 0.2],  # Lime Green
        [0.2, 0.4, 1],  # Indigo
        [1, 0.2, 0.6],  # Fuchsia
        [0.6, 1, 0.2],  # Apple Green
        [0.2, 0.6, 1],  # Azure
        [1, 0.6, 0.2],  # Apricot
        [0.6, 0.2, 1],  # Grape
        [0.2, 1, 0.6],  # Mint Green
        [0.6, 0.2, 0.4],  # Burgundy
        [0.4, 0.6, 1],  # Powder Blue
        [1, 0.4, 0.6],  # Cherry Red
        [0.6, 1, 0.4],  # Pale Lime
        [0.4, 1, 0.6],  # Pale Mint
        [0.6, 0.4, 1],  # Pale Violet
        [1, 0.6, 0.4],  # Peach
        [1, 0, 0],  # Red
        [0, 1, 0],  # Green
        [0, 0, 1],  # Blue
        [1, 1, 0],  # Yellow
        [0, 1, 1],  # Cyan
        [1, 0, 1],  # Magenta
        [0.5, 0, 0],  # Dark Red
    ])

color_names = [
    "Green",
    "Yellow",
    "Cyan",
    "Magenta",
    "Dark Red",
    "Dark Green",
    "Dark Blue",
    "Olive",
    "Teal",
    "Purple",
    "Light Red",
    "Light Green",
    "Light Blue",
    "Orange",
    "Violet",
    "Pink",
    "Aqua",
    "Lime",
    "Grey",
    "Dark Grey",
    "Muted Blue",
    "Muted Purple",
    "Muted Green",
    "Bright Orange",
    "Bright Lime",
    "Bright Aqua",
    "Bright Purple",
    "Bright Pink",
    "Bright Blue",
    "Light Yellow",
    "Light Cyan",
    "Light Magenta",
    "Soft Blue",
    "Soft Green",
    "Soft Red",
    "Amber",
    "Mint",
    "Lavender",
    "Rose",
    "Chartreuse",
    "Sky Blue",
    "Coral",
    "Spring Green",
    "Periwinkle",
    "Raspberry",
    "Lime Green",
    "Indigo",
    "Fuchsia",
    "Apple Green",
    "Azure",
    "Apricot",
    "Grape",
    "Mint Green",
    "Burgundy",
    "Powder Blue",
    "Cherry Red",
    "Pale Lime",
    "Pale Mint",
    "Pale Violet",
    "Peach",
    "Red",
    "Green",
    "Blue",
    "Yellow",
    "Cyan",
    "Magenta",
    "Dark Red"
]


NUM_SCANS = 64


def load_pc(num):
    file_path = os.path.join(PCD_PATH, "{}.txt".format(num))
    data = np.loadtxt(file_path, delimiter=' ')
    points = data[:, :3]  # Only x, y, z are needed
    r = data[:, 3]

    # Create Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd


def test_loading():
    file_path = os.path.join(PCD_PATH, "1.txt")
    data = np.loadtxt(file_path, delimiter=' ')
    points = data[:, :3]  # Only x, y, z are needed
    r = data[:, 3]

    # show_PC(points, r)
    # show_PC_o3d(points, r)
    show_PC_with_distance_map(points)


def test_time_stamps_ends():
    load_timestamps_ends()


def test_KITTI_scans(points):
    # data0 = np.loadtxt(os.path.join(PCD_PATH, "0.txt"), delimiter=' ')
    # points = data0[:, :3]  # Only x, y, z are needed

    # # # Load the point cloud from a .bin file (replace with your actual file path)
    # path = "/homes/maorat/Downloads/0000000000.bin"
    # # path = "/homes/maorat/Downloads/000000.bin"
    # points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    # points = points[:, 0:3]

    # Extract x, y, z coordinates
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Calculate the phi angle for each point
    # depth = np.linalg.norm(points[:, :3], 2, axis=1)
    phi = np.arcsin(z / np.sqrt(x**2 + y**2 + z**2)) * 180. / np.pi
    # phi = phi[(phi <= 2) & (phi >= -24.8)]

    # intervals = calc_phis_interval(phi)
    # intervals = get_scans_from_phi(phi)
    intervals = get_scans_from_phi2(phi)
    colors = colors_from_phi_intervals(intervals)

    def show_o3d(points, colors):
        # Create an Open3D point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # Only x, y, z
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Visualize the point cloud
        o3d.visualization.draw_geometries([pcd], window_name="Point Cloud with Phi Colors")

    show_o3d(points, colors)


def colors_from_scan_id(pnts, scan_start, scan_end):
    # Convert the normalized scan_ids to RGB colors using a colormap
    # Assign the colors based on scan_ids
    num_scans = len(scan_start)
    # Initialize an array to store the color for each point
    colors = np.zeros((len(pnts), 3))
    # Assign colors to points based on scan ranges
    for i in range(num_scans):
        start_idx = scan_start[i]
        end_idx = scan_end[i]
        colors[start_idx:end_idx + 1] = custom_colors[i]
    # colors = np.array([custom_colors[scan_id] for scan_id in scan_ids])

    return colors


def colors_from_phi_intervals(phi_intervals):
    # Convert the normalized scan_ids to RGB colors using a colormap

    # Assign the colors based on scan_ids
    colors = []
    for i in range(len(phi_intervals)):
        colors.append(custom_colors[phi_intervals[i]])

    return np.array(colors)


def colors_for_1_scan(intervals, scan_num):
    colors = []

    for interval in intervals:
        if interval == scan_num:
            colors.append([1, 0, 0])
            continue

        colors.append(gray)

    return np.array(colors)


def ant():
    path = "/homes/maorat/Downloads/000000.bin"

    # Load the point cloud from a .bin file (replace with your actual file path)
    points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)

    depth = np.linalg.norm(points[:, :3], 2, axis=1)
    pitch = np.arcsin(points[:, 2] / depth)
    fov_down = -24.8 / 180.0 * np.pi
    fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
    scan_ids = (pitch + abs(fov_down)) / fov
    scan_ids *= 64
    scan_ids = np.floor(scan_ids)
    scan_ids = np.minimum(64 - 1, scan_ids)
    scan_ids = np.maximum(0, scan_ids).astype(np.int32)

    # Assign the colors based on scan_ids
    colors = []
    for i in range(len(scan_ids)):
        colors.append(custom_colors[scan_ids[i]])
    # colors = np.array([custom_colors[scan_id] for scan_id in scan_ids])


def get_scan_ids(pcd, N_SCANS=64):
    depth = np.linalg.norm(pcd[:, :3], 2, axis=1)
    pitch = np.arcsin(pcd[:, 2] / depth)

    indices = (pitch <= 2 * np.pi / 180.) & (pitch >= -24.8 * np.pi / 180.)
    pcd = pcd[indices]
    pitch = pitch[indices]
    # fov_down = min(pitch)
    # fov = abs(fov_down) + abs(max(pitch))

    fov_down = -24.8 / 180.0 * np.pi
    fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
    scan_ids = (pitch + abs(fov_down)) / fov
    scan_ids *= (N_SCANS)
    scan_ids = np.floor(scan_ids)
    scan_ids -= min(scan_ids)
    # scan_ids = np.minimum(N_SCANS - 1, scan_ids)
    # scan_ids = np.maximum(0, scan_ids).astype(np.int32)
    return scan_ids, pcd


def reorder_pcd(pcd, N_SCANS=64):
    scan_start = np.zeros(N_SCANS, dtype=int)
    scan_end = np.zeros(N_SCANS, dtype=int)

    scan_ids, pcd = get_scan_ids(pcd)
    sorted_ind = np.argsort(scan_ids, kind='stable')
    sorted_pcd = pcd[sorted_ind]
    sorted_scan_ids = scan_ids[sorted_ind]

    elements, elem_cnt = np.unique(sorted_scan_ids, return_counts=True)

    start = 0
    for ind, cnt in enumerate(elem_cnt):
        scan_start[ind] = start
        start += cnt
        scan_end[ind] = start

    laser_cloud = np.hstack((sorted_pcd, sorted_scan_ids.reshape((-1, 1))))
    return laser_cloud, scan_start, scan_end


def get_item(folder_path, file_ind):
    path = os.path.join(folder_path, file_ind)
    pcd = np.fromfile(path, dtype=np.float32).reshape(-1, 4)[:, :3]
    return reorder_pcd(pcd)


def get_item_from_points(points):
    return reorder_pcd(points)


def show_pcd(points, colors=None):
    # Create an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # Only x, y, z
    if colors:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd], window_name="Point Cloud with Scan ID Colors")


def test_ant():
    file = "0000000000.bin"
    # file = "000000.bin"

    pcd, scan_start, scan_end = get_item("/homes/maorat/Downloads/", file)
    colors = colors_from_scan_id(pcd, scan_start, scan_end)
    # colors = colors_from_phi_intervals(pcd[:, 3].flatten().astype(np.int32))
    show_pcd(pcd, colors)


def calc_phis_interval(phi_values):
    # path = "/homes/maorat/Downloads/000000.bin"
    #
    # # Load the point cloud from a .bin file (replace with your actual file path)
    # points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    NUM_SCANS = 64.
    # phi_min = min(phi_values)
    # phi_max = max(phi_values)
    # resolution = (phi_max - phi_min) / NUM_SCANS

    phi_min = -24.8
    phi_max = 2.
    resolution = 0.4

    # # Map each phi value to an interval number
    # def phi_to_interval(phi):
    #     # Ensure phi is within the defined range
    #     if phi < phi_min or phi > phi_max:
    #         return 64
    #
    #     # Calculate the interval number
    #     interval_number = int((phi - phi_min) / resolution)
    #
    #     return interval_number

    interval_numbers = []
    for phi in phi_values:
        if phi < phi_min or phi > phi_max:
            continue
        interval_number = int((phi - phi_min) / resolution)
        interval_numbers.append(interval_number)

    return np.array(interval_numbers)


def calc_phis():
    path = "/homes/maorat/Downloads/0000000000.bin"
    # path = "/homes/maorat/Downloads/000000.bin"
    # Load the point cloud from a .bin file (replace with your actual file path)
    points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)

    # data0 = np.loadtxt(os.path.join(PCD_PATH, "0.txt"), delimiter=' ')
    # points = data0[:, :3]  # Only x, y, z are needed

    # Extract x, y, z coordinates
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Calculate the phi angle for each point
    # depth = np.linalg.norm(points[:, :3], 2, axis=1)
    # phi_values = np.arctan2(z(x**2 + y**2))) * 180. / np.pi
    phi_values = np.arctan2(z, np.sqrt(x ** 2 + y ** 2)) * 180.0 / np.pi

    # Define the range and resolution
    phi_min = min(phi_values)
    phi_max = max(phi_values)
    resolution = (phi_max - phi_min) / 64.

    # Calculate the number of intervals
    num_intervals = int((phi_max - phi_min) / resolution)

    # Map each phi value to an interval number
    def phi_to_interval(phi):
        # Ensure phi is within the defined range
        if phi < phi_min or phi > phi_max:
            raise ValueError(f"phi value {phi} is out of bounds.")

        # Calculate the interval number
        interval_number = int((phi - phi_min) / resolution)

        return interval_number

    # Apply the function to the array of phi values
    interval_numbers = np.array([phi_to_interval(phi) for phi in phi_values])

    plt.figure(figsize=(14, 8))  # Increase figure size for better readability
    plt.hist(interval_numbers, bins=num_intervals, range=(0, num_intervals), edgecolor='black')
    plt.title('Histogram of Phi Intervals')
    plt.xlabel('Interval Number')
    plt.ylabel('Frequency')

    # Set x-axis ticks and labels
    tick_interval = 5  # Adjust this to control the number of ticks shown
    ticks = np.arange(0, num_intervals, tick_interval)
    tick_labels = [f"{phi_min + i * resolution:.1f}" for i in ticks]

    plt.xticks(ticks=ticks, labels=tick_labels, rotation=45, ha='right')  # Rotate labels for better visibility
    plt.grid(True)
    plt.tight_layout()  # Adjusts subplots to give some padding
    plt.show()


def get_scans_from_phi(phi_values):
    dphi = [phi_values[i] - phi_values[i - 1] for i in range(1, len(phi_values))]
    dphi2 = [dphi[i] - dphi[i - 1] for i in range(1, len(dphi))]
    scans = []
    scan_num = 0

    for i in range(len(dphi2)):
        if -0.6 < dphi[i] < -0.35:
            scan_num += 1
        scans.append(scan_num)

    # for i in range(len(dphi)):
    #     if -0.6 < dphi[i] < -0.35:
    #         scans.append(1)
    #     else:
    #         scans.append(0)

    return np.array(scans)


def get_scans_from_phi2(phi_values):
    max_phi = np.max(phi_values)
    min_phi = np.min(phi_values)
    reference_phis = np.linspace(min_phi, max_phi, 64)
    # Step 2: Assign the closest reference point index to each phi value
    closest_indices = np.abs(phi_values[:, np.newaxis] - reference_phis).argmin(axis=1)
    return closest_indices


def check_phi():
    from scipy.ndimage import gaussian_filter1d
    # # Load the point cloud from a .bin file (replace with your actual file path)
    # path = "/homes/maorat/Downloads/0000000000.bin"
    path = "/homes/maorat/Downloads/000000.bin"
    points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)

    # Extract x, y, z coordinates
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Calculate the phi angle for each point
    # depth = np.linalg.norm(points[:, :3], 2, axis=1)
    phi = np.arcsin(z / np.sqrt(x ** 2 + y ** 2 + z ** 2)) * 180. / np.pi
    indices = (phi <= 2) & (phi >= -24.8)
    phi = phi[indices]
    # phi = np.sort(phi)

    # dphi = [phi[i] - phi[i - 1] for i in range(1, len(phi))]
    # dphi2 = [dphi[i] - dphi[i - 1] for i in range(1, len(dphi))]
    #
    # scans = []
    # scan_num = 0
    # for i in range(len(dphi2)):
    #     if -0.6 < dphi[i] < -0.35:
    #         scan_num += 1
    #     scans.append(scan_num)
    #
    # print(scan_num)

    # plt.plot(dphi)
    # plt.plot(gaussian_filter1d(dphi, sigma=1.5))
    # plt.plot(phi)
    scan_ids, pcd = get_scan_ids(points)
    fig, ax = plt.subplots()

    i=3
    for i in np.unique(scan_ids).astype(int):
        mask = scan_ids == i
        scan_id_mask = phi[mask]
        clr = custom_colors[i, :]
        ax.hist(scan_id_mask, bins=4,  facecolor=clr)

    peaks = [ -24.8 + 0.4*i for i in range(64)]

    for elem in peaks:
        ax.axvline(elem, color='red')
    # plt.plot(scans)
    fig.show()
    plt.show()
    stop = 2


def check_phi_gpt(points):
    # Calculate phi for each point
    phi_values = np.arcsin(points[:, 2] / np.linalg.norm(points, axis=1))  # in radians
    phi_values = np.degrees(phi_values)  # convert to degrees

    # Round phi values to match sensor resolution
    phi_resolution = 0.4
    rounded_phi_values = np.round(phi_values / phi_resolution) * phi_resolution

    # Find unique rounded phi values and assign an index to each
    unique_phi_values, group_indices = np.unique(rounded_phi_values, return_inverse=True)

    # Now, group_indices is an array with the same size as points, where each element is the group index
    return group_indices


def test_feature_points(pc1, scan_num=None):
    scans = find_feature_points_in_point_cloud(pc1, scan_num)

    if not scan_num:
        show_feature_points(pc1, scans)
    else:
        show_feature_points_single_scan(pc1, scans)


def test_feature_points_2pc(pc0, pc1, scan_num=None):
    scans0 = find_feature_points_in_point_cloud(pc0, scan_num)
    scans1 = find_feature_points_in_point_cloud(pc1, scan_num)

    if not scan_num:
        show_feature_points_2_pc(pc0, pc1, scans0, scans1)
    else:
        show_feature_points_2_pc(pc0, pc1, scans0, scans1)


def show_feature_points_single_scan(pc, scan):
    if len(scan.edge_points) > 0:
        print("Edge points:")
        print(scan.edge_points)
        print(" Edge points Indices(rel to scan):")
        print(scan.edge_points_indices)
        print(" C vals:")
        print(scan.edges_c_vals)
        print(" C vals without norm:")
        c_vals_without_norms = [scan.edges_c_vals[i] * np.linalg.norm(scan.edge_points[i]) for i in range(len(scan.edge_points))]
        print(c_vals_without_norms)
    else:
        print("No edge points")

    print("Planar points:")
    print(scan.planar_points)
    print(" Planar points Indices(rel to scan):")
    print(scan.planar_points_indices)
    print(" C vals:")
    print(scan.planar_c_vals)
    print(" C vals without norm:")
    c_vals_without_norms_planes = [scan.planar_c_vals[i] * np.linalg.norm(scan.planar_points[i]) for i in
                            range(len(scan.planar_points))]
    print(c_vals_without_norms_planes)

    # pcd = color_edges_and_planars_in_scan(scan)
    pcd = color_edges_and_planars_in_scan(scan)

    o3d.visualization.draw_geometries([pcd])


def color_a_scan_in_pc(point_cloud, scan_ind, p_p=None, e_p=None, start_ind_in_scan=0, end_ind_in_scan=None, start_ind_in_pc=None, end_ind_in_pc=None):
    colors = np.full(point_cloud.points.shape, 0.8)  # RGB for gray
    scan_start_pc_ind = point_cloud.scan_start[scan_ind] + start_ind_in_scan

    if end_ind_in_scan:
        scan_end_pc_ind = point_cloud.scan_start[scan_ind] + end_ind_in_scan
    else:
        scan_end_pc_ind = point_cloud.scan_end[scan_ind]

    if start_ind_in_pc:
        scan_start_pc_ind = start_ind_in_pc
    if end_ind_in_pc:
        scan_end_pc_ind = end_ind_in_pc

    colors[scan_start_pc_ind: scan_end_pc_ind + 1] = [0., 1., 0.]  # Green

    if p_p:
        colors[point_cloud.scan_start[scan_ind] + p_p] = [0., 0., 1.]  # Blue
    if e_p:
        colors[point_cloud.scan_start[scan_ind] + e_p] = [1., 0., 0.]  # Read

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud.points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])


def color_part_of_a_scan(point_cloud, scan_ind, p_p=None, e_p=None, start_ind_in_scan=0, end_ind_in_scan=None, win=None):
    scan_start_in_pc = point_cloud.scan_start[scan_ind]
    scan_end_in_pc = point_cloud.scan_end[scan_ind]
    scan_points = point_cloud.points[scan_start_in_pc:scan_end_in_pc, :]

    colors = np.full(scan_points.shape, 0.8)  # RGB for gray

    if win and e_p:
        for i, e_point in enumerate(e_p):
            colors[e_point - win: e_point + win + 1] = custom_colors[i]  # Green
            colors[e_point] = [1., 0., 0.]  # Blue
            print("{}) {} : {}".format(i, e_point, color_names[i]))
    elif win and p_p:
        for i, p_point in enumerate(p_p):
            colors[p_point - win: p_point + win + 1] = custom_colors[i]  # Green
            colors[p_point] = [0., 0., 1.]  # Blue
            print("{}) {} : {}".format(i, p_point, color_names[i]))
    else:
        colors[start_ind_in_scan: end_ind_in_scan + 1] = [0., 1., 0.]  # Green

    if p_p:
        colors[p_p] = [0., 0., 1.]  # Blue
    if e_p:
        colors[e_p] = [1., 0., 0.]  # Read

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(scan_points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])


def color_part_of_a_scan_2_pc(pc0, pc1, scan_ind, p_p=None, e_p=None, start_ind_in_scan=0, end_ind_in_scan=None, win=None):
    scan_start_in_pc0 = pc0.scan_start[scan_ind]
    scan_end_in_pc0 = pc0.scan_end[scan_ind]
    scan_points0 = pc0.points[scan_start_in_pc0:scan_end_in_pc0, :]

    scan_start_in_pc1 = pc1.scan_start[scan_ind]
    scan_end_in_pc1 = pc1.scan_end[scan_ind]
    scan_points1 = pc1.points[scan_start_in_pc1:scan_end_in_pc1, :]

    colors0 = np.full(scan_points0.shape, 0.8)  # RGB for gray
    colors1 = np.full(scan_points1.shape, 0.4)  # RGB for gray
    #
    # if win and e_p:
    #     colors[e_p - win: e_p + win + 1] = [0., 1., 0.]  # Green
    # elif win and p_p:
    #     colors[p_p - win: p_p + win + 1] = [0., 1., 0.]  # Green
    # else:
    #     colors[start_ind_in_scan: end_ind_in_scan + 1] = [0., 1., 0.]  # Green
    #
    # if p_p:
    #     colors[p_p] = [0., 0., 1.]  # Blue
    # if e_p:
    #     colors[e_p] = [1., 0., 0.]  # Read


    pcd0 = o3d.geometry.PointCloud()
    pcd0.points = o3d.utility.Vector3dVector(scan_points0)
    pcd0.colors = o3d.utility.Vector3dVector(colors0)

    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(scan_points1)
    pcd1.colors = o3d.utility.Vector3dVector(colors1)

    o3d.visualization.draw_geometries([pcd0, pcd1])


def color_edges_and_planars_in_pc(point_cloud, scans):
    colors = np.full(point_cloud.points.shape, 0.8)  # RGB for gray

    for i, scan in enumerate(scans):
        scan_start_pc_ind = point_cloud.scan_start[i]
        edges_indices_in_pc = scan.edge_points_indices + scan_start_pc_ind
        planar_indices_in_pc = scan.planar_points_indices + scan_start_pc_ind
        if len(edges_indices_in_pc) > 0:
            colors[edges_indices_in_pc] = [1, 0, 0]   # Red
        if len(planar_indices_in_pc) > 0:
            colors[planar_indices_in_pc] = [0, 0, 1]   # Blue

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud.points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def color_edges_and_planars_in_scan(scan, win=0):
    # start = 1140
    # end = 1150

    start = 0
    end = len(scan.scan_points)
    colors = np.full(scan.scan_points.shape, 0.8)  # RGB for gray

    edge_points_indices = scan.edge_points_indices[(scan.edge_points_indices >= start) & (scan.edge_points_indices <= end)]
    planar_points_indices = scan.planar_points_indices[(scan.planar_points_indices >= start) & (scan.planar_points_indices <= end)]

    print("Edges:")
    if len(edge_points_indices):
        for i, e_point in enumerate(edge_points_indices):
            colors[e_point - win: e_point + win + 1] = custom_colors[i]  # Green
            print("{}) {} : {}".format(i, e_point, color_names[i]))
        colors[edge_points_indices] = [1., 0., 0.]  # Blue

    print("Planars")
    if len(planar_points_indices):
        for i, p_point in enumerate(planar_points_indices):
            color_ind = i + len(edge_points_indices)
            colors[p_point - win: p_point + win + 1] = custom_colors[color_ind]  # Green
            colors[p_point] = [0., 0., 1.]  # Blue
            print("{}) {} : {}".format(i, p_point, color_names[color_ind]))
        colors[planar_points_indices] = [0., 0., 1.]  # Blue


    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(scan.scan_points[start:end])
    pcd.colors = o3d.utility.Vector3dVector(colors[start:end])
    return pcd


def show_feature_points(pc, scans):
    for i, scan in enumerate(scans):
        if len(scan.edge_points) > 0 or len(scan.planar_points) > 0:
            print("Scan {} has: ".format(i), end=" ")
            if len(scan.edge_points) > 0:
                print("{} edges".format(len(scan.edge_points)), end="  ")
            else:
                print("       ", end= "  ")
            if len(scan.planar_points) > 0:
                print("{} planars".format(len(scan.planar_points)))
            else:
                print()
        # print("Scan {}:".format(i))
    #     print(" Edge points:")
    #     print(scan.edge_points)
    #     print(" Edge points Indices:")
    #     print(scan.edge_points_indices)
    #     print(" C vals:")
    #     print(scan.edges_c_vals)
    #
    #     print(" Planar points:")
    #     print(scan.planar_points)
    #     print(" Planar points Indices:")
    #     print(scan.planar_points_indices)
    #     print(" C vals:")
    #     print(scan.planar_c_vals)
    planar_features = 0
    edges_features = 0
    for scan in scans:
        edges_features += len(scan.edge_points_indices)
        planar_features += len(scan.planar_points_indices)

    print("Edges Num: ", edges_features)
    print("Planar Num: ", planar_features)

    pcd = color_edges_and_planars_in_pc(pc, scans)
    o3d.visualization.draw_geometries([pcd])


def show_feature_points_2_pc(pc0, pc1, scans0, scans1):
    # for i, scan in enumerate(scans):
    #     print("Scan {}:".format(i))
    #     print(" Edge points:")
    #     print(scan.edge_points)
    #     print(" Edge points Indices:")
    #     print(scan.edge_points_indices)
    #     print(" C vals:")
    #     print(scan.edges_c_vals)
    #
    #     print(" Planar points:")
    #     print(scan.planar_points)
    #     print(" Planar points Indices:")
    #     print(scan.planar_points_indices)
    #     print(" C vals:")
    #     print(scan.planar_c_vals)
    planar_features0 = 0
    edges_features0 = 0
    for scan in scans0:
        edges_features0 += len(scan.edge_points_indices)
        planar_features0 += len(scan.planar_points_indices)

    planar_features1 = 0
    edges_features1 = 0
    for scan in scans1:
        edges_features1 += len(scan.edge_points_indices)
        planar_features1 += len(scan.planar_points_indices)

    print("Edges Num0: ", edges_features0)
    print("Planar Num0: ", planar_features0)
    print("Edges Num1: ", edges_features1)
    print("Planar Num1: ", planar_features1)

    pcd0 = color_edges_and_planars_in_pc(pc0, scans0)
    pcd1 = color_edges_and_planars_in_pc(pc1, scans1)
    o3d.visualization.draw_geometries([pcd0, pcd1])


def show_edges_planar_pcd(points):
    edges = np.array([[1., 0., 0.], [2., 0., 0.]])
    planar = np.array([[0., 5., 0.], [0., 6., 0.]])
    pcd = create_point_cloud(points, color=[0.5, 0.5, 0.5])  # Light gray color
    edge_pcd = create_point_cloud(edges, color=[1., 0., 0.])  # Light gray color
    planar_pcd = create_point_cloud(planar, color=[0., 0., 1.])  # Light gray color

    # o3d.visualization.draw_geometries([pcd], window_name="Point Cloud with Highlights")
    # o3d.visualization.draw_geometries([edge_pcd], window_name="Point Cloud with Highlights")
    # o3d.visualization.draw_geometries([edges_pcd, planar_pcd, scan_pcd, pcd], window_name="Point Cloud with Highlights")
    o3d.visualization.draw_geometries([pcd, edge_pcd, planar_pcd], window_name="")


def show_large(edges_pcd, planar_pcd, scan_pcd, pcd):
    pcd.paint_uniform_color([0.5, 0.5, 0.5])  # Set base color
    scan_pcd.paint_uniform_color([0.3, 0.3, 0.3])  # Set scan color
    edges_pcd.paint_uniform_color([1, 0, 0])  # Set edges color
    planar_pcd.paint_uniform_color([0, 0, 1])  # Set planar color

    pcd.colors = o3d.utility.Vector3dVector(np.array(pcd.colors) * 0.5)  # Reduce opacity by darkening colors
    scan_pcd.colors = o3d.utility.Vector3dVector(np.array(scan_pcd.colors) * 0.5)  # Reduce opacity

    o3d.visualization.draw_geometries([edges_pcd, planar_pcd, scan_pcd, pcd], window_name="Point Cloud with Highlights")


def test_correspondence_features(pc0, pc1, NUM_SCANS_TO_TAKE=None):
    scans0 = find_feature_points_in_point_cloud(pc0)
    scans = find_feature_points_in_point_cloud(pc1)

    if NUM_SCANS_TO_TAKE:
        scans0 = scans0[: NUM_SCANS_TO_TAKE]
        scans = scans[: NUM_SCANS_TO_TAKE]

    scans0_pts = np.concatenate([s.scan_points for s in scans0], axis=0)
    scans_pts = np.concatenate([s.scan_points for s in scans], axis=0)

    edge_points_lst = np.concatenate([s.edge_points for s in scans if s.edge_points != []], axis=0)
    planar_points_lst = np.concatenate([s.planar_points for s in scans if s.planar_points != []], axis=0)

    corresponding_edges_lines = find_corresponding_edges_lines(edge_points_lst, pc0)
    corresponding_planar_patches = find_corresponding_planar_patches(planar_points_lst, pc0)

    edge_points_lst = np.array(edge_points_lst)
    corresponding_planar_patches = np.array(corresponding_planar_patches)

    # Create PointCloud objects
    pcd1 = create_point_cloud(scans0_pts, color=[0.8, 0.8, 0.8])  # Light gray color
    pcd2 = create_point_cloud(scans_pts, color=[0.6, 0.6, 0.6])  # Gray color
    edge_points = create_point_cloud(edge_points_lst, color=[1, 0, 0])  # Red
    planar_points = create_point_cloud(planar_points_lst, color=[0, 1, 0])

    corresponding_edges_lines_points = []
    # Iterate over each EdgeCorr object and add the e0 and e1 points to the list
    for edge_corr in corresponding_edges_lines:
        corresponding_edges_lines_points.append(edge_corr.e1)
        corresponding_edges_lines_points.append(edge_corr.e2)
    corresponding_edges_lines_points = np.array(corresponding_edges_lines_points)

    corresponding_planar_patches_points = []
    # Iterate over each EdgeCorr object and add the e0 and e1 points to the list
    for planar_patch in corresponding_planar_patches:
        corresponding_planar_patches_points.append(planar_patch.e1)
        corresponding_planar_patches_points.append(planar_patch.e2)
        corresponding_planar_patches_points.append(planar_patch.e3)
    corresponding_planar_patches_points = np.array(corresponding_planar_patches_points)

    # Convert the list of points to a numpy array
    corresponding_edges_lines_list = create_point_cloud(corresponding_edges_lines_points, color=[0, 0, 1])  # Blue
    corresponding_planar_patches_list = create_point_cloud(corresponding_planar_patches_points, color=[0, 1, 0.8])  # Blue

    # o3d.visualization.draw_geometries([pcd1, pcd2, corresponding_edges_lines_list], window_name="Two Point Clouds")
    o3d.visualization.draw_geometries([pcd1, pcd2, edge_points, planar_points, corresponding_edges_lines_list, corresponding_planar_patches_list], window_name="Two Point Clouds")


def test_correspondence_edges_single_scan(pc0, pc1, scan_num=63):
    scan0 = find_feature_points_in_point_cloud(pc0, scan_num)
    scan1 = find_feature_points_in_point_cloud(pc1, scan_num)

    corresponding_edges_lines = find_corresponding_edges_lines(scan1.edge_points, pc0)

    # Create PointCloud objects
    scan0_pcd = create_point_cloud(scan0.scan_points, color=[0.8, 0.8, 0.8])  # Light gray color
    scan1_pcd = create_point_cloud(scan1.scan_points, color=[0.6, 0.6, 0.6])  # Gray color

    scan0_colors = np.full(scan0.scan_points.shape, 0.8)
    corresponding_edges_lines_indices = []
    edges_points_from_corr = []
    for edge_corr in corresponding_edges_lines:
        corresponding_edges_lines_indices.append(edge_corr.e1_ind_in_scan)
        corresponding_edges_lines_indices.append(edge_corr.e2_ind_in_scan)
        edges_points_from_corr.append(edge_corr.point)
    print("Found {} corr".format(len(corresponding_edges_lines_indices)))

    corresponding_edges_lines_indices = np.array(corresponding_edges_lines_indices)
    edges_points_from_corr = np.array(edges_points_from_corr)

    scan0_colors[corresponding_edges_lines_indices] = [0., 1., 0.]
    scan0_pcd.colors = o3d.utility.Vector3dVector(scan0_colors)

    scan1_colors = np.full(scan1.scan_points.shape, 0.8)
    scan1_colors[scan1.edge_points_indices] = [0.8, 0., 0.]
    scan1_pcd.colors = o3d.utility.Vector3dVector(scan1_colors)

    arrows1 = create_arrows_between_points(scan0.scan_points[corresponding_edges_lines_indices[::2]], edges_points_from_corr, arrow_radius=0.005, arrow_color=[0, 1, 0])
    arrows2 = create_arrows_between_points(scan0.scan_points[corresponding_edges_lines_indices[1::2]], edges_points_from_corr, arrow_radius=0.005, arrow_color=[0, 1, 0])

    o3d.visualization.draw_geometries([scan0_pcd, scan1_pcd] + arrows1 + arrows2, window_name="Two Point Clouds")


# def test_correspondence_edges_single_scan_in_whole_pc(pc0, pc1, scan_num=63):
#     scan1 = find_feature_points_in_point_cloud(pc1, scan_num)
#
#     corresponding_edges_lines = find_corresponding_edges_lines(scan1.edge_points, pc0)
#
#     corresponding_edges_lines_indices = []
#     edges_points_from_corr = []
#     for edge_corr in corresponding_edges_lines:
#
#         corresponding_edges_lines_indices.append(pc0.scan_start[edge_corr.e1_scan] + edge_corr.e1_ind_in_scan)
#         corresponding_edges_lines_indices.append(pc0.scan_start[edge_corr.e2_scan] + edge_corr.e2_ind_in_scan)
#         edges_points_from_corr.append(edge_corr.point)
#     print("Found {} corr".format(len(corresponding_edges_lines_indices)))
#
#     pcd0_colors = np.full(pc0.points.shape, 0.8)
#     corresponding_edges_lines_indices = np.array(corresponding_edges_lines_indices)
#     edges_points_from_corr = np.array(edges_points_from_corr)
#
#     # Create PointCloud objects
#     pcd0 = create_point_cloud(pc0.points, color=[0.5, 0.5, 0.5])  # Light gray color
#     scan1_pcd = create_point_cloud(scan1.scan_points, color=[0.8, 0.8, 0.8])  # Gray color
#
#     pcd0_colors[corresponding_edges_lines_indices] = [0., 0., 1.]
#     pcd0.colors = o3d.utility.Vector3dVector(pcd0_colors)
#
#     scan1_colors = np.full(scan1.scan_points.shape, 0.8)
#     scan1_colors[scan1.edge_points_indices] = [1., 0., 0.]
#     scan1_pcd.colors = o3d.utility.Vector3dVector(scan1_colors)
#
#     arrows1 = create_arrows_between_points(pc0.points[corresponding_edges_lines_indices[::2]], edges_points_from_corr, arrow_radius=0.005, arrow_color=[0, 1, 0])
#     arrows2 = create_arrows_between_points(pc0.points[corresponding_edges_lines_indices[1::2]], edges_points_from_corr, arrow_radius=0.005, arrow_color=[0, 1, 0])
#     o3d.visualization.draw_geometries([pcd0, scan1_pcd] + arrows1 + arrows2, window_name="Two Point Clouds")
#     # o3d.visualization.draw_geometries([pcd0, scan1_pcd], window_name="Two Point Clouds")


def test_correspondence_features_single_scan_in_whole_pc(pc0, pc1, scan_num=63):

    # Find feature points in scan1 and prints the information
    scan1 = find_feature_points_in_point_cloud(pc1, scan_num)

    print("Edge points:")
    print(scan1.edge_points)
    print(" Edge points Indices(rel to scan):")
    print(scan1.edge_points_indices)
    # print(" C vals:")
    # print(scan1.edges_c_vals)

    print("Planar points:")
    print(scan1.planar_points)
    print(" Planar points Indices(rel to scan):")
    print(scan1.planar_points_indices)
    # print(" C vals:")
    # print(scan1.planar_c_vals)

    # Find correspondences in pc0

    # Edges
    corresponding_edges_lines = find_corresponding_edges_lines(scan1.edge_points, pc0)
    corresponding_edges_lines_indices = []
    edges_points_from_corr = []
    print("Found {} Edges corr".format(len(corresponding_edges_lines)))
    for edge_corr in corresponding_edges_lines:
        e1_pc_ind = pc0.scan_start[edge_corr.e1_scan] + edge_corr.e1_ind_in_scan
        e2_pc_ind = pc0.scan_start[edge_corr.e2_scan] + edge_corr.e2_ind_in_scan
        corresponding_edges_lines_indices.append(e1_pc_ind)
        corresponding_edges_lines_indices.append(e2_pc_ind)
        edges_points_from_corr.append(edge_corr.point)
        print("Point {} matches to {}, {}".format(edge_corr.point, e1_pc_ind, e2_pc_ind))
    corresponding_edges_lines_indices = np.array(corresponding_edges_lines_indices)
    edges_points_from_corr = np.array(edges_points_from_corr)

    # Planars
    corresponding_planar_patches_indices = []
    planar_points_from_corr = []
    corresponding_planar_patches = find_corresponding_planar_patches(scan1.planar_points, pc0)
    print("Found {} Planar corr".format(len(corresponding_planar_patches)))
    for planar_corr in corresponding_planar_patches:
        e1_pc_ind = pc0.scan_start[planar_corr.e1_scan] + planar_corr.e1_ind_in_scan
        e2_pc_ind = pc0.scan_start[planar_corr.e2_scan] + planar_corr.e2_ind_in_scan
        e3_pc_ind = pc0.scan_start[planar_corr.e3_scan] + planar_corr.e3_ind_in_scan
        corresponding_planar_patches_indices.append(e1_pc_ind)
        corresponding_planar_patches_indices.append(e2_pc_ind)
        corresponding_planar_patches_indices.append(e3_pc_ind)
        print("Point {} matches to {}, {}, {}".format(planar_corr.point, e1_pc_ind, e2_pc_ind, e3_pc_ind))
        planar_points_from_corr.append(planar_corr.point)
    corresponding_planar_patches_indices = np.array(corresponding_planar_patches_indices)
    planar_points_from_corr = np.array(planar_points_from_corr)

    pcd0 = create_point_cloud(pc0.points, color=[0.5, 0.5, 0.5])  # Light gray color
    pcd0_colors = np.full(pc0.points.shape, 0.8)
    pcd0_colors[corresponding_edges_lines_indices] = [0.9, 0.1, 0.1]   # Light Red
    pcd0_colors[corresponding_planar_patches_indices] = [0.1, 0.1, 0.9]  # Light Blue
    pcd0.colors = o3d.utility.Vector3dVector(pcd0_colors)

    # [0.9, 0.1, 0.1],  # Light Red
    # [0.1, 0.9, 0.1],  # Light Green
    # [0.1, 0.1, 0.9],  # Light Blue
    scan1_pcd = create_point_cloud(scan1.scan_points, color=[0.8, 0.8, 0.8])  # Gray color
    scan1_colors = np.full(scan1.scan_points.shape, 0.8)
    scan1_colors[scan1.edge_points_indices] = [1., 0., 0.]
    scan1_colors[scan1.planar_points_indices] = [0., 0., 0.1]
    scan1_pcd.colors = o3d.utility.Vector3dVector(scan1_colors)

    arrows1 = create_arrows_between_points(pc0.points[corresponding_edges_lines_indices[::2]], edges_points_from_corr, arrow_radius=0.0005, arrow_color=[0.8, 0.1, 0.1])
    arrows2 = create_arrows_between_points(pc0.points[corresponding_edges_lines_indices[1::2]], edges_points_from_corr, arrow_radius=0.0005, arrow_color=[0.8, 0.1, 0.1])

    arrows3 = create_arrows_between_points(pc0.points[corresponding_planar_patches_indices[::3]], planar_points_from_corr, arrow_radius=0.0005, arrow_color=[0.1, 0.1, 0.9])
    arrows4 = create_arrows_between_points(pc0.points[corresponding_planar_patches_indices[1::3]], planar_points_from_corr, arrow_radius=0.0005, arrow_color=[0.1, 0.1, 0.9])
    arrows5 = create_arrows_between_points(pc0.points[corresponding_planar_patches_indices[2::3]], planar_points_from_corr, arrow_radius=0.0005, arrow_color=[0.1, 0.1, 0.9])

    o3d.visualization.draw_geometries([pcd0, scan1_pcd] + arrows1 + arrows2 + arrows3 + arrows4 + arrows5, window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0, scan1_pcd], window_name="Two Point Clouds")


def test_correspondence_edges_single_scan_in_whole_pc(pc0, pc1, num_corr_to_take=None, scan_num=None):
    scans0 = find_feature_points_in_point_cloud(pc0)
    pc0.set_scans_obj_lst(scans0)

    # Find feature points in scan1 and prints the information
    scan1 = find_feature_points_in_point_cloud(pc1, scan_num)

    if scan_num is not None:
        print("Edges points:")
        for i in range(len(scan1.edge_points)):
            print("{}: {}".format(scan1.edge_points_indices[i], scan1.edge_points[i]))
    # print(" C vals:")
    # print(scan1.planar_c_vals)

    # Find correspondences in pc0

    # Planars
    corresponding_edges_lines_indices = []
    edge_points_from_corr = []
    corresponding_edges_lines = find_corresponding_edges_lines(scan1, pc0)

    if scan_num is not None:
        print()
        print("Edges Corr:")
        for edge_corr in corresponding_edges_lines:
            point_pc_ind = edge_corr.point_ind_in_scan
            e1_pc_ind = pc0.scan_start[edge_corr.e1_scan] + edge_corr.e1_ind_in_scan
            e2_pc_ind = pc0.scan_start[edge_corr.e2_scan] + edge_corr.e2_ind_in_scan
            print("{} {} --> ".format(edge_corr.point, point_pc_ind), end="")
            print("{} {}".format(e1_pc_ind, edge_corr.e1), end=" | ")
            print("{} {}".format(e2_pc_ind, edge_corr.e2), end=" | ")
            print()

        print()

    for i, edge_corr in enumerate(corresponding_edges_lines):
        if num_corr_to_take is None or i == num_corr_to_take:
            e1_pc_ind = pc0.scan_start[edge_corr.e1_scan] + edge_corr.e1_ind_in_scan
            e2_pc_ind = pc0.scan_start[edge_corr.e2_scan] + edge_corr.e2_ind_in_scan
            corresponding_edges_lines_indices.append(e1_pc_ind)
            corresponding_edges_lines_indices.append(e2_pc_ind)
            print(
                "Point {} matches to {}, {}".format(edge_corr.point_ind_in_scan, e1_pc_ind, e2_pc_ind))
            edge_points_from_corr.append(edge_corr.point)

    print("Found {} Edges corr".format(len(corresponding_edges_lines_indices)))
    corresponding_edges_lines_indices = np.array(corresponding_edges_lines_indices)
    edge_points_from_corr = np.array(edge_points_from_corr)

    pcd0 = create_point_cloud(pc0.points, color=[0.5, 0.5, 0.5])  # Light gray color
    pcd0_colors = np.full(pc0.points.shape, 0.8)
    if len(corresponding_edges_lines_indices) > 0:
        pcd0_colors[corresponding_edges_lines_indices] = [0.9, 0.1, 0.1]  # Light Red
    pcd0.colors = o3d.utility.Vector3dVector(pcd0_colors)

    # [0.9, 0.1, 0.1],  # Light Red
    # [0.1, 0.9, 0.1],  # Light Green
    # [0.1, 0.1, 0.9],  # Light Blue
    scan1_pcd = create_point_cloud(scan1.scan_points, color=[0.8, 0.8, 0.8])  # Gray color
    scan1_colors = np.full(scan1.scan_points.shape, 0.8)

    if num_corr_to_take is not None:
        scan1_colors[scan1.edge_points_indices[num_corr_to_take]] = [1., 0., 0.]
    else:
        scan1_colors[scan1.edge_points_indices] = [1., 0., 0.]
    scan1_pcd.colors = o3d.utility.Vector3dVector(scan1_colors)

    lines1 = create_lines_between_points(pc0.points[corresponding_edges_lines_indices[::2]], edge_points_from_corr,
                                         line_color=[0, 1, 0])
    lines2 = create_lines_between_points(pc0.points[corresponding_edges_lines_indices[1::2]],
                                         edge_points_from_corr, line_color=[0, 1, 0])

    o3d.visualization.draw_geometries([pcd0, scan1_pcd, lines1, lines2], window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0, scan1_pcd], window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0], window_name="Two Point Clouds")

    # corr_pcd = create_point_cloud(pc0.points[corresponding_planar_patches_indices],
    #                               color=[1., 0, 0])  # Light gray color
    # o3d.visualization.draw_geometries([corr_pcd, scan1_pcd, lines1, lines2], window_name="Two Point Clouds")


def test_correspondence_edges_all_scans_in_whole_pc(pc0, pc1, num_corr_to_take=None):
    scans0 = find_feature_points_in_point_cloud(pc0)
    pc0.set_scans_obj_lst(scans0)

    # Find feature points in scan1 and prints the information
    scan1 = find_feature_points_in_point_cloud(pc1)

    # Edges
    corresponding_edges_lines_indices = []
    edge_points_from_corr = []
    edge_points_indices = []
    corresponding_edges_lines = []

    for scan in scan1:
        corresponding_edges_lines += find_corresponding_edges_lines(scan, pc0)

    for i, edge_corr in enumerate(corresponding_edges_lines):
        if num_corr_to_take is None or i == num_corr_to_take:
            point_pc_ind = pc0.scan_start[edge_corr.point_scan] + edge_corr.point_ind_in_scan
            e1_pc_ind = pc0.scan_start[edge_corr.e1_scan] + edge_corr.e1_ind_in_scan
            e2_pc_ind = pc0.scan_start[edge_corr.e2_scan] + edge_corr.e2_ind_in_scan
            corresponding_edges_lines_indices.append(e1_pc_ind)
            corresponding_edges_lines_indices.append(e2_pc_ind)
            edge_points_indices.append(point_pc_ind)
            print("Point {} matches to {}, {}".format(edge_corr.point_ind_in_scan, e1_pc_ind, e2_pc_ind))
            edge_points_from_corr.append(edge_corr.point)

    print("Found {} Edges corr".format(len(corresponding_edges_lines_indices)))
    corresponding_edges_lines_indices = np.array(corresponding_edges_lines_indices)
    edge_points_from_corr = np.array(edge_points_from_corr)
    edge_points_indices = np.array(edge_points_indices)

    pcd0 = create_point_cloud(pc0.points, color=[0.5, 0.5, 0.5])  # Light gray color
    pcd0_colors = np.full(pc0.points.shape, 0.9)
    pcd0_colors[corresponding_edges_lines_indices] = [0.9, 0.1, 0.1]  # Light Red
    pcd0.colors = o3d.utility.Vector3dVector(pcd0_colors)

    # [0.9, 0.1, 0.1],  # Light Red
    # [0.1, 0.9, 0.1],  # Light Green
    # [0.1, 0.1, 0.9],  # Light Blue
    pcd1 = create_point_cloud(pc1.points, color=[0.8, 0.8, 0.8])  # Gray color
    pcd1_color = np.full(pc1.points.shape, 0.7)

    if num_corr_to_take is not None:
        pcd1_color[edge_points_indices[num_corr_to_take]] = [1., 0., 0.]
    else:
        pcd1_color[edge_points_indices] = [1., 0., 0.]
    pcd1.colors = o3d.utility.Vector3dVector(pcd1_color)

    lines1 = create_lines_between_points(pc0.points[corresponding_edges_lines_indices[::2]], edge_points_from_corr,
                                         line_color=[0, 1, 0])
    lines2 = create_lines_between_points(pc0.points[corresponding_edges_lines_indices[1::2]],
                                         edge_points_from_corr, line_color=[0, 1, 0])

    o3d.visualization.draw_geometries([pcd0, pcd1, lines1, lines2], window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0, scan1_pcd], window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0], window_name="Two Point Clouds")

    # corr_pcd = create_point_cloud(pc0.points[corresponding_planar_patches_indices],
    #                               color=[1., 0, 0])  # Light gray color
    # o3d.visualization.draw_geometries([corr_pcd, scan1_pcd, lines1, lines2], window_name="Two Point Clouds")


def test_correspondence_planars_all_scans_in_whole_pc(pc0, pc1, num_corr_to_take=None):
    scans0 = find_feature_points_in_point_cloud(pc0)
    if len(pc0.scans_obj_lst) == 0:
        pc0.set_scans_obj_lst(scans0)

    # Find feature points in scan1 and prints the information
    scan1 = find_feature_points_in_point_cloud(pc1)

    # Planars
    corresponding_planars_lines_indices = []
    planar_points_from_corr = []
    planar_points_indices = []
    corresponding_planars_lines = []

    for scan in scan1:
        corresponding_planars_lines += find_corresponding_planar_patches(scan, pc0)

    for i, planar_corr in enumerate(corresponding_planars_lines):
        if num_corr_to_take is None or i == num_corr_to_take:
            point_pc_ind = pc0.scan_start[planar_corr.point_scan] + planar_corr.point_ind_in_scan
            e1_pc_ind = pc0.scan_start[planar_corr.e1_scan] + planar_corr.e1_ind_in_scan
            e2_pc_ind = pc0.scan_start[planar_corr.e2_scan] + planar_corr.e2_ind_in_scan
            e3_pc_ind = pc0.scan_start[planar_corr.e3_scan] + planar_corr.e3_ind_in_scan
            corresponding_planars_lines_indices.append(e1_pc_ind)
            corresponding_planars_lines_indices.append(e2_pc_ind)
            corresponding_planars_lines_indices.append(e3_pc_ind)
            planar_points_indices.append(point_pc_ind)
            print("Point {} matches to {}, {}, {}".format(planar_corr.point_ind_in_scan, e1_pc_ind, e2_pc_ind, e3_pc_ind))
            planar_points_from_corr.append(planar_corr.point)

    print("Found {} Planars corr".format(len(corresponding_planars_lines_indices)))
    corresponding_planars_lines_indices = np.array(corresponding_planars_lines_indices)
    planar_points_from_corr = np.array(planar_points_from_corr)
    planar_points_indices = np.array(planar_points_indices)

    pcd0 = create_point_cloud(pc0.points, color=[0.5, 0.5, 0.5])  # Light gray color
    pcd0_colors = np.full(pc0.points.shape, 0.9)
    pcd0_colors[corresponding_planars_lines_indices] = [0.1, 0.1, 0.9]  # Light Red
    pcd0.colors = o3d.utility.Vector3dVector(pcd0_colors)

    # [0.9, 0.1, 0.1],  # Light Red
    # [0.1, 0.9, 0.1],  # Light Green
    # [0.1, 0.1, 0.9],  # Light Blue
    pcd1 = create_point_cloud(pc1.points, color=[0.8, 0.8, 0.8])  # Gray color
    pcd1_color = np.full(pc1.points.shape, 0.7)

    if num_corr_to_take is not None:
        pcd1_color[planar_points_indices[num_corr_to_take]] = [0., 0., 1.]
    else:
        pcd1_color[planar_points_indices] = [0., 0., 1.]
    pcd1.colors = o3d.utility.Vector3dVector(pcd1_color)

    lines1 = create_lines_between_points(pc0.points[corresponding_planars_lines_indices[::3]], planar_points_from_corr,
                                         line_color=[0, 1, 0])
    lines2 = create_lines_between_points(pc0.points[corresponding_planars_lines_indices[1::3]],
                                         planar_points_from_corr, line_color=[0, 1, 0])
    lines3 = create_lines_between_points(pc0.points[corresponding_planars_lines_indices[2::3]],
                                         planar_points_from_corr, line_color=[0, 1, 0])

    o3d.visualization.draw_geometries([pcd0, pcd1, lines1, lines2, lines3], window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0, scan1_pcd], window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0], window_name="Two Point Clouds")

    # corr_pcd = create_point_cloud(pc0.points[corresponding_planar_patches_indices],
    #                               color=[1., 0, 0])  # Light gray color
    # o3d.visualization.draw_geometries([corr_pcd, scan1_pcd, lines1, lines2], window_name="Two Point Clouds")


def test_correspondence_planars_single_scan_in_whole_pc(pc0, pc1, num_corr_to_take=None, scan_num=63):
    scans0 = find_feature_points_in_point_cloud(pc0)
    pc0.set_scans_obj_lst(scans0)

    # Find feature points in scan1 and prints the information
    scan1 = find_feature_points_in_point_cloud(pc1, scan_num)

    print("Planar points:")
    for i in range(len(scan1.planar_points)):
        print("{}: {}".format(scan1.planar_points_indices[i], scan1.planar_points[i]))
    # print(" C vals:")
    # print(scan1.planar_c_vals)

    # Find correspondences in pc0

    # Planars
    corresponding_planar_patches_indices = []
    planar_points_from_corr = []
    corresponding_planar_patches = find_corresponding_planar_patches(scan1, pc0)

    print()
    print("Planar Corr:")
    for planar_corr in corresponding_planar_patches:
        point_pc_ind = planar_corr.point_ind_in_scan
        e1_pc_ind = pc0.scan_start[planar_corr.e1_scan] + planar_corr.e1_ind_in_scan
        e2_pc_ind = pc0.scan_start[planar_corr.e2_scan] + planar_corr.e2_ind_in_scan
        e3_pc_ind = pc0.scan_start[planar_corr.e3_scan] + planar_corr.e3_ind_in_scan
        print("{} {} --> ".format(planar_corr.point, point_pc_ind), end="")
        print("{} {}".format(e1_pc_ind, planar_corr.e1), end=" | ")
        print("{} {}".format(e2_pc_ind, planar_corr.e2), end=" | ")
        print("{} {}".format(e3_pc_ind, planar_corr.e3), end="")
        print()

    print()
    for i, planar_corr in enumerate(corresponding_planar_patches):
        if num_corr_to_take is None or i == num_corr_to_take:
            e1_pc_ind = pc0.scan_start[planar_corr.e1_scan] + planar_corr.e1_ind_in_scan
            e2_pc_ind = pc0.scan_start[planar_corr.e2_scan] + planar_corr.e2_ind_in_scan
            e3_pc_ind = pc0.scan_start[planar_corr.e3_scan] + planar_corr.e3_ind_in_scan
            corresponding_planar_patches_indices.append(e1_pc_ind)
            corresponding_planar_patches_indices.append(e2_pc_ind)
            corresponding_planar_patches_indices.append(e3_pc_ind)
            print("Point {} matches to {}, {}, {}".format(planar_corr.point_ind_in_scan, e1_pc_ind, e2_pc_ind, e3_pc_ind))
            planar_points_from_corr.append(planar_corr.point)

    print("Found {} Planar corr".format(len(corresponding_planar_patches_indices)))
    corresponding_planar_patches_indices = np.array(corresponding_planar_patches_indices)
    planar_points_from_corr = np.array(planar_points_from_corr)

    pcd0 = create_point_cloud(pc0.points, color=[0.5, 0.5, 0.5])  # Light gray color
    pcd0_colors = np.full(pc0.points.shape, 0.8)
    pcd0_colors[corresponding_planar_patches_indices] = [0., 0., 1.]  # Light Blue
    pcd0.colors = o3d.utility.Vector3dVector(pcd0_colors)

    # [0.9, 0.1, 0.1],  # Light Red
    # [0.1, 0.9, 0.1],  # Light Green
    # [0.1, 0.1, 0.9],  # Light Blue
    scan1_pcd = create_point_cloud(scan1.scan_points, color=[0.8, 0.8, 0.8])  # Gray color
    scan1_colors = np.full(scan1.scan_points.shape, 0.8)
    scan1_colors[scan1.planar_points_indices[num_corr_to_take]] = [0., 0., 1.]
    scan1_pcd.colors = o3d.utility.Vector3dVector(scan1_colors)


    lines1 = create_lines_between_points(pc0.points[corresponding_planar_patches_indices[::3]], planar_points_from_corr,
                                         line_color=[1, 0, 0])
    lines2 = create_lines_between_points(pc0.points[corresponding_planar_patches_indices[1::3]],
                                         planar_points_from_corr, line_color=[1, 0, 0])
    lines3 = create_lines_between_points(pc0.points[corresponding_planar_patches_indices[2::3]],
                                         planar_points_from_corr, line_color=[1, 0, 0])

    # o3d.visualization.draw_geometries([pcd0, scan1_pcd, lines1, lines2, lines3], window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0, scan1_pcd], window_name="Two Point Clouds")
    # o3d.visualization.draw_geometries([pcd0], window_name="Two Point Clouds")

    corr_pcd = create_point_cloud(pc0.points[corresponding_planar_patches_indices], color=[1., 0, 0])  # Light gray color
    o3d.visualization.draw_geometries([corr_pcd, scan1_pcd, lines1, lines2, lines3], window_name="Two Point Clouds")


def test_odometry_toy(pc0, pc1, NUM_SCANS_TO_TAKE=1):
    scans = find_feature_points_in_point_cloud(pc1)[:NUM_SCANS_TO_TAKE]

    edge_points_lst = np.concatenate([s.edge_points for s in scans if s.edge_points != []], axis=0)
    planar_points_lst = np.concatenate([s.planar_points for s in scans if s.planar_points != []], axis=0)

    corresponding_edges_lines = find_corresponding_edges_lines(edge_points_lst, pc0)
    corresponding_planar_patches = find_corresponding_planar_patches(planar_points_lst, pc0)

    corresponding_planar_patches = np.array(corresponding_planar_patches)

    # Convert the list of points to a numpy array
    T_result_1_to_0 = optimize_test(corresponding_edges_lines, corresponding_planar_patches)
    print("1to0 vec: \n", T_result_1_to_0)
    print("0to1 vec: \n", inverse_pose(T_result_1_to_0))
    T_result_0_to_1_mat = get_inverse_Rt(T_result_1_to_0)
    T_result_1_to_0_mat = pose_to_mat(T_result_1_to_0[:3], T_result_1_to_0[3:])

    test_res(pc0, pc1, T_result_0_to_1_mat)


def create_point_cloud(points, color, indices=[], start=None, end=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if start != None and end != None:
        colors = np.full(points.shape, 0.8)  # RGB for gray
        colors[start:end] = [1, 0, 0]  # RGB for red
        pcd.colors = o3d.utility.Vector3dVector(colors)
    elif len(indices) > 0:
        colors = np.full(points.shape, 0.8)  # RGB for gray
        colors[indices] = [1, 0, 0]
        pcd.colors = o3d.utility.Vector3dVector(colors)
    else:
        pcd.paint_uniform_color(color)

    return pcd


def plot_2_pc(points1, points2):
    pcd1 = create_point_cloud(points1, color=[0, 1, 1])  # Cyan
    pcd2 = create_point_cloud(points2, color=[0.5, 0, 0])  # Magenta

    # Visualize both point clouds
    o3d.visualization.draw_geometries([pcd1, pcd2], window_name="")


def plot_3_pc(points1, points2, points3):
    pcd1 = create_point_cloud(points1, color=[0, 1, 1])  # cyan
    pcd2 = create_point_cloud(points2, color=[0, 1, 0])  # Green color for the second point cloud
    pcd3 = create_point_cloud(points3, color=[0.5, 0, 0])  # dark red

    # Visualize both point clouds
    o3d.visualization.draw_geometries([pcd1, pcd2, pcd3], window_name="")


def plot_pcd(points):
    o3d_pcd = create_point_cloud(points, color=[0.95, 0.95, 0.95])
    o3d.visualization.draw_geometries([o3d_pcd])


def test_plot2_pc(directory_path):
    pc0 = load_velodyne_single_pc(directory_path, "0")
    pc1 = load_velodyne_single_pc(directory_path, "1")
    plot_2_pc(pc0.points, pc1.points)


def change_pc(pc, T):
    R = T[:, :3]
    t = T[:, 3]

    new_pc = pc.points @ R.T + t.T
    return new_pc


def test_res(pc_first, pc_sec, T=None):
    moved_pc1_points = change_pc(pc_sec, T)

    plot_3_pc(pc_first.points, moved_pc1_points, pc_sec.points)
    # plot_2_pc(moved_pc0_points, pc1.points)


def get_gt(num):
    poses = pd.read_csv(GT_POSES, delimiter=' ', header=None)

    ground_truth = np.zeros((len(poses), 3, 4))
    # end = len(poses)
    end = num
    for i in range(1, end):
        ground_truth[i] = np.array(poses.iloc[i]).reshape((3, 4))

    return ground_truth[:, :, 3][:, 0], ground_truth[:, :, 3][:, 2]


def get_od(num):
    odometries = np.load('/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/odo50.npy')
    od_tv = left_cameras_trajectory(odometries)
    # od_tv = odometries[:, :, 3]
    od_tv = od_tv[:num, :]
    return -od_tv[:, 1], od_tv[:, 0]


def test_gt(end):
    poses = pd.read_csv(GT_POSES, delimiter=' ', header=None)

    ground_truth = np.zeros((len(poses), 3, 4))
    # end = len(poses)
    # end = 50
    for i in range(1, end):
        ground_truth[i] = np.array(poses.iloc[i]).reshape((3, 4))

    fig = plt.figure(figsize=(7, 6))
    traj = fig.add_subplot(111)
    traj.scatter(ground_truth[:, :, 3][:, 0], ground_truth[:, :, 3][:, 2])
    traj.set_xlabel('x')
    traj.set_ylabel('z')

    plt.show()


def test_odometry(pc_f, pc_sec):
    scans = find_feature_points_in_point_cloud(pc_f)
    pc_f.set_scans_obj_lst(scans)

    result_1_to_0_mat, Pk1_undistorted = lidar_odometry_between_2_pc_noTS(pc_f, pc_sec, True)
    test_res(pc_f, pc_sec, result_1_to_0_mat)


def test_saved_od():
    odometries = np.load('/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/odo50.npy')
    od_tv = left_cameras_trajectory(odometries)
    # od_tv = odometries[:, :, 3]
    od_tv = od_tv[:10, :]

    fig = plt.figure(figsize=(7, 6))
    traj = fig.add_subplot(111)
    traj.plot(od_tv[:, 1], od_tv[:, 0])
    traj.set_xlabel('x')
    traj.set_ylabel('z')

    plt.show()


def test_gt_od(num):
    gt_x, gt_y = get_gt(num)
    od_x, od_y = get_od(num)
    fig = plt.figure(figsize=(7, 6))
    traj = fig.add_subplot(111)
    traj.plot(od_x, od_y)
    traj.plot(gt_x, gt_y)

    traj.set_xlabel('x')
    traj.set_ylabel('z')

    plt.show()

#
# def test_toy_odometry(pc0, pc1):
#     pcd0 = convert_points_to_point_cloud(pc0)
#     pcd1 = convert_points_to_point_cloud(pc1)
#     T0to1 = lidar_odometry_whole_data_no_TS([pcd0, pcd1])[0]
#     test_res(pcd0, pcd1, T0to1)


def toy_data():
    scan_start = np.array([0, 20, 40, 60, 80])
    scan_end = np.array([19, 39, 59, 79, 99])
    pc0_toy_point_cloud = PointCloud(pc0_toy_points, pc0_toy_points[:, 2], scan_start, scan_end, None)

    pc1_toy_points = pc0_toy_points + np.array([.5, .5, 0])
    pc1_toy_point_cloud = PointCloud(pc1_toy_points, pc1_toy_points[:, 2], scan_start, scan_end, None)

    return pc0_toy_point_cloud, pc1_toy_point_cloud


def get_pc0_pc1(f=0,s=1):
    directory_path = r'/homes/maorat/Downloads/v_data'
    pc0 = load_velodyne_single_pc(directory_path, f)
    pc1 = load_velodyne_single_pc(directory_path, s)
    return pc0, pc1


def create_arrows_between_points(points1, points2, arrow_radius=0.005, arrow_color=[0, 1, 0]):
    assert points1.shape == points2.shape, "points1 and points2 must have the same shape."

    arrows = []
    for p1, p2 in zip(points1, points2):
        # Compute the direction and length of the arrow
        direction = p1 - p2
        length = np.linalg.norm(direction)

        if length > 0:  # Ensure the points are not the same
            # Normalize the direction
            direction_normalized = direction / length

            # Create the arrow mesh
            arrow = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=arrow_radius,
                cone_radius=2 * arrow_radius,
                cylinder_height=0.8 * length,
                cone_height=0.2 * length
            )

            # Rotate the arrow to align with the direction vector
            rotation = arrow.get_rotation_matrix_from_xyz([
                np.arctan2(direction_normalized[1], direction_normalized[0]),
                np.arccos(direction_normalized[2]),
                0
            ])
            arrow.rotate(rotation, center=(0, 0, 0))

            # Translate the arrow to the start point
            arrow.translate(p2)

            # Color the arrow
            arrow.paint_uniform_color(arrow_color)

            # Add the arrow to the list
            arrows.append(arrow)

    return arrows


def create_lines_between_points(points1, points2, line_color=[1, 0, 0]):
    assert points1.shape == points2.shape, "points1 and points2 must have the same shape."

    # Combine points into a single array for the LineSet
    points = np.vstack((points1, points2))

    # Create line indices that connect each point in points1 to the corresponding point in points2
    lines = [[i, i + len(points1)] for i in range(len(points1))]

    # Create the LineSet object
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    # Color the lines
    colors = np.tile(line_color, (len(lines), 1))
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


def test_closing_points(scan, inds, window_side_size):
    for ind in inds:
        check_is_point_close_to_its_neighbors(scan, ind, window_side_size)


def test_c_vals_several_indices(scan, indices, window_side_size):
    all_diffs = []
    for ind in indices:
        summ_diff = test_diff_distribution(scan, ind, window_side_size)
        all_diffs.append(summ_diff)

    results = np.array(all_diffs).squeeze()  # Squeeze to remove single-dimensional entries
    show_bar_plot_all(indices, results)
    # show_separate_bar_plots(indices, results)


def test_diff_distribution_for_several_indices(scan, indices, window_side_size):
    all_diffs = []
    for ind in indices:
        summ_diff = test_diff_distribution(scan, ind, window_side_size)
        all_diffs.append(np.abs(summ_diff))

    results = np.array(all_diffs).squeeze()  # Squeeze to remove single-dimensional entries
    show_bar_plot_all(indices, results)
    # show_separate_bar_plots(indices, results)


def test_diff_ratio(scan, ind, window_side_size):
    start = max(ind - window_side_size, 0)
    end = min(ind + window_side_size + 1, len(scan))

    scan_points = scan[start:end]
    sum_differences_point_and_scan_points = np.sort(np.abs(np.sum(scan[ind] - scan_points, axis=0)))
    return sum_differences_point_and_scan_points[1] / sum_differences_point_and_scan_points[0]


def test_diff_ratio_for_several_indices(scan, indices, window_side_size):
    all_diffs = []
    for ind in indices:
        ratio = test_diff_ratio(scan, ind, window_side_size)
        print("{}) {}".format(ind, ratio))

    # results = np.array(all_diffs).squeeze()  # Squeeze to remove single-dimensional entries
    # plt.scatter(np.arange(len(results)), results)
    # plt.show()
    # show_separate_bar_plots(indices, results)


def show_bar_plot_all(indices, results):
    # Plot the results
    fig, ax = plt.subplots(figsize=(10, 6))
    bar_width = 0.2
    x = np.arange(len(indices))

    # Plot bars for each component
    ax.bar(x - bar_width, results[:, 0], width=bar_width, label='X')
    ax.bar(x, results[:, 1], width=bar_width, label='Y')
    ax.bar(x + bar_width, results[:, 2], width=bar_width, label='Z')

    # Add labels and title
    ax.set_xlabel('Index')
    ax.set_ylabel('Value')
    ax.set_title('Results of test_c_vals Function')
    ax.set_xticks(x)
    ax.set_xticklabels(indices)
    ax.legend()

    plt.show()


def show_separate_bar_plots(indices, results):
    # Plot the results in subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 18), sharex=True)

    # Titles for subplots
    titles = ['X Component', 'Y Component', 'Z Component']
    x = np.arange(len(indices))

    # Plot bars for each component in separate subplots
    for i in range(3):
        # axs[i].bar(indices, results[:, i], width=0.4)
        axs[i].scatter(x, results[:, i], label=titles[i])
        axs[i].set_ylabel('Value')
        axs[i].set_title(titles[i])

    # Add common labels
    plt.xlabel('Index')
    plt.suptitle('Results of test_c_vals Function')

    plt.show()


def test_diff_distribution(scan, ind, window_side_size):
    start = max(ind - window_side_size, 0)
    end = min(ind + window_side_size + 1, len(scan))

    scan_points = scan[start:end]
    cov_mat = scan_points.T @ scan_points
    eigen_val = np.linalg.eig(cov_mat)[0]
    return eigen_val


def test_mapping(pc_f, pc_s, pose_1_to0):
    pc_f.set_scans_obj_lst(find_feature_points_in_point_cloud(pc_f))
    pc_s.set_scans_obj_lst(find_feature_points_in_point_cloud(pc_s))

    mat_1to0 = pose_to_mat(pose_1_to0)
    initial_T = pose_to_mat(np.array([0, 0, 0, 0, 0, 0]))
    mapping([initial_T, mat_1to0], [pc_f, pc_s], show_map=True, print_msg=2)


def visualize_map_corr(world_map, P_k_in_map_initial, features_corresponding, feature_clr, feature_clr_darker, nearby_clr, default_clr=[0.8, 0.8, 0.8], edge=True):
    # Convert points to an Open3D point cloud
    map_pcd = o3d.geometry.PointCloud()
    map_pcd.points = o3d.utility.Vector3dVector(world_map.points)

    P_k_pcd = o3d.geometry.PointCloud()
    P_k_pcd_points = []

    # Initialize the colors array with the default color
    map_colors = np.full(world_map.points.shape, default_clr)  # Gray color by default

    # Loop through each feature object and set the corresponding colors
    for feature_obj in features_corresponding:
        # feature_point_index = feature_obj.nearby_points_indices
        P_k_pcd_points.append(feature_obj.point)
        map_colors[feature_obj.nearby_points_indices] = nearby_clr
        if edge:
            map_colors[feature_obj.nearby_edges_indices] = feature_clr_darker
        else:
            map_colors[feature_obj.nearby_planars_indices] = feature_clr_darker


    map_pcd.colors = o3d.utility.Vector3dVector(map_colors)

    P_k_pcd_points = np.array(P_k_pcd_points)
    P_k_pcd_colors = np.full(P_k_pcd_points.shape, feature_clr)
    P_k_pcd.points = o3d.utility.Vector3dVector(P_k_pcd_points)
    P_k_pcd.colors = o3d.utility.Vector3dVector(P_k_pcd_colors)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([map_pcd, P_k_pcd])


def test_map_corr(pc0, pc1, T_1_to_0_mat):
    scans = find_feature_points_in_point_cloud(pc1)
    pc1.set_scans_obj_lst(scans)

    world_map = Map(pc0.points)

    P_k_in_map_initial = pc1.apply_transformation(T_1_to_0_mat)
    plot_3_pc(pc0.points, P_k_in_map_initial.points, pc1.points)

    corresponding_edges_lines = find_edge_corr(world_map.points, P_k_in_map_initial.all_edges_pts,
                                               P_k_in_map_initial.all_edges_pts_indices_in_pc,
                                               radius=0.1,
                                               ratio_threshold=5)
    corresponding_planar_patches = find_plane_corr(world_map.points, P_k_in_map_initial.all_planar_pts,
                                                   P_k_in_map_initial.all_planar_pts_indices_in_pc,
                                                   radius=0.1,
                                                   ratio_threshold=3)

    # visualize_map_corr(pc0.points, corresponding_edges_lines, [1., 0. ,0.], [0, 1, 1], default_clr=[0.8, 0.8, 0.8])
    visualize_map_corr(pc0.points, corresponding_planar_patches , [0, 0, 1], [0, 1, 1], default_clr=[0.8, 0.8, 0.8])


def test_edges_corr(P_ks, T_1_to_0_mat):
    for i in range(len(P_ks)):
        scans = find_feature_points_in_point_cloud(P_ks[i])
        P_ks[i].set_scans_obj_lst(scans)

    world_map = Map(P_ks[0].points, P_ks[0].all_edges_pts, P_ks[0].all_edges_pts_indices_in_pc, P_ks[0].all_planar_pts, P_ks[0].all_planar_pts_indices_in_pc)

    P_k_in_map_initial = P_ks[1].apply_transformation(T_1_to_0_mat)
    # plot_3_pc(pc0.points, P_k_in_map_initial.points, pc1.points)

    corresponding_edges_lines = find_edge_corr(world_map, P_k_in_map_initial, radius=0.1, ratio_threshold=2)

    print("Mapping: Num Edges: {}".format(len(corresponding_edges_lines)))

    if len(corresponding_edges_lines) > 0:
        visualize_map_corr(world_map, P_k_in_map_initial, corresponding_edges_lines, [1., 0., 0.], [0.5, 0, 0], [0, 1, 1], default_clr=[0.8, 0.8, 0.8])


def test_voxeling(pc1, voxel_size=0.2):
    radius = 5.
    scans = find_feature_points_in_point_cloud(pc1)
    pc1.set_scans_obj_lst(scans)

    pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(pc1.all_edges_pts)
    pcd.points = o3d.utility.Vector3dVector(pc1.points)
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    kdtree = o3d.geometry.KDTreeFlann(down_pcd)

    [k, nearby_points_indices, _] = kdtree.search_radius_vector_3d(pc1.all_edges_pts[0], radius)
    print("Num nearby: {}".format(len(nearby_points_indices)))
    nearby_points = np.asarray(pcd.points)[nearby_points_indices, :]

    colors = np.full(pc1.points.shape, [0.8, 0.8, 0.8])  # Gray color by default
    colors[nearby_points_indices] = [0, 1, 1]  # Cyan
    colors[pc1.all_edges_pts_indices_in_pc[0]] = [1., 0, 0]

    # pcd.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([pcd])

    # Show nearby only
    new_points = np.vstack((pc1.all_edges_pts[0], nearby_points))
    new_colors = np.full(new_points.shape, [0.8, 0.8, 0.8])  # Gray color by default
    new_colors[0] = [1, 0, 0]
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(new_points)
    new_pcd.colors = o3d.utility.Vector3dVector(new_colors)
    o3d.visualization.draw_geometries([new_pcd])


def test_search_near(pc1, voxel_size=0.2):
    radius = 0.1
    scans = find_feature_points_in_point_cloud(pc1)
    pc1.set_scans_obj_lst(scans)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc1.points)
    # pcd.points = o3d.utility.Vector3dVector(pc1.points)
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    nearby_edges_indices_all = np.array([])
    for i, edge_pt in enumerate(pc1.all_edges_pts):
        [k, nearby_points_indices, _] = kdtree.search_radius_vector_3d(edge_pt, radius)
        print("Num nearby: {}".format(len(nearby_points_indices)))
        # nearby_points = np.asarray(pcd.points)[nearby_points_indices, :]
        nearby_edges_indices_all = np.hstack((nearby_edges_indices_all, nearby_points_indices))

    # start = 58
    # end = 59
    # edge_ind = 58
    # [k, nearby_points_indices, _] = kdtree.search_radius_vector_3d(pc1.all_edges_pts[edge_ind], radius)
    # print("Num nearby: {}".format(len(nearby_points_indices)))
    # nearby_points = np.asarray(pcd.points)[nearby_points_indices, :]
    # # nearby_edges_indices_all = np.hstack((nearby_edges_indices_all, nearby_points_indices))
    # nearby_edges_indices_all = nearby_points_indices

    nearby_edges_indices_all = nearby_edges_indices_all.astype(int)
    nearby_edges_indices_all = np.intersect1d(nearby_edges_indices_all, pc1.all_edges_pts_indices_in_pc)
    colors = np.full(pc1.points.shape, [0.8, 0.8, 0.8])  # Gray color by default
    colors[nearby_edges_indices_all] = [0, 1, 1]  # Cyan
    # colors[pc1.all_edges_pts_indices_in_pc[start:end]] = [1., 0, 0]
    # colors[pc1.all_edges_pts_indices_in_pc[edge_ind]] = [1., 0, 0]
    colors[pc1.all_edges_pts_indices_in_pc] = [1., 0, 0]

    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])

    # # Show nearby only
    # new_points = np.vstack((pc1.all_edges_pts[0], nearby_points))
    # new_colors = np.full(new_points.shape, [0.8, 0.8, 0.8])  # Gray color by default
    # new_colors[0] = [1, 0, 0]
    # new_pcd = o3d.geometry.PointCloud()
    # new_pcd.points = o3d.utility.Vector3dVector(new_points)
    # new_pcd.colors = o3d.utility.Vector3dVector(new_colors)
    # o3d.visualization.draw_geometries([new_pcd])

def test_plot_map_edge_corr():
    pass


def test_plane_corr(P_ks, T_1_to_0_mat):

    for i in range(len(P_ks)):
        scans = find_feature_points_in_point_cloud(P_ks[i])
        P_ks[i].set_scans_obj_lst(scans)

    world_map = Map(P_ks[0].points, P_ks[0].all_edges_pts, P_ks[0].all_edges_pts_indices_in_pc, P_ks[0].all_planar_pts,
                    P_ks[0].all_planar_pts_indices_in_pc)

    P_k_in_map_initial = P_ks[1].apply_transformation(T_1_to_0_mat)
    # plot_3_pc(pc0.points, P_k_in_map_initial.points, pc1.points)

    corresponding_planar_patches = find_plane_corr(world_map, P_k_in_map_initial, radius=0.1, ratio_threshold=2)

    print("Mapping: Num planars: {}".format(len(corresponding_planar_patches)))

    if len(corresponding_planar_patches) > 0:
        visualize_map_corr(world_map, P_k_in_map_initial, corresponding_planar_patches, [0., 0., 1], [1, 0, 1],
                           [0, 1, 1], default_clr=[0.8, 0.8, 0.8], edge=False)


def test_gt_first_pose(ind=0):
    poses = pd.read_csv(GT_POSES, delimiter=' ', header=None)

    # end = len(poses)
    # end = 50
    gt = np.array(poses.iloc[ind]).reshape((3, 4))
    print(mat_to_pose(gt))


def test_map_imrpoving(pc1, pc0, od, map):
    pc1_moved_od = change_pc(pc1, od)
    pc1_moved_map = change_pc(pc1, map)
    plot_3_pc(pc0.points, pc1_moved_map, pc1_moved_od)


def test_maps():
    print("Map")
    cam2map_final_path = '/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/map10.npy'
    cam2map_final = np.load(cam2map_final_path)
    test_poses(cam2map_final)

    # rel_od_path = '/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/rel_od10.npy'
    # rel_od = np.load(rel_od_path)
    # global_od = [rel_od[0]]
    # for i in range(len(rel_od)):
    #     last = global_od[-1]
    #     global_od.append(compose_transformations(last, rel_od[i]))

    # od_path = '/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/od10.npy'
    # od = np.load(od_path)
    # test_poses(od)
    print("Rel od")
    rel_od_path = '/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/rel_od10.npy'
    rel_od = np.load(rel_od_path)
    test_poses(rel_od)
    print()

    # plt.scatter(od[:, :, 3][:, 1], od[:, :, 3][:, 0])
    # plt.scatter(cam2map_final[:, :, 3][:, 1], cam2map_final[:, :, 3][:, 0])
    # plt.show()


def test_poses(matrices):
    poses = []
    for ind in range(11):
        pc0, pc9 = get_pc0_pc1(0, ind)
        # plot_2_pc(pc0.points, pc9.points)
        # test_res(pc0, pc9, cam2map_final[ind])
        print("{}) {}".format(ind, mat_to_pose(matrices[ind])[3:]))
        poses.append(mat_to_pose(matrices[ind])[3:])

    poses = np.array(poses)
    plt.scatter(poses[:, 1], poses[:, 0])
    plt.show()



if __name__ == '__main__':

###########################
######## GT ###############`
###########################
    # test_gt(500)
    # test_saved_od()
    # test_gt_od(10)
    # test_gt_first_pose(1)

###########################
####### Toy ###############
###########################
    # pc0_toy_point_cloud, pc1_toy_point_cloud = toy_data()
    # test_feature_points(pc0_toy_point_cloud, -1)
    # test_odometry_toy(pc0_toy_point_cloud, pc1_toy_point_cloud, -1)

    # show_edges_planar_pcd(pc0_toy)
    # test_plot2_pc(direcotry_path)

###########################
####### Real ##############
###########################
    pc0, pc1 = get_pc0_pc1()

    # test_KITTI_scans(pc0.points)
    # plot_pcd(pc0.points)
    # plot_2_pc(pc0.points, pc1.points)
    # color_a_scan_in_pc(pc1, 45)
    # color_a_scan_in_pc(pc1, 45, 1740, 1765)  # ~ 1750 Wall with an edge
    # color_part_of_a_scan(pc1, 45, win=5, e_p=1721)

    # color_a_scan_in_pc(pc1, 50)

    # color_a_scan_in_pc(pc1, 46, start_ind_in_scan=2300, end_ind_in_scan=2310, e_p=2305)
    # color_part_of_a_scan(pc1, 46, start_ind_in_scan=2000, end_ind_in_scan=2010, e_p=2005)
    # color_part_of_a_scan_2_pc(pc0, pc1, 45)

    # color_a_scan_in_pc(pc1, 50, start_ind_in_scan=1700, end_ind_in_scan=1710, e_p=1705)
    # color_part_of_a_scan(pc1, 50, win=5, p_p=1050)

###########################
##### Feature Points ######
###########################
    # test_feature_points(pc1)
    # test_feature_points(pc1, 48)
    # test_feature_points(pc1, 48)

    # scan_ind = 45
    # pc = pc0
    # p_p = "1796 1798 1976 2055 2131"
    # p_p = list(map(int, p_p.split()))
    # win = 5

    # test_feature_points(pc, scan_ind)
    # color_part_of_a_scan(pc, scan_ind, win=5, p_p=p_p)
    # color_part_of_a_scan(pc, scan_ind, start_ind_in_scan=1796, end_ind_in_scan=1798)
    # test_diff_distribution_for_several_indices(pc.scans_list[scan_ind], p_p, win)
    # test_diff_ratio_for_several_indices(pc.scans_list[scan_ind], p_p, win)
###########################
##### Correspondenes ######
###########################
    # test_feature_points_2pc(pc0, pc1)

    # test_correspondence_features(pc0, pc1)
    # test_correspondence_edges_single_scan(pc0, pc1, 45)
    # test_correspondence_edges_single_scan_in_whole_pc(pc0, pc1, 46)

    # test_correspondence_features_single_scan_in_whole_pc(pc0, pc1, None)
    # test_correspondence_edges_single_scan_in_whole_pc(pc0, pc1, 1, scan_num=20)
    # test_correspondence_edges_all_scans_in_whole_pc(pc0, pc1)
    # test_correspondence_planars_single_scan_in_whole_pc(pc0, pc1, None, scan_num=45)
    # test_correspondence_planars_all_scans_in_whole_pc(pc0, pc1)

    # plot_2_pc(pc0_toy_points, pc1_toy_points)
    # test_correspondence_edges(pc0_toy_point_cloud, pc1_toy_point_cloud)

###########################
####### Odometry ##########
###########################
    # test_odometry()
    # pose = np.array([-0.004262332063605034, -0.003687375212532842, 0.006821049458197604,0.4216724916835413,0.006114902783919981,-0.02313760848003912])
    # pose = np.array([-0.0006486884420419435, -0.02240999146768109, -0.003204126907454595, 0.6597904047657178, -0.0028312749771891325, 0.06745413111632526])
    # pose1to0 = np.array([-0.00166637,  0.01393997,  0.00359723, -0.85,  0.01255943, -0.07656728])
    pose1to0_od = np.array([0.00184494, 0.00907854, 0.00097846, 0.63369234, 0.01642826, 0.00962726])
    pose1to0_map = np.array([0.0008534827389256954,-0.0018528624363395561,0.004712494235048642,0.6559973466559963,-0.021954434493218725,-0.00768123050957483])

    # test_res(pc0, pc1, pose_to_mat(pose1to0_map))
    # velo2cam = np.array([[6.927964000000e-03, -9.999722000000e-01, -2.757829000000e-03, -2.457729000000e-02],
    #                     [-1.162982000000e-03, 2.749836000000e-03, -9.999955000000e-01, -6.127237000000e-02],
    #                     [9.999753000000e-01, 6.931141000000e-03, -1.143899000000e-03, -3.321029000000e-01]])
    # print(pose_to_mat(pose_1_to_0))
    # print(mat_to_pose(compose_transformations(velo2cam, pose_to_inverse_mat(pose_1_to_0))))

###########################
####### Mapping ##########
###########################
    # test_voxeling(pc1)
    # test_search_near(pc1)
    # test_edges_corr([pc0, pc1], pose_to_mat(pose1to0_od))
    # test_plane_corr([pc0, pc1], pose_to_mat(pose1to0_od))
    # test_mapping(pc0, pc1)

    od =  np.array([0.0010015417690943567,-0.0025269016111534055,0.005059975603542424,0.6390697790297146,-0.031330652448839015,-0.015176447276427604])
    map =   np.array([-0.0024222658717191624, -0.004849156521401527,0.005740356823095244,0.6840281955374403,-0.019958288054430555,-0.04975945038826015])
    # test_map_imrpoving(pc1, pc0, pose_to_mat(od), pose_to_mat(map))

###########################
####### Test 8-9 ##########
###########################
    pc_f, pc_s = get_pc0_pc1(0, 7)

    # color_a_scan_in_pc(pc_s, 45)
    # color_part_of_a_scan(pc_s, 45, win=5, p_p=[1746])
    # color_part_of_a_scan(pc_s, 45, start_ind_in_scan=1815, end_ind_in_scan=1820)  #1810-1840

    # test_feature_points(pc_f)
    # test_feature_points(pc_s)
    # test_feature_points(pc_0, 45)


    # test_correspondence_edges_all_scans_in_whole_pc(pc_f, pc_s)
    # test_correspondence_planars_all_scans_in_whole_pc(pc_f, pc_s)

    # test_odometry(pc_f, pc_s)
    pose_9to8 = np.array([0.0017789356210419413,-0.0012465324818731124,0.033960561872698296,0.4023576335412677,0.023296903819808423,0.01698072904637613])
    # test_mapping(pc_f, pc_s, pose_9to8)
    # test_res(pc7, pc8, pose_to_mat(pose_8_to_7))

    # test_maps()
    cam2map_final_path = '/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/map10.npy'
    cam2map_final = np.load(cam2map_final_path)
    # test_mapping(pc_f, pc_s, pose_9to8)
    test_res(pc_f, pc_s, cam2map_final[7])
