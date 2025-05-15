#! /homes/maorat/work/mefusionlib/venv/bin/python
from scipy.optimize import least_squares
import tqdm
from scipy.spatial.transform import Rotation as R

import open3d as o3d
import os
import numpy as np
import matplotlib.pyplot as plt

# import cv2

PCD_PATH = r'/homes/maorat/Downloads/v_data'
NUM_PCDS = 2
TS_PATH = "/homes/maorat/Downloads/timestamps_end.txt"
GT_POSES = "/homes/maorat/Downloads/LOAM/gt_00.txt"

import numpy as np

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
        [0.5, 0, 0]  # Dark Red
])
class MapEdgeCorr:
    def __init__(self, newPk_point, newPk_idx_in_pc, e1, e2, nearby_edges, nearby_edges_indices, nearby_points, nearby_points_indices):
        self.point = newPk_point
        self.point_idx = newPk_idx_in_pc
        self.e1 = e1
        self.e2 = e2
        self.nearby_points = nearby_points
        self.nearby_points_indices = nearby_points_indices
        self.nearby_edges = nearby_edges
        self.nearby_edges_indices = nearby_edges_indices


class MapPlaneCorr:
    def __init__(self, point, point_idx, e1, e2, e3, nearby_planars, nearby_planars_indices, nearby_points, nearby_points_indices):
        self.point = point
        self.point_idx = point_idx
        self.e1 = e1
        self.e2 = e2
        self.e3 = e3
        self.nearby_points = nearby_points
        self.nearby_points_indices = nearby_points_indices
        self.nearby_planars = nearby_planars
        self.nearby_planars_indices = nearby_planars_indices



class EdgeCorrespondence:
    def __init__(self, point,
                 e1, e1_dist,
                 e2, e2_dist,
                 point_ind_in_scan, e1_ind_in_scan, e2_ind_in_scan,
                 point_scan, e1_scan, e2_scan):
        self.point = point
        self.e1 = e1
        self.e1_dist = e1_dist
        self.e2 = e2
        self.e2_dist = e2_dist

        self.point_ind_in_scan = point_ind_in_scan
        self.e1_ind_in_scan = e1_ind_in_scan
        self.e2_ind_in_scan = e2_ind_in_scan

        self.point_scan = point_scan
        self.e1_scan = e1_scan
        self.e2_scan = e2_scan


class PlanarCorrespondence:
    def __init__(self, point, e1, e2, e3,
                 e1_dist, e2_dist, e3_dist,
                 point_ind_in_scan, e1_ind_in_scan, e2_ind_in_scan, e3_ind_in_scan,
                 point_scan, e1_scan, e2_scan, e3_scan):
        self.point = point
        self.e1 = e1
        self.e1_dist = e1_dist
        self.e2 = e2
        self.e2_dist = e2_dist
        self.e3 = e3
        self.e3_dist = e3_dist

        self.point_ind_in_scan = point_ind_in_scan
        self.e1_ind_in_scan = e1_ind_in_scan
        self.e2_ind_in_scan = e2_ind_in_scan
        self.e3_ind_in_scan = e3_ind_in_scan

        self.point_scan = point_scan
        self.e1_scan = e1_scan
        self.e2_scan = e2_scan
        self.e3_scan = e3_scan


class Scan:
    def __init__(self, scan_points, scan_ind, edge_points, edges_c_vals, planar_points, planar_c_vals, edge_points_indices, planar_points_indices):
        self.scan_points = scan_points
        self.scan_ind = scan_ind
        self.edge_points = edge_points
        self.edges_c_vals = edges_c_vals
        self.planar_points = planar_points
        self.planar_c_vals = planar_c_vals
        self.edge_points_indices = edge_points_indices  # Indices relative to scan
        self.planar_points_indices = planar_points_indices  # Indices relative to scan

    def apply_transformation(self, Rt):
        def transform_points(points, Rt):
            if len(points) == 0:
                return points
            new_points = points @ Rt[:, :3].T + Rt[:, 3].T
            return new_points

        new_scan_points = transform_points(self.scan_points, Rt)
        new_edge_points = transform_points(self.edge_points, Rt)
        new_planar_points = transform_points(self.planar_points, Rt)

        return Scan(
            scan_points=new_scan_points,
            scan_ind=self.scan_ind,
            edge_points=new_edge_points,
            edges_c_vals=self.edges_c_vals,
            planar_points=new_planar_points,
            planar_c_vals=self.planar_c_vals,
            edge_points_indices=self.edge_points_indices,
            planar_points_indices=self.planar_points_indices
        )


class Map:
    def __init__(self, points, edges_points, edge_points_indices, planar_points, planar_points_indices):
        self.num_point_cloud = 1
        self.points = points
        self.points_cloud_start = [0]
        self.points_cloud_end = [len(points) - 1]
        self.edges_points = edges_points
        self.edge_points_indices = edge_points_indices
        self.planar_points = planar_points
        self.planar_points_indices = planar_points_indices
        self.colors = [custom_colors[0] for _ in range(len(points))]

    def add_points(self, new_points, new_edge_points, new_edge_points_indices, new_planar_points, new_planar_points_indices):
        self.num_point_cloud += 1
        self.points_cloud_start = [self.points]
        self.points = np.vstack((self.points, new_points))
        self.points_cloud_end = [self.points - 1]

        self.edges_points = np.vstack((self.edges_points, new_edge_points))
        self.edge_points_indices = np.hstack((self.edge_points_indices, new_edge_points_indices + len(new_points)))
        self.planar_points = np.vstack((self.planar_points, new_planar_points))
        self.planar_points_indices = np.hstack((self.planar_points_indices, new_planar_points_indices + len(new_points)))
        color_index = (self.num_point_cloud - 1) % len(custom_colors)
        self.colors += [custom_colors[color_index] for _ in range(len(new_points))]

    def show_map(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(self.colors)
        o3d.visualization.draw_geometries([pcd], window_name="")


class PointCloud:
    def __init__(self, points, scans_ids, scan_start, scan_end, r):
        self.points = points
        self.scans_ids = scans_ids.astype('int')
        self.scan_start = scan_start
        self.scan_end = scan_end
        self.scans_list = [self.points[self.scan_start[i]:self.scan_end[i]] for i in range(len(self.scan_start))]
        self.r = r
        self.scans_obj_lst = []
        self.all_edges_pts = []
        self.all_planar_pts = []
        self.all_edges_pts_indices_in_pc = []
        self.all_planar_pts_indices_in_pc = []

    def point_ind_to_scan_ind(self, idx):
        return self.scans_ids[idx]

    def point_ind_in_scan_to_point_cloud_ind(self, idx, scan_ind):
        return self.scan_start[scan_ind] + idx

    def set_scans_obj_lst(self, scans_obj_lst):
        self.scans_obj_lst = scans_obj_lst

        for scan_obj in tqdm.tqdm(self.scans_obj_lst):
            if len(scan_obj.edge_points) > 0:
                self.all_edges_pts.append(scan_obj.edge_points)
                self.all_edges_pts_indices_in_pc.append(scan_obj.edge_points_indices + self.scan_start[scan_obj.scan_ind])
            if len(scan_obj.planar_points) > 0:
                self.all_planar_pts.append(scan_obj.planar_points)
                self.all_planar_pts_indices_in_pc.append(scan_obj.planar_points_indices + self.scan_start[scan_obj.scan_ind])

        # Concatenate all edge_points and planar_points into separate np arrays
        if len(self.all_edges_pts) > 0:
            self.all_edges_pts = np.concatenate(self.all_edges_pts, axis=0)
            self.all_edges_pts_indices_in_pc = np.concatenate(self.all_edges_pts_indices_in_pc, axis=0)
        else:
            self.all_edges_pts = np.array([])
            self.all_edges_pts_indices_in_pc = np.array([])

        if len(self.all_planar_pts) > 0:
            self.all_planar_pts = np.concatenate(self.all_planar_pts, axis=0)
            self.all_planar_pts_indices_in_pc = np.concatenate(self.all_planar_pts_indices_in_pc, axis=0)
        else:
            self.all_planar_pts = np.array([])
            self.all_planar_pts_indices_in_pc = np.array([])

    def apply_transformation(self, Rt):
        def transform_points(points, Rt):
            if len(points) == 0:
                return points
            new_points = points @ Rt[:, :3].T + Rt[:, 3].T
            return new_points

        new_points = transform_points(self.points, Rt)
        new_scan_obj_lst = [scan.apply_transformation(Rt) for scan in self.scans_obj_lst]

        new_point_cloud = PointCloud(new_points, self.scans_ids, self.scan_start, self.scan_end, self.r)
        new_point_cloud.set_scans_obj_lst(new_scan_obj_lst)
        return new_point_cloud

    def add_another_point_cloud_data(self, pc):
        self.points = np.vstack((self.points, pc.points))

        # Update scans_ids by offsetting with the current maximum scans_ids + 1
        max_scan_id = self.scans_ids.max() + 1
        self.scans_ids = np.hstack((self.scans_ids, pc.scans_ids + max_scan_id))

        # Adjust and append scan_start and scan_end
        offset = len(self.points) - len(pc.points)
        self.scan_start = np.hstack((self.scan_start, pc.scan_start + offset))
        self.scan_end = np.hstack((self.scan_end, pc.scan_end + offset))

        # Update scans_list
        new_scans_list = [self.points[start:end] for start, end in zip(pc.scan_start + offset, pc.scan_end + offset)]
        self.scans_list.extend(new_scans_list)

        # Append the additional data (edges, planar points, etc.)
        self.all_edges_pts = np.vstack((self.all_edges_pts, pc.all_edges_pts))
        self.all_planar_pts = np.vstack((self.all_planar_pts, pc.all_planar_pts))
        self.all_edges_pts_indices_in_pc = np.hstack(
            (self.all_edges_pts_indices_in_pc, pc.all_edges_pts_indices_in_pc + offset))
        self.all_planar_pts_indices_in_pc = np.hstack(
            (self.all_planar_pts_indices_in_pc, pc.all_planar_pts_indices_in_pc + offset))


def load_velodyne_single_pc(directory_path, ind):
    path = "00000{}.bin".format(ind)
    if ind >= 10:
        path = "0000{}.bin".format(ind)
    if ind >= 100:
        path = "000{}.bin".format(ind)
    if ind >= 1000:
        path = "00{}.bin".format(ind)

    file_path = os.path.join(directory_path, path)
    data = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    points = data[:, :3]  # Only x, y, z are needed
    r = data[:, 3]
    points, scan_start, scan_end = get_item_from_points(points)

    point_cloud = PointCloud(points[:, :3], points[:, 3], scan_start, scan_end, r)

    return point_cloud


def convert_points_to_point_cloud(points):
    points, scan_start, scan_end = get_item_from_points(points)
    point_cloud = PointCloud(points[:, :3], points[:, 3], scan_start, scan_end, None)
    return point_cloud


def load_all_velodyne_files(directory_path=PCD_PATH, start_frame=0, end_frame=NUM_PCDS):
    point_clouds = []

    for i in range(start_frame, end_frame + 1):
        point_cloud = load_velodyne_single_pc(directory_path, i)
        point_clouds.append(point_cloud)
    return point_clouds


def load_timestamps_ends(file_path=TS_PATH):
    with open(file_path, 'r') as file:
        timestamps = file.readlines()

    timestamps_canon = []
    for ts in timestamps:
        date_time_part, fractional_part = ts.split('.')
        seconds_with_fraction = date_time_part.split(':')[-1] + '.' + fractional_part
        timestamps_canon.append(float(seconds_with_fraction))

    time_deltas = [(timestamps_canon[i] - timestamps_canon[0]) for i in range(len(timestamps_canon))]
    time_deltas_array = np.array(time_deltas)

    return time_deltas_array


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


def get_item_from_points(points):
    return reorder_pcd(points)


def c_val(point, scan_points, ind, num_pts_to_look):
    start = max(ind - num_pts_to_look, 0)
    end = min(ind + num_pts_to_look + 1, len(scan_points))

    scan_points = scan_points[start:end]
    S_size = 2 * num_pts_to_look + 1
    pt_norm = np.linalg.norm(point)

    sum_differences_point_and_scan_points = np.sum(point - scan_points, axis=0)
    sum_differences_point_and_scan_points_norm = np.linalg.norm(sum_differences_point_and_scan_points)
    c = sum_differences_point_and_scan_points_norm / (S_size * pt_norm)
    return c


def find_feature_points(scan):
    # TODO: Testing
    NUM_PLANAR_PTS_TO_TAKE = 4
    NUM_EDGES_PTS_TO_TAKE = 1
    c_vals = [c_val(scan[i], scan[:i, i + 1:]) for i in range(len(scan))]
    increase_c_vals_sorted = sorted(c_vals)
    decrease_c_vals_sorted = sorted(c_vals, reverse=True)

    edges_points = decrease_c_vals_sorted[:NUM_PLANAR_PTS_TO_TAKE]
    planar_points = increase_c_vals_sorted[:NUM_EDGES_PTS_TO_TAKE]
    return edges_points, planar_points


def take_parts(lst, M):
    n = len(lst)
    part_size = n // 4
    part1 = lst[0:part_size][:M]
    part2 = lst[part_size:2 * part_size][:M]
    part3 = lst[2 * part_size:3 * part_size][:M]
    part4 = lst[3 * part_size:][:M]
    result = np.concatenate([part1, part2, part3, part4])
    return result


def filter_planar_points_with_dist_in_2_dim(scan, ind, window_side_size):
    start = max(ind - window_side_size, 0)
    end = min(ind + window_side_size + 1, len(scan))
    scan_window = scan[start:end]
    sum_differences_point_and_scan_points = np.sort(np.abs(np.sum(scan[ind] - scan_window, axis=0)))
    threshold = 0.01
    dist_cond = np.sum(sum_differences_point_and_scan_points < threshold)
    return dist_cond == 2


def check_is_point_close_to_its_neighbors(scan, ind, window_side_size, thresh=0.02):
    if ind == 0 or ind == len(scan) - 1:
        return False

    start = max(ind - window_side_size, 0)
    end = min(ind + window_side_size + 1, len(scan))

    scan_window = scan[start:end]

    squared_distances = np.sum(np.diff(scan_window, axis=0) ** 2, axis=1)

    mean_dist = np.mean(squared_distances)

    diff_dist = min(mean_dist, 0.005)  # Allow 10% dist from mean
    mean_cond = squared_distances[window_side_size - 1] < diff_dist and squared_distances[window_side_size] < diff_dist
    var_cond  = np.all(squared_distances < thresh)

    return mean_cond and var_cond


def is_point_an_edge(i, scan):
    precision = 0.01  # 2cm
    edge_max_thresh = 1.1 * precision
    edge_min_thresh = 0.7 * precision
    planar_max_thresh = 0.04 * precision

    NUM_POINTS_TO_LOOK_AT = 5
    point_c_val = c_val(scan[i], scan, i, NUM_POINTS_TO_LOOK_AT)
    # print(point_c_val)
    passed = point_c_val >= edge_min_thresh and point_c_val <= edge_max_thresh

    return passed


def is_point_a_planar(i, scan):
    precision = 0.01  # 2cm
    edge_max_thresh = 1.1 * precision
    edge_min_thresh = 0.7 * precision
    planar_max_thresh = 0.04 * precision

    NUM_POINTS_TO_LOOK_AT = 5
    point_c_val = c_val(scan[i], scan, i, NUM_POINTS_TO_LOOK_AT)

    return point_c_val <= planar_max_thresh


def find_feature_points_in_scan_part(scan_part):
    # TODO: Testing
    window_side_size = 5
    window_size = 2 * window_side_size + 1

    if len(scan_part) < window_size:
        empty_n_array = np.array([])
        return empty_n_array, empty_n_array, empty_n_array, empty_n_array, empty_n_array, empty_n_array

    NUM_PLANAR_PTS_TO_TAKE = 4
    NUM_EDGES_PTS_TO_TAKE = 2
    c_vals = [c_val(scan_part[i], scan_part, i, window_side_size) for i in range(len(scan_part))]
    c_vals = np.array(c_vals)
    # sorted_c_vals = np.sort(c_vals)

    increase_c_vals_sorted_indices = np.argsort(c_vals)
    decrease_c_vals_sorted_indices = increase_c_vals_sorted_indices[::-1]

    close_to_its_neighbors_increase_indices = np.array([check_is_point_close_to_its_neighbors(scan_part, i, window_side_size) for i in increase_c_vals_sorted_indices])
    close_to_its_neighbors_decrease_indices = np.array([check_is_point_close_to_its_neighbors(scan_part, i, window_side_size) for i in decrease_c_vals_sorted_indices])

    in_window_increase_indices = (increase_c_vals_sorted_indices > window_side_size) & (increase_c_vals_sorted_indices < len(scan_part) - window_side_size)
    in_window_decrease_indices = (decrease_c_vals_sorted_indices > window_side_size) & (decrease_c_vals_sorted_indices < len(scan_part) - window_side_size)

    precision = 0.01  # 2cm
    edge_max_thresh = 1.5 * precision
    edge_min_thresh = 0.7 * precision
    planar_max_thresh = 0.04 * precision

    good_c_val_edges = (c_vals[decrease_c_vals_sorted_indices] <= edge_max_thresh) & (c_vals[decrease_c_vals_sorted_indices] >= edge_min_thresh)
    good_c_val_planars = c_vals[increase_c_vals_sorted_indices] <= planar_max_thresh

    # Don't take ground points - Lidar height is 1.73
    high_planar_pts = scan_part[increase_c_vals_sorted_indices][:, 2] >= -1.5
    high_planar_pts[0] = True  # Take the first one anyway

    high_edges_pts = scan_part[decrease_c_vals_sorted_indices][:, 2] >= -1.3
    high_edges_pts[0] = True

    # good_distribution_planar_pts = np.array([filter_planar_points_with_dist_in_2_dim(scan_part, i, window_side_size) for i in increase_c_vals_sorted_indices])
    # good_increase_indices = in_window_increase_indices & close_to_its_neighbors_increase_indices & good_c_val_planars & good_distribution_planar_pts

    good_increase_indices = in_window_increase_indices & close_to_its_neighbors_increase_indices & good_c_val_planars & high_planar_pts
    good_decrease_indices = in_window_decrease_indices & close_to_its_neighbors_decrease_indices & good_c_val_edges & high_edges_pts

    increase_middle_pts = increase_c_vals_sorted_indices[good_increase_indices]
    decrease_middle_pts = decrease_c_vals_sorted_indices[good_decrease_indices]

    edge_points_indices = decrease_middle_pts[:NUM_EDGES_PTS_TO_TAKE]
    planar_points_indices = increase_middle_pts[:NUM_PLANAR_PTS_TO_TAKE]

    planar_points = scan_part[planar_points_indices]
    edge_points = scan_part[edge_points_indices]

    return edge_points, planar_points, edge_points_indices, planar_points_indices, c_vals[edge_points_indices], c_vals[planar_points_indices]


def show_pcd(points, colors=None):
    # Create an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # Only x, y, z
    if colors:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd], window_name="Point Cloud with Scan ID Colors")


def find_feature_points_in_scan(scan):
    part_size = len(scan) // 6
    part_size = max(part_size, 50)  # Todo: consider divide it to 6 only if the scan size is higher than some number.

    edge_points_total, planar_points_total, edge_points_indices_total, planar_points_indices_total = [], [], [], []
    edges_c_vals_total, planar_c_vals_total = [], []

    for i in range(0, len(scan), part_size):
        start = i
        end = min(start + part_size, len(scan))
        scan_part = scan[start:end]
        edge_points, planar_points, edge_points_indices, planar_points_indices, edges_c_vals, planar_c_vals = find_feature_points_in_scan_part(scan_part)
        edge_points_indices += start
        planar_points_indices += start

        if len(edge_points) > 0:
            edge_points_total.append(edge_points)
        if len(edge_points_indices) > 0:
            edge_points_indices_total.append(edge_points_indices)
        if len(planar_points) > 0:
            planar_points_total.append(planar_points)
        if len(planar_points_indices) > 0:
            planar_points_indices_total.append(planar_points_indices)
        if len(edges_c_vals) > 0:
            edges_c_vals_total.append(edges_c_vals)
        if len(planar_c_vals) > 0:
            planar_c_vals_total.append(planar_c_vals)

    if len(edge_points_total) > 0:
        edge_points_total = np.concatenate(edge_points_total, axis=0)
    else:
        edge_points_total = np.array([])
    if len(edge_points_indices_total) > 0:
        edge_points_indices_total = np.hstack(edge_points_indices_total)
    else:
        edge_points_indices_total = np.array([])
    if len(planar_points_total) > 0:
        planar_points_total = np.concatenate(planar_points_total, axis=0)
    else:
        planar_points_total = np.array([])
    if len(planar_points_indices_total) > 0:
        planar_points_indices_total = np.hstack(planar_points_indices_total)
    else:
        planar_points_indices_total = np.array([])
    if len(edges_c_vals_total) > 0:
        edges_c_vals_total = np.concatenate(edges_c_vals_total, axis=0)
    else:
        edges_c_vals_total = np.array([])
    if len(planar_c_vals_total) > 0:
        planar_c_vals_total = np.hstack(planar_c_vals_total)
    else:
        planar_c_vals_total = np.array([])

    # c_vals = [c_val(scan[i], np.delete(scan, i, axis=0)) for i in range(len(scan))]
    # stop = 2

    return edge_points_total, planar_points_total, edge_points_indices_total, planar_points_indices_total, edges_c_vals_total, planar_c_vals_total


def compose_transformations(first_ex_mat, second_ex_mat):
    """
    Compute the composition of two extrinsic camera matrices.
    first_cam_mat : A -> B
    second_cam_mat : B -> C
    composed mat : A -> C
    """
    # [R2 | t2] @ [ R1 | t1] = [R2 @ R1 | R2 @ t1 + t2]
    #             [000 | 1 ]
    hom1 = np.append(first_ex_mat, [np.array([0, 0, 0, 1])], axis=0)
    return second_ex_mat @ hom1


def pose_to_mat(pose):
    """
    Compute rotation matrix from rotation vector and returns it as
    numpy array [R|t] with shape 3 X 4
    :param R_vec: Rotation vector
    :param t_vec: Translation vector
    """
    rotation = R.from_rotvec(pose[:3])
    R_mat = rotation.as_matrix()
    # R_mat, _ = cv2.Rodrigues(R_vec)
    return np.hstack((R_mat, pose[3:].reshape(-1, 1)))


def rot_mat_to_euler_angles(R_mat):
    import math
    sy = math.sqrt(R_mat[0, 0] * R_mat[0, 0] + R_mat[1, 0] * R_mat[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R_mat[2, 1], R_mat[2, 2])
        y = math.atan2(-R_mat[2, 0], sy)
        z = math.atan2(R_mat[1, 0], R_mat[0, 0])
    else:
        x = math.atan2(-R_mat[1, 2], R_mat[1, 1])
        y = math.atan2(-R_mat[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def mat_to_rodriguez(translation):
    R_mat, t_vec = translation[:, :3], translation[:, 3]
    loc = - R_mat.T @ t_vec
    euler_angles = rot_mat_to_euler_angles(R_mat)

    return np.hstack((euler_angles, loc))


def mat_to_pose(Rt):
    R_mat, t_vec = Rt[:, :3], Rt[:, 3]
    euler_angles = rot_mat_to_euler_angles(R_mat)

    return np.hstack((euler_angles, t_vec))

def mat_to_inverse_pose(Rt):
    R_mat, t_vec = Rt[:, :3], Rt[:, 3]
    euler_angles = rot_mat_to_euler_angles(R_mat.T)

    return np.hstack((euler_angles, -R_mat.T @ t_vec))


def m2c_mat_to_vec(m2c_mat):
    R_mat, t_vec = m2c_mat[:, :3], m2c_mat[:, 3]
    loc = - R_mat.T @ t_vec
    euler_angles = rot_mat_to_euler_angles(R_mat)

    return np.hstack((euler_angles, loc))


def c2m_mat_to_vec(c2m):
    R_mat, t_vec = c2m[:, :3], c2m[:, 3]
    loc = t_vec
    euler_angles = rot_mat_to_euler_angles(R_mat)

    return np.hstack((euler_angles, loc))


def rotation_matrix_to_axis_angle(R_matrix):
    """
    Converts a rotation matrix to axis-angle representation.

    Parameters:
    R_matrix (numpy.ndarray): A 3x3 rotation matrix.

    Returns:
    axis (numpy.ndarray): The axis of rotation (unit vector).
    angle (float): The rotation angle in radians.
    """
    # Create a Rotation object from the rotation matrix
    rotation = R.from_matrix(R_matrix)

    # Convert to axis-angle (rotation vector)
    axis_angle = rotation.as_rotvec()

    # Extract the angle and axis
    angle = np.linalg.norm(axis_angle)
    axis = axis_angle / angle if angle != 0 else np.zeros(3)

    return axis, angle


def vector_to_pose_matrix(axis_angle_vector, translation):
    from scipy.spatial.transform import Rotation as R
    """
    Converts a vector in R^3 (axis-angle) and a translation vector to a 4x4 pose matrix.

    Parameters:
    axis_angle_vector (numpy.ndarray): The axis-angle vector (3 elements).
    translation (numpy.ndarray): The translation vector (3 elements).

    Returns:
    pose_matrix (numpy.ndarray): The 4x4 transformation matrix.
    """
    # Calculate the angle (magnitude of the axis-angle vector)
    angle = np.linalg.norm(axis_angle_vector)

    # Normalize the axis-angle vector to get the axis
    axis = axis_angle_vector / angle if angle != 0 else np.zeros(3)

    # Create the rotation object from the rotation vector (axis * angle)
    rotation = R.from_rotvec(axis_angle_vector)

    # Convert to rotation matrix
    R_matrix = rotation.as_matrix()

    # Create the 4x4 pose matrix
    pose_matrix = np.eye(4)  # Start with an identity matrix
    pose_matrix[:3, :3] = R_matrix  # Insert the rotation matrix
    pose_matrix[:3, 3] = translation  # Insert the translation vector

    return pose_matrix


def RT_mul_p(RT, p):
    """
    :param RT: 3x4 matrix
    :param p:
    :return:
    """
    return RT[:, :3] * p + RT[:, 3]


def vec_to_mat(vec):
    pose_mat = vector_to_pose_matrix(vec[:3], vec[3:])
    return pose_mat


def fix_points_in_time(points, T_1to0, points_times_stamps, time_interval):
    transformed_points = []

    def calc_partial_T(T, time_stamp, time_interval):
        factor = time_stamp / time_interval
        return factor * T

    for i in range(len(points)):
        partial_T = calc_partial_T(T_1to0, points_times_stamps[i], time_interval)
        partial_RT = vec_to_mat(partial_T)
        transformed_point = RT_mul_p(partial_RT, points[i])
        transformed_points.append(transformed_point)

    return transformed_points


def search_nearest_neighbor(point, points, num_nearest_neighbors=1):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    kdtree = o3d.geometry.KDTreeFlann(pcd)

    [k, idx, dist] = kdtree.search_knn_vector_3d(point, num_nearest_neighbors)

    nearest_point = points[idx[:num_nearest_neighbors]]
    return nearest_point, idx[:num_nearest_neighbors], dist[:num_nearest_neighbors]


def get_best_point_edge(scan_id, pc0, point):
    chosen_point, choose_point_ind_in_scan, chosen_point_dist, chosen_scan_id = None, None, None, None

    if scan_id == 0:
        if len(pc0.scans_obj_lst[scan_id + 1].edge_points) > 0:
            chosen_point, choose_point_ind_in_edge_points, chosen_point_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id + 1].edge_points)
            chosen_point, choose_point_ind_in_edge_points, chosen_point_dist = chosen_point[0], choose_point_ind_in_edge_points[0], chosen_point_dist[0]
            chosen_scan_id = scan_id + 1
            choose_point_ind_in_scan = pc0.scans_obj_lst[scan_id + 1].edge_points_indices[choose_point_ind_in_edge_points]
    elif scan_id == len(pc0.scans_list) - 1:
        if len(pc0.scans_obj_lst[scan_id - 1].edge_points) > 0:
            chosen_point, choose_point_ind_in_edge_points, chosen_point_dist = search_nearest_neighbor(point,pc0.scans_obj_lst[scan_id - 1].edge_points)
            chosen_point, choose_point_ind_in_edge_points, chosen_point_dist = chosen_point[0], choose_point_ind_in_edge_points[0], chosen_point_dist[0]
            chosen_scan_id = scan_id - 1
            choose_point_ind_in_scan = pc0.scans_obj_lst[scan_id - 1].edge_points_indices[choose_point_ind_in_edge_points]
    else:
        l_dist, m_dist = None, None
        if len(pc0.scans_obj_lst[scan_id - 1].edge_points) > 0:
            l_nn, l_idx_in_edge_points, l_dist = search_nearest_neighbor(point,pc0.scans_obj_lst[scan_id - 1].edge_points)
            l_nn, l_idx_in_edge_points, l_dist = l_nn[0], l_idx_in_edge_points[0], l_dist[0]
            l_idx_in_scan = pc0.scans_obj_lst[scan_id - 1].edge_points_indices[l_idx_in_edge_points]
        if len(pc0.scans_obj_lst[scan_id + 1].edge_points) > 0:
            m_nn, m_idx_in_edge_points, m_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id + 1].edge_points)
            m_nn, m_idx_in_edge_points, m_dist = m_nn[0], m_idx_in_edge_points[0], m_dist[0]
            m_idx_in_scan = pc0.scans_obj_lst[scan_id + 1].edge_points_indices[m_idx_in_edge_points]

        if l_dist and m_dist:
            if l_dist < m_dist:
                chosen_point = l_nn
                choose_point_ind_in_scan = l_idx_in_scan
                chosen_point_dist = l_dist
                chosen_scan_id = scan_id - 1
            else:
                chosen_point = m_nn
                choose_point_ind_in_scan = m_idx_in_scan
                chosen_point_dist = m_dist
                chosen_scan_id = scan_id + 1
        elif l_dist:
            chosen_point = l_nn
            choose_point_ind_in_scan = l_idx_in_scan
            chosen_point_dist = l_dist
            chosen_scan_id = scan_id - 1
        elif m_dist:
            chosen_point = m_nn
            choose_point_ind_in_scan = m_idx_in_scan
            chosen_point_dist = m_dist
            chosen_scan_id = scan_id + 1

    return chosen_point, choose_point_ind_in_scan, chosen_point_dist, chosen_scan_id


def get_best_point_planar(scan_id, pc0, point):
    chosen_point, choose_point_ind_in_scan, chosen_point_dist, chosen_scan_id = None, None, None, None
    if scan_id == 0:
        if len(pc0.scans_obj_lst[scan_id+1].planar_points) > 0:
            chosen_point, choose_point_ind_in_planar_points, chosen_point_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id+1].planar_points)
            chosen_point, choose_point_ind_in_planar_points, chosen_point_dist = chosen_point[0], choose_point_ind_in_planar_points[0], chosen_point_dist[0]
            chosen_scan_id = scan_id + 1
            choose_point_ind_in_scan = pc0.scans_obj_lst[scan_id+1].planar_points_indices[choose_point_ind_in_planar_points]
    elif scan_id == len(pc0.scans_list) - 1:
        if len(pc0.scans_obj_lst[scan_id-1].planar_points) > 0:
            chosen_point, choose_point_ind_in_planar_points, chosen_point_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id-1].planar_points)
            chosen_point, choose_point_ind_in_planar_points, chosen_point_dist = chosen_point[0], choose_point_ind_in_planar_points[0], chosen_point_dist[0]
            chosen_scan_id = scan_id - 1
            choose_point_ind_in_scan = pc0.scans_obj_lst[scan_id-1].planar_points_indices[choose_point_ind_in_planar_points]
    else:
        l_dist, m_dist = None, None
        if len(pc0.scans_obj_lst[scan_id-1].planar_points) > 0:
            l_nn, l_idx_in_planar_points, l_dist = search_nearest_neighbor(point,pc0.scans_obj_lst[scan_id-1].planar_points)
            l_nn, l_idx_in_planar_points, l_dist = l_nn[0], l_idx_in_planar_points[0], l_dist[0]
            l_idx_in_scan = pc0.scans_obj_lst[scan_id-1].planar_points_indices[l_idx_in_planar_points]
        if len(pc0.scans_obj_lst[scan_id+1].planar_points) > 0:
            m_nn, m_idx_in_planar_points, m_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id+1].planar_points)
            m_nn, m_idx_in_planar_points, m_dist = m_nn[0], m_idx_in_planar_points[0], m_dist[0]
            m_idx_in_scan = pc0.scans_obj_lst[scan_id+1].planar_points_indices[m_idx_in_planar_points]

        if l_dist and m_dist:
            if l_dist < m_dist:
                chosen_point = l_nn
                choose_point_ind_in_scan = l_idx_in_scan
                chosen_point_dist = l_dist
                chosen_scan_id = scan_id - 1
            else:
                chosen_point = m_nn
                choose_point_ind_in_scan = m_idx_in_scan
                chosen_point_dist = m_dist
                chosen_scan_id = scan_id + 1
        elif l_dist:
            chosen_point = l_nn
            choose_point_ind_in_scan = l_idx_in_scan
            chosen_point_dist = l_dist
            chosen_scan_id = scan_id - 1
        elif m_dist:
            chosen_point = m_nn
            choose_point_ind_in_scan = m_idx_in_scan
            chosen_point_dist = m_dist
            chosen_scan_id = scan_id + 1

    return chosen_point, choose_point_ind_in_scan, chosen_point_dist, chosen_scan_id


def get_best_point(scan_id, pc0, point):
    if scan_id == 0:
        chosen_point, choose_point_ind_in_scan, chosen_point_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id + 1].scan_points)
        chosen_point, choose_point_ind_in_scan, chosen_point_dist = chosen_point[0], choose_point_ind_in_scan[0], chosen_point_dist[0]
        chosen_scan_id = scan_id + 1
    elif scan_id == len(pc0.scans_list) - 1:
        chosen_point, choose_point_ind_in_scan, chosen_point_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id - 1].scan_points)
        chosen_point, choose_point_ind_in_scan, chosen_point_dist = chosen_point[0], choose_point_ind_in_scan[0], chosen_point_dist[0]
        chosen_scan_id = scan_id - 1
    else:
        l_nn, l_idx_in_scan, l_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id - 1].scan_points)
        l_nn, l_idx_in_scan, l_dist = l_nn[0], l_idx_in_scan[0], l_dist[0]

        m_nn, m_idx_in_scan, m_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[scan_id + 1].scan_points)
        m_nn, m_idx_in_scan, m_dist = m_nn[0], m_idx_in_scan[0], m_dist[0]

        if l_dist < m_dist:
            chosen_point = l_nn
            choose_point_ind_in_scan = l_idx_in_scan
            chosen_point_dist = l_dist
            chosen_scan_id = scan_id - 1
        else:
            chosen_point = m_nn
            choose_point_ind_in_scan = m_idx_in_scan
            chosen_point_dist = m_dist
            chosen_scan_id = scan_id + 1


    return chosen_point, choose_point_ind_in_scan, chosen_point_dist, chosen_scan_id


def find_corresponding_edges_line_for_point(point_ind_in_scan, point_scan_ind, point, pc0):
    # j,l,m are notations from the paper
    j_nn, j_idx_all_edges_lst, j_dist = search_nearest_neighbor(point, pc0.all_edges_pts)
    j_nn, j_idx_all_edges_lst, j_dist = j_nn[0], j_idx_all_edges_lst[0], j_dist[0]  # j_idx it's the idx in the all_edges_pts lst
    j_idx_in_pc = pc0.all_edges_pts_indices_in_pc[j_idx_all_edges_lst]
    j_scan_id = pc0.point_ind_to_scan_ind(j_idx_in_pc)
    j_ind_in_scan = j_idx_in_pc - pc0.scan_start[j_scan_id]

    chosen_point, chosen_point_ind_in_scan, chosen_point_dist, chosen_scan_id = get_best_point_edge(j_scan_id, pc0, point)
    if chosen_point is None:
        return False, None

    dist_thresh = 1.
    passed_dist_thresh = j_dist < dist_thresh and chosen_point_dist < dist_thresh

    if not passed_dist_thresh:
        return False, None


    edge_corr = EdgeCorrespondence(point,
                                   j_nn, j_dist,
                                   chosen_point, chosen_point_dist,
                                   point_ind_in_scan, j_ind_in_scan, chosen_point_ind_in_scan,
                                   point_scan_ind, j_scan_id, chosen_scan_id)

    return True, edge_corr


def find_corresponding_edges_lines(scan, pc0):
    # print("    Find Corresponding Edges Lines...")
    corresponding_edges_lines = []
    for i, point in enumerate(scan.edge_points):
        point_ind_in_scan = scan.edge_points_indices[i]
        success, edge_corr = find_corresponding_edges_line_for_point(point_ind_in_scan, scan.scan_ind, point, pc0)
        if success:
            corresponding_edges_lines.append(edge_corr)

    return corresponding_edges_lines


def find_corresponding_planar_patch_for_point(point_ind_in_scan, point_scan_ind, point, pc0):
    # j,l,m are notations from the paper
    j_nn, j_nn_idx_all_planar_lst, j_dist = search_nearest_neighbor(point, pc0.all_planar_pts)
    j_nn, j_nn_idx_all_planar_lst, j_dist = j_nn[0], j_nn_idx_all_planar_lst[0], j_dist[0]
    j_idx_in_pc = pc0.all_planar_pts_indices_in_pc[j_nn_idx_all_planar_lst]
    j_scan_id = pc0.point_ind_to_scan_ind(j_idx_in_pc)
    j_ind_in_scan = j_idx_in_pc - pc0.scan_start[j_scan_id]


    two_nn, two_nn_idx_in_planar_points, two_dist = search_nearest_neighbor(point, pc0.scans_obj_lst[j_scan_id].planar_points, num_nearest_neighbors=2)  # best point in the same scan
    l_nn, l_nn_idx_in_planar_points, l_dist = two_nn[0], two_nn_idx_in_planar_points[0], two_dist[0]
    l_scan_id = j_scan_id
    l_ind_in_scan = pc0.scans_obj_lst[j_scan_id].planar_points_indices[l_nn_idx_in_planar_points]

    if np.array_equal(l_nn, j_nn):
        if len(two_nn) == 2:
            l_nn, l_nn_idx_in_planar_points, l_dist = two_nn[1], two_nn_idx_in_planar_points[1], two_dist[1]
            l_ind_in_scan = pc0.scans_obj_lst[j_scan_id].planar_points_indices[l_nn_idx_in_planar_points]
        else:
            return False, None

    chosen_point, chosen_point_ind_in_scan, chosen_point_dist, chosen_scan_id = get_best_point_planar(j_scan_id, pc0, point)
    if chosen_point is None:
        return False, None

    dist_thresh = 1.
    passed_dist_thresh = j_dist < dist_thresh and l_dist < dist_thresh and chosen_point_dist < dist_thresh

    if not passed_dist_thresh:
        return False, None

    planar_corr = PlanarCorrespondence(point, j_nn, l_nn, chosen_point,
                                       j_dist, l_dist, chosen_point_dist,
                                       point_ind_in_scan, j_ind_in_scan, l_ind_in_scan, chosen_point_ind_in_scan,
                                       point_scan_ind, j_scan_id, l_scan_id, chosen_scan_id)

    return True, planar_corr


def find_corresponding_planar_patches(scan1, pc0):
    corresponding_planar_patches = []
    for i, point in enumerate(scan1.planar_points):
        point_ind_in_scan = scan1.planar_points_indices[i]
        success, planar_corr = find_corresponding_planar_patch_for_point(point_ind_in_scan, scan1.scan_ind, point, pc0)
        if success:
            corresponding_planar_patches.append(planar_corr)

    return corresponding_planar_patches


def pose_to_inverse_mat(pose):
    pose_mat = pose_to_mat(pose)
    inverse_pose_mat = get_inverse_Rt(pose_mat)
    return inverse_pose_mat


def inverse_pose(pose):
    pose_mat = pose_to_mat(pose)
    inverse_pose_mat = get_inverse_Rt(pose_mat)
    return mat_to_pose(inverse_pose_mat)


def inverse_pose_c2m(cam2map):
    inverse_T_mat = get_inverse_Rt(cam2map)
    return m2c_mat_to_vec(inverse_T_mat)


def get_inverse_Rt(Rt):
    R = Rt[:, :3]
    t = Rt[:, 3]
    inverse_translation = -R.T @ t
    inverse_T_mat = np.hstack((R.T, inverse_translation.reshape(-1, 1)))
    return inverse_T_mat


def find_feature_points_in_point_cloud(point_cloud, scan_num=None):
    # print("    Find features points in point cloud...")
    scans = []

    if scan_num:
        scan_points = point_cloud.scans_list[scan_num]
        edge_points, planar_points, edge_points_indices, planar_points_indices, edges_c_vals, planar_c_vals = find_feature_points_in_scan(
            scan_points)
        scan = Scan(scan_points, scan_num, edge_points, edges_c_vals, planar_points, planar_c_vals, edge_points_indices, planar_points_indices)
        return scan

    for i, scan_points in enumerate(point_cloud.scans_list):
        edge_points, planar_points, edge_points_indices, planar_points_indices, edges_c_vals, planar_c_vals = find_feature_points_in_scan(scan_points)
        scan = Scan(scan_points, i, edge_points, edges_c_vals, planar_points, planar_c_vals, edge_points_indices, planar_points_indices)
        scans.append(scan)

    return scans


def formatted_pose(pose, np=True):
    if not np:
        return f"[{','.join(map(str, pose))}]"
    else:
        return f"np.array([{','.join(map(str, pose))}])"


def lidar_odometry_between_2_pc_noTS(previous_undistorted_pc, Pk1, print_msg=False):

    if print_msg: print("  Lidar Odometry Between 2 Points...")

    scans = find_feature_points_in_point_cloud(Pk1)
    Pk1.set_scans_obj_lst(scans)

    corresponding_edges_lines = []
    corresponding_planar_patches = []

    if print_msg: print("    Searching for features points in point cloud...")
    for scan_ind, scan in tqdm.tqdm(enumerate(scans), total=len(scans)):
        corresponding_edges_lines += find_corresponding_edges_lines(scan, previous_undistorted_pc)
        corresponding_planar_patches += find_corresponding_planar_patches(scan, previous_undistorted_pc)

    if print_msg:
        print("Odometry: Num Edge constraints: ", len(corresponding_edges_lines))
        print("Odometry: Num Planar constraints: ", len(corresponding_planar_patches))

    result_1_to_0_pose = optimize(corresponding_edges_lines, corresponding_planar_patches)
    if print_msg:
        print("Odometry: 1to0: ", formatted_pose(result_1_to_0_pose))

    return pose_to_mat(result_1_to_0_pose), Pk1


def relative_camera_pos(ex_cam_mat):
    """
    Finds and returns the Camera position at the "world" d2_points
    :param ex_cam_mat: [Rotation mat|translation vec]
    """
    # R = extrinsic_camera_mat[:, :3]
    # t = extrinsic_camera_mat[:, 3]
    return -1 * ex_cam_mat[:, :3].T @ ex_cam_mat[:, 3]


def convert_trans_from_rel_to_global(T_arr):
    relative_T_arr = []
    last = T_arr[0]

    for t in T_arr:
        last = compose_transformations(last, t)
        relative_T_arr.append(last)

    return relative_T_arr


def left_cameras_trajectory(relative_T_arr):
    """
    Computes the left cameras 3d positions relative to the starting position
    :param T_arr: relative to first camera transformations array
    :return: numpy array with dimension num T_arr X 3
    """
    relative_cameras_pos_arr = []
    for t in relative_T_arr:
        relative_cameras_pos_arr.append(relative_camera_pos(t))
    return np.array(relative_cameras_pos_arr)


def compare_left_cam_2d_trajectory_to_ground_truth(od_horiz, od_ver, gt_horiz, gt_ver):
    """
    Compare the left cameras relative 2d positions to the ground truth
    """
    fig = plt.figure()

    ax = fig.add_subplot()
    ax.set_title(f"Left cameras 2d trajectory of {len(od_horiz)} frames_ind (ground truth - cyan)")
    ax.scatter(od_horiz, od_ver, s=1, c='red')
    ax.scatter( gt_horiz, gt_ver, s=1, c='cyan')
    ax.set_xticks([i * 0.1 for i in range(int(min(gt_horiz) * 10), int(max(gt_horiz) * 10) + 1)])

    fig.savefig("/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/Compare Left cameras 2d trajectory.png")
    plt.close(fig)


def plot_2d_traj(x, z, title):
    """
    Compare the left cameras relative 2d positions to the ground truth
    """
    import matplotlib.ticker as ticker
    fig = plt.figure()

    ax = fig.add_subplot()
    ax.set_title(f"LOAM trajectory of {len(x)} frames_ind")
    ax.scatter(x, z, s=1, c='red')

    ax.set_xlim(min(x) - 0.1, max(x) + 0.1)
    ax.xaxis.set_major_locator(plt.MultipleLocator(0.1))

    fig.savefig("/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/LeftCamTraj{}.png".format(title))
    plt.close(fig)


def lidar_odometry_whole_data_no_TS(pcds, print_msg=False):
    if print_msg: print("Lidar Odometry...")

    initial_T = pose_to_mat(np.array([0, 0, 0, 0, 0, 0]))
    cur2prev_matrices = [initial_T]
    previous_undistorted_pc = pcds[0]

    scans = find_feature_points_in_point_cloud(previous_undistorted_pc)
    previous_undistorted_pc.set_scans_obj_lst(scans)
    P_ks = [previous_undistorted_pc]

    for i in tqdm.tqdm(range(1, len(pcds))):
        if print_msg: print(" Working on Pc{}...".format(i))

        result_1_to_0_mat, Pk1_undistorted = lidar_odometry_between_2_pc_noTS(previous_undistorted_pc, pcds[i], print_msg)

        cur2prev_matrices.append(result_1_to_0_mat)
        P_ks.append(Pk1_undistorted)
        previous_undistorted_pc = Pk1_undistorted

    return cur2prev_matrices, P_ks


def residuals_edge(Rt, edge_corr):
    new_point = Rt[:, :3] @ edge_corr.point + Rt[:, 3]

    a = new_point - edge_corr.e1
    b = new_point - edge_corr.e2
    c = edge_corr.e1 - edge_corr.e2
    r_line = np.linalg.norm(np.cross(a, b)) / np.linalg.norm(c)

    return r_line


def residuals_plane(Rt, plane_corr):
    new_point = Rt[:, :3] @ plane_corr.point + Rt[:, 3]
    a = plane_corr.e1 - plane_corr.e2
    b = plane_corr.e1 - plane_corr.e3
    c = new_point - plane_corr.e1

    ab_cross = np.cross(a, b)
    abcross_dot_c = np.dot(ab_cross, c)

    return np.linalg.norm(abcross_dot_c) / np.linalg.norm(ab_cross)


def residuals(pose, edges_corr, planes_corr):
    all_residuals = []
    Rt = pose_to_mat(pose)

    for edge_corr in edges_corr:
        edge_r = residuals_edge(Rt, edge_corr)
        all_residuals.append(edge_r)

    for plane_corr in planes_corr:
        plane_r = residuals_plane(Rt, plane_corr)
        all_residuals.append(plane_r)

    # print(all_residuals)
    return np.array(all_residuals)


def optimize(edge_corr, plane_corr):
    from scipy.optimize import least_squares

    pose_initial = np.array([0, 0, 0, 0, 0, 0])
    # Optimize using Levenberg-Marquardt
    # result = least_squares(residuals, pose_initial, args=(edge_corr, plane_corr), xtol=1e-10, loss='huber')
    result = least_squares(residuals, pose_initial, args=(edge_corr, plane_corr), method="lm")
    return result.x


def optimize_test(edge_corr, plane_corr):
    from scipy.optimize import least_squares

    # Initial guess for the pose
    pose_initial = np.array([0, 0, 0, 0, 0, 0])
    # Optimize using Levenberg-Marquardt
    # result = least_squares(residuals, pose_initial, args=(edge_corr, plane_corr), method="lm")
    result = least_squares(residuals, pose_initial, args=(edge_corr, plane_corr), method="lm")
    return result.x


def undistort_pcds(pcds):
    return np.array([])


def check_if_eig_values_represent_an_edge(eigenvalues, ratio_threshold=10):
    one_dominant = eigenvalues[2] >= ratio_threshold * eigenvalues[1]
    return one_dominant


def check_if_eig_values_represent_a_plane(eigenvalues, ratio_threshold):
    # Check for two dominant eigenvalues
    two_dominant = eigenvalues[2] >= ratio_threshold * eigenvalues[0] and eigenvalues[1] >= 0.5 * ratio_threshold * \
                   eigenvalues[0]
    return two_dominant


def find_edge_corr(map, newPk, radius=0.2, ratio_threshold=5):
    # Convert points to an Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(map.points)
    kdtree = o3d.geometry.KDTreeFlann(pcd)
    corresponding_edges_lines = []

    for i, Pk_edge_point in enumerate(newPk.all_edges_pts):

        [k, nearby_points_indices, _] = kdtree.search_radius_vector_3d(Pk_edge_point, radius)

        nearby_edges_indices = (np.intersect1d(nearby_points_indices, map.edge_points_indices)).astype(int)
        nearby_edges = map.points[nearby_edges_indices, :]

        if len(nearby_edges_indices) >= 2:

            centroid = np.mean(nearby_edges, axis=0)

            covariance_matrix = np.cov(nearby_edges - centroid, rowvar=False)

            eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)

            if check_if_eig_values_represent_an_edge(eigenvalues, ratio_threshold=ratio_threshold):
                edge_corr = MapEdgeCorr(Pk_edge_point, newPk.all_edges_pts_indices_in_pc[i], centroid, centroid + eigenvectors[2], nearby_edges, nearby_edges_indices, map.points[nearby_points_indices, :], nearby_points_indices)  # Assuming they are ordered
                corresponding_edges_lines.append(edge_corr)

    return corresponding_edges_lines


def find_plane_corr(map, newPk, radius=0.1, ratio_threshold=3):
    # Convert points to an Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(map.points)
    kdtree = o3d.geometry.KDTreeFlann(pcd)


    corresponding_planar_patches = []

    for i, newPk_plane_point in enumerate(newPk.all_planar_pts):
        [k, nearby_points_indices, _] = kdtree.search_radius_vector_3d(newPk_plane_point, radius)
        nearby_points = np.asarray(pcd.points)[nearby_points_indices, :]

        nearby_planar_indices = np.intersect1d(nearby_points_indices, map.planar_points_indices)
        nearby_planars = map.points[nearby_planar_indices, :]

        if len(nearby_planar_indices) >= 3:
            centroid = np.mean(nearby_points, axis=0)
            covariance_matrix = np.cov(nearby_points - centroid, rowvar=False)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)

            if check_if_eig_values_represent_a_plane(eigenvalues, ratio_threshold=ratio_threshold):
                plane_corr = MapPlaneCorr(newPk_plane_point, newPk.all_planar_pts_indices_in_pc[i], centroid, centroid + eigenvectors[2], centroid + eigenvectors[1], nearby_planars, nearby_planar_indices, map.points[nearby_points_indices, :], nearby_points_indices)  # Assuming they are ordered
                corresponding_planar_patches.append(plane_corr)

    return corresponding_planar_patches


def find_corr_in_map_and_solve(world_map, P_k_in_map_initial, print_msg=False):
    edge_ratio_threshold = 3
    planar_ratio_threshold = 3
    corresponding_edges_lines = find_edge_corr(world_map, P_k_in_map_initial, radius=0.2, ratio_threshold=edge_ratio_threshold)
    corresponding_planar_patches = find_plane_corr(world_map, P_k_in_map_initial, radius=0.4, ratio_threshold=planar_ratio_threshold)
    if print_msg:
        print("Mapping: Num Edges: {}".format(len(corresponding_edges_lines)))
        print("Mapping: Num Planes: {}".format(len(corresponding_planar_patches)))
    residual_cur_to_map_pose = optimize(corresponding_edges_lines, corresponding_planar_patches)
    return pose_to_mat(residual_cur_to_map_pose)


def plot_2_pc(points1, points2):
    pcd1 = create_point_cloud(points1, color=[0, 1, 1])  # Cyan
    pcd2 = create_point_cloud(points2, color=[0.5, 0, 0])  # Magenta

    # Visualize both point clouds
    o3d.visualization.draw_geometries([pcd1, pcd2], window_name="")


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


def plot_3_pc(points1, points2, points3):
    pcd1 = create_point_cloud(points1, color=[0, 1, 1])  # cyan
    pcd2 = create_point_cloud(points2, color=[0, 1, 0])  # Green color for the second point cloud
    pcd3 = create_point_cloud(points3, color=[0.5, 0, 0])  # dark red

    # Visualize both point clouds
    o3d.visualization.draw_geometries([pcd1, pcd2, pcd3], window_name="")


def mapping(cur2prev_matrices, P_ks, show_map=False, print_msg=0):
    if print_msg >= 1:
        print("Start Mapping")
    prev2map_mat = cur2prev_matrices[0]  # Should be identity
    world_map = Map(P_ks[0].points, P_ks[0].all_edges_pts, P_ks[0].all_edges_pts_indices_in_pc, P_ks[0].all_planar_pts, P_ks[0].all_planar_pts_indices_in_pc)
    cam2map_matrices = [prev2map_mat]

    for i in range(1, len(cur2prev_matrices)):  # index i represent the transform from i-1 to i
        if print_msg: print(" Working on Pc{}...".format(i))
        cur_to_prev_mat = cur2prev_matrices[i]
        initial_curr_to_map = compose_transformations(prev2map_mat, cur_to_prev_mat)

        P_k_in_map_initial = P_ks[i].apply_transformation(initial_curr_to_map)


        residual_cur_to_map = find_corr_in_map_and_solve(world_map, P_k_in_map_initial, print_msg)

        # Update Trans
        cur2map_final = compose_transformations(residual_cur_to_map, initial_curr_to_map)
        if print_msg >= 2:
            print("Mapping: residual_cur_to_map: ", formatted_pose(mat_to_pose(residual_cur_to_map)))
            print("Mapping: cur2map_final: ",       formatted_pose(mat_to_pose(cur2map_final)))

        # Update map
        P_k_in_map = P_k_in_map_initial.apply_transformation(residual_cur_to_map)

        world_map.add_points(P_k_in_map.points, P_k_in_map.all_edges_pts, P_k_in_map.all_edges_pts_indices_in_pc,
                             P_k_in_map.all_planar_pts, P_k_in_map.all_planar_pts_indices_in_pc)  # Color points from different pc in different colr to see diff

        # Update for next iteration
        prev2map_mat = cur2map_final
        cam2map_matrices.append(cur2map_final)

    if show_map:
        world_map.show_map()
    return cam2map_matrices



###################################################################################
###################################################################################
###################################################################################
def show_PC(points, reflectivity):
    # Extract x, y, z coordinates
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Create a 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, s=1)  # s is the size of points

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Enable interactive mode
    plt.ion()

    # Show plot
    plt.show()

    # Keep the plot open
    plt.ioff()
    plt.show()


def show_PC_o3d(points, reflectivity):
    import matplotlib.cm as cm
    reflectivity_normalized = (reflectivity - reflectivity.min()) / (reflectivity.max() - reflectivity.min())

    # Map reflectivity to colors using a colormap (e.g., 'viridis')
    cmap = cm.get_cmap('viridis')
    colors = cmap(reflectivity_normalized)[:, :3]  # Only take RGB (exclude alpha channel)

    # Create Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Assign colors to the point cloud
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])


def show_PC_with_distance_map(points):
    import matplotlib.cm as cm
    # Calculate the Euclidean distance from the origin (or another reference point)
    distances = np.linalg.norm(points, axis=1)

    # Normalize the distances to the range [0, 1]
    distances_normalized = (distances - distances.min()) / (distances.max() - distances.min())

    # Map normalized distances to colors using a colormap (e.g., 'viridis')
    colormap = cm.get_cmap('viridis')
    colors = colormap(distances_normalized)[:, :3]  # Use only RGB values

    # Create Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Assign colors to the point cloud
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])


def get_ground_truth_transformations(left_cam_trans_path=GT_POSES, movie_len=100):
    """
    Reads the ground truth transformations
    :return: array of transformation
    """
    T_ground_truth_arr = []
    with open(left_cam_trans_path) as f:
        lines = f.readlines()
    for i in range(movie_len):
        left_mat = np.array(lines[i][:-1].split(" ")).astype(float).reshape((3, 4))
        T_ground_truth_arr.append(left_mat)
    return np.array(T_ground_truth_arr)


def get_gt(poses):
    return poses[:, :, 3][:, 0], poses[:, :, 3][:, 2]


def get_od(odometries):
    return -odometries[:, :, 3][:, 1], odometries[:, :, 3][:, 0]


def save_show_results(save_matrices=False, map2cam_mat=None, compare_traj=False, cam2map_matrices=None,
                      cam2map_matrices_final=None, plot_trajs=None, show_first_res=False, sf=None, ef=None):
    gt_poses = get_ground_truth_transformations(left_cam_trans_path=GT_POSES)
    if save_matrices:
        np.save('/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/odo50.npy', map2cam_mat)
    if plot_trajs:
        gt_horizontal, gt_vertical = get_gt(gt_poses)
        od_horizontal, od_vertical = get_od(cam2map_matrices_final)
        plot_2d_traj(od_horizontal, od_vertical, "Odometry")
        plot_2d_traj(gt_horizontal, gt_vertical, "GT")
        compare_left_cam_2d_trajectory_to_ground_truth(od_horizontal, od_vertical, gt_horizontal, gt_vertical)

    if show_first_res:
        print("gt1:\n", c2m_mat_to_vec(gt_poses[1]))
        # print("od1:\n", c2m_mat_to_vec(cam2map_matrices[1]))
        print("map1:\n", c2m_mat_to_vec(cam2map_matrices_final[1]))
##################################################################################################


def parser():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-ml", type=int, help="movie len")
    parser.add_argument("-pm", type=int, help="print msg", default=0)
    parser.add_argument("-sr", action='store_true', help="save results")
    parser.add_argument("-sf", type=int, help="start frame")
    parser.add_argument("-ef", type=int, help="end frame")
    parser.add_argument("-sm", type=int, help="save matrices")
    return parser.parse_args()


def main():
    print("LOAM Starts")

    # Arguments
    args = parser()

    movie_len = args.ml if args.ml else 2
    start_frame = args.sf if args.sf else 0
    end_frame =  args.ef if args.ef else movie_len
    point_clouds = load_all_velodyne_files(PCD_PATH, start_frame=start_frame, end_frame=end_frame)

    # Odometry
    cur2prev_matrices, P_ks = lidar_odometry_whole_data_no_TS(point_clouds, args.pm)

    cam2_map_od = [cur2prev_matrices[0]]
    for i in range(len(cur2prev_matrices)):
        last = cam2_map_od[-1]
        cam2_map_od.append(compose_transformations(last, cur2prev_matrices[i]))

    # Mapping
    cam2map_matrices_final = mapping(cur2prev_matrices, P_ks, print_msg=args.pm)

    if args.sm:
        np.save('/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/rel_od10.npy', cur2prev_matrices)
        np.save('/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/od10.npy', cam2_map_od)
        np.save('/homes/maorat/work/mefusionlib/mefusionlib/localization/small_scripts/map10.npy', cam2map_matrices_final)

    cam2map_matrices_final = np.array(cam2map_matrices_final)
    if args.sr:
        save_show_results(plot_trajs=True, cam2map_matrices_final=cam2map_matrices_final, sf=start_frame, ef=end_frame, show_first_res=True)

    print("LOAM is DONE")


if __name__ == "__main__":
    main()
