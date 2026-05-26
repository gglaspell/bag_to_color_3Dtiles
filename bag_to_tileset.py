#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_to_tileset.py

ROS 2 bag -> registered point cloud -> (optional RGB colorization) ->
georeferenced 3D Tiles (PNTS / tileset.json).

Pipeline:
1. Read sensor_msgs/NavSatFix messages -> average GPS origin (lat0, lon0, alt0).
2. Read PointCloud2 frames + optional camera images + optional odometry.
3. Register frames with ICP + pose graph optimisation.
4. [colour path] Project the closest camera image onto each world-frame cloud,
   then merge with smart gray-fill filtering.
   [XYZ-only path] Merge raw frames with view-ray normals (no camera topic).
5. Optional floor leveling.
6. Voxel-downsample, ROR, SOR, DBSCAN cleaning.
7. Georeference local ENU -> ECEF (EPSG:4978) via pyproj,
   preserving RGB colours in the PLY when present.
8. py3dtiles.convert() -> tileset.json + *.pnts files.
"""

import argparse
import bisect
import copy
import math
import sys
from io import BytesIO
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image
from pyproj import CRS, Transformer
from py3dtiles.convert import convert as py3dtiles_convert
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)

# sensor_msgs/PointField datatypes
_POINTFIELD_TO_DTYPE = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}

_GPS_STATUS_NO_FIX = -1

# ---------------------------------------------------------------------------
# ROS message converters
# ---------------------------------------------------------------------------

def convert_ros_pc2_to_o3d(msg):
    """Convert sensor_msgs/PointCloud2 -> Open3D PointCloud (XYZ only).

    Returns o3d.geometry.PointCloud or None on failure.
    """
    try:
        fields = {f.name: (int(f.offset), int(f.datatype)) for f in msg.fields}
        if not {"x", "y", "z"}.issubset(fields):
            return None

        x_off, x_dt = fields["x"]
        y_off, y_dt = fields["y"]
        z_off, z_dt = fields["z"]

        for dt in (x_dt, y_dt, z_dt):
            if dt not in _POINTFIELD_TO_DTYPE:
                print(f"Warning: unsupported PointField datatype {dt}; skipping frame.")
                return None

        n_points = int(msg.width) * int(msg.height)
        itemsize = int(msg.point_step)
        if n_points <= 0 or itemsize <= 0:
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        x_np = _POINTFIELD_TO_DTYPE[x_dt]
        y_np = _POINTFIELD_TO_DTYPE[y_dt]
        z_np = _POINTFIELD_TO_DTYPE[z_dt]

        # Build structured dtype for the three fields individually
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [x_np, y_np, z_np],
            "offsets": [x_off, y_off, z_off],
            "itemsize": itemsize,
        })
        arr = np.frombuffer(msg.data, dtype=dtype, count=n_points)
        pts = np.column_stack([
            arr["x"].astype(np.float64),
            arr["y"].astype(np.float64),
            arr["z"].astype(np.float64),
        ])
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] < 10:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd
    except Exception:
        return None


def convert_ros_image(msg):
    """Convert sensor_msgs/Image or CompressedImage -> PIL Image (RGB).

    Returns PIL Image or None on failure.
    """
    try:
        msgtype = type(msg).__name__
        if "Compressed" in msgtype:
            return Image.open(BytesIO(bytes(msg.data))).convert("RGB")

        encoding = getattr(msg, "encoding", "rgb8")
        h, w = int(msg.height), int(msg.width)
        if encoding == "rgb8":
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return Image.fromarray(data, "RGB")
        if encoding == "bgr8":
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return Image.fromarray(data[:, :, ::-1], "RGB")
        if encoding == "mono8":
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            return Image.fromarray(data).convert("RGB")
        if encoding == "mono16":
            data = (np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w) >> 8).astype(np.uint8)
            return Image.fromarray(data).convert("RGB")
        # Fallback: try raw bytes
        return Image.open(BytesIO(bytes(msg.data))).convert("RGB")
    except Exception as e:
        print(f"Warning: image decode failed ({e})")
        return None


def intrinsics_from_camera_info(msg):
    """Extract (fx, fy, cx, cy, width, height) from sensor_msgs/CameraInfo."""
    k = list(msg.k)
    return float(k[0]), float(k[4]), float(k[2]), float(k[5]), int(msg.width), int(msg.height)


def get_odom_transform(odom_msg):
    """Extract 4x4 pose matrix from nav_msgs/Odometry. Returns ndarray or None."""
    try:
        pos = odom_msg.pose.pose.position
        quat = odom_msg.pose.pose.orientation
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
        T[:3, 3] = [pos.x, pos.y, pos.z]
        return T
    except Exception:
        return None


def get_closest_timestamp(ts, sorted_keys):
    """O(log N) bisect lookup of the nearest timestamp in a sorted list."""
    if not sorted_keys:
        return None
    idx = bisect.bisect_left(sorted_keys, ts)
    if idx == 0:
        return sorted_keys[0]
    if idx == len(sorted_keys):
        return sorted_keys[-1]
    before, after = sorted_keys[idx - 1], sorted_keys[idx]
    return before if (ts - before) <= (after - ts) else after

# ---------------------------------------------------------------------------
# ICP registration helpers
# ---------------------------------------------------------------------------

def compute_fpfh_descriptor(pcd, voxel_size):
    """Compute FPFH descriptor; estimates normals if missing."""
    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 2.0, max_nn=30))
    return o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0, max_nn=100))


def ransac_coarse_alignment(src, tgt, src_fpfh, tgt_fpfh, voxel_size):
    """RANSAC feature-matching coarse alignment. Returns 4x4 ndarray or None."""
    dist = voxel_size * 5.0
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src, tgt, src_fpfh, tgt_fpfh,
        mutual_filter=False,
        max_correspondence_distance=dist,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000, 0.999),
    )
    return result.transformation if result.fitness > 0.1 else None


def detect_loop_closure(
    current_idx, current_pcd, current_fpfh,
    historical_pcds, historical_fpfhs, historical_poses,
    voxel_size, search_radius, loop_fitness_thresh, temporal_window=100,
):
    """Find loop closure candidates. Returns list of (cand_idx, T, fitness)."""
    if current_idx < temporal_window:
        return []
    search_indices = list(range(0, current_idx - temporal_window))
    if not search_indices:
        return []

    current_pos = historical_poses[current_idx][:3, 3]
    hist_positions = np.array([historical_poses[i][:3, 3] for i in search_indices])

    pos_pcd = o3d.geometry.PointCloud()
    pos_pcd.points = o3d.utility.Vector3dVector(hist_positions)
    kdtree = o3d.geometry.KDTreeFlann(pos_pcd)

    _, raw_idxs, _ = kdtree.search_radius_vector_3d(current_pos, float(search_radius))
    if not raw_idxs:
        return []

    closures = []
    for raw_i in raw_idxs:
        cand_idx = search_indices[raw_i]
        cand_fpfh = historical_fpfhs[cand_idx]
        if cand_fpfh is None:
            continue
        coarse = ransac_coarse_alignment(
            current_pcd, copy.deepcopy(historical_pcds[cand_idx]),
            current_fpfh, cand_fpfh, voxel_size,
        )
        if coarse is None:
            continue
        icp = o3d.pipelines.registration.registration_icp(
            current_pcd, historical_pcds[cand_idx],
            voxel_size * 2.0, coarse,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        if icp.fitness >= loop_fitness_thresh:
            closures.append((cand_idx, icp.transformation, icp.fitness))
    return closures

# ---------------------------------------------------------------------------
# View-ray normals
# ---------------------------------------------------------------------------

def _safe_normalize(v, eps=1e-9):
    norms = np.linalg.norm(v, axis=1, keepdims=True)
    return v / np.clip(norms, eps, None)


def attach_view_rays_as_normals(pcd_world, sensor_origin):
    """Store unit vectors from each point toward the sensor as pcd normals."""
    pts = np.asarray(pcd_world.points)
    if len(pts) == 0:
        return
    dirs = _safe_normalize(sensor_origin.reshape(1, 3) - pts)
    pcd_world.normals = o3d.utility.Vector3dVector(dirs)

# ---------------------------------------------------------------------------
# Colorization
# ---------------------------------------------------------------------------

def color_point_cloud_from_image(pcd, img, camera_pose, fx, fy, cx, cy, img_w, img_h,
                                  min_depth=0.1, max_depth=None):
    """Project camera image colours onto a world-frame point cloud in-place.

    Points not visible by the camera receive neutral gray fill (0.5, 0.5, 0.5).
    Returns the same pcd with .colors populated.
    """
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        pcd.colors = o3d.utility.Vector3dVector(np.empty((0, 3)))
        return pcd

    img_arr = np.asarray(img)  # H x W x 3, uint8
    cam_pos = camera_pose[:3, 3]
    cam_rot = R.from_matrix(camera_pose[:3, :3])

    # World points into camera body frame
    body = cam_rot.inv().apply(pts - cam_pos)

    # ROS body (x-fwd, y-left, z-up) -> camera optical (x-right, y-down, z-fwd)
    opt_x = -body[:, 1]   # right = -left
    opt_y = -body[:, 2]   # down  = -up
    opt_z =  body[:, 0]   # fwd   = fwd

    depth = np.linalg.norm(body, axis=1)
    valid = (opt_z > 1e-6) & (depth >= min_depth)
    if max_depth is not None:
        valid &= (depth <= max_depth)

    z_safe = np.where(opt_z > 1e-6, opt_z, 1e-6)
    u = fx * (opt_x / z_safe) + cx
    v = fy * (opt_y / z_safe) + cy
    valid &= (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)

    colors = np.full((len(pts), 3), 0.5, dtype=np.float64)
    if np.any(valid):
        ui = np.clip(u[valid].astype(np.int32), 0, img_w - 1)
        vi = np.clip(v[valid].astype(np.int32), 0, img_h - 1)
        colors[valid] = img_arr[vi, ui] / 255.0

    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def merge_colored_point_clouds(colored_pcds, voxel_size, gray_filter_radius):
    """Concatenate per-frame coloured clouds, remove gray fill near real colour,
    then voxel-downsample.

    Gray fill = per-channel std-dev < 0.08 AND mean within 0.15 of 0.5.
    Gray-fill points with at least one coloured neighbour within
    gray_filter_radius are removed; isolated fills are kept.
    """
    all_pts, all_cols, all_nors, all_is_gray = [], [], [], []

    for pcd in colored_pcds:
        if len(pcd.points) == 0 or not pcd.has_colors():
            continue
        pts = np.asarray(pcd.points, dtype=np.float64)
        cols = np.asarray(pcd.colors, dtype=np.float64)
        nors = (np.asarray(pcd.normals, dtype=np.float64)
                if pcd.has_normals() else np.zeros((len(pts), 3)))
        ch_std = np.std(cols, axis=1)
        ch_mean = np.mean(cols, axis=1)
        is_gray = (ch_std < 0.08) & (np.abs(ch_mean - 0.5) < 0.15)
        all_pts.append(pts)
        all_cols.append(cols)
        all_nors.append(nors)
        all_is_gray.append(is_gray)

    if not all_pts:
        raise ValueError("No valid coloured point clouds to merge.")

    pts = np.vstack(all_pts)
    cols = np.vstack(all_cols)
    nors = np.vstack(all_nors)
    is_gray = np.hstack(all_is_gray)

    colored_pts = pts[~is_gray]
    if len(colored_pts) > 0 and gray_filter_radius > 0.0:
        print(f"  Gray-fill filtering (radius={gray_filter_radius} m)...")
        tree = cKDTree(colored_pts)
        gray_idx = np.where(is_gray)[0]
        neighbors = tree.query_ball_point(pts[gray_idx], r=gray_filter_radius)
        has_col = np.array([len(n) > 0 for n in neighbors], dtype=bool)
        keep = np.ones(len(pts), dtype=bool)
        keep[gray_idx[has_col]] = False
        pts = pts[keep]
        cols = cols[keep]
        nors = nors[keep]
    elif len(colored_pts) == 0:
        print("  Warning: no coloured points found; keeping all gray fills.")

    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(pts)
    merged.colors = o3d.utility.Vector3dVector(cols)
    if np.any(np.linalg.norm(nors, axis=1) > 0):
        merged.normals = o3d.utility.Vector3dVector(_safe_normalize(nors))

    print(f"  Voxel downsampling (voxel_size={voxel_size} m)...")
    return merged.voxel_down_sample(voxel_size)

# ---------------------------------------------------------------------------
# GPS
# ---------------------------------------------------------------------------

def parse_gps_fixes(bag_path, gps_topic, typestore):
    """Read NavSatFix messages; return averaged (lat0, lon0, alt0) or exit."""
    fixes = []
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == gps_topic]
        if not conns:
            sys.exit(
                f"Error: GPS topic '{gps_topic}' not found in bag.\n"
                f"Use --gps_topic to specify the correct topic."
            )

        for conn, _ts, raw in reader.messages(connections=conns):
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                if int(msg.status.status) == _GPS_STATUS_NO_FIX:
                    continue
                lat, lon, alt = float(msg.latitude), float(msg.longitude), float(msg.altitude)
                if math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt):
                    fixes.append((lat, lon, alt))
            except Exception:
                continue

    if not fixes:
        sys.exit(
            f"Error: No valid GPS fixes on topic '{gps_topic}'.\n"
            "All messages had STATUS_NO_FIX or could not be parsed."
        )

    n = len(fixes)
    lat0 = sum(f[0] for f in fixes) / n
    lon0 = sum(f[1] for f in fixes) / n
    alt0 = sum(f[2] for f in fixes) / n
    print(f"GPS: {n} fix(es) averaged -> lat={lat0:.7f} lon={lon0:.7f} alt={alt0:.3f} m")
    return lat0, lon0, alt0

# ---------------------------------------------------------------------------
# Georeferencing: local ENU -> ECEF
# ---------------------------------------------------------------------------

def _enu_to_ecef_rotation(lat_deg, lon_deg):
    """3x3 rotation: ENU column vectors -> ECEF."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    return np.array([
        [-so,       co,       0.0],
        [-sl * co, -sl * so,  cl ],
        [ cl * co,  cl * so,  sl ],
    ], dtype=np.float64)


def transform_local_enu_to_ecef(pts_enu, lat0_deg, lon0_deg, alt0_m):
    """Convert (N,3) local-ENU points to ECEF XYZ (float64, metres)."""
    transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(4978), always_xy=True)
    ox, oy, oz = transformer.transform(lon0_deg, lat0_deg, alt0_m)
    ecef_origin = np.array([ox, oy, oz], dtype=np.float64)
    R_mat = _enu_to_ecef_rotation(lat0_deg, lon0_deg)
    return pts_enu @ R_mat.T + ecef_origin

# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def process_bag(args):
    bag_path = Path(args.bagpath)
    out_dir  = Path(args.outputdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not bag_path.exists():
        sys.exit(f"Error: Bag file not found: {bag_path}")

    if args.camera_topic and not args.camera_info_topic:
        sys.exit(
            "Error: --camera_topic requires --camera_info_topic.\n"
            "Set --camera_info_topic to the sensor_msgs/CameraInfo topic."
        )

    odom_max_latency_ns = int(args.odom_max_latency * 1e9)

    # -------------------------------------------------------------------
    # 0) GPS origin
    # -------------------------------------------------------------------
    print(f"Reading GPS fixes from topic: {args.gps_topic}")
    lat0, lon0, alt0 = parse_gps_fixes(bag_path, args.gps_topic, TYPESTORE)

    # -------------------------------------------------------------------
    # 1) Read bag messages
    # -------------------------------------------------------------------
    topics_to_read = [args.pc_topic]
    if args.odom_topic:
        topics_to_read.append(args.odom_topic)
    if args.camera_topic:
        topics_to_read.append(args.camera_topic)
        topics_to_read.append(args.camera_info_topic)

    pointclouds   = []   # [(ts_ns, o3d.PointCloud)]
    odom_data     = {}   # ts_ns -> 4x4 ndarray
    camera_images = {}   # ts_ns -> PIL Image
    cam_info_data = {}   # ts_ns -> (fx, fy, cx, cy, w, h)

    print(f"\nReading bag:      {bag_path}")
    print(f"Output dir:       {out_dir}")
    print(f"PointCloud topic: {args.pc_topic}")
    if args.camera_topic:
        print(f"Camera topic:     {args.camera_topic}")
        print(f"CameraInfo topic: {args.camera_info_topic}")
    if args.odom_topic:
        print(f"Odometry topic:   {args.odom_topic} (max latency: {args.odom_max_latency} s)")
    lc_status = (
        f"ENABLED (every {args.loop_closure_search_interval} frames)"
        if args.enable_loop_closure else "disabled"
    )
    print(f"Loop closure:     {lc_status}\n")

    with AnyReader([bag_path], default_typestore=TYPESTORE) as reader:
        conns = [c for c in reader.connections if c.topic in topics_to_read]
        if not conns:
            sys.exit(f"Error: No messages found for topics: {topics_to_read}")

        for conn, ts, raw in tqdm(reader.messages(connections=conns), desc="Reading"):
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                if conn.topic == args.pc_topic:
                    pcd = convert_ros_pc2_to_o3d(msg)
                    if pcd is not None and len(pcd.points) >= 100:
                        pointclouds.append((ts, pcd))
                elif args.odom_topic and conn.topic == args.odom_topic:
                    T = get_odom_transform(msg)
                    if T is not None:
                        odom_data[ts] = T
                elif args.camera_topic and conn.topic == args.camera_topic:
                    img = convert_ros_image(msg)
                    if img is not None:
                        camera_images[ts] = img
                elif args.camera_info_topic and conn.topic == args.camera_info_topic:
                    cam_info_data[ts] = intrinsics_from_camera_info(msg)
            except Exception:
                continue

    if not pointclouds:
        sys.exit("Error: No valid point clouds were extracted.")
    print(f"Extracted {len(pointclouds)} point clouds.")

    if args.odom_topic:
        if not odom_data:
            print("Warning: --odom_topic set but no messages found; "
                  "falling back to identity initial guess.")
        else:
            print(f"Extracted {len(odom_data)} odometry messages.")

    # Resolve camera intrinsics / color mode
    color_mode = bool(args.camera_topic)
    fx = fy = cx = cy = cam_w = cam_h = None
    if color_mode:
        if not cam_info_data:
            sys.exit(
                f"Error: No CameraInfo messages found on '{args.camera_info_topic}'.\n"
                "Verify the topic name and that the bag contains CameraInfo messages."
            )
        if not camera_images:
            print("Warning: --camera_topic set but no images decoded; "
                  "falling back to XYZ-only mode.")
            color_mode = False
        else:
            first_ts = min(cam_info_data.keys())
            fx, fy, cx, cy, cam_w, cam_h = cam_info_data[first_ts]
            print(f"Extracted {len(camera_images)} camera images.")
            print(f"Camera intrinsics: fx={fx:.1f} fy={fy:.1f} "
                  f"cx={cx:.1f} cy={cy:.1f} {cam_w}x{cam_h}")

    odom_ts_sorted = sorted(odom_data.keys())

    # -------------------------------------------------------------------
    # 2) ICP + pose graph
    # -------------------------------------------------------------------
    posegraph = o3d.pipelines.registration.PoseGraph()
    current_transform = np.eye(4, dtype=np.float64)
    posegraph.nodes.append(
        o3d.pipelines.registration.PoseGraphNode(current_transform.copy()))

    _, source_raw = pointclouds[0]
    source = source_raw.voxel_down_sample(args.voxel_size)
    source.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.voxel_size * 2.0, max_nn=30))

    lc_pcds, lc_fpfhs, lc_poses = [], [], []
    if args.enable_loop_closure:
        lc_pcds.append(source)
        lc_fpfhs.append(compute_fpfh_descriptor(copy.deepcopy(source), args.voxel_size))
        lc_poses.append(current_transform.copy())

    previous_odom_T = None
    if odom_ts_sorted:
        cts = get_closest_timestamp(pointclouds[0][0], odom_ts_sorted)
        if cts is not None and abs(cts - pointclouds[0][0]) < odom_max_latency_ns:
            previous_odom_T = odom_data[cts]

    successful_pc_indices = [0]
    loop_closures_found = 0

    print("\nRegistering point clouds...")
    for i in tqdm(range(1, len(pointclouds)), desc="Registering"):
        ts, target_raw = pointclouds[i]
        target = target_raw.voxel_down_sample(args.voxel_size)
        target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=args.voxel_size * 2.0, max_nn=30))

        initial_guess = np.eye(4, dtype=np.float64)
        if odom_ts_sorted:
            cts = get_closest_timestamp(ts, odom_ts_sorted)
            if cts is not None and abs(cts - ts) < odom_max_latency_ns:
                current_odom_T = odom_data[cts]
                if previous_odom_T is not None:
                    initial_guess = np.linalg.inv(previous_odom_T) @ current_odom_T
                previous_odom_T = current_odom_T
            else:
                previous_odom_T = None

        try:
            reg = o3d.pipelines.registration.registration_icp(
                source, target,
                args.icp_dist_thresh, initial_guess,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
            )
        except Exception:
            continue

        if reg.fitness < args.icp_fitness_thresh:
            continue

        current_transform = reg.transformation @ current_transform
        posegraph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(current_transform)))

        info = np.eye(6, dtype=np.float64) * max(reg.fitness, 1e-6)
        posegraph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                len(posegraph.nodes) - 2,
                len(posegraph.nodes) - 1,
                reg.transformation, info, uncertain=False,
            ))

        if args.enable_loop_closure:
            do_lc = (i % args.loop_closure_search_interval == 0)
            tgt_fpfh = compute_fpfh_descriptor(copy.deepcopy(target), args.voxel_size) if do_lc else None
            lc_pcds.append(target)
            lc_fpfhs.append(tgt_fpfh)
            lc_poses.append(current_transform.copy())
            if do_lc:
                closures = detect_loop_closure(
                    len(lc_pcds) - 1, target, tgt_fpfh,
                    lc_pcds, lc_fpfhs, lc_poses,
                    args.voxel_size,
                    search_radius=args.loop_closure_radius,
                    loop_fitness_thresh=args.loop_closure_fitness_thresh,
                )
                for cand_idx, lc_T, lc_fit in closures:
                    lc_info = np.eye(6, dtype=np.float64) * max(lc_fit * 100.0, 1e-6)
                    posegraph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(
                            cand_idx, len(posegraph.nodes) - 1,
                            lc_T, lc_info, uncertain=True,
                        ))
                    loop_closures_found += 1

        successful_pc_indices.append(i)
        source = target

    if len(posegraph.nodes) < 2:
        sys.exit(
            "Error: Registration failed (too few successful registrations).\n"
            "Try --icp_fitness_thresh 0.3 or --icp_dist_thresh 0.5."
        )

    if args.enable_loop_closure:
        print(f"Loop closures detected: {loop_closures_found}")

    # -------------------------------------------------------------------
    # 3) Pose graph optimisation
    # -------------------------------------------------------------------
    print("\nOptimizing pose graph...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=args.icp_dist_thresh,
        edge_prune_threshold=0.25,
        reference_node=0,
    )
    try:
        o3d.pipelines.registration.global_optimization(
            posegraph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option,
        )
    except Exception as e:
        print(f"Warning: Global optimization failed ({e}); using unoptimized poses.")

    def node_pose(node_idx):
        return np.asarray(posegraph.nodes[node_idx].pose, dtype=np.float64)

    # -------------------------------------------------------------------
    # 4a) COLOUR PATH
    # -------------------------------------------------------------------
    if color_mode:
        cam_ts_sorted = sorted(camera_images.keys())
        print("\nColoring point clouds...")
        colored_pcds = []
        for node_idx, pc_idx in tqdm(
            enumerate(successful_pc_indices),
            total=len(successful_pc_indices), desc="Coloring",
        ):
            pc_ts, pc_raw = pointclouds[pc_idx]
            pose = node_pose(node_idx)
            pcd_world = copy.deepcopy(pc_raw)
            pcd_world.transform(pose)
            attach_view_rays_as_normals(pcd_world, pose[:3, 3])

            closest_cam_ts = get_closest_timestamp(pc_ts, cam_ts_sorted)
            if abs(closest_cam_ts - pc_ts) / 1e9 <= args.max_time_diff:
                pcd_world = color_point_cloud_from_image(
                    pcd_world, camera_images[closest_cam_ts],
                    pose, fx, fy, cx, cy, cam_w, cam_h,
                    min_depth=args.color_min_depth,
                    max_depth=args.color_max_depth,
                )
            else:
                n = len(pcd_world.points)
                pcd_world.colors = o3d.utility.Vector3dVector(
                    np.full((n, 3), 0.5, dtype=np.float64))

            colored_pcds.append(pcd_world)

        print("\nMerging colored point clouds...")
        pcd_clean = merge_colored_point_clouds(
            colored_pcds,
            voxel_size=args.voxel_size,
            gray_filter_radius=args.gray_filter_radius,
        )
        skip_voxel = True  # merge_colored_point_clouds already downsamples

    # -------------------------------------------------------------------
    # 4b) XYZ-ONLY PATH
    # -------------------------------------------------------------------
    else:
        print("\nMerging point clouds...")
        pcd_combined = o3d.geometry.PointCloud()
        for node_idx, pc_idx in tqdm(
            enumerate(successful_pc_indices),
            total=len(successful_pc_indices), desc="Merging",
        ):
            _, pcd_raw = pointclouds[pc_idx]
            pose = node_pose(node_idx)
            pcd_world = copy.deepcopy(pcd_raw)
            pcd_world.transform(pose)
            attach_view_rays_as_normals(pcd_world, pose[:3, 3])
            pcd_combined += pcd_world

        if len(pcd_combined.points) == 0:
            sys.exit("Error: Combined point cloud is empty.")
        pcd_clean = pcd_combined
        skip_voxel = False

    # -------------------------------------------------------------------
    # 5) Optional floor leveling
    # -------------------------------------------------------------------
    if args.level_floor:
        print("\nAttempting floor leveling...")
        try:
            tmp = pcd_clean.voxel_down_sample(args.voxel_size * 2.0)
            plane_model, _ = tmp.segment_plane(
                distance_threshold=args.voxel_size * 2.0,
                ransac_n=3, num_iterations=1000,
            )
            a, b, c, _d = plane_model
            n_vec = np.array([a, b, c], dtype=np.float64)
            n_vec /= np.linalg.norm(n_vec) + 1e-12
            if np.dot(n_vec, [0, 0, 1]) < 0:
                n_vec = -n_vec

            v = np.cross(n_vec, [0, 0, 1])
            s = np.linalg.norm(v)
            if s > 1e-12:
                cang = float(np.dot(n_vec, [0, 0, 1]))
                vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]],
                               dtype=np.float64)
                R3 = np.eye(3) + vx + vx @ vx * ((1.0 - cang) / (s * s))
                pcd_clean.points = o3d.utility.Vector3dVector(
                    np.asarray(pcd_clean.points) @ R3.T)
                if pcd_clean.has_normals():
                    pcd_clean.normals = o3d.utility.Vector3dVector(
                        np.asarray(pcd_clean.normals) @ R3.T)
                print("  Floor leveling applied.")
            else:
                print("  Map is already level.")
        except Exception as e:
            print(f"  Warning: Floor leveling failed ({e}).")

    # -------------------------------------------------------------------
    # 6) Cleaning: voxel (XYZ path only), ROR, SOR, DBSCAN
    # -------------------------------------------------------------------
    pts_before = len(pcd_clean.points)

    if not skip_voxel:
        print(f"\nVoxel downsampling (voxel_size={args.voxel_size} m)...")
        pcd_clean = pcd_clean.voxel_down_sample(args.voxel_size)

    print(f"ROR outlier removal (nb_points={args.ror_nb_points}, radius={args.ror_radius} m)...")
    pcd_clean, _ = pcd_clean.remove_radius_outlier(
        nb_points=args.ror_nb_points, radius=args.ror_radius)

    print(f"SOR outlier removal (nb_neighbors={args.sor_nb_neighbors}, std_ratio={args.sor_std_ratio})...")
    pcd_clean, _ = pcd_clean.remove_statistical_outlier(
        nb_neighbors=args.sor_nb_neighbors, std_ratio=args.sor_std_ratio)

    if args.dbscan_eps > 0:
        print(f"DBSCAN clustering (eps={args.dbscan_eps} m, min_points={args.dbscan_min_points})...")
        labels = np.array(pcd_clean.cluster_dbscan(
            eps=args.dbscan_eps, min_points=args.dbscan_min_points, print_progress=False))
        keep = labels >= 0
        pcd_clean = pcd_clean.select_by_index(np.where(keep)[0])

    pts_after = len(pcd_clean.points)
    print(f"Points: {pts_before} -> {pts_after}")

    if pts_after == 0:
        sys.exit("Error: Point cloud is empty after cleaning. "
                 "Try relaxing --ror_nb_points or --dbscan_eps.")

    # -------------------------------------------------------------------
    # 7) Georeference ENU -> ECEF
    # -------------------------------------------------------------------
    print("\nGeoreferencing (ENU -> ECEF)...")
    pts_enu = np.asarray(pcd_clean.points, dtype=np.float64)
    pts_ecef = transform_local_enu_to_ecef(pts_enu, lat0, lon0, alt0)

    has_color = pcd_clean.has_colors()
    colors_uint8 = None
    if has_color:
        colors_uint8 = (np.asarray(pcd_clean.colors) * 255).clip(0, 255).astype(np.uint8)

    # -------------------------------------------------------------------
    # 8) Write PLY and generate 3D Tiles
    # -------------------------------------------------------------------
    ply_path = out_dir / "cloud.ply"
    print(f"Writing PLY: {ply_path}  (colours: {'yes' if has_color else 'no'})")

    pcd_ecef = o3d.geometry.PointCloud()
    pcd_ecef.points = o3d.utility.Vector3dVector(pts_ecef)
    if has_color:
        pcd_ecef.colors = o3d.utility.Vector3dVector(
            np.asarray(pcd_clean.colors, dtype=np.float64))

    o3d.io.write_point_cloud(str(ply_path), pcd_ecef, write_ascii=False)

    print("\nGenerating 3D Tiles...")
    py3dtiles_convert(
        str(ply_path),
        outfolder=str(out_dir),
        crs_in="EPSG:4978",
        jobs=args.workers,
    )
    print(f"\nDone.  Tileset: {out_dir}/tileset.json")

# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(
        description="Convert a ROS 2 bag to a georeferenced 3D Tiles tileset.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # positional
    p.add_argument("bagpath",   help="Path to the ROS 2 .bag / .db3 / .mcap file.")
    p.add_argument("outputdir", help="Output directory for tileset.json + *.pnts.")

    # topics
    p.add_argument("--pc_topic",          default="/points",  help="sensor_msgs/PointCloud2 topic.")
    p.add_argument("--gps_topic",         default="/gps/fix", help="sensor_msgs/NavSatFix topic.")
    p.add_argument("--odom_topic",        default="",         help="nav_msgs/Odometry topic (optional).")
    p.add_argument("--camera_topic",      default="",         help="sensor_msgs/Image or CompressedImage topic (optional).")
    p.add_argument("--camera_info_topic", default="",         help="sensor_msgs/CameraInfo topic (required when --camera_topic is set).")

    # camera colorization
    p.add_argument("--max_time_diff",     type=float, default=0.1,  help="Max camera-lidar timestamp gap (s).")
    p.add_argument("--color_min_depth",   type=float, default=0.1,  help="Min projection depth (m).")
    p.add_argument("--color_max_depth",   type=float, default=None, help="Max projection depth (m); no limit if omitted.")
    p.add_argument("--gray_filter_radius",type=float, default=0.05, help="Suppress gray fill within this radius of coloured points (m).")

    # ICP
    p.add_argument("--voxel_size",        type=float, default=0.05, help="Downsampling voxel size (m).")
    p.add_argument("--icp_dist_thresh",   type=float, default=0.2,  help="ICP max correspondence distance (m).")
    p.add_argument("--icp_fitness_thresh",type=float, default=0.6,  help="Min ICP fitness to accept a frame [0-1].")
    p.add_argument("--odom_max_latency",  type=float, default=0.5,  help="Max odom-pointcloud timestamp gap (s).")

    # loop closure
    p.add_argument("--enable_loop_closure",          action="store_true", help="Enable loop closure detection.")
    p.add_argument("--loop_closure_radius",          type=float, default=5.0,  help="Spatial search radius for loop closures (m).")
    p.add_argument("--loop_closure_fitness_thresh",  type=float, default=0.3,  help="Min ICP fitness for a loop closure edge.")
    p.add_argument("--loop_closure_search_interval", type=int,   default=10,   help="Check for loop closures every N frames.")

    # cleaning
    p.add_argument("--ror_nb_points",    type=int,   default=6,   help="ROR: min neighbour count.")
    p.add_argument("--ror_radius",       type=float, default=0.5, help="ROR: search radius (m).")
    p.add_argument("--sor_nb_neighbors", type=int,   default=20,  help="SOR: neighbour count.")
    p.add_argument("--sor_std_ratio",    type=float, default=2.0, help="SOR: std-dev multiplier.")
    p.add_argument("--dbscan_eps",       type=float, default=0.5, help="DBSCAN epsilon (m); 0 disables.")
    p.add_argument("--dbscan_min_points",type=int,   default=10,  help="DBSCAN minimum cluster size.")

    # misc
    p.add_argument("--level_floor", action="store_true", help="Attempt RANSAC floor leveling.")
    p.add_argument("--workers",     type=int, default=4,  help="py3dtiles worker threads.")

    return p


if __name__ == "__main__":
    args = build_parser().parse_args()
    process_bag(args)
