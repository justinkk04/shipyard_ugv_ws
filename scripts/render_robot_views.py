#!/usr/bin/env python3
"""Render 6 orthographic views of an SDF robot model with proper pose resolution."""

import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os
import sys
import math


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])


def parse_pose(pose_str):
    if pose_str is None or pose_str.strip() == "":
        return np.zeros(3), np.eye(3)
    parts = [float(x) for x in pose_str.strip().split()]
    pos = np.array(parts[:3])
    R = rotation_matrix(parts[3], parts[4], parts[5]) if len(parts) >= 6 else np.eye(3)
    return pos, R


def combine_transforms(pos1, R1, pos2, R2):
    """Combine two transforms: first apply (pos1, R1), then (pos2, R2) relative to it."""
    pos = pos1 + R1 @ pos2
    R = R1 @ R2
    return pos, R


def box_verts(sx, sy, sz):
    hx, hy, hz = sx/2, sy/2, sz/2
    v = np.array([
        [-hx,-hy,-hz],[hx,-hy,-hz],[hx,hy,-hz],[-hx,hy,-hz],
        [-hx,-hy,hz],[hx,-hy,hz],[hx,hy,hz],[-hx,hy,hz],
    ])
    f = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,3,7,4],[1,2,6,5]]
    return v, f


def cylinder_verts(radius, length, n=24):
    angles = np.linspace(0, 2*np.pi, n, endpoint=False)
    v = []
    for a in angles:
        v.append([radius*np.cos(a), radius*np.sin(a), -length/2])
    for a in angles:
        v.append([radius*np.cos(a), radius*np.sin(a), length/2])
    v = np.array(v)
    f = []
    for i in range(n):
        j = (i+1) % n
        f.append([i, j, j+n, i+n])
    f.append(list(range(n)))
    f.append(list(range(n, 2*n)))
    return v, f


def sphere_verts(radius, n_lat=12, n_lon=16):
    v = [[0, 0, radius]]
    for i in range(1, n_lat):
        lat = np.pi * i / n_lat
        for j in range(n_lon):
            lon = 2 * np.pi * j / n_lon
            v.append([radius*np.sin(lat)*np.cos(lon),
                       radius*np.sin(lat)*np.sin(lon),
                       radius*np.cos(lat)])
    v.append([0, 0, -radius])
    v = np.array(v)
    f = []
    for j in range(n_lon):
        f.append([0, 1+j, 1+(j+1)%n_lon])
    for i in range(n_lat-2):
        for j in range(n_lon):
            a = 1+i*n_lon+j; b = 1+i*n_lon+(j+1)%n_lon
            c = 1+(i+1)*n_lon+(j+1)%n_lon; d = 1+(i+1)*n_lon+j
            f.append([a, b, c, d])
    south = len(v)-1
    base = 1+(n_lat-2)*n_lon
    for j in range(n_lon):
        f.append([south, base+(j+1)%n_lon, base+j])
    return v, f


def parse_color(mat):
    if mat is None:
        return (0.7, 0.7, 0.7, 0.9)
    d = mat.find("diffuse")
    if d is not None:
        p = [float(x) for x in d.text.strip().split()]
        return (p[0], p[1], p[2], p[3] if len(p) > 3 else 0.9)
    return (0.7, 0.7, 0.7, 0.9)


def collect_visuals(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find(".//model")

    # Build a frame lookup: name -> (position, rotation) in model frame
    # First, resolve all frames by building the pose graph
    frame_poses = {}

    # base_footprint is the root link at origin
    frame_poses['base_footprint'] = (np.zeros(3), np.eye(3))

    # Parse joints to get their poses (they define where child links are)
    joint_map = {}  # joint_name -> {pose, relative_to, child}
    for joint in model.findall("joint"):
        jname = joint.get("name")
        pose_elem = joint.find("pose")
        relative_to = pose_elem.get("relative_to", None) if pose_elem is not None else None
        pose_text = pose_elem.text if pose_elem is not None else None
        child = joint.find("child").text
        pos, R = parse_pose(pose_text)

        # Resolve relative_to
        if relative_to and relative_to in frame_poses:
            parent_pos, parent_R = frame_poses[relative_to]
            abs_pos, abs_R = combine_transforms(parent_pos, parent_R, pos, R)
        else:
            abs_pos, abs_R = pos, R

        frame_poses[jname] = (abs_pos, abs_R)
        joint_map[jname] = {'child': child, 'pos': abs_pos, 'R': abs_R}

    # Parse frame elements
    for frame in model.findall("frame"):
        fname = frame.get("name")
        attached = frame.get("attached_to", None)
        pose_elem = frame.find("pose")
        pose_text = pose_elem.text if pose_elem is not None else None
        pos, R = parse_pose(pose_text)

        if attached and attached in frame_poses:
            parent_pos, parent_R = frame_poses[attached]
            abs_pos, abs_R = combine_transforms(parent_pos, parent_R, pos, R)
        else:
            abs_pos, abs_R = pos, R

        frame_poses[fname] = (abs_pos, abs_R)

    # Now resolve link poses
    link_poses = {}
    for link in model.findall("link"):
        lname = link.get("name")
        pose_elem = link.find("pose")

        if pose_elem is not None:
            relative_to = pose_elem.get("relative_to", None)
            pos, R = parse_pose(pose_elem.text)

            if relative_to and relative_to in frame_poses:
                parent_pos, parent_R = frame_poses[relative_to]
                abs_pos, abs_R = combine_transforms(parent_pos, parent_R, pos, R)
            else:
                abs_pos, abs_R = pos, R
        elif lname in frame_poses:
            abs_pos, abs_R = frame_poses[lname]
        else:
            abs_pos, abs_R = np.zeros(3), np.eye(3)

        link_poses[lname] = (abs_pos, abs_R)

    print(f"  Resolved link poses:")
    for name, (pos, _) in link_poses.items():
        print(f"    {name}: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

    # Collect all visual geometries
    geometries = []
    for link in model.findall("link"):
        lname = link.get("name")
        link_pos, link_R = link_poses[lname]

        for visual in link.findall("visual"):
            pose_elem = visual.find("pose")
            vis_pos, vis_R = parse_pose(pose_elem.text if pose_elem is not None else None)

            # Combine link pose + visual pose
            world_pos, world_R = combine_transforms(link_pos, link_R, vis_pos, vis_R)

            color = parse_color(visual.find("material"))
            geom = visual.find("geometry")
            verts, faces = None, None

            box = geom.find("box")
            if box is not None:
                size = [float(x) for x in box.find("size").text.strip().split()]
                verts, faces = box_verts(*size)

            cyl = geom.find("cylinder")
            if cyl is not None:
                radius = float(cyl.find("radius").text)
                length = float(cyl.find("length").text)
                verts, faces = cylinder_verts(radius, length)

            sph = geom.find("sphere")
            if sph is not None:
                radius = float(sph.find("radius").text)
                verts, faces = sphere_verts(radius)

            if verts is not None:
                verts = (world_R @ verts.T).T + world_pos
                geometries.append((verts, faces, color))

    return geometries


def render_view(geometries, elev, azim, title, output_path, figsize=(8, 8)):
    fig = plt.figure(figsize=figsize, facecolor='#2b2b2b')
    ax = fig.add_subplot(111, projection='3d', facecolor='#2b2b2b')

    for verts, faces, color in geometries:
        polys = [[verts[idx] for idx in face] for face in faces]
        edge_color = tuple(max(0, c - 0.2) for c in color[:3])
        coll = Poly3DCollection(polys, alpha=color[3] if len(color) > 3 else 0.9)
        coll.set_facecolor(color[:3])
        coll.set_edgecolor((*edge_color, 0.3))
        coll.set_linewidth(0.5)
        ax.add_collection3d(coll)

    all_verts = np.vstack([v for v, _, _ in geometries])
    max_range = np.ptp(all_verts, axis=0).max() / 2 * 1.3
    mid = (all_verts.max(axis=0) + all_verts.min(axis=0)) / 2
    ax.set_xlim(mid[0]-max_range, mid[0]+max_range)
    ax.set_ylim(mid[1]-max_range, mid[1]+max_range)
    ax.set_zlim(mid[2]-max_range, mid[2]+max_range)

    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, color='white', fontsize=16, fontweight='bold', pad=15)
    ax.set_xlabel('X', color='gray'); ax.set_ylabel('Y', color='gray'); ax.set_zlabel('Z', color='gray')
    ax.tick_params(colors='gray', labelsize=7)
    ax.xaxis.pane.fill = False; ax.yaxis.pane.fill = False; ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('#444444')
    ax.yaxis.pane.set_edgecolor('#444444')
    ax.zaxis.pane.set_edgecolor('#444444')
    ax.grid(True, alpha=0.2)

    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#2b2b2b')
    plt.close()
    print(f"  Saved: {output_path}")


def main():
    sdf_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/shipyard_ugv_ws/src/models/shipyard_rover/model.sdf")
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser(
        "~/shipyard_ugv_ws/robot_views")

    os.makedirs(output_dir, exist_ok=True)
    print(f"Parsing SDF: {sdf_path}")
    geometries = collect_visuals(sdf_path)
    print(f"Found {len(geometries)} visual geometries\n")

    views = [
        (25,  -60,  "Perspective (3/4 View)", "perspective.png"),
        (90,  -90,  "Top View",               "top.png"),
        (-90, -90,  "Bottom View",            "bottom.png"),
        (0,   0,    "Front View",             "front.png"),
        (0,   180,  "Back View",              "back.png"),
        (0,   -90,  "Left View",              "left.png"),
        (0,   90,   "Right View",             "right.png"),
    ]

    print(f"Rendering {len(views)} views to: {output_dir}/")
    for elev, azim, title, filename in views:
        render_view(geometries, elev, azim, title, os.path.join(output_dir, filename))

    # Combined grid
    fig, axes_arr = plt.subplots(2, 3, figsize=(18, 12),
                                  subplot_kw={'projection': '3d'}, facecolor='#2b2b2b')
    for ax_obj, (elev, azim, title, _) in zip(axes_arr.flat, views[1:]):
        ax_obj.set_facecolor('#2b2b2b')
        for verts, faces, color in geometries:
            polys = [[verts[idx] for idx in face] for face in faces]
            edge_color = tuple(max(0, c - 0.2) for c in color[:3])
            coll = Poly3DCollection(polys, alpha=color[3] if len(color) > 3 else 0.9)
            coll.set_facecolor(color[:3])
            coll.set_edgecolor((*edge_color, 0.3))
            coll.set_linewidth(0.3)
            ax_obj.add_collection3d(coll)
        all_verts = np.vstack([v for v, _, _ in geometries])
        mr = np.ptp(all_verts, axis=0).max() / 2 * 1.3
        mid = (all_verts.max(axis=0) + all_verts.min(axis=0)) / 2
        ax_obj.set_xlim(mid[0]-mr, mid[0]+mr); ax_obj.set_ylim(mid[1]-mr, mid[1]+mr)
        ax_obj.set_zlim(mid[2]-mr, mid[2]+mr)
        ax_obj.view_init(elev=elev, azim=azim)
        ax_obj.set_title(title, color='white', fontsize=12, fontweight='bold')
        ax_obj.tick_params(colors='gray', labelsize=5)
        ax_obj.xaxis.pane.fill = False; ax_obj.yaxis.pane.fill = False; ax_obj.zaxis.pane.fill = False
        ax_obj.xaxis.pane.set_edgecolor('#444444')
        ax_obj.yaxis.pane.set_edgecolor('#444444')
        ax_obj.zaxis.pane.set_edgecolor('#444444')
        ax_obj.grid(True, alpha=0.2)

    plt.suptitle("Shipyard Rover - All Views", color='white', fontsize=18, fontweight='bold')
    plt.tight_layout()
    combined = os.path.join(output_dir, "all_views_combined.png")
    plt.savefig(combined, dpi=150, bbox_inches='tight', facecolor='#2b2b2b')
    plt.close()
    print(f"  Saved: {combined}\n\nDone!")


if __name__ == "__main__":
    main()
