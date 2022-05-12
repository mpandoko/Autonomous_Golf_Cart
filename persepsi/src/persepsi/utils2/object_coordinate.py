#tugas akhir by michael
import torch
import numpy as np
import pyrealsense2 as rs

#Calculating Depth
def calcdepth(xywh, aligned_df):
    dist_array = np.array([np.array([])])
    for  j in xywh: #loop for all bboxes
        xc = int(j[0].item())
        yc = int(j[1].item())
        dist = np.array([np.array([aligned_df.get_distance(xc, yc)])])
        # dist = np.array([np.array([distance[yc, xc]])])
        dist_array = np.concatenate((dist_array, dist), axis=0) if dist_array.size else dist
    # Get data scale from the device and convert to meters
    # z =  np.multiply(dist_array, depth_scale)
    # Same type with xywhs
    return torch.tensor(dist_array).cuda()

    #https://github.com/IntelRealSense/librealsense/issues/6749 this potentially is more accurate, try later
    
def calcdepth2(xyxy, verts):
    dist_array = np.array([np.array([])])
    for j in xyxy: #loop for all bboxes
        x1 = int(j[0].item())
        y1 = int(j[1].item())
        x2 = int(j[2].item())
        y2 = int(j[3].item())
        object_points = verts[y1:y2, x1:x2].reshape(-1, 3)
        # Get z points
        if not object_points.size:
            break
        zs = object_points[:, 2]
        dist = np.array([np.array([np.percentile(zs, 25)])])
        dist_array = np.concatenate((dist_array, dist), axis=0) if dist_array.size else dist
    return torch.tensor(dist_array).cuda()

# Pixel to point
def pixel_to_point(det, color_intrin):
    xy_m = rs.rs2_deproject_pixel_to_point(color_intrin, det[:2], det[4])
    det[:2] = xy_m[:2]
    return det

def point_to_pixel(det, color_intrin):
    xy = rs.rs2_project_point_to_pixel(color_intrin, [det[0], det[1], det[4]])
    det[:2] = xy
    return det

def track_point_to_pixel(det, color_intrin):
    ret = det.mean[:5].copy()
    xy = rs.rs2_project_point_to_pixel(color_intrin, [ret[0], ret[1], ret[4]])
    ret[:2] = xy
    ret[2] *= ret[3]
    ret[:2] -= ret[2:4] / 2
    return ret