# Copyright (c) Meta Platforms, Inc. and affiliates.

import os
import re
import time
from argparse import ArgumentParser

import cv2
import numpy as np
import open3d as o3d
from natsort import natsorted
from tqdm import tqdm


class InteractivePCDVisualizer(object):
    def __call__(self, pcd_list):
        o3d.visualization.draw_geometries(pcd_list)


class VideoPCDVisualizer(object):
    def __init__(self, save_path, frame_rate, up, lookat, front, zoom, size=(1920, 1043)):
        self.vis = o3d.visualization.Visualizer()
        self.frame_rate = float(frame_rate)
        self.save_path = save_path
        self.width, self.height = size
        self.up = up
        self.lookat = lookat
        self.front = front
        self.zoom = zoom

    def __call__(self, frames_pcds):
        """
        frames_pcds is a list of lists. The outer list holds the frame
        pointclouds for the video. The inner list holds the pointclouds for each frame.
        pointclouds must be o3d.geometry.PointCloud() objects
        """
        self.vis.create_window(width=self.width, height=self.height)

        rgb_list = []
        
        output_file = cv2.VideoWriter(
            filename=self.save_path,
            fourcc=cv2.VideoWriter_fourcc(*"mp4v"),
            fps=self.frame_rate,
            frameSize=(self.width, self.height),
            isColor=True,
        )

        for frame_index, frame_pcds in enumerate(frames_pcds):
            ctr = self.vis.get_view_control()

            for pcd in frame_pcds:
                reset_bounding_box = False if frame_index > 0 else True
                self.vis.add_geometry(pcd, reset_bounding_box=reset_bounding_box)

            if frame_index == 0:
                ctr.set_up(self.up)
                ctr.set_lookat(self.lookat)
                ctr.set_front(self.front)
                ctr.set_zoom(self.zoom)

            opt = self.vis.get_render_option()
            opt.point_size = args.point_size
            opt.background_color = [0, 0, 0]
            self.vis.poll_events()
            self.vis.update_renderer()

            for i, frame_pcd in enumerate(frame_pcds):
                self.vis.remove_geometry(frame_pcd, reset_bounding_box=False)

            rgb = self.vis.capture_screen_float_buffer()
            rgb = np.array(rgb) * 255
            rgb = rgb[:, :, ::-1].astype(np.uint8)
            print(rgb.shape)
            output_file.write(rgb)
            time.sleep(1.0 / self.frame_rate)

        output_file.release()


class PCDBuilder(object):
    def __init__(self, fx, fy, cx, cy, baseline):
        self.camera = o3d.camera.PinholeCameraIntrinsic()
        self.camera.intrinsic_matrix = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        self.baseline = baseline

    def pcd_from_rgbd(self, color, disp, disp_trunc, remove_flying):
        disp[disp < disp_trunc[0]] = 0.0
        disp[disp > disp_trunc[1]] = 0.0

        color_raw = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))

        depth_raw = self.camera.intrinsic_matrix[0, 0] / (disp + 1e-5) * self.baseline
        depth_raw = o3d.geometry.Image(depth_raw)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_raw, depth_raw, depth_trunc=3.0, convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.camera)
        if remove_flying:
            pcd, _ = pcd.remove_statistical_outlier(10, 5)
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        return pcd

    def __call__(self, color, depth, depth_trunc, remove_flying):
        frame_pcds = []
        for idx, img in enumerate(tqdm(color, desc="Creating pcds")):
            single_frame_pcds = [
                self.pcd_from_rgbd(img, depth[idx], depth_trunc, remove_flying)]
            frame_pcds.append(single_frame_pcds)

        return frame_pcds


def load_depth_path(color_path, revise_keys=[('img_left', 'Depth'), ('RGB_0_Rectified', 'Depth_sf')]):
    depth_path = color_path
    for p, r in revise_keys:
        depth_path = re.sub(p, r, depth_path)
    return depth_path


def main(args):

################################
    dir = '/media/shc/Elements/Twin-S_data/post_sync/post_sync_z/'
    T_c_p = np.load(os.path.join(dir, f'T_c_p_real_offset.npy'))
    zed_pcd_dir = os.path.join(dir, 'zed_depth_pcd')
    ambf_pcd_dir = os.path.join(dir, 'depth_pcd')
    T_c_p[...,:3 ,3] /= 1000

    zed_pointclouds = natsorted([os.path.join(zed_pcd_dir, f) for f in os.listdir(zed_pcd_dir) if ".ply" in f])
    ambf_pointclouds = natsorted([os.path.join(ambf_pcd_dir, f) for f in os.listdir(ambf_pcd_dir) if ".ply" in f])
    frame_pcds = []
    # visulize result
    if args.zed:
        for [count,pointcloud] in enumerate(zip(zed_pointclouds, ambf_pointclouds)):
            if count <150:
                continue
            else:
                pcd_zed = o3d.io.read_point_cloud(os.path.join(zed_pcd_dir, pointcloud[0]))
                pcd_ambf = o3d.io.read_point_cloud(os.path.join(ambf_pcd_dir, pointcloud[1]))
                pcd_ambf.paint_uniform_color([1, 0.706, 1])  
                pcd_target = o3d.io.read_point_cloud("../data/phantom_point-cloud_data/phacon_exp_3_ds.ply")
                pcd_target.transform(T_c_p[count])
                print(f'Appending {count} frames.')
                frame_pcds.append([pcd_zed, pcd_ambf, pcd_target])
    else:
        for [count,pointcloud] in enumerate(ambf_pointclouds):
            if count <0:
                continue
            else:
                pcd_ambf = o3d.io.read_point_cloud(os.path.join(ambf_pcd_dir, pointcloud))
                pcd_ambf.paint_uniform_color([1, 0.706, 1])  
                pcd_target = o3d.io.read_point_cloud("../data/phantom_point-cloud_data/phacon_exp_3_ds.ply")
                pcd_target.transform(T_c_p[count])
                print(f'Appending {count} frames.')
                frame_pcds.append([pcd_ambf, pcd_target])


    if not args.video:
        InteractivePCDVisualizer()(frame_pcds[0])
    else:
        front=[ -0.3509939981295529, -0.0013414501852417977, -0.93637674778287405 ]
        lookat=[ -0.0044998621935288937, -0.029191735052132683, 0.34626969113492007 ]
        up=[ 0.90668866085024358, 0.24932730419749116, -0.34022282061496678 ]
        zoom=0.38000000000000006
        VideoPCDVisualizer(args.output, args.frame_rate, up, lookat, front, zoom)(frame_pcds)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--video", action='store_true', help='Save visualization to video')
    parser.add_argument("--frame-rate", default=8)
    # parser.add_argument("--input", help="Directory to input images")
    # parser.add_argument("--depth", help="Directory to depth images")
    parser.add_argument("--output", help="A file or directory to save output visualizations. "
                                         "If not given, will show output in an OpenCV window.")
    # parser.add_argument("--fx", default=51.2 / 36 * 1024,
    #                     type=float, help="focal length along x-axis (longer side) in pixels")
    # parser.add_argument("--fy", default=None,
    #                     type=float, help="focal length along y-axis (shorter side) in pixels")
    # parser.add_argument("--cx", default=None, type=float, help="centre of image along x-axis")
    # parser.add_argument("--cy", default=None, type=float, help="centre of image along y-axis")
    # parser.add_argument("--baseline", default=1.0, type=float, help="baseline")
    # parser.add_argument("--shape", type=int, nargs="+", default=[1600, 1200], help="input image size [W, H]")
    # parser.add_argument("--disp_trunc", type=float, nargs='+', default=[1.0, 210.0])
    # parser.add_argument("--shrink", nargs='+', type=float, default=[0.1] * 4, help='left top right bottom')
    parser.add_argument("--point_size", type=int, default=3)
    parser.add_argument("--zed", action='store_true', help='With zed pointcloud.')
    # parser.add_argument("--num_frames", default=-1, type=int)
    # parser.add_argument("--remove_flying", action='store_true')
    # parser.add_argument("--start_frame", type=int, default=0)
    args = parser.parse_args()

    # point_size = args.point_size

    main(args)
