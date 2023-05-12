import h5py
import cv2
import matplotlib.pyplot as plt
import os

num = 4
name = f'/media/shc/Elements/Twin-S_data/0505/hdf5_data/data_{num}.hdf5'
hf = h5py.File(name, 'r')
print(hf.visit(print))
metadata = hf.get('metadata')
data = hf.get('data')
T_cb_c = metadata['T_cb_c'][:]
T_db_d = metadata['T_db_d'][:]
T_pb_p = metadata['T_pb_p'][:]
# print(T_cb_c)
# print(T_db_d)
# print(T_pb_p)
extrinsic = metadata['camera_intrinsic'][:]
print(extrinsic)
T_o_cb = data['pose_pan'][:]

# print(T_o_cb)
l_img = data['l_img'][:]
r_img = data['r_img'][:]
segm = data['segm'][:]
# print(l_img[0].shape)
idx = 0
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 9
size = (640, 360)

videoWriter = cv2.VideoWriter(f'/media/shc/Elements/Twin-S_data/0505/hdf5_data/{num}.mp4', fourcc, fps, size)
for idx in range(len(l_img)):
    img = cv2.addWeighted(l_img[idx], 0.5, segm[idx], 0.5, 1)
    # img = r_img[idx]
    # cv2.imshow('',img)
    # cv2.waitKey(0)
    videoWriter.write(img)
    print(f'finished {idx}.')
videoWriter.release()
# depth = data['depth'][:]
# plt.imshow(depth[0])
# plt.show()
