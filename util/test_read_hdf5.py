import h5py
import cv2
import matplotlib.pyplot as plt

hf = h5py.File('/media/shc/Elements/Twin-S_data/0505/hdf5_data/20230510_184304.hdf5', 'r')
print(hf.visit(print))
metadata = hf.get('metadata')
data = hf.get('data')
T_cb_c = metadata['T_cb_c'][:]
T_db_d = metadata['T_db_d'][:]
T_pb_p = metadata['T_pb_p'][:]
print(T_cb_c)
print(T_db_d)
print(T_pb_p)

T_o_cb = data['pose_pan'][:]
# print(T_o_cb)
l_img = data['l_img'][:]
# print(l_img[0].shape)
# cv2.imshow('',l_img[0])
# cv2.waitKey(0)

depth = data['depth'][:]
plt.imshow(depth[0])
plt.show()
