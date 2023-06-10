import h5py
import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os

class SqueezedNorm(matplotlib.colors.Normalize):
    def __init__(self, vmin=None, vmax=None, mid=0, s1=2, s2=2, clip=False):
        self.vmin = vmin # minimum value
        self.mid  = mid  # middle value
        self.vmax = vmax # maximum value
        self.s1=s1; self.s2=s2
        f = lambda x, zero,vmax,s: np.abs((x-zero)/(vmax-zero))**(1./s)*0.5
        self.g = lambda x, zero,vmin,vmax, s1,s2: f(x,zero,vmax,s1)*(x>=zero) - \
                                             f(x,zero,vmin,s2)*(x<zero)+0.5
        matplotlib.colors.Normalize.__init__(self, vmin, vmax, clip)

    def __call__(self, value, clip=None):
        r = self.g(value, self.mid,self.vmin,self.vmax, self.s1,self.s2)
        return np.ma.masked_array(r)
    
norm=SqueezedNorm(vmin=0, vmax=0.26, mid=0.2386, s1=1.79, s2=2.45)

def readHdf5(num):
    name = f'/media/shc/Elements/Twin-S_data/0505/hdf5_data/data_{num}.hdf5'
    hf = h5py.File(name, 'r')
    data = hf.get('data')
    # print(T_o_cb)
    l_img = data['l_img'][:]
    # r_img = data['r_img'][:]
    segm = data['segm'][:]
    depth = data['depth'][:]
    # print(l_img[0].shape)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 9
    size = (640, 360)

    videoWriter = cv2.VideoWriter(f'/media/shc/Elements/Twin-S_data/0505/hdf5_data/l_{num}.mp4', fourcc, fps, size)
    for idx in range(len(l_img)):
        # img = cv2.addWeighted(l_img[idx], 0.5, segm[idx], 0.5, 1)
        img = l_img[idx]
        # img = segm[idx]
        videoWriter.write(img)
        print(f'finished {idx}.')
    videoWriter.release()


num = 0

# for i in range(5):
#     num = i+1
#     name = f'/media/shc/Elements/Twin-S_data/0505/hdf5_data/data_{num}.hdf5'
#     hf = h5py.File(name, 'r')

#     # print(hf.visit(print))
#     # metadata = hf.get('metadata')
#     data = hf.get('data')
#     l_img = data['l_img'][:]
#     # r_img = data['r_img'][:]
#     segm = data['segm'][:]
#     depth = data['depth'][:]
#     # print(l_img[0].shape)

#     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#     fps = 9
#     size = (640, 360)

#     videoWriter = cv2.VideoWriter(f'/media/shc/Elements/Twin-S_data/0505/hdf5_data/segm_{num}.mp4', fourcc, fps, size)
#     for idx in range(len(l_img)):
#         # img = cv2.addWeighted(l_img[idx], 0.5, segm[idx], 0.5, 1)
#         # img = l_img[idx]
#         img = segm[idx]
#         videoWriter.write(img)
#         print(f'finished {idx}.')
#     videoWriter.release()

readHdf5('5_1')

    # plt.imshow(depth[0], vmin=0.1, vmax=0.4, cmap='Blues', norm=norm) #twilight #binary
    # # plt.colorbar(label='depth m', orientation='vertical', shrink=0.8)
    # plt.axis('off')
    # plt.show()
    # break