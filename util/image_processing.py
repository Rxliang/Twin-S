import os
import cv2
from natsort import natsorted
import numpy as np
import matplotlib.pyplot as plt
import sys
from util.show_depthmap import SqueezedNorm
from matplotlib.widgets import Slider
import matplotlib.pyplot as plt
import matplotlib.colors
from Solver import solver

# Define the colorbar norm
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


fig, (ax, ax2, ax3) = plt.subplots(nrows=3, 
                                   gridspec_kw={"height_ratios":[3,2,1], "hspace":0.25})

# Visulize a adjustable colorbar
class adjustCbar:
    def __init__(self, colormap):
        self.x = np.linspace(0,1.4, 110)
        norm=SqueezedNorm(vmin=0, vmax=0.14, mid=0, s1=1.7, s2=4)

        self.line,  = ax.plot(self.x, norm(self.x))
        ax.margins(0)
        ax.set_ylim(0,1)

        self.im = ax2.imshow(np.atleast_2d(self.x).T, cmap=colormap, norm=norm, aspect="auto")
        self.cbar = fig.colorbar(self.im ,cax=ax3,ax=ax2, orientation="horizontal")
        midax = plt.axes([0.1, 0.04, 0.2, 0.03], facecolor="lightblue")
        s1ax = plt.axes([0.4, 0.04, 0.2, 0.03], facecolor="lightblue")
        s2ax = plt.axes([0.7, 0.04, 0.2, 0.03], facecolor="lightblue")

        self.mid = Slider(midax, 'Midpoint', self.x[0], self.x[-1], valinit=0)
        self.s1 = Slider(s1ax, 'S1', 0.5, 6, valinit=1.7)
        self.s2 = Slider(s2ax, 'S2', 0.5, 6, valinit=4)

    def update(self, val):
        norm=SqueezedNorm(vmin=0, vmax=0.26, mid=self.mid.val, s1=self.s1.val, s2=self.s2.val)
        self.im.set_norm(norm)
        self.cbar.update_bruteforce(self.im) 
        self.line.set_ydata(norm(self.x))
        fig.canvas.draw_idle() 

    def change_cbar(self):
        self.mid.on_changed(self.update)
        self.s1.on_changed(self.update)
        self.s2.on_changed(self.update)
        fig.subplots_adjust(bottom=0.15)
        plt.show()


def zoom_center(img, zoom_factor=1.0):

    y_size = img.shape[0]
    x_size = img.shape[1]
    
    # define new boundaries
    x1 = int(0.5*x_size*(1-1/zoom_factor))
    x2 = int(x_size-0.5*x_size*(1-1/zoom_factor))
    y1 = int(0.5*y_size*(1-1/zoom_factor))
    y2 = int(y_size-0.5*y_size*(1-1/zoom_factor))
    
    # first crop image then scale
    img_cropped = img[y1:y2,x1:x2]
    return cv2.resize(img_cropped, None, fx=zoom_factor, fy=zoom_factor)

def creatFolder(output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print("Create Folder at Designated Address...")


# this function is for read image,the input is directory name
def save_modified_images(limage_path, segm_path):
    global overlay_path, modified_limg_path, modified_seg_path
    array_of_img = [] # this if for store all of the image data
    img_list = os.listdir(limage_path)
    img_list = natsorted(img_list)
    count = 0
    creatFolder(overlay_path)
    creatFolder(modified_seg_path)
    creatFolder(modified_limg_path)
    
    for file in img_list:
        limg = cv2.imread(limage_path + "/" + file)
        segm = cv2.imread(segm_path + "/" + file)
        # new_limg = cv2.copyMakeBorder(limg, 230, 130, 0, 0, cv2.BORDER_CONSTANT)
        new_limg = cv2.copyMakeBorder(limg, 200 , 160, 0 , 0, cv2.BORDER_CONSTANT)
        new_limg = cv2.resize(new_limg, (640, 480))
        cv2.imwrite('segm.jpeg', segm)
        segm = zoom_center(segm)
        segm = segm[:480, :640]

        overlap = cv2.addWeighted(new_limg, 0.8, segm, 0.4, 0)
        count += 1
        print(count)
        # new_img = np.vstack([img, t])
        # print(new_img.shape)
        cv2.imshow('img', overlap)
        # cv2.waitKey(0)

        cv2.imwrite(overlay_path + "/" + file, overlap)
        cv2.imwrite(modified_seg_path + "/" + file, segm)
        cv2.imwrite(modified_limg_path + "/" + file, new_limg)
        # break
        # array_of_img.append(img)
        # print(array_of_img)


def modify_images(limage_path, segm_path):
    global overlay_path, modified_limg_path, modified_seg_path

    cv2.namedWindow('Image Pad Modify')
    cv2.createTrackbar('x1','Image Pad Modify',0,300,nothing)
    cv2.createTrackbar('x2','Image Pad Modify',0,300,nothing)
    cv2.createTrackbar('y1','Image Pad Modify',0,300,nothing)
    cv2.createTrackbar('y2','Image Pad Modify',0,300,nothing)

    switch = '0:OFF\n1:ON'
    cv2.createTrackbar(switch, 'Image Pad Modify',0,1,nothing)

    array_of_img = [] # this if for store all of the image data
    img_list = os.listdir(limage_path)
    img_list = natsorted(img_list)
    count = 0
    for file in img_list:
        while True:
        # print(file)
            limg = cv2.imread(limage_path + "/" + file)
            # print(limg.shape)
            segm = cv2.imread(segm_path + "/" + file)
            # print(segm.shape)
            x1 = cv2.getTrackbarPos('x1','Image Pad Modify')
            x2 = cv2.getTrackbarPos('x2','Image Pad Modify')
            y1 = cv2.getTrackbarPos('y1','Image Pad Modify')
            y2 = cv2.getTrackbarPos('y2','Image Pad Modify')
            print(x1,",",x2,",",y1,",",y2)
            # new_limg = cv2.copyMakeBorder(limg, 230, 220, 100, 0, cv2.BORDER_CONSTANT)
            new_limg = cv2.copyMakeBorder(limg, x1, x2, y1, y2, cv2.BORDER_CONSTANT)
            new_limg = cv2.resize(new_limg, (640, 480))

            # cv2.imwrite('segm.jpeg', segm)
            # segm = zoom_center(segm, 1.07)
            segm = segm[:480, :640]

            overlap = cv2.addWeighted(new_limg, 0.5, segm, 0.5, 0)
            count += 1
            print(count)
            # new_img = np.vstack([img, t])
            # print(new_img.shape)
            cv2.imshow('Image Pad Modify', overlap)
            cv2.waitKey(0)
        # cv2.imwrite(overlay_path + "/" + file, overlap)
        # cv2.imwrite(modified_seg_path + "/" + file, segm)
        # cv2.imwrite(modified_limg_path + "/" + file, new_limg)
        # break
        # array_of_img.append(img)
        # print(array_of_img)


def showDepth(scaled_depth, vmax, i):
    # norm=SqueezedNorm(vmin=0, vmax=0.0026, mid=0.13, s1=1.79, s2=2.45)
    fig = plt.figure()
    plt.imshow(scaled_depth, vmin=0, vmax=vmax, cmap='plasma') #twilight #binary
    # plt.colorbar(label='depth m', orientation='vertical', shrink=0.8)
    plt.axis('off')
    # plt.show()
    print(str(i))
    save_path = os.path.join(path, f'MR/{str(i)}.png')
    plt.savefig(save_path,  pad_inches=0, dpi=130,bbox_inches = 'tight')
    plt.close()

def colorFilter(img_path):
    frame = cv2.imread(img_path)
    # It converts the BGR color space of image to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # print(hsv[335,390,:])
    # Threshold of blue in HSV space
    lower_blue = np.array([121, 242, 133])
    upper_blue = np.array([121, 242, 135])

    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.bitwise_not(mask)
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv2.bitwise_and(frame, frame, mask = mask)

    tmp = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    _, alpha = cv2.threshold(tmp, 0, 255, cv2.THRESH_BINARY)
    b, g, r = cv2.split(result)
    rgba = [b, g, r, alpha]
    dst = cv2.merge(rgba, 4)
    # cv2.imshow('frame', frame)
    # cv2.imshow('mask', mask)
    cv2.imshow('result', result)
    cv2.imshow('dst', dst)
    # cv2.imwrite('result.png',dst)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



def processMR(MR_path, seg_path, limg_path):
    save_path =os.path.join(path, 'save')
    demo_path =os.path.join(path, 'demo2')
    mr_file_list = os.listdir(MR_path)
    mr_file_list = natsorted(mr_file_list)
    file_list = os.listdir(seg_path)
    file_list = natsorted(file_list)

    # for file in file_list:
    #     img = cv2.imread(os.path.join(seg_path, file))
    #     src = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #     # _, alpha = cv2.threshold(src,23, 60, cv2.THRESH_BINARY)
    #     # _, alpha = cv2.threshold(src,110, 255, cv2.THRESH_BINARY)
    #     alpha = np.array(src)
    #     alpha[alpha < 180] = 0
    #     alpha[alpha > 230] = 0
    #     b, g, r = cv2.split(img)
    #     print(alpha)
    #     rgba = [b, g, r, alpha]
    #     dst = cv2.merge(rgba, 4)
    #     plt.imshow(alpha, 'gray')
    #     plt.show()
    #     cv2.imshow('img', dst)
    #     print(dst.shape)
    #     # cv2.imwrite(os.path.join(save_path, file), dst)
    #     cv2.waitKey(0)
    #     break

    # for file in mr_file_list:
    #     img = cv2.imread(os.path.join(MR_path, file))
    #     src = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #     _, alpha = cv2.threshold(src,30, 100, cv2.THRESH_BINARY)
    #     b, g, r = cv2.split(img)
    #     # print(alpha)
    #     rgba = [b, g, r, alpha]
    #     dst = cv2.merge(rgba, 4)
    #     plt.imshow(alpha, 'gray')
    #     # plt.show()
    #     cv2.imshow('img', dst)
    #     # print(dst.shape)
    #     cv2.imwrite(os.path.join(save_path, file), dst)
    #     # cv2.waitKey(0)
    #     # break

    i=0
    for (mr, file) in zip(mr_file_list, file_list):
        segm = cv2.imread(os.path.join(seg_path, file))
        mr = cv2.imread(os.path.join(save_path, mr), cv2.IMREAD_UNCHANGED)
        limg = cv2.imread(os.path.join(limg_path, file))
        # over1 = cv2.addWeighted(limg, 1, mr, 0.6, 0)
        # over2 = cv2.addWeighted(limg, 0.5,over1, 0.5, 0)
        
        plt.imshow(limg[:,:,::-1])
        plt.imshow(mr)
        plt.axis('off')
        # plt.show()
        # cv2.imshow('img', over1)
        # cv2.waitKey(0)
        # cv2.imwrite(os.path.join(demo_path, file), over1)
        plt_path = os.path.join(demo_path, f'{str(i)}.png')
        plt.savefig(plt_path,  pad_inches=0, dpi=130,bbox_inches = 'tight')
        plt.close()
        i+=1
        # break

def nothing(x):
    pass

if __name__ == '__main__':
    sol = solver()
    # path = '/home/shc/Desktop/data/1023/sync_phacon_4'
    path = sys.argv[1]
    # path = '/home/shc/Desktop/data/1026/phacon_segm_depth_2'
    seg_path = os.path.join(path, 'segm_mask')
    limg_path = os.path.join(path, 'modified_limg')
    overlay_path = os.path.join(path, 'overlay')
    modified_seg_path = os.path.join(path, 'modified_segm_mask')
    modified_limg_path = os.path.join(path, 'modified_limg')
    MR_path = os.path.join(path, 'MR')
    # modify_images(limg_path, seg_path)
    # save_modified_images(limg_path, seg_path)
    # sol.img2video(path, 15, (640,480))
    # for i in range(630):
    #     img_1 = np.load(os.path.join(path, f'depth_map/{str(i)}.npy'))
    #     # img_2 = np.load(os.path.join(path, 'depth_map/278.npy'))
    #     # img_3 = np.load(os.path.join(path, 'depth_map/1366.npy'))
    #     # img_4 = np.load(os.path.join(path, 'depth_map/2205.npy'))
    #     img_5 = np.load(os.path.join(path, 'depth_map/630.npy'))
    #     depth_map = img_5-img_1
    #     depth_map[depth_map >= 0.004] = 0
    #     showDepth(depth_map, 1.2/100, i)
    # showDepth(img_1, 0.2)
    # 1 567 1556 2123 2554
    # 22 4 129
    # exp4: 0 278 725 1366 2205 2940
    processMR(MR_path, seg_path, limg_path)
    # acb = adjustCbar('jet')
    # acb.change_cbar()