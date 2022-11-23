
import os
import cv2
from natsort import natsorted
import numpy as np

# def adjust_img(img1,img2):
def nothing(x):
    pass

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
def modify_images(limage_path, segm_path):
    global overlay_path, modified_limg_path, modified_seg_path
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
            print(x1,x2,y1,y2)
            # new_limg = cv2.copyMakeBorder(limg, 230, 220, 100, 0, cv2.BORDER_CONSTANT)
            new_limg = cv2.copyMakeBorder(limg, x1, x2, y1, y2, cv2.BORDER_CONSTANT)
            new_limg = cv2.resize(new_limg, (640, 480))

            cv2.imwrite('segm.jpeg', segm)
            segm = zoom_center(segm)
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

cv2.namedWindow('Image Pad Modify')
cv2.createTrackbar('x1','Image Pad Modify',0,300,nothing)
cv2.createTrackbar('x2','Image Pad Modify',0,300,nothing)
cv2.createTrackbar('y1','Image Pad Modify',0,300,nothing)
cv2.createTrackbar('y2','Image Pad Modify',0,300,nothing)

switch = '0:OFF\n1:ON'
cv2.createTrackbar(switch, 'Image Pad Modify',0,1,nothing)
# cv2.createTrackbar('R_red','Image Pad Modify',0,255,nothing)

if __name__ == '__main__':
    path = '/Users/simon/Desktop/未命名文件夹/img_mask'
    seg_path = os.path.join(path, 'segm_mask')
    limg_path = os.path.join(path, 'limg')
    overlay_path = os.path.join(path, 'overlay')
    modified_seg_path = os.path.join(path, 'modified_segm_mask')
    modified_limg_path = os.path.join(path, 'modified_limg')
    creatFolder(overlay_path)
    creatFolder(modified_seg_path)
    creatFolder(modified_limg_path)
    modify_images(limg_path, seg_path)