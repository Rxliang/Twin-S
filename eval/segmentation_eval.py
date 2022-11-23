import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
from natsort import natsorted


def dice_coef(y_true, y_pred):

    y_true_f = y_true.flatten()
    y_pred_f = y_pred.flatten()
    intersection = np.sum(y_true_f * y_pred_f)
    smooth = 0.0001
    return (2. * intersection + smooth) / (np.sum(y_true_f) + np.sum(y_pred_f) + smooth)


def dice_coef_multilabel(y_true, y_pred, numLabels):
    dice=0
    for index in range(numLabels):
        dice += dice_coef(y_true[:,:,:,index], y_pred[:,:,:,index])
    return dice/numLabels # taking average


def get_masks(imgA):

    phacon_color = 76
    drill_color = 0
    ret, thresh1 = cv2.threshold(imgA, 200, 255, cv2.THRESH_BINARY)
    thresh1 = cv2.cvtColor(thresh1, cv2.COLOR_BGR2GRAY)

    ret, thresh2 = cv2.threshold(imgA, 135, 255, cv2.THRESH_BINARY)
    thresh2 = cv2.cvtColor(thresh2, cv2.COLOR_BGR2GRAY)

    phacon_mask = thresh1 == phacon_color
    drill_mask = thresh2 == drill_color

    # plt.imshow(masks[:, :, 1])
    # plt.show()
    return phacon_mask, drill_mask


if __name__ == '__main__':

    img_list = []
    source_masks = np.zeros([103, 480, 640, 2])
    target_masks = np.zeros([103, 480, 640, 2])
    source_path = '../../IPCAI/eval_segmentation/'
    target_path = '../../IPCAI/annotated_mask/'
    source_dir_name = ['segm_1023_4', 'segm_1026_1', 'segm_1026_2']
    target_dir_name = ['1023_4', '1026_1', '1026_2']
    count = 0

    # load all source masks
    for name in source_dir_name:
        source_dirs = os.listdir(source_path + f'{name}')
        for imgs in natsorted(source_dirs):
            img_list.append(os.path.join(source_path + 'limg' + f'{name}'[-7:], imgs))
            imgs = cv2.imread(os.path.join(source_path + f'{name}', imgs))
            cropped_imgs = imgs[75:425, :]
            imgs = cv2.copyMakeBorder(cropped_imgs, 75, 55, 0, 0, cv2.BORDER_CONSTANT, value=255)
            phacon_mask, drill_mask = get_masks(imgs)
            source_masks[count, :, :, 0] = phacon_mask
            source_masks[count, :, :, 1] = drill_mask
            count += 1

    # load all target masks
    count = 0
    for name in target_dir_name:
        target_dirs = os.listdir(target_path + f'{name}')
        for imgs in natsorted(target_dirs):
            entity = imgs[-6]
            imgs = cv2.imread(os.path.join(target_path + f'{name}', imgs))
            imgs = cv2.cvtColor(imgs, cv2.COLOR_BGR2GRAY)
            if entity == '1':
                target_masks[count, :, :, 0] = imgs >= 127
            else:
                target_masks[count, :, :, 1] = imgs >= 127
                count += 1


    for num in range(103):
        masks1 = []
        masks2 = []
        limg = cv2.imread(img_list[num])
        for index in range(2):
            # overlay = cv2.addWeighted(source_masks[num, :, :, index], 0.5, target_masks[num, :, :, index], 0.5, 0)
            temp1 = cv2.cvtColor((source_masks[num, :, :, index]*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            temp2 = cv2.cvtColor((target_masks[num, :, :, index]*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            # change color of masks
            if index == 0:
                temp1[source_masks[num, :, :, index] == 1] = (120, 240, 52)
                temp2[target_masks[num, :, :, index] == 1] = (120, 240, 52)
            else:
                temp1[source_masks[num, :, :, index] == 1] = (220, 0, 50)
                temp2[target_masks[num, :, :, index] == 1] = (220, 0, 50)

            masks1.append(temp1)
            masks2.append(temp2)
        m1 = cv2.addWeighted(masks1[0], 0.5, masks1[1], 0.5, 0)
        m2 = cv2.addWeighted(masks2[0], 0.5, masks2[1], 0.5, 0)
        overlay1 = cv2.addWeighted(m1, 0.5, limg, 0.5, 0)
        overlay2 = cv2.addWeighted(m2, 0.5, limg, 0.5, 0)

        overlay1 = overlay1[75:425, 20:]
        overlay2 = overlay2[75:425, 20:]

        plt.subplot(2, 1, 1)
        plt.imshow(overlay2)
        plt.axis('off')
        plt.subplot(2, 1, 2)
        plt.imshow(overlay1)
        plt.axis('off')
        # plt.show()
        # plt.savefig(source_path+f'/results/{num}.png')
    print('Finish saving results.')

    dice_score = dice_coef_multilabel(source_masks, target_masks, 2)
    print(f'For target and source {dice_score}')