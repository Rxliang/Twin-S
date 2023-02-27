# -*-coding: utf-8- -*-

import numpy as np
# from dataLoader import dataLoader
import matplotlib.pyplot as plt
import pandas as pd
from decimal import Decimal
from scipy import stats
import seaborn as sns

# ld = dataLoader()


def getPlacement(dir):
    csv_data = pd.read_csv(dir)
    tool_placement = csv_data.iloc[:, 9:12] * 1000
    tool_placement = tool_placement.to_numpy()
    return tool_placement


def trajectory(dir, points):
    tool_placement = getPlacement(dir)
    tool_placement = tool_placement - tool_placement[0]
    gt_1 = np.array([np.array([0, 0, 0]), np.array([0, -2, 0]), np.array([2, -2, 0]), np.array([2, -2, -2])])
    gt_2 = np.array([np.array([0, 0, 0]), np.array([0, -5, 0]), np.array([5, -5, 0]), np.array([5, -5, 5])])
    # print(np.linalg.norm(gt_2[3] - tool_placement[-1]))
    points = points - points[0]
    print(np.linalg.norm(gt_1[3] - points[3]))
    x = tool_placement[:, 0]
    y = tool_placement[:, 1]
    z = tool_placement[:, 2]
    # x_eval = points[:, 0]
    # y_eval = points[:, 1]
    # z_eval = points[:, 2]
    x_gt1 = gt_1[:, 0]
    y_gt1 = gt_1[:, 1]
    z_gt1 = gt_1[:, 2]

    x_gt2 = gt_2[:, 0]
    y_gt2 = gt_2[:, 1]
    z_gt2 = gt_2[:, 2]

    ax = plt.subplot(projection='3d')
    ax.set_title('5mm placement for each axis')
    ax.scatter(x, y, z, c='r', alpha=0.1, linewidths=0.1)

    # ax.scatter(x_gt1, y_gt1, z_gt1, c='b', linewidths=5)
    # ax.plot(x_gt1, y_gt1, z_gt1, c='b', linewidth=2)

    ax.scatter(x_gt2, y_gt2, z_gt2, c='b', linewidths=5)
    ax.plot(x_gt2, y_gt2, z_gt2, c='b', linewidth=2)

    ax.set_xlabel('X/mm')
    ax.set_ylabel('Y/mm')
    ax.set_zlabel('Z/mm')
    plt.show()


def l2dis(dir):
    tool_placement = getPlacement(dir)
    dis_list = np.zeros(len(tool_placement))
    for i in range(len(tool_placement)):
        dis_list[i] = np.linalg.norm(tool_placement[i] - tool_placement[0])
        dis_list[i] = Decimal(dis_list[i]).quantize(Decimal('0.00'))
    # import collections
    # c = collections.Counter(dis_list)
    # print('c', c)
    # sns.histplot(data=dis_list)
    # print(np.sqrt(3 * 2 ** 2))
    return dis_list


def Analyze(dir1, dir2, dir5):
    def placement(input):
        p = np.zeros(len(input)-1)
        for i in range(len(input)-1):
            p[i] = np.linalg.norm(input[i+1] - input[i])
        return p

    tool_placement1 = getPlacement(dir1)
    tool_placement2 = getPlacement(dir2)
    tool_placement5 = getPlacement(dir5)
    dict0 = {0: 0, 1: 5000, 2: 11000, 3: 14500}  # dict for test1
    dict1 = {0: 0, 1: 4000, 2: 8900, 3: 13000}  # dict for test2
    dict2 = {0: 300, 1: 2800, 2: 6500, 3: 9900}  # dict for test3
    points0 = np.array([tool_placement1[dict0[i]:dict0[i] + 50] for i in range(4)])
    points1 = np.array([tool_placement2[dict1[i]:dict1[i] + 50] for i in range(4)])
    dis_list0 = []
    dis_list1 = []
    dis_list2 = []
    dis_listx = []
    dis_listy = []
    dis_listz = []
    points2 = np.array([tool_placement5[dict2[i]:dict2[i] + 50] for i in range(4)])
    for i in range(4):
        np.random.shuffle(points0[i])

    for j in range(len(points0[0])):
        # dis_list0.append(np.linalg.norm(points0[0, j] - points0[1, j])-5)
        # dis_list0.append(np.linalg.norm(points0[1, j] - points0[2, j])-5)
        dis_list0.append(np.linalg.norm(points0[2, j] - points0[3, j])-4.10)

        dis_listx.append(np.linalg.norm(points0[0, j] - points0[1, j])-5)
        dis_listy.append(np.linalg.norm(points0[1, j] - points0[2, j])-5)
        dis_listz.append(np.linalg.norm(points0[2, j] - points0[3, j]))
    dis_list0 = np.array(dis_list0)

    for i in range(4):
        np.random.shuffle(points1[i])
    for j in range(len(points1[0])):
        # dis_list1.append(np.linalg.norm(points1[0, j] - points1[1, j])-5)
        # dis_list1.append(np.linalg.norm(points1[1, j] - points1[2, j])-5)
        dis_list1.append(np.linalg.norm(points1[2, j] - points1[3, j])-3.65)
        
        dis_listx.append(np.linalg.norm(points0[0, j] - points0[1, j])-5)
        dis_listy.append(np.linalg.norm(points0[1, j] - points0[2, j])-5)
        dis_listz.append(np.linalg.norm(points0[2, j] - points0[3, j]))
    dis_list1 = np.array(dis_list1)
    
    for i in range(4):
        np.random.shuffle(points2[i])
    for j in range(len(points2[0])):
        # dis_list2.append(np.linalg.norm(points2[0, j] - points2[1, j])-5)
        # dis_list2.append(np.linalg.norm(points2[1, j] - points2[2, j])-5)
        dis_list2.append(np.linalg.norm(points2[2, j] - points2[3, j])-4.10)

        dis_listx.append(np.linalg.norm(points0[0, j] - points0[1, j])-5)
        dis_listy.append(np.linalg.norm(points0[1, j] - points0[2, j])-5)
        dis_listz.append(np.linalg.norm(points0[2, j] - points0[3, j]))
        # dis_list2.append(np.linalg.norm(points2[3, j]))
    dis_list2 = np.array(dis_list2)

    dis_listx = np.array(dis_listx)
    dis_listy = np.array(dis_listy)
    dis_listz = np.array(dis_listz)

    return dis_list0, dis_list1, dis_list2, dis_listx, dis_listy, dis_listz

if __name__ == '__main__':
    dir1 = '../data/atracsys_sensitivity_eval_data/atracsys_5mm_test1.csv'
    dir2 = '../data/atracsys_sensitivity_eval_data/atracsys_5mm_test2.csv'
    dir3 = '../data/atracsys_sensitivity_eval_data/atracsys_5mm_test3.csv'
    dis_list0, dis_list1, dis_list2, dis_list = Analyze(dir1, dir2, dir3)
    # total_list = np.hstack((2-dis_list1, 5-dis_list2))
    # x = np.array([i for i in range(150)])
    # plt.plot(x, dis_list1)

    # plt.subplot(1,3,1)
    # sns.distplot(dis_list0, bins=50, hist=True, kde=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"})
    # plt.subplot(1, 3, 2)
    # sns.distplot(dis_list1, bins=50, hist=True, kde=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"})
    # plt.subplot(1, 3, 3)
    # sns.distplot(dis_list2, bins=50, hist=True, kde=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"})
    plt.rc('xtick', labelsize=16)
    plt.rc('ytick', labelsize=16)
    plt.rc('axes', labelsize=16)

    plt.xlabel('mm')
    plt.subplot(1, 1, 1)
    sns.distplot(dis_list, bins=30, hist=True, kde=True, rug=False,
                 hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"})

    # sns.set(font_scale=1.8)
    # plt.subplot(1,3,1)
    # sns.distplot(dis_list0, bins=20, hist=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"},axlabel='mm')
    # plt.subplot(1, 3, 2)
    # sns.distplot(dis_list1, bins=20, hist=True, kde=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"}, axlabel='mm')
    # plt.subplot(1, 3, 3)
    # sns.distplot(dis_list2, bins=20, hist=True, kde=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"}, axlabel='mm')

    print('test1', np.mean(dis_list), np.std(dis_list),np.max(dis_list))
    # print('test2', np.mean(dis_list1), np.std(dis_list2))
    # print('test3', np.mean(dis_list2), np.std(dis_list2))
    plt.show()
    # dis_list = l2dis(dir3)
    # x = np.array([i for i in range(len(dis_list))])
    # plt.plot(x, dis_list)
    # plt.show()