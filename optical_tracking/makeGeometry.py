import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Helper functions
def obtain_positions(data, num_markers):
    """
    Obtains all positions in stream for each valid marker in input stream.
    Args:
        data: input data (dataframe)
        num_markers: desired number of markers for position analysis
    """
    poses_idxs = [i for i, j in enumerate(data.Values.str.find("x").tolist()) if j != -1]
    print(poses_idxs)
    dashed_idxs = [i for i, j in enumerate(data.Values.str.find("---").tolist()) if j != -1]

    pose_dict = {}
    for m in range(num_markers):
        pose_dict[m] = {"x": [], "y": [], "z": []}

    for p, idx in enumerate(poses_idxs):
        if data.Values.iloc[idx] == "poses: []":  # Filter timestamps with empty position info
            continue

        timestamp = data[idx:dashed_idxs[p + 1]]
        mpos_idxs = [i for i, j in enumerate(timestamp.Values.str.find("position:").tolist()) if j != -1]

        if len(mpos_idxs) != num_markers:  # Filter timestamps with more/less than established number of markers
            continue

        for pos_num, mpos_idx in enumerate(mpos_idxs):
            pose_dict[pos_num]["x"].append(float(timestamp.Values.iloc[mpos_idx + 1].strip().split(" ")[-1]))
            pose_dict[pos_num]["y"].append(float(timestamp.Values.iloc[mpos_idx + 2].strip().split(" ")[-1]))
            pose_dict[pos_num]["z"].append(float(timestamp.Values.iloc[mpos_idx + 3].strip().split(" ")[-1]))

    return pose_dict


def create_matrix(marker_dict, row_number):
    new_matrix = np.zeros((row_number, len(marker_dict.keys())))
    for m, means in marker_dict.items():
        new_matrix[:, m] = means

    return new_matrix


def marker_level_analysis(pose_dict):
    """
    Calculates mean position for each marker and returns 'leftmost' marker.
    Args:
        pose_dict: dictionary with absolute positions per valid marker
    """
    marker_means = {}
    for m in pose_dict.keys():
        marker_means[m] = [0, 0, 0]

    for marker, _ in pose_dict.items():
        if len(pose_dict[marker]["x"]) == 0:
            marker_means[marker] = [None, None, None]
        else:
            marker_means[marker][0] = np.mean(pose_dict[marker]["x"])
            marker_means[marker][1] = np.mean(pose_dict[marker]["y"])
            marker_means[marker][2] = np.mean(pose_dict[marker]["z"])

    return marker_means


def plot_points(marker_means, save=False):
    """
    Plot relative marker positions (relative to p1).
    Args:
        marker_means: dictionary with mean absolute positions per valid marker
        save (opt): Save a still copy of the visualization
    """
    fig = plt.figure()
    ax = Axes3D(fig)
    colors = ["b", "r", "g", "k"]

    for m_num, marker in enumerate(marker_means.values()):
        if None in marker:
            continue
        if m_num == 0:
            ax.scatter(marker[0], marker[1], marker[2], color=colors[m_num], label="Reference Marker", marker="X")
        else:
            ax.scatter(marker[0], marker[1], marker[2], color=colors[m_num], label="Marker no. {}".format(m_num + 1))

    ax.set_xlabel('x-axis (m)')
    ax.set_ylabel('y-axis (m)')
    ax.set_zlabel('z-axis (m)')
    plt.legend()
    plt.show()
    if save:
        plt.savefig("marker_visualization.png")


# Main workflow
def main(data_path, num_markers, reference, plot=True):
    in_data = pd.read_csv(data_path, delimiter="\n", header=None)
    in_data.columns = ["Values"]

    pose_dict = obtain_positions(data=in_data, num_markers=num_markers)
    marker_means = marker_level_analysis(pose_dict=pose_dict)
    mean_matrix = create_matrix(marker_dict=marker_means, row_number=3).T

    m1 = mean_matrix[0] - mean_matrix[reference]
    m2 = mean_matrix[1] - mean_matrix[reference]
    m3 = mean_matrix[2] - mean_matrix[reference]
    m4 = mean_matrix[3] - mean_matrix[reference]

    dict = {1: m1, 2: m2, 3: m3, 4: m4}
    plot_points(dict, save=plot)

    print("-" * 60)

    print("Relative position of M1 to Reference:")
    print(m1 * 1000)
    print("\n")
    print("Relative position of M2 to Reference:")
    print(m2 * 1000)
    print("\n")
    print("Relative position of M3 to Reference:")
    print(m3 * 1000)
    print("\n")
    print("Relative position of M4 to Reference:")
    print(m4 * 1000)
    print("-" * 60)


if __name__ == "__main__":
    data_path = "Atracsys_test_1_GUI_out.csv"
    main(data_path=data_path, num_markers=4, reference=1, plot=True)