import pandas as pd
import numpy as np
import json
from util.Solver import solver
sol = solver()

class dataLoader:

    def __init__(self):
        return

    def getMarkerPose(self, frame, marker_num, csv_data):
        '''
            Read the marker pose and list as numpy array [tx,ty,tz].
        '''
        marker_pose = csv_data.iloc[frame, 10*(marker_num+1):10*(marker_num+1)+3] * 1000
        marker_pose = marker_pose.to_numpy()
        return marker_pose

    def makeFrame(self, frame_idx, csv_data, marker_num=5):
        '''
        makes 5 markers coordinates an array of frame
        '''
        markers = []
        for i in range(marker_num):
            markers.append(self.getMarkerPose(frame_idx, i, csv_data))
        markers = np.vstack(markers)
        markers = markers.astype('float64')
        return markers

    def makePanelFrame(self, frame_idx, csv_data):
        '''
        makes 4 markers coordinates an array of frame
        '''
        markers = []
        marker_num = [5, 6, 7, 8]
        for i in marker_num:
            markers.append(self.getMarkerPose(frame_idx, i, csv_data))
        markers = np.vstack(markers)
        markers = markers.astype('float64')
        return markers

    def getCentroid(self, initFilePath, markerNum, panel=False):
        '''
        get the centroid of 5 markers
        centralize those markers
        '''
        csv_data = pd.read_csv(initFilePath)
        frames = []
        normal_frames = []
        norms = []
        for i in range(400):
            if panel == False:
                frames.append(self.makeFrame(10+i, csv_data, markerNum))
            else:
                frames.append(self.makePanelFrame(10 + i, csv_data))
        frames = np.array(frames)
        frame0 = frames.mean(0)
        frame = frame0.mean(0)
        print("center", frame)
        for i in range(len(frame0)):
            normal_frames.append(frame0[i] - frame)
        normal_frames = np.vstack(normal_frames)
        # for i in range(len(normal_frames)):
        #     for j in range(len(normal_frames)):
        #         norms.append(np.linalg.norm(normal_frames[i] - normal_frames[j]))
        # print(norms)
        return normal_frames

    def frames(self, num_frames, csv_data, panel=False):
        if panel == False:
            fs = [self.makeFrame(0, csv_data)]
            # print('len csv', len(csv_data))
            start = int(np.random.random() * 50)
            frame_num = np.linspace(start, len(csv_data), num_frames, dtype=int)
            print('frame:', len(frame_num))
            # for i in range(num_frames):
            for i in frame_num:
                # frame_num = int(np.random.random() * len(csv_data))
                # fs.append(self.makeFrame(frame_num, csv_data))
                fs.append(self.makeFrame(i-1, csv_data))
        else:
            fs = [self.makePanelFrame(0, csv_data)]
            start = int(np.random.random() * 50)
            frame_num = np.linspace(start, len(csv_data), num_frames, dtype=int)
            print('frame:', len(frame_num))
            for i in frame_num:
                fs.append(self.makePanelFrame(i - 1, csv_data))
        return fs

    def getToolPose(self, frame, csv_path):
        '''
        get the pose data from Surgical_tool topic as:
        [tx, ty, tz, qx, qy, qz, qw]
        @param frame: index of the frame get the data from
        '''
        csv_data = pd.read_csv(csv_path, header=None)
        tool_placement = csv_data.iloc[frame, 1:4] * 1000
        tool_orientation = csv_data.iloc[frame, 4:]
        tool_placement = tool_placement.to_numpy()
        tool_orientation = tool_orientation.to_numpy()
        tool_pose = np.hstack((tool_placement, tool_orientation))
        tool_pose = tool_pose.astype('float64')
        return tool_pose


    def loadJson(self, path):
        '''
        load json file from path.
        '''
        with open(path, 'r') as load_f:
            load_dict = json.load(load_f)
        # print('dict', load_dict)
        ls = [load_dict['pose']['position']['x']*1000, load_dict['pose']['position']['y']*1000,
              load_dict['pose']['position']['z']*1000, load_dict['pose']['orientation']['x'],
              load_dict['pose']['orientation']['y'], load_dict['pose']['orientation']['z'],
              load_dict['pose']['orientation']['w']]
        ls = np.array(ls)
        return ls

    def loadHandeyeJson(self, path):
        f = open(path + 'calibration_optimized.json')
        hand_eye_info = json.load(f)
        quaternion_X = []
        for j in hand_eye_info["translation"]:
            quaternion_X.append(float(hand_eye_info["translation"][j])*1000)
        for i in hand_eye_info["rotation"]:
            quaternion_X.append(float(hand_eye_info["rotation"][i]))

        _, X = sol.seven2trans(np.array(quaternion_X))
        np.save(path+'hand_eye_X.npy', X)
        return X

    def getRealPose(self, idx, dirPath):
        df = pd.read_csv(dirPath)
        pose_x = df['pose.position.x'][idx] * 1000
        pose_y = df['pose.position.y'][idx] * 1000
        pose_z = df['pose.position.z'][idx] * 1000
        orin_x = df['pose.orientation.x'][idx]
        orin_y = df['pose.orientation.y'][idx]
        orin_z = df['pose.orientation.z'][idx]
        orin_w = df['pose.orientation.w'][idx]
        pose = np.array([pose_x, pose_y, pose_z, orin_x, orin_y, orin_z, orin_w])
        # print(real_pose, '\n', pose)
        return pose

    def getTimestamp_list(self, csv_path):
        csv_data = pd.read_csv(csv_path, header=None)
        timestamp_list = []
        for i in range(len(csv_data)):
            timestamp = round(csv_data.iloc[i, 0], 5)
            timestamp_list.append(timestamp)
        # timestamp_list = timestamp_list.astype('float64')
        return timestamp_list

if __name__ == '__main__':
    ld = dataLoader()
    # tool_pose = ld.getToolPose(0, '/home/shc/Desktop/data/915/handeye_imu_2/cam_pose.csv')
    # tl = ld.getTimestamp_list('/home/shc/Desktop/data/915/handeye_imu_2/cam_pose.csv')
    # print(tl)