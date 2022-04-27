# Copyright 2021 Stuart Duncan
from dataLoader import dataLoader
from scipy.spatial.transform import Rotation
import numpy as np
import pandas as pd
from Solver import solver
sol = solver()

def calibrate_stylus(data, swap_w = True ):
  '''Pivot calibration for a 3D stylus.

  data is an Nx7 numpy array of the poses in translation-quaternion encoding
  with collumns in tx,ty,tz,qw,qx,qy,qz order. If your quaternions are in
  qx,qy,qz,qw order then set swap_w to False

  Returns the calibration and RMS residual error
  t_stylus: The stylus tip position in the local coordinate space of the tracked object (t_tip)
  t_pivotp: The pivot point that was used during calibration (p_dimple)
  residual: RMS residual error

  '''
  t_i = data[:, 0:3]
  q_i = data[:, 3:7]

  if swap_w:
    # from_quat expects xyzw but data is commonly in wxyz order
    # this swaps the first and last components of the quaternion data
    q_i[:, [0, 3]] = q_i[:, [3, 0]]

  # Create arrays which match the dims and shapes expected by linalg.lstsq
  t_i_shaped = np.array(t_i.reshape(-1,1))
  r_i = Rotation.from_quat(q_i)
  r_i_shaped = []
  for r in r_i:
    r_i_shaped.extend(np.concatenate((r.as_matrix(), -np.identity(3)),axis=1))
  r_i_shaped = np.stack(r_i_shaped)

  # Run least-squares, extract the positions
  lstsq = np.linalg.lstsq(r_i_shaped, -t_i_shaped, rcond=None)
  t_stylus = lstsq[0][0:3]
  t_pivotp = lstsq[0][3:6]
  # print('lstsq0',t_i_shaped)
  # Calculate the 3D residual RMS error
  residual_vectors = np.array((t_i_shaped + r_i_shaped@lstsq[0]).reshape(len(t_i),3))
  residual_norms   = np.linalg.norm(residual_vectors,axis=1)
  residual_rms     = np.sqrt(np.mean(residual_norms ** 2))

  return {
    't_tip': t_stylus[:, 0],
    'p_dimple': t_pivotp[:, 0],
    'residual': residual_rms}

if __name__ == '__main__':
  ld = dataLoader()
  data = []
  csv_data = pd.read_csv('drill_calibration/drill_calibration_3.csv')
  # print(csv_data.head())
  num_frames = 500
  for i in range(num_frames):
    data.append(ld.getToolPose(i, csv_data))
  data = np.vstack(data)
  res = calibrate_stylus(data, swap_w=False)
  t_tip = res['t_tip']
  p = res['p_dimple']
  print('t_tip:',t_tip, "\np:", p)
  # np.linalg.norm(res)
