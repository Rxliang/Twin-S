# RoboMaster

Author: Ruixing Liang (rliang7@jh.edu) Hongchao Shu (hshu4@jhu.edu)

## Introduction

This is Repo tailored for accurate Synchronization between simulation in Virtual Reality (VR) and Real world, which is tracked by optical tracker using Atracsys fusionTrack 500 and Microscope (Haag-Streit Allegra 500). This repo is initallized for Computer Integrated Surgery II in Johns Hopkins University Robotics Course will further optimized and incorporated in MICCAI submission open source repo subsequently please follow and star for tracking the status. This could also be seen as a pre-release of AMBF plugin for optical tracker extensions. 

![](./Resources/Readme_1.png)

##Status

For now, we implemented and evaluated:

1. Pivot Calibration, Geometry Generation Functions to define concisely and compactly which only need marker information which is intended to expand usage to different platforms you may have in hand.
2. Camera Calibration (Still in Work Progress), Early implementation using preliminary OpenCV API has been identified as one of the problem we have in progress, since we pursue the highest accuracy we could possibly get. Stereo Camera Rectification with evaluations has been implemented.
3. Fixed Transformation from Known Geometry to Phantom or in other case your object of interest has been implement based on Open3D ICP API.
4. Hand Eye Calibration (Still in Work Progress), Early implementations based on ETH Dual Quaternion Algorithm has been tested. Further results will be needed before our final version of update.
5. VR Set Up(Still in Work Progress), Implementation based on AMBF framework and Pyvista has been tested, functional and refinement will be carried out before the release of first complete version.
6. Data Generation