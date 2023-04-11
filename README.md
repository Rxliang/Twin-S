# Twin-S

This is the official code for our paper accepted at IPCAI 2023 and IJCARS - [Twin-S: A Digital Twin Paradigm for Skull Base Surgery](https://arxiv.org/abs/2211.11863).

![](Resources/demo_video.gif)

If you find our project useful, please cite
```
@article{shu2022twin,
  title={Twin-S: A Digital Twin for Skull-base Surgery},
  author={Shu, Hongchao and Liang, Ruixing and Li, Zhaoshuo and Goodridge, Anna and Zhang, Xiangyu and Ding, Hao and Nagururu, Nimesh and Sahu, Manish and Creighton, Francis X and Taylor, Russell H and others},
  journal={arXiv preprint arXiv:2211.11863},
  year={2022}
}
```

## Methodology

We present a digital twin framework for skull base surgery named Twin-S. It models and tracks the critical
components of skull-base surgery, including the surgical tool, patient anatomy,
and surgical camera. Significantly, Twin-S updates patient anatomy to account
for the real-world tool to tissue interactions in real-time.
![](Resources/overview_setup.png)

### Usage

To run our project as a whole requires you to implement many adaptations based on your current hardware API. For instance, Camera Acquisition Pipelines and Optical Tracker may be faced a large fix if we are using different platform when you are pursuing the equivalent accuracy as we have evaluated.

Except for VR part, test and evaluations have been tested on three main streams including Windows OS, Mac OS, Ubuntu 20.04, 18.04, etc. VR part explicitly has been tested on Ubuntu 16.04 and Ubuntu 18.04 and Mac Mojave without ROS support. For better usage we simply recommend you to use docker to pull our image and run your container.

**Quicker Start**

1. Download and Configure Docker

**Complete Build**

1. Create AMBF Framework and Set it up

```bash
sudo apt install libasound2-dev libgl1-mesa-dev xorg-dev
cd ~
git clone https://github.com/WPI-AIM/ambf.git
cd ambf && mkdir build
cd build
cmake ..
make
cd ..
git clone https://github.com/LCSR-SICKKIDS/volumetric_drilling
cd volumetric_drilling
mkdir build
cd build
cmake ..
make
```

2. Set up the Communication between ROS nodes in .bashrc  

**Pivot Calibration**

1. Record the ROS bag of pivot tool's poses.

2. Load the poses from bag to csv.

3. Perform pivot calibration and save the tip.

```bash
./pivot_calibration.sh -p <$path> -s <$save_name> -t <$topic>
