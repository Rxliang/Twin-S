# cleanup() {
#     echo "Cleaning up..."
#     # look for processes whose parent process ID is the same as the shell script's process ID
#     kill -1 $$
# }

# # Set a trap to run the cleanup function when the script receives a 2 signal (Ctrl+C)
# trap cleanup 1

DURATION=60;

while getopts ":h:r:e:d:p:o:s:m:" opt
do
    case $opt in
        h)
        rosbag record -O "$OPTARG" /atracsys/Camera_hand/measured_cp /zedm/zed_node/left/image_rect_color/compressed
        ;;
        r)
        timeout $DURATION rosbag record -O "$OPTARG" /fwd_limage/compressed /fwd_rimage/compressed /fwd_pose_drill /fwd_pose_camhand /fwd_pose_pan #/fwd_depthData /fwd_segm/compressed /fwd_realsense_depthData
        echo "Finish data recording!"
        ;;
        e)
        rosbag record -O "$OPTARG" /fwd_limage/compressed /fwd_rimage/compressed /fwd_pose_pan /fwd_pose_camhand
        ;;
        d)
        rosbag record -O "$OPTARG" /atracsys/Surgical_drill/measured_cp
        ;;
        p)
        rosbag record -O "$OPTARG" /atracsys/Pointer/measured_cp
        ;;
        o)
        rosbag record -O "$OPTARG" /fwd_limage/compressed /fwd_rimage/compressed /fwd_pointcloud /fwd_pose_camhand /fwd_pose_pan
        ;;
        s)
        rosbag record -O "$OPTARG" /sync_limage/compressed /sync_rimage/compressed /sync_segm/compressed /sync_pose_camhand /sync_pose_pan /sync_pose_drill /sync_pcd/DepthData -b 10000
        ;;
        m)
        rosbag record -O "$OPTARG" /atracsys/Camera_hand/measured_cp /atracsys/Surgical_drill/measured_cp /atracsys/Panel/measured_cp /zedm/zed_node/left/image_rect_color/compressed /zedm/zed_node/right/image_rect_color/compressed
        
        echo "unkown"
        exit 1;;
    esac
done