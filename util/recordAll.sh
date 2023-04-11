while getopts ":h:r:e:d:p:o:s:" opt
do
    case $opt in
        h)
        rosbag record -O "$OPTARG" /atracsys/Camera_hand/measured_cp /zedm/zed_node/left/image_rect_color/compressed
        ;;
        r)
        rosbag record -O "$OPTARG" /fwd_limage/compressed /fwd_rimage/compressed /fwd_pose_drill /fwd_pose_camhand /fwd_pose_pan #/fwd_depthData /fwd_segm/compressed /fwd_realsense_depthData
        ;;
        e)
        rosbag record -O "$OPTARG" /fwd_limage/compressed /fwd_rimage/compressed /fwd_pose_pan /fwd_pose_camhand
        ;;
        d)
        rosbag record -O "$OPTARG" /atracsys/Surgical_Drill/measured_cp
        ;;
        p)
        rosbag record -O "$OPTARG" /atracsys/Pointer/measured_cp
        ;;
        o)
        rosbag record -O "$OPTARG" /fwd_limage/compressed /fwd_rimage/compressed /fwd_pointcloud /fwd_pose_camhand /fwd_pose_pan
        ;;
        s)
        rosbag record -O "$OPTARG" /sync_limage/compressed /sync_segm/compressed /sync_pcd/DepthData /sync_pose_camhand /sync_pose_pan
        echo "unkown"
        exit 1;;
    esac
done