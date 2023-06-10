cleanup() {
    echo "Cleaning up..."
    # look for processes whose parent process ID is the same as the shell script's process ID
    pkill -P $$
}

# Set a trap to run the cleanup function when the script receives a 2 signal (Ctrl+C)
trap cleanup 2

while getopts h:b:s: flag
do
    case "${flag}" in
        h) handeyeFile=${OPTARG};;
        b) bagFile=${OPTARG};;
        s) saveFile=${OPTARG};;
    esac
done

# # sync the recorded data with the simulation data
# ./recordAll.sh -s "$saveFile" &
echo "\n"
read -p "**********************Open the AMBF simulation and Press enter to continue...************************" t

# python3 ../util/data_generation.py --output_dir "$saveFile" &
cd ;
cd ~/test\ pub;
# python ros_pose_update.py --split_file split_data_val.txt --base_dir . --pose_dir motion_e2e_output --gt_pose --drop_poses_idx 50 100 130 145 170 190 240 260 300 330 345 370 390 440&
# python ros_pose_update.py --split_file split_data_val.txt --base_dir . --pose_dir motion_e2e_output &
python ros_pose_update.py --split_file split_data_val.txt --base_dir . --pose_dir motion_e2e_output --gt_pose --drop_poses_idx 50 100 130 145 170 190 240 260 300 330 345 370 390 440 &
# python ros_pose_update.py --split_file split_data_3.txt --base_dir . --pose_dir pete_drilling_output --gt_pose --drop_poses_idx 50 80 90 110 115 140 &
# python ros_pose_update.py --split_file split_data_3.txt --base_dir . --pose_dir pete_drilling_output &

python /home/shc/volumetric_drilling/scripts/offline_video_gen.py &

# echo "\n"
# read -p "************************************Press enter to continue...**************************************" t
# rosbag play "$bagFile" >/dev/null &

wait %1
sleep 3

cleanup
# pids=$(pgrep -f "sim_sync")
# kill $pids
echo "Finish data generating!"