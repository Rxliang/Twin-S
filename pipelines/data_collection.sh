cleanup() {
    echo "Cleaning up..."
    # look for processes whose parent process ID is the same as the shell script's process ID
    pkill -P $$
}

# Set a trap to run the cleanup function when the script receives a 2 signal (Ctrl+C)
trap cleanup 2

while getopts c:s: flag
do
    case "${flag}" in
        c) config=${OPTARG};;
        s) saveFile=${OPTARG};;
    esac
done

# start synchronizer
python3 ../util/synchronizer.py --data &
# start the drill move script
python3 ~/volumetric_drilling/scripts/drill_move_imgPub.py --sim_sync --config "$config"&
# offline sync to the simulation
python3 ~/volumetric_drilling/scripts/offline_segm_eval.py &
# record the data
echo "\n"
read -p "**********************Press enter to start recording...************************" t
./recordAll.sh -r "$saveFile" &
wait