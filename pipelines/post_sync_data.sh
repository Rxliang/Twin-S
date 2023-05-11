cleanup() {
    echo "Cleaning up..."
    # look for processes whose parent process ID is the same as the shell script's process ID
    pkill -P $$
}

# Set a trap to run the cleanup function when the script receives a 2 signal (Ctrl+C)
trap cleanup 2

while getopts h: flag
do
    case "${flag}" in
        h) handeyeFile=${OPTARG};;
    esac
done

# start synchronizer
python3 ../util/synchronizer.py --sim_sync &
# start the drill move script
python3 ~/volumetric_drilling/scripts/drill_move_imgPub.py --handeye ~/Twin-S/params/"$handeyeFile" --sim_sync &

wait