cleanup() {
    echo "Cleaning up..."
    # look for processes whose parent process ID is the same as the shell script's process ID
    pkill -P $$
}

# Set a trap to run the cleanup function when the script receives a 2 signal (Ctrl+C)
trap cleanup 2

while getopts h:c: flag
do
    case "${flag}" in
        c) config=${OPTARG};;
    esac
done
# # Change to volumatric drilling folder
cd ~/volumetric_drilling/scripts;

# start the drill move script
python3 drill_move_imgPub.py --sim_sync --config "$config"&
# offline sync to the simulation
python3 offline_segm_eval.py &

wait