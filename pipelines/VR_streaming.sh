cleanup() {
    echo "Cleaning up..."
    # look for processes whose parent process ID is the same as the shell script's process ID
    pkill -P $$
}

# Set a trap to run the cleanup function when the script receives a 2 signal (Ctrl+C)
trap cleanup 2

while getopts c: flag
do
    case "${flag}" in
        c) config=${OPTARG};;
    esac
done

# start synchronizer
python3 ../util/synchronizer.py --vr &
# start the drill move script
python3 ~/volumetric_drilling/scripts/vr_drill_move.py --config "$config"&
echo "\n"
read -p "**********************Press enter to start streaming...************************" t
wait