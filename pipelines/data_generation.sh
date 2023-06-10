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

python3 ../util/data_generation.py --output_dir "$saveFile" &
./post_sync_data.sh -h "$handeyeFile" >/dev/null & 

echo "\n"
read -p "************************************Press enter to continue...**************************************" t
rosbag play "$bagFile" >/dev/null &

wait %3
sleep 60

cleanup
pids=$(pgrep -f "sim_sync")
kill $pids
echo "Finish data generating!"