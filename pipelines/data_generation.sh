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

DURATION=10;

# # sync the recorded data with the simulation data
./recordAll.sh -s "$saveFile" &
./post_sync_data.sh -h "$handeyeFile" &
echo "\n"
read -p "**********************Open the AMBF simulation and Press enter to continue...************************" t
rosbag play "$bagFile" &
wait
echo "Finish data generating!"