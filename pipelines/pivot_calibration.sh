while getopts p:s:t: flag
do
    case "${flag}" in
        p) path=${OPTARG};;
        s) saveName=${OPTARG};;
        t) topic=${OPTARG};;
    esac
done
# # Data collection
cd ../util;
# ./recordAll.sh -p "$path";

# Bag to csv
#Pointer tool topic: /atracsys/Pointer/measured_cp
#Surgical Drill topic: /atracsys/Surgical_Drill/measured_cp 
python3 tf_to_csv.py --bag "$path".bag --tf_target_frame "$topic"; 

# Pivot calibration
cd ../optical_tracking;
python3 sksurgery_pivot_calibration.py -i "$path".csv -s "$saveName" -c ransac_config.json;
