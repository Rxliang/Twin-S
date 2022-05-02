import bagpy
def bag2csv(file_path, axxb):
    b = bagpy.bagreader(file_path)
    if axxb:
        # b.message_by_topic('/fwd_pose_pan')
        b.message_by_topic('/fwd_pose_camhand')
    else:
        b.message_by_topic('/fwd_pose_pan')
        b.message_by_topic('/fwd_pose_drill')
        b.message_by_topic('/fwd_pose_camhand')

if __name__ == '__main__':

    bag2csv('F:\AXXB_Good_small_motion_419.bag', axxb=True)