#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2021-10-29

# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import sys
import argparse
import rospy
import numpy as np
import json
import scipy.spatial
import scipy.optimize

import geometry_msgs.msg

if sys.version_info.major < 3:
    input = raw_input


# create subscription with callback to handle marker messages
def get_pose_data(ros_topic, expected_marker_count):
    records = []
    collecting = False
    reference = []

    def display_sample_count():
        sys.stdout.write("\rNumber of samples collected: %i" % len(records))
        sys.stdout.flush()

    def pose_array_callback(msg):
        # skip if not recording marker pose messages
        if not collecting:
            return

        # make sure the number of poses matches the number of expected markers
        if len(msg.poses) != expected_marker_count:
            return

        record = np.array([
            (marker.position.x, marker.position.y, marker.position.z)
            for marker in msg.poses
        ])

        # use first sample as reference order
        if reference == []:
            reference.extend(record)

        # each record has n poses but we don't know if they are sorted by markers
        # find correspondence to reference marker that minimizes pair-wise distance
        correspondence = scipy.spatial.distance.cdist(record, reference).argmin(axis=0)

        # skip records where naive-correspondence isn't one-to-one
        if len(np.unique(correspondence)) != len(reference):
            return

        # put record markers into the same order as the corresponding reference markers
        ordered_record = record[correspondence]
        records.append(ordered_record)
        display_sample_count()

    pose_array_subscriber = rospy.Subscriber(ros_topic, geometry_msgs.msg.PoseArray, pose_array_callback)

    input("Press Enter to start collection using topic %s" % ros_topic)
    print("Collection started\nPress Enter to stop")
    display_sample_count()
    collecting = True

    input("")
    collecting = False
    pose_array_subscriber.unregister()

    return records


# Apply PCA to align markers, and if is_planar to project to plane.
# Points data should have mean zero (i.e. be centered at origin).
# planar_threshold is maximium relative variance along third axis that is considerd planar
def principal_component_analysis(points, is_planar, planar_threshold=1e-2):
    # SVD for PCA
    _, sigma, Vt = np.linalg.svd(points, full_matrices=False)

    # Orientation should be (close to) +/-1
    basis_orientation = np.linalg.det(Vt)
    # Select positive orientation of basis
    if basis_orientation < 0.0:
        Vt[2, :] = -Vt[2, :]

    # Three markers will always be planar, so we can ignore minor computation errors
    marker_count = np.size(is_planar)
    is_planar = is_planar or marker_count == 3

    # Project markers to best-fit plane
    if is_planar:
        print("Planar flag enabled, projecting markers onto plane...")
        # Remove 3rd (smallest) principal componenent to collapse points to plane
        Vt[2, :] = 0

    planarity = sigma[2] / sigma[1]
    if is_planar and planarity > planar_threshold:
        print("WARNING: planar flag is enabled, but markers don't appear to be planar!")
    elif not is_planar and planarity < planar_threshold:
        print(
            "Markers appear to be planar. If so, add '--planar' flag for better results"
        )

    return np.matmul(points, Vt.T)


def process_marker_records(records, is_planar):
    # average position of each marker
    averaged_marker_poses = np.mean(records, axis=0)
    # center (average) of individual average marker positions
    isocenter = np.mean(averaged_marker_poses, axis=0)
    # center coordinate system on isocenter
    points = averaged_marker_poses - isocenter
    # align using PCA and project to plane is is_planar flag is set
    points = principal_component_analysis(points, is_planar)

    return points

supported_units = {
    "mm": 0.001,
    "cm": 0.01,
    "m": 1.0,
}

def convert_units(marker_points, output_units):
    # Input marker pose data is always in meters
    input_units = "m"

    print("Converting units from {} to {}".format(input_units, output_units))
    return marker_points * supported_units[input_units] / supported_units[output_units]


def write_data(points, id, output_file_name):
    fiducials = [{"x": x, "y": y, "z": z} for [x, y, z] in points]
    origin = {"x": 0.0, "y": 0.0, "z": 0.0}

    data = {
        "count": len(fiducials),
        "fiducials": fiducials,
        "pivot": origin,
    }

    if id is not None:
        data["id"] = id

    with open(output_file_name, "w") as f:
        json.dump(data, f, indent=4, sort_keys=True)
        f.write("\n")

    print("Generated tool geometry file {}".format(output_file_name))


if __name__ == "__main__":
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node("tool_maker", anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()

    # required arguments
    parser.add_argument(
        "-t",
        "--topic",
        type=str,
        required=True,
        help="topic to use to receive PoseArray without namespace. Use __ns:= to specify the namespace",
    )
    parser.add_argument(
        "-n",
        "--number-of-markers",
        type=int,
        choices=range(3, 10),
        required=True,
        help="number of markers on the tool. Used to filter messages with incorrect number of markers",
    )
    parser.add_argument(
        "-o", "--output", type=str, required=True, help="output file name"
    )

    # optional arguments
    parser.add_argument(
        "-p",
        "--planar",
        action="store_true",
        help="indicates all markers lie in a plane",
    )
    parser.add_argument(
        "-i", "--id", type=int, required=False, help="specify optional id"
    )
    parser.add_argument(
        "-u",
        "--units",
        type=str,
        choices=supported_units.keys(),
        default="mm",
        required=False,
        help="units to use for output data in tool config",
    )

    args = parser.parse_args(argv[1:])  # skip argv[0], script name

    # Arbitrary number to make sure we have enough records to average out noise etc.
    minimum_records_required = 10

    # create the callback that will collect data
    records = get_pose_data(args.topic, args.number_of_markers)
    if len(records) < minimum_records_required:
        sys.exit("Not enough records ({} minimum)".format(minimum_records_required))

    points = process_marker_records(records, args.planar)
    points = convert_units(points, args.units)
    write_data(points, args.id, args.output)
