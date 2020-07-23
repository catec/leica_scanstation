#!/usr/bin/env python2

import sys
import rospy
import os.path
from datetime import datetime
import numpy as np

"""
How to call script:

rosrun leica_scanstation_utils ptx_2_pcd.py file_to_convert visualize 

Args:
    file_to_convert: ptx file to be converted to pcd. Do not include extension
    visualize (optional): boolean to set to visualize converted pcd file on pcl_viewer
""" 

def main():
    rospy.init_node('ptx_2_pcd')

    file_name = sys.argv[1]
    try:
        pc_path = rospy.get_param("/pointcloud_folder")
    except:
        rospy.logwarn("No param /pointcloud_folder. Set to default")
        pc_path = "/home/catec/catkin_ws/src/leica_scanstation_utils/pointclouds"        

    infile_path = os.path.join(pc_path, file_name+".ptx")
    outfile_path = os.path.join(pc_path, file_name+".pcd")
    rospy.loginfo("Converting file: %s",infile_path)

    if infile_path.endswith('.ptx'):
        process_file(infile_path,outfile_path)
    else:
        rospy.logerr("File must have .ptx format")
    

    if len(sys.argv)>2 and sys.argv[2]:
        visualize_converted_file(outfile_path)
    

def visualize_converted_file(filename):
    os.system("pcl_viewer " + filename)


def process_line(line, transform):
    vals = np.array(map(float, str(line).split(" ")[:3]) + [1])

    pos = vals.dot(transform)

    return "{0} {1} {2}".format(pos[0], pos[1], pos[2])


def process_file(input_file, output_file):
    with open(input_file, 'r') as ptx, open(output_file, 'w') as pcd:
        col = int(ptx.readline())
        row = int(ptx.readline())
        # transformation matrix for view point
        view_point_tm = np.zeros([4, 3])
        view_point_tm[3] = map(float, ptx.readline().rstrip().split(" "))
        view_point_tm[0] = map(float, ptx.readline().rstrip().split(" "))
        view_point_tm[1] = map(float, ptx.readline().rstrip().split(" "))
        view_point_tm[2] = map(float, ptx.readline().rstrip().split(" "))
        cloud_tm = np.zeros([4, 3])  # transformation matrix for cloud
        cloud_tm[0] = map(float, ptx.readline().rstrip().split(" ")[:3])
        cloud_tm[1] = map(float, ptx.readline().rstrip().split(" ")[:3])
        cloud_tm[2] = map(float, ptx.readline().rstrip().split(" ")[:3])
        cloud_tm[3] = map(float, ptx.readline().rstrip().split(" ")[:3])
        # TODO: VIEW POINT: add quotanion
        pcd_head = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {0}
HEIGHT {1}
VIEWPOINT {2} {3} {4} 1 0 0 0
POINTS {5}
DATA ascii\n'''.format(col, row,
                       view_point_tm[3, 0],
                       view_point_tm[3, 1],
                       view_point_tm[3, 2],
                       col * row)
        pcd.writelines(pcd_head)
        count = 0
        line = ptx.readline()
        while line:
            pcd.write(process_line(line, cloud_tm) + "\n")
            count += 1
            line = ptx.readline()
    rospy.loginfo("File converted")


if __name__ == '__main__':
    main()