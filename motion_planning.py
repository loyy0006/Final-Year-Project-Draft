import bpy
import math 
import csv

obj = bpy.data.objects['Waterjet Tool'];
obj.rotation_mode = 'XYZ';

'''
Local transformation matrix from SE3 filtering to Blender (set object frame)
'''

with open('C:/Users/Admin/Documents/Yongz/NTU/Final Year Project/Motion Capture Codes/15-03-2021/motion_planning.csv',newline='') as csvfile:
    csv_reader = csv.reader(csvfile, delimiter=',')

    for row in csv_reader:


        obj.location = (float(row[0]),float(row[1]),float(row[2]));
        #obj.rotation_euler = (float(row[3]),float(row[4]),float(row[5]));
        
        obj.keyframe_insert(data_path="location",frame=float(row[6]));
        obj.keyframe_insert(data_path="rotation_euler",frame=float(row[6]));
        print('done')
     