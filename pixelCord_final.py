import cv2
import math 
import csv
import bpy
import bmesh
import numpy as np
import mathutils
from mathutils import Vector, Matrix

#----------------------------------------------
filename = "C:/Users/Admin/Documents/Downloads/calculus_colour_2.79.png"
img = cv2.imread(filename, 0)
height, width = img.shape[:2]

#----------------------------------------------
obj=bpy.context.scene.objects['teeth']
#bpy.ops.object.mode_set(mode='EDIT')
me=obj.data
bm=bmesh.from_edit_mesh(me)
size=len(bm.faces)
uv_layer = bm.loops.layers.uv.active
x = []
y = []
with open('C:/Users/Admin/Documents/Yongz/NTU/Final Year Project/CAD files/OpenCV Datasets/Segmentation_v2/coordinates.csv',newline='') as csvfile:
    csv_reader = csv.reader(csvfile,delimiter=',')
    for row in csv_reader:
        x.append(row[0])
        y.append(row[1])
#loop through the pixels

pixel_s_copy = []
for i in range(len(x)): #because size is the same
    x[i] = int(x[i])/height
    y[i] = int(y[i])/height
    pixel_s_copy.append([x[i],y[i]])
    #pixel_s_copy = np.append(pixel_s_copy,[[x[i],y[i]]])

#pixel_s = (x,y)
#pixel_s = (437/1024,547/1024) # pixel coordinates (your output from image processing)

#----------------------------------------------
import mathutils
from mathutils import Vector, Matrix

face_index_array = []
for i in range(len(pixel_s_copy)):
    for face in bm.faces:
        uv1, uv2, uv3 = [l[uv_layer].uv.to_3d() for l in face.loops]
        if mathutils.geometry.intersect_point_tri_2d(pixel_s_copy[i], uv1, uv2, uv3):
            face_index = face.index
            face_index_array.append(face.index)
            break  
            #print(face_index_array)
# at the end of the loop will have an array of pixels and their corresponding faces

#----------------------------------------------
obj=bpy.context.scene.objects['teeth']
#bpy.ops.object.mode_set(mode='EDIT')
me=obj.data
bm=bmesh.from_edit_mesh(me)
size=len(bm.faces)

##defining mesh structure
def delta (X, Y):
    return (X-Y)
    

class my_struct:
    xyz1 = np.zeros((size,3))
    xyz2 = np.zeros((size,3))
    xyz3 = np.zeros((size,3))
    uv1 = np.zeros((size,3))
    uv2 = np.zeros((size,3))
    uv3 = np.zeros((size,3))
    X0 = np.zeros((size,3))
    U0 = np.zeros((size,2))
    Proj = np.zeros((size,6))
    Lift = np.zeros((size,6))
    
s1 = my_struct()
    
bm.verts.ensure_lookup_table()
bm.faces.ensure_lookup_table()  
bm.faces[0].verts[1].index    

uv_layer = bm.loops.layers.uv.active 
    
sel_faces = [f for f in bm.faces if f.select]

for face in bm.faces:
    f=face.index
    [u1,u2,u3] = [l[bm.loops.layers.uv.active].uv.to_3d() for l in face.loops]
    s1.uv1[f] = u1
    s1.uv2[f] = u2
    s1.uv3[f] = u3
    [x1,x2,x3] = [face.verts[0].co, face.verts[1].co, face.verts[2].co]
    s1.xyz1[f] = x1
    s1.xyz2[f] = x2
    s1.xyz3[f] = x3
    #s1.Lift[f] = [delta(s1.xyz1[f], s1.xyz2[f]), delta(s1.xyz1[f], s1.xyz3[f])] * np.linalg.inv([delta(s1.uv1[f], s1.uv2[f]), delta(s1.uv1[f], s1.uv3[f])])
    a = np.array([delta(s1.xyz1[f], s1.xyz2[f]), delta(s1.xyz1[f], s1.xyz3[f])] )
    a = np.matrix(a.transpose())
    b = np.array([delta(s1.uv1[f][0:2], s1.uv2[f][0:2]), delta(s1.uv1[f][0:2], s1.uv3[f][0:2])])
    b = np.matrix(b.transpose())
    c= np.cross(a[:,0].T,a[:,1].T).T
    if np.linalg.det(b)!=0.0:
        originalLift = a*np.linalg.inv(b)
        s1.Lift[f] = np.reshape(a*np.linalg.inv(b),(1,6)) #reshape back to 3x2
        s1.X0[f] = s1.xyz1[f]-np.dot(originalLift,s1.uv1[f][0:2])
    if np.linalg.det(np.append(a,c,axis=1))!=0.0:
        originalProj = np.append(b, [[0],[0]], axis=1)* np.linalg.inv(np.append(a,c,axis=1))
        s1.Proj[f] = np.reshape(np.append(b, [[0],[0]], axis=1)* np.linalg.inv(np.append(a,c,axis=1)),(1,6))
        s1.U0[f] = s1.uv1[f][0:2]-np.dot(originalProj,s1.xyz1[f].T)
    
##testing mesh structure
#i = 613
#p2D= np.array(pixel_s_copy[i])
#p3D=s1.X0[face_index_array[i]]+np.dot(np.reshape(s1.Lift[face_index_array[i]],(3,2)),p2D.T)
#p3D


#p3D=np.array([0.1445, 0.30713, 0.1345])  
#p2D=s1.U0[face_index_array[i]]+np.dot(np.reshape(s1.Proj[face_index_array[i]],(2,3)),p3D.T)

##Save into csv file for matlab
p3D_array=[]
for i in range(len(face_index_array)):
    p2D= np.array(pixel_s_copy[i])
    p3D=s1.X0[face_index_array[i]]+np.dot(np.reshape(s1.Lift[face_index_array[i]],(3,2)),p2D.T) # corresponding 3D point
    p3D_array.append(p3D)

with open('C:/Users/Admin/Documents/Yongz/NTU/Final Year Project/CAD files/OpenCV Datasets/Segmentation_v2/3D_coords.csv', "w", newline='') as f:
    writer = csv.writer(f,delimiter=',')
    writer.writerows(p3D_array)   
