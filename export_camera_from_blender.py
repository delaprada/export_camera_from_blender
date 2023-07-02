import bpy
import numpy as np
from mathutils import Matrix
from pathlib import *

def get_calibration_matrix_K_from_blender(camd):
    f_in_mm = camd.lens
        
    scene = bpy.context.scene
    width = scene.render.resolution_x
    height = scene.render.resolution_y
    orientation = camd.sensor_fit
    
    calibrated_sensor_width = camd.sensor_width
    calibrated_sensor_height = camd.sensor_height

    pixel_width = calibrated_sensor_width / width
    pixel_height = calibrated_sensor_height / height
    
    shift_x = camd.shift_x
    shift_y = camd.shift_y
    
    larger_dim = calibrated_sensor_width
    orientation = "HORIZONTAL"
    if calibrated_sensor_width < calibrated_sensor_height:
        orientation = "VERTICAL"
        larger_dim = calibrated_sensor_height
    ratio = larger_dim / min(calibrated_sensor_height, calibrated_sensor_width)
    
    calibrated_focal_length_x = f_in_mm
    
    fx = calibrated_focal_length_x / pixel_width
    fy = calibrated_focal_length_x / pixel_height
    
    print(fx, fy)
    
    shift_x = shift_x * larger_dim
    shift_y = -shift_y * larger_dim
    
    principal_point_x = (calibrated_sensor_width / 2) - shift_x
    principal_point_y = (calibrated_sensor_height / 2) - shift_y
    
    cx = principal_point_x / pixel_width
    cy = principal_point_y / pixel_height

    K = [ fx,    0,    cx,
        0  ,  fy,    cy,
        0  ,   0,     1]
    return K, width, height

def get_4x4_RT_matrix_from_blender(cam):
    # bcam stands for blender camera
    R_bcam2cv = Matrix(
        ((1, 0,  0),
         (0, -1, 0),
         (0, 0, -1)))

    # Transpose since the rotation is object rotation, 
    # and we want coordinate rotation
    # R_world2bcam = cam.rotation_euler.to_matrix().transposed()
    # T_world2bcam = -1*R_world2bcam * location
    #
    # Use matrix_world instead to account for all constraints
    location, rotation = cam.matrix_world.decompose()[0:2]
    R_world2bcam = rotation.to_matrix().transposed()

    # Convert camera location to translation vector used in coordinate changes
    # T_world2bcam = -1*R_world2bcam*cam.location
    # Use location from matrix_world to account for constraints:     
    T_world2bcam = -1 * R_world2bcam @ location

    # Build the coordinate transform matrix from world to computer vision camera
    R_world2cv = R_bcam2cv @ R_world2bcam
    T_world2cv = R_bcam2cv @ T_world2bcam

    # put into 3x4 matrix
    RT = Matrix((
        R_world2cv[0][:] + (T_world2cv[0],),
        R_world2cv[1][:] + (T_world2cv[1],),
        R_world2cv[2][:] + (T_world2cv[2],),
         ))
    
    motionMat = np.zeros((4,4))
    motionMat[3][3] = 1
    motionMat[0:3,0:4] = np.asarray(RT)
    
    R = [R_world2cv[0][0], R_world2cv[0][1], R_world2cv[0][2],
    R_world2cv[1][0], R_world2cv[1][1], R_world2cv[1][2],
    R_world2cv[2][0], R_world2cv[2][1], R_world2cv[2][2]]
    T = T_world2cv
    
    return R, T, RT


if __name__ == "__main__":
    for scene in bpy.data.scenes:
        bpy.context.window.scene = scene
        
        for obj in scene.objects:
            if obj.type == 'CAMERA':    

                print(obj.name)
                K, w, h= get_calibration_matrix_K_from_blender(obj.data)
                R,T, RT = get_4x4_RT_matrix_from_blender(obj)
                print(K)
                print(R)
                print(T)
                print(RT)
            
            if obj.type == 'MESH':
                if obj.name == 'rand_0_skin':
#                    vertices = obj.data.vertices
#                    print(vertices[0].co)
                    verts = [vert.co for vert in obj.data.vertices]
                    print(verts[0])
                    plain_verts = [np.array(vert) for vert in verts]
                    print(plain_verts[0])
                