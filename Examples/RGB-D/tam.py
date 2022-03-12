import pyrealsense2 as rs
import cv2
import numpy as np
#import os
import time
# Configure depth and color streams
class Realsense_para:
    def __init__(self,ci,di,ex):
        self.color_inner_matirx=np.mat(np.array([[ci.fy,0,ci.ppy],[0,ci.fx,ci.ppx],[0,0,1]]))
        self.depth_inner_matrix=np.mat(np.array([[di.fx,0,di.ppx],[0,di.fy,di.ppy],[0,0,1]]))
       # print(self.color_inner_matirx.I)
        #print(self.depth_inner_matrix)
        self.color_to_depth_rotation=np.mat(np.array(ex.rotation).reshape(3,3))##
       # print(np.mat(np.array(ex.rotation)))
        self.color_to_depth_translation=np.mat(np.array(ex.translation))###
       #print(self.color_to_depth_translation)
    def refresh_mat(self):
        self.frames = pipeline.wait_for_frames()
        #self.frames = aligned_stream.process(frames)
        self.depth = self.frames.get_depth_frame()
        self.color = self.frames.get_color_frame()
        self.left_ifr = self.frames.get_infrared_frame(1)
        self.righ_ifr = self.frames.get_infrared_frame(2)
        self.left_ifr_=np.asanyarray(self.left_ifr.get_data())
        self.righ_ifr_=np.asanyarray(self.righ_ifr.get_data())
        #hole_filling = rs.hole_filling_filter()
        #self.depth = hole_filling.process(self.depth)
        colorizer = rs.colorizer()
        #self.colorized_depth = np.asanyarray(colorizer.colorize(self.depth).get_data())
        # depth_profile = depth.get_profile()
        # color_profile = color.get_profile()
        # print(depth_profile)
        # print(color_profile)
        #self.depthmat=np.asanyarray(self.depth.get_data())
        self.colormat=np.asanyarray(self.color.get_data())
        align = rs.align(rs.stream.color)
        self.frames  = align.process(self.frames)
        self.aligned_depth_frame = self.frames.get_depth_frame()
        hole_filling = rs.hole_filling_filter()
        self.aligned_depth_frame = hole_filling.process(self.aligned_depth_frame)
        self.depthmat=np.asanyarray(self.aligned_depth_frame.get_data())
        #print(self.depthmat[130:150])
        self.colorized_depth = np.asanyarray(colorizer.colorize(self.aligned_depth_frame).get_data())
        self.depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale() 
        #print(self.depth_scale)
#matrix = np.array([[1,5],[1,4]])
#matrix_a=np.mat(np.array([[1,0,2],[0,3,4],[0,0,1]]))
#print(matrix_a*matrix_a.I)

#print(matrix_a*matrix_b)
#print(a)d

pipeline = rs.pipeline()
# CONFIG：
config = rs.config()
#config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)
#config.enable_stream(rs.stream.depth,1280, 720, rs.format.z16, 6)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) #/ ung voi size nao thi framerate do 
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)# vd : (640.480) ==> framerate = 30. 
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30) #@@@@@@@@@@@@@@@@@@@@
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30) #@@@@@@@@@@@@@@@@@@@@
# Start streaming
pipeline.start(config)
#aligned_stream = rs.align(rs.stream.color)
#se = pipeline.get_active_profile().get_device().query_sensors()[1]
#print(se)  
pipeline.get_active_profile().get_device().as_auto_calibrated_device # auto calib (modified)
sensor = pipeline.get_active_profile().get_device().query_sensors()[1] # 1 is color
#print(type(pipeline.get_active_profile()))#==> <class 'pyrealsense2.pipeline_profile'>
#print(type(sensor)) #==> <class 'pyrealsense2.sensor'>
#sensor_depth = pipeline.get_active_profile().get_device().query_sensors()[1] 
#print(type( pipeline.get_active_profile().get_device()))
#print(options.get_option_range(rs.option.max_distance))
frames = pipeline.wait_for_frames() 
#print(type(frames)) # <class 'pyrealsense2.composite_frame'>
depth = frames.get_depth_frame()
depthmat=np.asanyarray(depth.get_data())
print(depthmat)
color = frames.get_color_frame()
#infrared = frames.get_infrared_frame()
depth_profile = depth.get_profile()
color_profile = color.get_profile()
#print(depth_profile)
print(color_profile)
#print(infrared_profile)# 
print(type(depth_profile)) # <class 'pyrealsense2.stream_profile'>
cvsprofile = rs.video_stream_profile(color_profile)
dvsprofile = rs.video_stream_profile(depth_profile)
print(type(cvsprofile)) # <class 'pyrealsense2.video_stream_profile'>
color_intrin = cvsprofile.get_intrinsics() # Get stream profile instrinsics attributes
depth_intrin = dvsprofile.get_intrinsics() # Get stream profile instrinsics attributes
print('COLOR INTRIN')
print(color_intrin)
print('depth INTRIN')
print(depth_intrin)

extrin = depth_profile.get_extrinsics_to(color_profile)
print('extrin')
print(extrin)
print('------------------------------------------------------')
#print(type(color_intrin)) # <class 'pyrealsense2.intrinsics'>
print(color_intrin) # [ 640x480  p[313.947 236.609]  f[612.916 613.482]  Inverse Brown Conrady [0 0 0 0 0] ]
# [ 1280x720  p[630.921 354.913]  f[919.374 920.224]  Inverse Brown Conrady [0 0 0 0 0] ]
print(depth_intrin) # [ 640x480  p[319.487 239.149]  f[384.684 384.684]  Brown Conrady [0 0 0 0 0] ]
#[ 1280x720  p[639.144 358.581]  f[641.14 641.14]  Brown Conrady [0 0 0 0 0] ]
D435_para=Realsense_para(color_intrin,depth_intrin,extrin)
#D435_para.refresh_mat()
# print(D435_para.color_to_depth_translation)

# print(color_to_depth_rotation,color_to_depth_translation)
pipeline.stop()## dang thu bo di chay van ok