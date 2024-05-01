import time
import numpy as np
from main import *



filename="E:\\Experiments Library 2021\\EE_LIB_EXPS_2020_02_05\\rb1\\single_reader\\001625127C5D\\"
rfid_measurements_filelist=[]
rfid_measurements_filelist.append(filename+"results_001625127C5F.txt") 
rfid_measurements_filelist.append(filename+"results_001625127C5D.txt") 

rfid_antennas_poses_filelist=[]
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5D_antenna_1.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5D_antenna_2.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5D_antenna_3.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5D_antenna_4.txt") 

rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5F_antenna_1.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5F_antenna_2.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5F_antenna_3.txt")
rfid_antennas_poses_filelist.append(filename+"pose_reader_001625127C5F_antenna_4.txt")


robot_velocity=filename+"velocities.txt"

rfid_locations=filename+"rfid_results_by_python.txt"

tags_infos=[]


execution_timestamp=-1

existed_tags=[]



start_time = time.time()
rfid_localization_output = rfid_localization(rfid_locations,tags_infos,robot_velocity,rfid_antennas_poses_filelist,rfid_measurements_filelist,existed_tags,execution_timestamp)
elapsed_time = time.time() - start_time


existed_tags = rfid_localization_output[0]
execution_timestamp = rfid_localization_output[1]