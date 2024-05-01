import numpy as np
import os 
import pandas

from functions_main import *




def rfid_localization(rfid_locations,tags_infos,robot_velocity,rfid_antennas_poses_filelist,rfid_measurements_filelist,existed_tags,execution_timestamp):
    
    #parsing the data and put them in existed_tags
    existed_tags, execution_timestamp, flag_localization = preprocessing_stage(rfid_measurements_filelist,rfid_antennas_poses_filelist,robot_velocity,existed_tags,execution_timestamp)
    
    if flag_localization:
        existed_tags = localization_stage(existed_tags)

    Results=pandas.DataFrame([])
    for tag in existed_tags:
        Estimation=np.hstack((tag.EPC,tag.x,tag.y,tag.z,tag.CI))

        Results=Results.append((pandas.DataFrame(Estimation)).T,ignore_index='True')


    if os.path.exists(rfid_locations):    
        os.remove(rfid_locations)
        
    Results.to_csv(rfid_locations, header=False, index=False)


    return [existed_tags, execution_timestamp]




