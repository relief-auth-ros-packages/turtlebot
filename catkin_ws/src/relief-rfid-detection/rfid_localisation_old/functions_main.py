

import numpy as np
import copy




from functions_parser import *
from functions_estimate_location import *
from classes import *



def create_existed_tags(existed_tags,new_tags):

    existed_tags_EPCs = [x.EPC for x in existed_tags]
    for specific_new_tag in new_tags: #[x.EPC for x in measured_tags]:
        #specific_new_tag.print()

        if specific_new_tag.EPC in existed_tags_EPCs:


            specific_new_tag_index=existed_tags_EPCs.index(specific_new_tag.EPC)
            specific_existed_tag=existed_tags[specific_new_tag_index]

            concatenate_old_new_tag_measurements(specific_new_tag,specific_existed_tag)
            #specific_existed_tag.print()
        else:
            specific_new_tag=unwrap_for_new_tag(specific_new_tag)
            existed_tags.append(specific_new_tag)
            #specific_new_tag.print()


    return existed_tags


# %%

def parsing_data(rfid_measurements_filelist,rfid_antennas_poses_filelist,robot_velocity,execution_timestamp):

    readers= read_reader_readings(rfid_measurements_filelist,execution_timestamp)

    # ama o pinakas readers den periexei kanena object tote simainei oti den eixa txt gai rfid
    if len(readers)==0:
        print("No readers found--No rfid_measurements_filelist found")
        #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    readers=create_antenna_objects_for_each_reader(readers,rfid_antennas_poses_filelist,execution_timestamp)
    if len(readers)==0:
        print('No antenna poses')
        #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    Velocities=read_velocities(robot_velocity,execution_timestamp)
    if len(Velocities)==0:
        print('No velocities')
        #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    first_time_pose, last_time_pose=find_first_last_pose(readers[0].antennas[0].poses)


    Velocities = keep_robot_velocities_within_existed_poses(Velocities,first_time_pose,last_time_pose)
    if len(Velocities)==0:
        print('Velocities and Poses do not overlap')
        # return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    readers=keep_readers_with_overlapped_poses(readers,first_time_pose,last_time_pose)

    if len(readers)==0:
        print('No Reader measurements overlap with poses')
            #return tag_measurements,tag_unwrapped_measurements,unique_tag,execution_timestamp
        return [],[],execution_timestamp

    velocity_values, velocity_time = find_nonmoving_periods(Velocities,readers[0].antennas[0].poses)

    unique_measured_tags_epcs=get_unique_measured_tag_epcs(readers)
    readers=create_poses_measurements(readers,velocity_time,velocity_values)

    execution_timestamp=last_time_pose

    return readers, unique_measured_tags_epcs,execution_timestamp



# %%
def initialise_measured_tags(readers,unique_measured_tags_epcs):

    measured_tags=[]
    for epc in unique_measured_tags_epcs:

        tag_temp=Tag(epc)
        for r in readers:  #den einai adeio
            reader_temp=Reader(r.IP)

            for a in r.antennas:  #den einai adeio
                antenna_temp=Antenna(a.index)
                antenna_temp.measurements=a.measurements[np.where(a.epcs==epc)[0],:] #replaceeee
                if len(antenna_temp.measurements)!=0:
                    reader_temp.antennas.append(antenna_temp)

            if len(reader_temp.antennas)!=0:
                tag_temp.readers.append(reader_temp)

        measured_tags.append(tag_temp)

    return measured_tags



# %%
def preprocessing_stage(rfid_measurements_filelist,rfid_antennas_poses_filelist,robot_velocity,existed_tags,execution_timestamp):

    readers, unique_measured_tags_epcs, execution_timestamp = parsing_data(rfid_measurements_filelist,rfid_antennas_poses_filelist,robot_velocity,execution_timestamp)

    #print(execution_timestamp)

    if len(readers):

        #discriminate data according to each tag
        new_tags = initialise_measured_tags(readers,unique_measured_tags_epcs)

        existed_tags = create_existed_tags(existed_tags,new_tags)

        print(len(new_tags))
        print(len(existed_tags))

        flag_localization=1
    else:
        flag_localization=0

    return existed_tags, execution_timestamp, flag_localization


def localization_stage(existed_tags):

    print(len([x for x in existed_tags if x.to_locate==1 or (x.to_locate==0 and (not(x.CI)))]))

    #perform localization only for tags with new data
    #for tag in [x for x in existed_tags if ((x.to_locate))]:
    for tag in [x for x in existed_tags if ((x.to_locate) or (x.to_locate==0 and (not(x.CI))))]: # commented-out 10/09/2021
        #if (tag.to_locate) or (tag.to_locate==0 and (not(tag.CI))):
        #tag.to_locate=1
        tag=estimate_location(tag)

            #tag.print_location()

    return existed_tags
