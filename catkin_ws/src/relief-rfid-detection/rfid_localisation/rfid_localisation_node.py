#!/usr/bin/env python

import time
import rospy
from rfid_localisation_main import *
import pandas

class RFIDLocalisation():

################################################################################
# constructor
################################################################################
  def __init__(self):

    ############################################################################
    if rospy.has_param('~callback_frequency'):
      self.callback_frequency = rospy.get_param('~callback_frequency')
    else:
      print("ERROR: callback_frequency not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~rfid_locations'):
      self.rfid_locations = rospy.get_param('~rfid_locations')
    else:
      print("WARNING: rfid_locations file not set; setting default value")
      self.rfid_locations = "~/rfid_locations.txt"

    ############################################################################
    if rospy.has_param('~tags_infos'):
      self.tags_infos = rospy.get_param('~tags_infos')
    else:
      print("ERROR: tags_infos file not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~robot_velocity'):
      self.robot_velocity = rospy.get_param('~robot_velocity')
    else:
      print("ERROR: robot_velocity file not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~rfid_antennas_poses_filelist'):
      self.antenna_poses_filelist = \
      rospy.get_param('~rfid_antennas_poses_filelist')
    else:
      print("ERROR: rfid_antennas_poses_filelist param not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~rfid_measurements_filelist'):
      self.rfid_measurements_filelist = \
      rospy.get_param('~rfid_measurements_filelist')
    else:
      print("ERROR: rfid_measurements_filelist file not set; aborting")
      return

    ############################################################################
    if rospy.has_param('~furniture_coords_storage_file'):
      self.storage_file = \
      rospy.get_param('~furniture_coords_storage_file')
    else:
      print("ERROR: furniture_coords_storage_file file not set; aborting")
      return



    ############################################################################
    # Both input and output lists
    self.existed_tags  = []
    self.last_timestamp = -1.0
    self.Results = pandas.DataFrame([])

    #self.timer = \
        #rospy.Timer(rospy.Duration(self.callback_frequency), self.callback)

    self.localise_tags_please()

    if False:
      print(self.callback_frequency)
      print(self.rfid_locations)
      print(self.tags_infos)
      print(self.robot_velocity)
      print(self.antenna_poses_filelist)
      print(self.rfid_measurements_filelist)



################################################################################
# periodic callback
################################################################################
  def callback(self, timer):
    print("Entering callback")

    #try:

    start_time = time.time()

    ret_list = rfid_localization(\
      self.rfid_locations, \
      self.tags_infos, \
      self.storage_file, \
      self.robot_velocity, \
      self.antenna_poses_filelist, \
      self.rfid_measurements_filelist, \
      self.existed_tags, \
      self.last_timestamp, \
      self.Results)

    end_time = time.time()
    print "Tag localisation executed in", end_time-start_time, "sec"

    self.existed_tags = ret_list[0]
    self.last_timestamp = ret_list[1]
    self.Results = ret_list[2]

#    except:
      #print "*********************************"
      #print "------- EXCEPTION CAUGHT ------- "
      #print "*********************************"


################################################################################
# instead of a periodic callback, wait until execution finishes
################################################################################
  def localise_tags_please(self):

    while not rospy.is_shutdown():
      print("Calling tag localisation main")

      #try:

      start_time = time.time()

      ret_list = rfid_localization(\
        self.rfid_locations, \
        self.tags_infos, \
        self.storage_file, \
        self.robot_velocity, \
        self.antenna_poses_filelist, \
        self.rfid_measurements_filelist, \
        self.existed_tags, \
        self.last_timestamp, \
        self.Results)

      end_time = time.time()
      print "Tag localisation executed in", end_time-start_time, "sec"

      self.existed_tags = ret_list[0]
      self.last_timestamp = ret_list[1]
      self.Results = ret_list[2]

      rospy.sleep(1.0)

  #    except:
        #print "*********************************"
        #print "------- EXCEPTION CAUGHT ------- "
        #print "*********************************"



################################################################################
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('rfid_localisation_node')

  try:
    RFIDLocalisation()
    rospy.spin()
  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
