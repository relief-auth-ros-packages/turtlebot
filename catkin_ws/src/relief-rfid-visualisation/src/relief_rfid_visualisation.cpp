#include "relief_rfid_visualisation.h"

/*******************************************************************************
 * Constructor. Initializes params / timer / publisher.
 */
RfidVisualisation::RfidVisualisation(void) :
  view_locked_(false), service_lock_(false), latest_service_requested_(0)
{
  // Load params as always
  loadParams();

  // Store the content of tag_info.txt into a hash map
  loadTagInfo(&tag_info_map_);

  // Publish the images that indicate tag types.
  // THE FILENAMES SHOULD COMPLETELY REFLECT THE CONTENTS (second column) OF
  // THE TAG_INFO FILE IN NAME; TAGS WHOSE TYPE IS NOT PRESENT IN THE FILELIST
  // SHALL NOT BE VISUALISED (e.g.: TAG_INFO: EPCXXXXX, book; should have
  // a filelist entry called ~/some_path/imgs/book_123.png)
  publishTagTypesImages(rfid_tag_images_directory_, tag_info_map_);

  findTagTypesTopics(&tag_info_map_);

  marker_pub_ = nodehandle_.advertise<visualization_msgs::Marker>(
    "visualization_marker", 100);

  periodic_timer_ = nodehandle_.createWallTimer(
    ros::WallDuration(callback_period_), &RfidVisualisation::periodicCallback, this);

  // Visualisation services
  visualise_all_epcs_srv_ = nodehandle_.advertiseService(
    ros::this_node::getName() + "/" + visualise_all_tags_topic_,
    &RfidVisualisation::visualiseAllEPCsSRV, this);
  visualise_no_epcs_srv_ = nodehandle_.advertiseService(
    ros::this_node::getName() + "/" + visualise_no_tags_topic_,
    &RfidVisualisation::visualiseNoEPCsSRV, this);
  visualise_this_epc_srv_ = nodehandle_.advertiseService(
    ros::this_node::getName() + "/" + visualise_this_tag_topic_,
    &RfidVisualisation::visualiseThisEPCSRV, this);
  visualise_this_type_srv_ = nodehandle_.advertiseService(
    ros::this_node::getName() + "/" +visualise_this_tag_type_topic_,
    &RfidVisualisation::visualiseThisTypeSRV, this);

  // dynamic reconfigure callback
  dr_server_.setCallback(
    boost::bind(&RfidVisualisation::parametersDynamicCallback, this, _1, _2));

  ROS_INFO("[RfidVisualisation] Node initialised");
}


/*******************************************************************************
 * Destructor
 */
RfidVisualisation::~RfidVisualisation(void)
{
  ROS_INFO("[RfidVisualisation] Node destroyed");
}


/*******************************************************************************
 * Calls the rviz_plugin_manager PluginSetConfig service with custom string
 */
int RfidVisualisation::callRvizPluginManagerConfigSetService(
  const int& plugin_uid,
  const std::string& config_string)
{
  // The service client
  ros::ServiceClient rviz_set_config_client =
    nodehandle_.serviceClient<rviz_plugin_manager::PluginSetConfig>(
      "rviz_plugin_set_config");

  // The actual service
  rviz_plugin_manager::PluginSetConfig set_config_srv;

  // Set the plugin_uid request entry
  set_config_srv.request.plugin_uid = plugin_uid;

  // Set the config request entry
  set_config_srv.request.config = config_string;

  // Call the service
  if (rviz_set_config_client.call(set_config_srv))
    return set_config_srv.response.code;
  else
    return -1;
}


/*******************************************************************************
 * Calls the rviz_plugin_manager PluginLoad service with custom string
 */
int RfidVisualisation::callRvizPluginManagerPluginLoadService(
  const std::string& plugin_class,
  const std::string& plugin_name,
  const std::string& plugin_topic,
  const std::string& plugin_data_type,
  const std::string& plugin_config,
  int* plugin_uid)
{
  // The service client
  ros::ServiceClient rviz_load_client =
    nodehandle_.serviceClient<rviz_plugin_manager::PluginLoad>(
      "rviz_plugin_load");

  // The actual service
  rviz_plugin_manager::PluginLoad load_srv;

  // Fill-in all request entries
  load_srv.request.plugin_class = plugin_class;
  load_srv.request.plugin_name = plugin_name;
  load_srv.request.plugin_topic = plugin_topic;
  load_srv.request.plugin_data_type = plugin_data_type;
  load_srv.request.plugin_config = plugin_config;

  // Call the service
  if (rviz_load_client.call(load_srv))
  {
    *plugin_uid = load_srv.response.plugin_uid;
    return load_srv.response.code;
  }
  else
  {
    ROS_ERROR("[RfidVisualisation] Could not call \
      PluginManagerPluginLoadService for input string %s", plugin_name.c_str());

    return -1;
  }
}


/*******************************************************************************
*/
int RfidVisualisation::convertEPCToInt(const std::string& EPC)
{
  int n = EPC.length();
  char id_c[n + 1];
  strcpy(id_c, EPC.c_str());

  int i = 0;
  int id = 0;
  while (id_c[i] != 0)
  {
    id = (id_c[i] - '0')  + (id * 10);
    i++;
  }

  return id;
}


/*******************************************************************************
 * Deletes ALL MARKERS from rviz
 */
void RfidVisualisation::deleteAllMarkers()
{
  // Create the marker
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker. This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.
  // Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::DELETEALL;

  // Set the pose of the marker.
  // This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker_pub_.publish(marker);
}


/*******************************************************************************
*/
void RfidVisualisation::findTagTypesTopics(
  std::unordered_map<std::string, std::vector<std::string> >* tag_info_map)
{
  // Get topics
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);

  // Store topics' names in `topic_names`
  std::vector<std::string> topic_names;
  for (ros::master::V_TopicInfo::iterator it = topic_infos.begin();
    it != topic_infos.end(); it++)
  {
    topic_names.push_back(it->name);
  }


  // Now find these topics whose name includes the epc of this tag
  for  (std::unordered_map<std::string, std::vector<std::string> >::iterator
    it = tag_info_map->begin();
    it != tag_info_map->end(); it++)
  {
    std::string pattern(it->first + ".*image_raw$");
    std::regex rx(pattern);
    for (std::vector<std::string>::iterator tit = topic_names.begin();
      tit != topic_names.end(); tit++)
    {
      std::ptrdiff_t number_of_matches = std::distance(
        std::sregex_iterator(tit->begin(), tit->end(), rx),
        std::sregex_iterator());

      if (number_of_matches > 0)
        it->second.push_back(*tit);
    }
  }

  /*
     for  (std::unordered_map<std::string, std::vector<std::string> >::const_iterator
     it = tag_info_map->cbegin();
     it != tag_info_map->cend(); it++)
     {
     ROS_ERROR("%s", it->second[1].c_str());
     }
     */
}


/*******************************************************************************
 * Param loader.
 */
void RfidVisualisation::loadParams(void)
{
  nodehandle_.getParam(ros::this_node::getName() + "/rfid_locations_filename",
    rfid_locations_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/rfid_tag_info_filename",
    rfid_tag_info_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/rfid_tag_images_directory",
    rfid_tag_images_directory_);
  nodehandle_.getParam(ros::this_node::getName() + "/callback_period",
    callback_period_);
  nodehandle_.getParam(ros::this_node::getName() + "/use_images",
    use_images_);

  nodehandle_.getParam(ros::this_node::getName() + "/visualise_all_tags_topic",
    visualise_all_tags_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/visualise_no_tags_topic",
    visualise_no_tags_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/visualise_this_tag_topic",
    visualise_this_tag_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/visualise_this_tag_type_topic",
    visualise_this_tag_type_topic_);
}


/*******************************************************************************
 * Load tag information into a hash map for fast retrieval
 */
void RfidVisualisation::loadTagInfo(
  std::unordered_map<std::string, std::vector<std::string> >* map)
{
  std::ifstream rfid_tag_info_file(
    rfid_tag_info_filename_.c_str(), std::ios::in);

  if (rfid_tag_info_file.fail())
  {
    ROS_ERROR("[RfidVisualisation] FILE tag_info.txt does not exist");
    return;
  }

  if (rfid_tag_info_file.is_open())
  {
    std::string line;
    while (getline(rfid_tag_info_file,line))
    {
      size_t pos = 0;
      std::string delimiter = ",";
      std::string token;
      std::vector<std::string> split_string_v;

      while ((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        split_string_v.push_back(token);
        line.erase(0, pos + delimiter.length());
      }
      split_string_v.push_back(line);

      int num_nonempty = 0;
      for (unsigned int i = 0; i < split_string_v.size(); i++)
      {
        if (split_string_v[i].size() > 0)
          num_nonempty++;
      }

      if (num_nonempty == split_string_v.size())
      {
        std::vector<std::string> second_v;
        second_v.push_back(split_string_v[1]);
        map->emplace(split_string_v[0], second_v);
      }
    }

    rfid_tag_info_file.close();
  }
  else
  {
    ROS_ERROR("[RfidVisualisation] Unable to open tag_info file");
  }

  /* // For verification purposes
     for (std::unordered_map<std::string, std::string>::iterator it = map->begin();
     it != map->end(); it++)
     {
     ROS_ERROR("%s,%s", it->first.c_str(), it->second.c_str());
     }
     */
}


/*******************************************************************************
*/
void RfidVisualisation::parametersDynamicCallback(
  const relief_rfid_visualisation::relief_rfid_visualisation_cfgConfig& config,
  const uint32_t& level)
{
  ROS_INFO("[RfidVisualisation] Dynamic reconfigure callback");

  if (use_images_ ^ config.use_images)
  {
    use_images_ = config.use_images;

    if (use_images_)
      deleteAllMarkers();
    else
    {
      for (std::unordered_map<std::string, unsigned int>::const_iterator it =
        tag_plugin_uid_map_.begin(); it != tag_plugin_uid_map_.end(); it++)
      {
        std::string str = "Enabled: False";
        int code = callRvizPluginManagerConfigSetService(it->second, str);
      }
    }

    if (latest_service_requested_ == 0)
      visualiseAllEPCs();
    else if (latest_service_requested_ == 1)
      visualiseNoEPCs();
    else if (latest_service_requested_ == 2)
      visualiseThisEPC(latest_epc_requested_);
    else if (latest_service_requested_ == 3)
      visualiseThisType(latest_type_requested_);
    else
      visualiseAllEPCs();
  }
}


/*******************************************************************************
 * Is called every `callback_period_` seconds.
 * Reads rfid locations in file `rfid_locations_filename`.
 * Publishes them to rviz.
 *
 * The convention is that the content of file `rfid_locations_filename` is:
 * rfid 1: | EPC | kind | r | g | b | x | y | z |
 * rfid 2: | EPC | kind | r | g | b | x | y | z |
 *              ...
 * rfid n: | EPC | kind | r | g | b | x | y | z |
 */
void RfidVisualisation::periodicCallback(const ros::WallTimerEvent& event)
{
  if (view_locked_)
    return;

  std::ifstream rfid_locations_file(
    rfid_locations_filename_.c_str(), std::ios::in);

  if (rfid_locations_file.fail())
  {
    ROS_ERROR("[RfidVisualisation] FILE rfid_locations.txt does not exist");
    return;
  }

  if (rfid_locations_file.is_open())
  {
    std::string line;
    while (getline(rfid_locations_file,line))
    {
      size_t pos = 0;
      std::string delimiter = ",";
      std::string token;
      std::vector<std::string> split_string_v;

      while ((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        split_string_v.push_back(token);
        line.erase(0, pos + delimiter.length());
      }
      split_string_v.push_back(line);

      int num_nonempty = 0;
      for (unsigned int i = 0; i < split_string_v.size(); i++)
      {
        if (split_string_v[i].size() > 0)
          num_nonempty++;
      }

      /*
         if (num_nonempty == split_string_v.size())
         publishMarker(split_string_v);
         */

      if (num_nonempty == split_string_v.size())
      {
        publishImage(split_string_v);
        publishMarker(split_string_v);
      }
    }

    rfid_locations_file.close();
  }
  else
  {
    ROS_ERROR("[RfidVisualisation] Unable to open input file");
  }
}


/*******************************************************************************
 * Publishes the position of this EPC as a frame
 */
void RfidVisualisation::publishEPCFrame(const std::vector<std::string>& info)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = info[0];
  static_transformStamped.transform.translation.x = stod(info[1]) / 100;
  static_transformStamped.transform.translation.y = stod(info[2]) / 100;
  static_transformStamped.transform.translation.z = stod(info[3]) / 100;

  tf2::Quaternion q;
  q.setRPY(-M_PI/2, 0, 0);

  static_transformStamped.transform.rotation.x = q.x();
  static_transformStamped.transform.rotation.y = q.y();
  static_transformStamped.transform.rotation.z = q.z();
  static_transformStamped.transform.rotation.w = q.w();

  static_broadcaster.sendTransform(static_transformStamped);
}


/*******************************************************************************
 * Publishes the rviz plugin that shows the image of the type that the tag
 * is associated with.
 */
int RfidVisualisation::publishEPCRvizPlugin(
  const std::vector<std::string>& info)
{
  // lookup info[0] in tag_info_map_, find details
  std::unordered_map<std::string, std::vector<std::string> >::const_iterator
    it = tag_info_map_.find(info[0]);

  // If tag info[0] is not included in tag_info, then the image associated with
  // it is image 'unknown.png'. But info[0] is not included in tag_info_map_,
  // and hence there is no entry at all at tag_info_map_, therefore no
  // topic to be associated with it either.
  // If tag info[0] is not included in tag_info, find this topic and
  // use it. Else all is good.
  std::string image_topic;
  if (it == tag_info_map_.end())
  {
    // Get topics
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    // Store topics' names in `topic_names`
    std::vector<std::string> topic_names;
    for (ros::master::V_TopicInfo::iterator itt = topic_infos.begin();
      itt != topic_infos.end(); itt++)
    {
      topic_names.push_back(itt->name);
    }

    std::string pt = "unknown.*image_raw$";
    std::string pattern(pt);
    std::regex rx(pattern);
    for (std::vector<std::string>::iterator tit = topic_names.begin();
      tit != topic_names.end(); tit++)
    {
      std::ptrdiff_t number_of_matches = std::distance(
        std::sregex_iterator(tit->begin(), tit->end(), rx),
        std::sregex_iterator());

      if (number_of_matches > 0)
      {
        image_topic = *tit;
        break;
      }
    }
  }
  else
    image_topic = it->second[1];

  std::vector<int> response_codes;

  std::string plugin_class = "rviz_textured_quads/MeshDisplayCustom";
  std::string plugin_name = info[0];
  std::string plugin_topic = "";
  std::string plugin_data_type = "";
  std::string plugin_config = "";
  int uid;

  // Load the plugin to rviz
  int code = callRvizPluginManagerPluginLoadService(plugin_class,
    plugin_name, plugin_topic, plugin_data_type, plugin_config, &uid);

  response_codes.push_back(code);

  std::string config_str1 = "Image Topic: '" + image_topic + "'";
  std::string config_str2 = "Quad Frame: '" + info[0] + "'";
  std::string config_str3 = "Meters per pixel: '0.001'";


  if (response_codes[0] != -1)
  {
    code = callRvizPluginManagerConfigSetService(uid, config_str1);
    response_codes.push_back(code);

    if (response_codes[1] != -1)
    {
      code = callRvizPluginManagerConfigSetService(uid, config_str2);
      response_codes.push_back(code);

      if (response_codes[2] != -1)
      {
        code = callRvizPluginManagerConfigSetService(uid, config_str3);
        response_codes.push_back(code);

        if (response_codes[3] != -1)
          return uid;
        else
        {
          ROS_ERROR("[RfidVisualisation] Failed to config at level 3");
          return -1;
        }
      }
      else
      {
        ROS_ERROR("[RfidVisualisation] Failed to config at level 2");
        return -1;
      }
    }
    else
    {
      ROS_ERROR("[RfidVisualisation] Failed to config at level 1");
      return -1;
    }
  }
  else
  {
    ROS_ERROR("[RfidVisualisation] Failed to config at level 0");
    return -1;
  }

  return -1;
}


/*******************************************************************************
 * Publishes one image per rfid tag
 */
void RfidVisualisation::publishImage(const std::vector<std::string>& info)
{
  // Publish the position of the tag as a frame
  publishEPCFrame(info);

  // Lookup this tag in the tag_plugin_uid map. If the tag exists do not
  // publish the plugin for the same tag. If it does not, publish the plugin
  // and update the tag_plugin_uid map.
  std::unordered_map<std::string, unsigned int>::const_iterator it =
    tag_plugin_uid_map_.find(info[0]);

  if (it == tag_plugin_uid_map_.end())
  {
    // Publish the rviz plugin for this tag
    int plugin_uid = publishEPCRvizPlugin(info);

    // Update the map with information on this tag and its corresponding
    // plugin uid
    if (plugin_uid >= 0)
      tag_plugin_uid_map_.emplace(info[0], plugin_uid);
    else
      ROS_ERROR("[RfidVisualisation] Invalid plugin uid");
  }
}


/*******************************************************************************
 * Publishes one marker whose info is contained in string-vector `info`
 * in the form of a shape and an identifying text.
 */
void RfidVisualisation::publishMarker(const std::vector<std::string>& info)
{
  // Publish the shape
  publishMarkerShape(info);

  // Publish the text
  publishMarkerText(info);
}


/*******************************************************************************
*/
void RfidVisualisation::publishMarkerShape(const std::vector<std::string>& info)
{
  //ROS_INFO("[relief_rfid_visualisation] publishing marker shape");

  // Unpack the vector
  // Convert EPC to unique id
  int id = convertEPCToInt(info[0]);

  double r = 255;
  double g = 0;
  double b = 0;
  double x = stod(info[1]) / 100;
  double y = stod(info[2]) / 100;
  double z = stod(info[3]) / 100;


  // Create the marker
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.
  // This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = id;

  // Set the marker type
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action
  // Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.
  // This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x+0.1;
  marker.pose.position.y = y+0.1;
  marker.pose.position.z = z+0.1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = r/255;
  marker.color.g = g/255;
  marker.color.b = b/255;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker_pub_.publish(marker);
}


/*******************************************************************************
*/
void RfidVisualisation::publishMarkerText(const std::vector<std::string>& info)
{
  // Unpack the vector
  // Convert EPC to unique id
  int id = convertEPCToInt(info[0]);

  /*
     std::string kind = info[1];

  // [r]ed
  double r = stod(info[2]);
  // [g]reen
  double g = stod(info[3]);
  // [b]lue
  double b = stod(info[4]);

  double x = stod(info[5]);
  double y = stod(info[6]);
  double z = stod(info[7]);
  */

  double r = 255;
  double g = 0;
  double b = 0;
  double x = stod(info[1]) / 100;
  double y = stod(info[2]) / 100;
  double z = stod(info[3]) / 100;


  // Create the marker
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.
  // This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = id + 100;

  // Set the marker type
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  // Set the marker action
  // Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.
  // This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = r/255;
  marker.color.g = g/255;
  marker.color.b = b/255;
  marker.color.a = 1.0;

  //marker.text = info[0];
  //marker.text = kind;

  // Display the type depending on the epc
  std::unordered_map<std::string, std::vector<std::string> >::const_iterator it =
    tag_info_map_.find(info[0]);

  if (it == tag_info_map_.end())
    marker.text = "";
  else
    marker.text = it->second[0];

  marker.lifetime = ros::Duration();

  marker_pub_.publish(marker);
}


/*******************************************************************************
 * Reads filelist `types_filelist` and publishes the images pointed at by each
 * file in the filelist. The topic names where these images are published
 * are not controllable, but they include the filename of each image.
 */
void RfidVisualisation::publishTagTypesImages(
  const std::string& images_directory,
  const std::unordered_map<std::string, std::vector<std::string> >& tag_info_map)
{
  for (std::unordered_map<std::string, std::vector<std::string> >::const_iterator
    it = tag_info_map.cbegin();
    it != tag_info_map.cend(); it++)
  {
    std::string rosrun_str =
      "rosrun relief_rfid_image_publisher relief_rfid_image_publisher";
    std::string exec_string =
      "cd " + images_directory + "/; " + rosrun_str + " " + it->first + ".png &";

    //ROS_ERROR("%s", exec_string.c_str());

    std::system(exec_string.c_str());
    sleep(1);
  }

  // Give it some more time
  sleep(2);
}


/*******************************************************************************
*/
void RfidVisualisation::visualiseAllEPCs()
{
  ROS_INFO("[RfidVisualisation] Visualising all tags");

  if (use_images_)
  {
    for (std::unordered_map<std::string, unsigned int>::const_iterator it =
      tag_plugin_uid_map_.begin(); it != tag_plugin_uid_map_.end(); it++)
    {
      std::string str = "Enabled: True";
      int code = callRvizPluginManagerConfigSetService(it->second, str);
    }
  }
  // else: no need when rviz markers are used; they are updated periodically as is
}


/*******************************************************************************
*/
bool RfidVisualisation::visualiseAllEPCsSRV(
  relief_rfid_visualisation::VisualiseAll::Request& req,
  relief_rfid_visualisation::VisualiseAll::Response& res)
{
  if (!service_lock_)
  {
    service_lock_ = true;

    // Store the id of this service
    latest_service_requested_ = 0;

    visualiseAllEPCs();

    view_locked_ = false;
    service_lock_ = false;

    return true;
  }
  else return false;
}


/*******************************************************************************
*/
void RfidVisualisation::visualiseNoEPCs()
{
  ROS_INFO("[RfidVisualisation] Visualising no tags");

  deleteAllMarkers();

  if (use_images_)
  {
    for (std::unordered_map<std::string, unsigned int>::const_iterator it =
      tag_plugin_uid_map_.begin(); it != tag_plugin_uid_map_.end(); it++)
    {
      std::string str = "Enabled: False";
      int code = callRvizPluginManagerConfigSetService(it->second, str);
    }
  }
}


/*******************************************************************************
*/
bool RfidVisualisation::visualiseNoEPCsSRV(
  relief_rfid_visualisation::VisualiseNone::Request& req,
  relief_rfid_visualisation::VisualiseNone::Response& res)
{
  if (!service_lock_)
  {
    service_lock_ = true;
    view_locked_ = true;

    // Store the id of this service
    latest_service_requested_ = 1;

    visualiseNoEPCs();

    service_lock_ = false;
    return true;
  }
  else
    return false;
}


/*******************************************************************************
*/
void RfidVisualisation::visualiseThisEPC(const std::string& EPC)
{
  ROS_INFO("[RfidVisualisation] Visualising tag %s", EPC.c_str());


  deleteAllMarkers();

  std::ifstream rfid_locations_file(
    rfid_locations_filename_.c_str(), std::ios::in);

  if (rfid_locations_file.fail())
  {
    ROS_ERROR("[RfidVisualisation] FILE rfid_locations.txt does not exist");
    return;
  }

  if (rfid_locations_file.is_open())
  {
    std::string line;
    while (getline(rfid_locations_file,line))
    {
      size_t pos = 0;
      std::string delimiter = ",";
      std::string token;
      std::vector<std::string> split_string_v;

      while ((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        split_string_v.push_back(token);
        line.erase(0, pos + delimiter.length());
      }

      split_string_v.push_back(line);

      if (split_string_v[0].compare(EPC) != 0)
        continue;

      int num_nonempty = 0;
      for (unsigned int i = 0; i < split_string_v.size(); i++)
      {
        if (split_string_v[i].size() > 0)
          num_nonempty++;
      }

      if (num_nonempty == split_string_v.size())
        publishMarker(split_string_v);
    }

    rfid_locations_file.close();
  }
  else
  {
    ROS_ERROR("[RfidVisualisation] Unable to open input file");
  }


  if (use_images_)
  {
    // lookup this epc in tag_plugin_uid_map_
    std::unordered_map<std::string, unsigned int>::const_iterator it_epc =
      tag_plugin_uid_map_.find(EPC);

    // if the tag exists at all
    if (it_epc != tag_plugin_uid_map_.end())
    {
      ROS_ERROR("[RfidVisualisation] Tag with EPC %s exists", EPC.c_str());

      for (std::unordered_map<std::string, unsigned int>::const_iterator it =
        tag_plugin_uid_map_.begin(); it != tag_plugin_uid_map_.end(); it++)
      {
        std::string str = "";
        if (it->first.compare(EPC) != 0)
          str = "Enabled: False";
        else
          str = "Enabled: True";

        int code = callRvizPluginManagerConfigSetService(it->second, str);
      }
    }
    else
    {
      ROS_ERROR("[RfidVisualisation] Tag with EPC %s does not exist",
        EPC.c_str());
      return;
    }
  }
}


/*******************************************************************************
*/
bool RfidVisualisation::visualiseThisEPCSRV(
  relief_rfid_visualisation::VisualiseThisEPC::Request& req,
  relief_rfid_visualisation::VisualiseThisEPC::Response& res)
{
  if (!service_lock_)
  {
    service_lock_ = true;
    view_locked_ = true;

    // Store the type
    latest_epc_requested_ = req.EPC;

    // Store the id of this service
    latest_service_requested_ = 2;

    visualiseThisEPC(req.EPC);

    service_lock_ = false;
    return true;
  }
  else
    return false;
}


/*******************************************************************************
*/
void RfidVisualisation::visualiseThisType(const std::string& type)
{
  ROS_INFO("[RfidVisualisation] Visualising tag type %s", type.c_str());

  deleteAllMarkers();

  std::ifstream rfid_locations_file(
    rfid_locations_filename_.c_str(), std::ios::in);

  if (rfid_locations_file.fail())
  {
    ROS_ERROR("[RfidVisualisation] FILE rfid_locations.txt does not exist");
    return;
  }

  if (rfid_locations_file.is_open())
  {
    std::string line;
    while (getline(rfid_locations_file,line))
    {
      size_t pos = 0;
      std::string delimiter = ",";
      std::string token;
      std::vector<std::string> split_string_v;

      while ((pos = line.find(delimiter)) != std::string::npos)
      {
        token = line.substr(0, pos);
        split_string_v.push_back(token);
        line.erase(0, pos + delimiter.length());
      }

      split_string_v.push_back(line);


      // Check if this tag's type is type in tag_info_map_
      std::unordered_map< std::string, std::vector<std::string> >::const_iterator
        it = tag_info_map_.find(split_string_v[0]);

      if (it == tag_info_map_.end() || it->second[0].compare(type) != 0)
        continue;

      int num_nonempty = 0;
      for (unsigned int i = 0; i < split_string_v.size(); i++)
      {
        if (split_string_v[i].size() > 0)
          num_nonempty++;
      }

      if (num_nonempty == split_string_v.size())
        publishMarker(split_string_v);
    }

    rfid_locations_file.close();
  }
  else
    ROS_ERROR("[RfidVisualisation] Unable to open input file");


  if (use_images_)
  {
    // Disable all at first
    for (std::unordered_map<std::string, unsigned int>::const_iterator it =
      tag_plugin_uid_map_.begin(); it != tag_plugin_uid_map_.end(); it++)
    {
      std::string str = "Enabled: False";
      int code = callRvizPluginManagerConfigSetService(it->second, str);
    }

    // Check to see if type exists at all. At the same time store the EPCs
    // for tags that tag `type` objects
    std::vector<std::string> type_epcs;

    for (std::unordered_map<std::string, std::vector<std::string> >::const_iterator
      it = tag_info_map_.begin();
      it != tag_info_map_.end(); it++)
    {
      if (it->second[0].compare(type) == 0)
        type_epcs.push_back(it->first);
    }

    // The type exists
    if (type_epcs.size() > 0)
    {
      for (unsigned int i = 0; i < type_epcs.size(); i++)
      {
        std::unordered_map<std::string, unsigned int>::const_iterator it =
          tag_plugin_uid_map_.find(type_epcs[i]);

        if (it != tag_plugin_uid_map_.end())
        {
          std::string str = "Enabled: True";
          int code = callRvizPluginManagerConfigSetService(it->second, str);
        }
      }
    }
    else
    {
      ROS_ERROR("[RfidVisualisation] Type %s does not exist", type.c_str());
      return;
    }
  }
}


/*******************************************************************************
*/
bool RfidVisualisation::visualiseThisTypeSRV(
  relief_rfid_visualisation::VisualiseThisType::Request& req,
  relief_rfid_visualisation::VisualiseThisType::Response& res)
{
  if (!service_lock_)
  {
    service_lock_ = true;
    view_locked_ = true;

    // Store the type
    latest_type_requested_ = req.type;

    // Store the id of this service
    latest_service_requested_ = 3;

    visualiseThisType(req.type);

    service_lock_ = false;
    return true;
  }
  else
    return false;
}
