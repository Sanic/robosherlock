#ifndef __QUERY_INTERFACE_H__
#define __QUERY_INTERFACE_H__

//rapidjson
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/pointer.h>

//STL
#include <vector>
#include <tuple>

//RS
#include <robosherlock/utils/output.h>
#include <robosherlock/utils/exception.h>
#include <robosherlock/queryanswering/ObjectDesignatorFactory.h>
#include <robosherlock/queryanswering/KnowledgeEngine.h>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>

// ROS
#include <ros/package.h>

class QueryInterface
{
private:

  std::shared_ptr<rs::KnowledgeEngine> knowledgeEngine_;

  struct QueryTermProperties {
    std::string key;
    std::vector<std::string> location;
    std::vector<std::string> check;
  };

  typedef std::map <std::string, QueryTermProperties> QueryTermDefinitions;
  QueryTermDefinitions queryTermDefs_;

  rapidjson::Document query_;

public:

  enum class QueryType {NONE, INSPECT, DETECT, SCAN, TRACK};

  //When answering queries, we'll always execute a method for pose estimation.
  //You can define here the name of the Annotator that should estimate the pose
  //and the name of the annotator after which the Pose Annotator should be ran
  // std::string pose_estimation_annotator_name_ = "Cluster3DGeometryAnnotator";
  // std::string pose_estimation_annotator_insert_after_ = "ClusterMerger";
  std::string pose_estimation_annotator_name_ = "Object6DPoseAnnotator";
  std::string pose_estimation_annotator_insert_after_ = "SceneGraphAnnotator";

  QueryInterface(std::shared_ptr<rs::KnowledgeEngine> ke)
  {
    knowledgeEngine_ = ke;
    query_ = rapidjson::Document(rapidjson::kObjectType);
    getQueryConfig();
  }

  ~QueryInterface()
  {
  }

  bool handleDetect(std::vector<std::string> &newPipelineOrder);


  bool handleInspect(std::vector<std::vector<std::string>> &newPipelineOrder);
  bool handleScan(std::vector<std::vector<std::string>> &newPipelineOrder);
  bool handleTrack(std::vector<std::string> &newPipelineOrder);

  bool getQueryConfig();

  rapidjson::Document &getQueryDocument();

  bool parseQuery(std::string query_);

  QueryType processQuery(std::vector<std::vector<std::string>> &new_pipeline_orders);


  bool filterResults(std::vector<std::string> &resultDesignators, std::vector<std::string> &filteredResponse, std::vector<bool> &designatorsToKeep);

  bool checkSubClass(const std::string &resultValue, const std::string &queryValue);

  bool checkClassProperty(const std::string &subject, const std::string &relation, const std::string &object);

  bool checkValueRestriction(const std::string &subject, const std::string &relation, const std::string &value);

  bool checkThresholdOnList(rapidjson::Value &list, const float threshold, std::string requestedKey, bool keepLower);

  bool extractQueryKeysFromDesignator(rapidjson::Value &desig, std::vector<std::string> &keys);

};

#endif // __QUERYINTERFACE_H__
