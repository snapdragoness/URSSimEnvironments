#include "reader.h"
#include "timer.h"
#include "planner.h"
#include "objects.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/tokenizer.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>

#include <rosbridge_ws_client.hpp>
#include <urs_wearable/GetPlan.h>

#define VERSION "1.2"

Reader reader;
Timer timer;

RosbridgeWsClient rbc("localhost:9090");
std::string domainDef;

void advertiseServiceCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::Message> message)
{
  // message->string() is destructive, so we have to buffer it first
  std::string messagebuf = message->string();

  std::cout << "Planning Request: " << messagebuf << std::endl;

  rapidjson::Document document;
  if (document.Parse(messagebuf.c_str()).HasParseError())
  {
    std::cerr << "Error in parsing service request message: " << messagebuf << std::endl;
    return;
  }

  // Define a set of objects for each type we have in domain definition
  Objects uavIdObjects("uav_id");
  Objects wpIdObjects("wp_id");

  // Construct initial state definition
  std::string initDef("");
  int nInit = 0;

  const rapidjson::Value& initialState = document["args"]["initial_state"];
  const rapidjson::Value& initialStateAt = initialState["at"];
  for (rapidjson::SizeType i = 0; i < initialStateAt.Size(); i++)
  {
    int uavId = initialStateAt[i]["uav_id"].GetUint();
    int wpId = initialStateAt[i]["wp_id"].GetUint();

    uavIdObjects.insert(uavId);
    wpIdObjects.insert(wpId);

    initDef += "(at " + uavIdObjects.intToObj(uavId) + " " + wpIdObjects.intToObj(wpId) + ")";
  }
  nInit += initialStateAt.Size();

  // Construct goal state definition
  std::string goalDef("");
  int nGoal = 0;

  const rapidjson::Value& goalState = document["args"]["goal_state"];
  const rapidjson::Value& goalStateAt = goalState["at"];
  for (rapidjson::SizeType i = 0; i < goalStateAt.Size(); i++)
  {
    int uavId = goalStateAt[i]["uav_id"].GetUint();
    int wpId = goalStateAt[i]["wp_id"].GetUint();

    uavIdObjects.insert(uavId);
    wpIdObjects.insert(wpId);

    goalDef += "(at " + uavIdObjects.intToObj(uavId) + " " + wpIdObjects.intToObj(wpId) + ")";
  }
  nGoal += goalStateAt.Size();

  // Construct object definition
  std::string problemDef =
    "(define (problem urs_prob)\n"
    "  (:domain urs)\n"
    "  (:objects";

  for (auto obj : uavIdObjects.objs)
  {
    problemDef += " " + obj;
  }
  problemDef += " - " + uavIdObjects.type;

  for (auto obj : wpIdObjects.objs)
  {
    problemDef += " " + obj;
  }
  problemDef += " - " + wpIdObjects.type;

  problemDef += ")\n";

  // There is a bug in CPA that :init must have "and"
//  if (nInit > 1)
//  {
//    problemDef += "  (:init (and " + initDef + "))\n";
//  }
//  else if (nInit > 0)
//  {
//    problemDef += "  (:init " + initDef + ")\n";
//  }
  problemDef += "  (:init (and " + initDef + "))\n";

  if (nGoal > 1)
  {
    problemDef += "  (:goal (and " + goalDef + "))\n";
  }
  else if (nGoal > 0)
  {
    problemDef += "  (:goal " + goalDef + ")\n";
  }
  problemDef += ")";

  std::ofstream pddlfile;
  pddlfile.open ("urs.pddl");
  pddlfile << domainDef << "\n" << problemDef;
  pddlfile.close();

  /*********************/
  /* Optimize and plan */
  /*********************/
  system("./swi_script.sh");

  Planner planner(&reader, &timer);

  FILE* f = fopen("theory_names", "r");

  // TODO: check open file
  char filename[16];
  while (!feof(f))
  {
    fscanf(f, "%s", filename);
    if (freopen(filename, "r", stdin) == NULL)
    {
      std::cerr << "ERROR: Cannot open " << filename << std::endl;
    }

    reader.read();
    planner.main();
  }

  /**************************************/
  /* Return a plan to Execution Monitor */
  /**************************************/
  // Construct planning response message
  rapidjson::Document values(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = values.GetAllocator();

  rapidjson::Value actions(rapidjson::kArrayType);

  for(map<string, int>::iterator it = planner.my_action_map.begin(); it != planner.my_action_map.end(); it++)
  {
    std::cout << it->first << std::endl;

    std::vector<std::string> tokens;
    boost::char_separator<char> sep {",()"};
    boost::tokenizer<boost::char_separator<char>> tok {it->first, sep};
    for (const auto &token : tok)
    {
      tokens.push_back((std::string)token);
      tokens.back().erase(0, 4);  // remove the leading "cpa_"
    }

    if (tokens[0].compare("goto") == 0)
    {
      rapidjson::Value actionGoto(rapidjson::kObjectType);
      actionGoto.AddMember("uav_id", uavIdObjects.objToInt(tokens[1]), allocator);
      actionGoto.AddMember("wp_id", wpIdObjects.objToInt(tokens[3]), allocator);

      rapidjson::Value action(rapidjson::kObjectType);
      action.AddMember("action_type", 0, allocator);
      action.AddMember("action_goto", actionGoto, allocator);

      actions.PushBack(action, allocator);
    }
  }

  values.AddMember("actions", actions, allocator);

  // Print planning response message
  rapidjson::StringBuffer strbuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
  values.Accept(writer);
  std::cout << "Planning Response: " << strbuf.GetString() << std::endl;

  // Send planning response message
  rbc.serviceResponse(document["service"].GetString(), document["id"].GetString(), true, values);
}

int main(int argc, char **argv)
{
  std::cout << "CPA+ version " << VERSION << std::endl;

  // Read our domain definition from PDDL file
  std::ifstream ursDomainPDDL("../urs_domain.pddl");
  std::stringstream strStream;
  strStream << ursDomainPDDL.rdbuf();
  domainDef = strStream.str();

  rbc.addClient("cpa_service_advertiser");
  rbc.advertiseService("cpa_service_advertiser", "/cpa/get_plan", "urs_wearable/GetPlan", advertiseServiceCallback);

  while(true);
}
