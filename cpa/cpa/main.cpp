#include "reader.h"
#include "timer.h"
#include "planner.h"

#include <iostream>
#include <fstream>
#include <regex>
#include <sstream>

#include <rosbridge_ws_client.hpp>

#define VERSION "1.2"

Reader reader;
Timer timer;

RosbridgeWsClient g_rbc("localhost:9090");
std::string g_domain_def;

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

  std::string problem_def = document["args"]["problem_def"].GetString();

  std::ofstream pddlfile;
  pddlfile.open ("urs.pddl");
  pddlfile << g_domain_def << "\n" << problem_def;
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

  rapidjson::Value plan(rapidjson::kArrayType);
  for (int i = 0; i < planner.my_action_map.size(); i++)
  {
    for (const auto& m : planner.my_action_map)
    {
      if (m.second == i)
      {
        std::string plan_step = std::regex_replace(m.first, std::regex("cpa_"), "");

        // Fluent API
        plan.PushBack(rapidjson::Value().SetString(plan_step.c_str(), plan_step.length(), allocator), allocator);

//        // Normal
//        rapidjson::Value str_val;
//        str_val.SetString(plan_step.c_str(), plan_step.length(), allocator);
//        plan.PushBack(str_val, allocator);
        break;
      }
    }
  }
  values.AddMember("plan", plan, allocator);

  // Print planning response message
  rapidjson::StringBuffer strbuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
  values.Accept(writer);
  std::cout << "Planning Response: " << strbuf.GetString() << std::endl;

  // Send planning response message
  g_rbc.serviceResponse(document["service"].GetString(), document["id"].GetString(), true, values);
}

int main(int argc, char **argv)
{
  std::cout << "CPA+ version " << VERSION << std::endl;

  // Read our domain definition from PDDL file
  std::ifstream domain_pddl("../urs_domain.pddl");
  std::stringstream str_stream;
  str_stream << domain_pddl.rdbuf();
  g_domain_def = str_stream.str();

  g_rbc.addClient("cpa_service_advertiser");
  g_rbc.advertiseService("cpa_service_advertiser", "/cpa/get_plan", "urs_wearable/GetPlan", advertiseServiceCallback);

  while(true);
}
