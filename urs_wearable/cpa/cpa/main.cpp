/* main.cc */

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

#include "../../include/protobuf_helper.h"
#include "planning.pb.h"
#include "state.pb.h"
#include "predicate.pb.h"
#include "action.pb.h"

#define VERSION "1.2"
#define DEBUG

#ifndef BUILT_DATE
#define BUILT_DATE "-"
#endif

const char* CPA_PLUS_PATH_NAME = "/tmp/cpa_plus_socket";

Reader reader;
Timer timer;

int main(int argc, char **argv)
{
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  std::cout << "CPA+ version " << VERSION << " - Built date: " << BUILT_DATE << std::endl;

  /*****************************************************/
  /* Establish a server for exec_monitor to connect to */
  /*****************************************************/
  int plannerSockFD;
  struct sockaddr_un plannerAddress;

  // Remove any old socket and create an unnamed socket for the server
  unlink(CPA_PLUS_PATH_NAME);
  if ((plannerSockFD = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
  {
    std::cerr << "Failed to create a socket" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Name the socket
  plannerAddress.sun_family = AF_UNIX;
  strcpy(plannerAddress.sun_path, CPA_PLUS_PATH_NAME);

  if (bind(plannerSockFD, (struct sockaddr *)&plannerAddress, sizeof(struct sockaddr_un)) == -1)
  {
    std::cerr << "Failed to bind the socket" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Create a connection queue and wait for clients
  if (listen(plannerSockFD, 5) == -1)
  {
    std::cerr << "Failed to enable the socket to accept connections" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::cout << "Waiting for a connection from Execution Monitor..." << std::endl;

  // Accept a connection
  int execMonitorSockFD;
  struct sockaddr_un execMonitorAddress;
  socklen_t execMonitorAddressLen = sizeof(sockaddr_un);
  if ((execMonitorSockFD = accept(plannerSockFD, (struct sockaddr*)&execMonitorAddress, &execMonitorAddressLen)) == -1)
  {
    std::cerr << "Failed to accept a connection request" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::cout << "Received the connection" << std::endl;

  // Read our domain definition from PDDL file
  std::ifstream ursDomainPDDL("urs_domain.pddl");
  std::stringstream strStream;
  strStream << ursDomainPDDL.rdbuf();
  std::string domainDef = strStream.str();

  while (true)
  {
    pb_urs::PlanningRequest planningRequest;
    if (!readDelimitedFromSockFD(execMonitorSockFD, planningRequest))
    {
      std::cerr << "Execution Monitor has disconnected" << std::endl;
      break;
    }

    std::cout << "planningRequest: " << std::endl << planningRequest.DebugString();

    std::string initDef("");
    int nInit = 0;
    std::string goalDef("");
    int nGoal = 0;

    // Define a set of objects for each type we have in domain definition
    Objects uav_id("uav_id");
    Objects wp_id("wp_id");

    pb_urs::State* initialState = planningRequest.mutable_initial();
    nInit += initialState->at_size();
    for (int i = 0; i < initialState->at_size(); i++) {
      const pb_urs::At& at = initialState->at(i);

      uav_id.insert(at.uav_id());
      wp_id.insert(at.wp_id());
      initDef += "(at " + uav_id.intToObj(at.uav_id()) + " " + wp_id.intToObj(at.wp_id()) + ")";
    }

    pb_urs::State* goalState = planningRequest.mutable_goal();
    nGoal += goalState->at_size();
    for (int i = 0; i < goalState->at_size(); i++) {
      const pb_urs::At& at = goalState->at(i);

      uav_id.insert(at.uav_id());
      wp_id.insert(at.wp_id());
      goalDef += "(at " + uav_id.intToObj(at.uav_id()) + " " + wp_id.intToObj(at.wp_id()) + ")";
    }

    std::string problemDef =
      "(define (problem urs_prob)\n"
      "  (:domain urs)\n"
      "  (:objects";

    for (auto obj : uav_id.objs)
    {
      problemDef += " " + obj;
    }
    problemDef += " - " + uav_id.type;

    for (auto obj : wp_id.objs)
    {
      problemDef += " " + obj;
    }
    problemDef += " - " + wp_id.type;

    problemDef += ")\n";
    if (nInit > 1)
    {
      problemDef += "  (:init (and " + initDef + "))\n";
    }
    else if (nInit > 0)
    {
      problemDef += "  (:init " + initDef + ")\n";
    }
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
    pb_urs::PlanningResponse planningResponse;
    for(map<string,int>::iterator it = planner.my_action_map.begin(); it != planner.my_action_map.end(); it++)
    {
      std::vector<std::string> tokens;
      boost::char_separator<char> sep {",()"};
      boost::tokenizer<boost::char_separator<char>> tok {it->first, sep};
      for (const auto &token : tok)
      {
        tokens.push_back((std::string)token);
        tokens.back().erase(0, 4);  // remove the leading "cpa_"
      }

      pb_urs::Action* action = planningResponse.add_actions();
      if (tokens[0].compare("goto") == 0)
      {
        action->set_type(pb_urs::Action_ActionType_GOTO);
        pb_urs::Goto* action_goto = action->mutable_goto_();

        action_goto->set_uav_id(uav_id.objToInt(tokens[1]));
        action_goto->set_wp_id(wp_id.objToInt(tokens[3]));
      }
    }

    std::cout << "planningResponse: " << std::endl << planningResponse.DebugString();
    writeDelimitedToSockFD(execMonitorSockFD, planningResponse);
  }

  // clean up
  close(execMonitorSockFD);
  close(plannerSockFD);

  // (optional) delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}
