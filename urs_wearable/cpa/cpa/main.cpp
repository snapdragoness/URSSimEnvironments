/* main.cc */

#include "reader.h"
#include "timer.h"
#include "planner.h"
#include "objects.h"

#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>

#include "../../include/protobuf_helper.h"
#include "../../include/proto_generated/planning.pb.h"
#include "../../include/proto_generated/state.pb.h"
#include "../../include/proto_generated/predicate.pb.h"
#include "../../include/proto_generated/action.pb.h"

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

  // remove any old socket and create an unnamed socket for the server
  unlink(CPA_PLUS_PATH_NAME);
  if ((plannerSockFD = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
  {
    std::cerr << "Failed to create a socket" << std::endl;
    exit(EXIT_FAILURE);
  }

  // name the socket
  plannerAddress.sun_family = AF_UNIX;
  strcpy(plannerAddress.sun_path, CPA_PLUS_PATH_NAME);

  if (bind(plannerSockFD, (struct sockaddr *)&plannerAddress, sizeof(struct sockaddr_un)) == -1)
  {
    std::cerr << "Failed to bind the socket" << std::endl;
    exit(EXIT_FAILURE);
  }

  // create a connection queue and wait for clients
  if (listen(plannerSockFD, 5) == -1)
  {
    std::cerr << "Failed to enable the socket to accept connections" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::cout << "Waiting for a connection from Execution Monitor..." << std::endl;

  // accept a connection
  int execMonitorSockFD;
  struct sockaddr_un execMonitorAddress;
  socklen_t execMonitorAddressLen = sizeof(sockaddr_un);
  if ((execMonitorSockFD = accept(plannerSockFD, (struct sockaddr*)&execMonitorAddress, &execMonitorAddressLen)) == -1)
  {
    std::cerr << "Failed to accept a connection request" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::cout << "Received the connection" << std::endl;

  while (true)
  {
    std::string domainDef
      ("(define (domain urs)\n"
      " (:requirements :conditional-effects :equality :typing)\n"
      " (:types id x y z)\n"
      " (:predicates (at ?id - id ?x - x ?y - y ?z - z)\n"
      " )\n"
      " (:action goto\n"
      "  :parameters (?id - id ?x0 - x ?y0 - y ?z0 - z ?x1 - x ?y1 - y ?z1 - z)\n"
      "  :precondition (at ?id ?x0 ?y0 ?z0)\n"
      "  :effect (and (not (at ?id ?x0 ?y0 ?z0)) (at ?id ?x1 ?y1 ?z1))\n"
      " )\n"
      ")\n");
    std::string initDef("");
    std::string goalDef("");

    // Define a set of objects for each type we have in domain definition
    Objects objs_id("id");
    Objects objs_x("x");
    Objects objs_y("y");
    Objects objs_z("z");

    google::protobuf::io::ZeroCopyInputStream* raw_input = new google::protobuf::io::FileInputStream(execMonitorSockFD);

    urs_wearable_pb::PlanningRequest planningRequest;
    if (!readDelimitedFrom(raw_input, &planningRequest))
    {
      std::cerr << "Execution Monitor has disconnected" << std::endl;
      delete raw_input;
      break;
    }
    delete raw_input;

    std::cout << "planningRequest: " << std::endl << planningRequest.DebugString();

    urs_wearable_pb::State* initialState = planningRequest.mutable_initial();
    for (int i = 0; i < initialState->at_size(); i++) {
      const urs_wearable_pb::At& at = initialState->at(i);

      objs_id.insert(at.uav_id());
      objs_x.insert(at.x());
      objs_y.insert(at.y());
      objs_z.insert(at.z());
      initDef += "(at " + Objects::intToObj("id", at.uav_id()) +
          " " + Objects::doubleToObj("x", at.x()) +
          " " + Objects::doubleToObj("y", at.y()) +
          " " + Objects::doubleToObj("z", at.z()) + ")";
    }

    urs_wearable_pb::State* goalState = planningRequest.mutable_goal();
    for (int i = 0; i < goalState->at_size(); i++) {
      const urs_wearable_pb::At& at = goalState->at(i);

      objs_id.insert(at.uav_id());
      objs_x.insert(at.x());
      objs_y.insert(at.y());
      objs_z.insert(at.z());
      goalDef += "(at " + Objects::intToObj("id", at.uav_id()) +
          " " + Objects::doubleToObj("x", at.x()) +
          " " + Objects::doubleToObj("y", at.y()) +
          " " + Objects::doubleToObj("z", at.z()) + ")";
    }

    ofstream pddlfile;
    pddlfile.open ("urs.pddl");
    pddlfile << domainDef
        << "(define (problem urs_prob)\n"
        << "  (:domain urs)\n"
        << "  (:objects";

    for (auto obj : objs_id.objs)
    {
      pddlfile << " " << obj;
    }
    pddlfile << " - " << objs_id.type;

    for (auto obj : objs_x.objs)
    {
      pddlfile << " " << obj;
    }
    pddlfile << " - " << objs_x.type;

    for (auto obj : objs_y.objs)
    {
      pddlfile << " " << obj;
    }
    pddlfile << " - " << objs_y.type;

    for (auto obj : objs_z.objs)
    {
      pddlfile << " " << obj;
    }
    pddlfile << " - " << objs_z.type;

    pddlfile << ")\n"
        << "  (:init (and " << initDef << "))\n"
        << "  (:goal (and " << goalDef << "))\n"
        << ")";
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
    urs_wearable_pb::PlanningResponse planningResponse;
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

      urs_wearable_pb::Action* action = planningResponse.add_actions();
      if (tokens[0].compare("goto") == 0)
      {
        action->set_type(urs_wearable_pb::Action_ActionType_GOTO);
        urs_wearable_pb::Goto* action_goto = action->mutable_goto_();

        action_goto->set_uav_id(Objects::objToInt("id", tokens[1]));
        action_goto->set_x(Objects::objToDouble("x", tokens[5]));
        action_goto->set_y(Objects::objToDouble("y", tokens[6]));
        action_goto->set_z(Objects::objToDouble("z", tokens[7]));
      }
    }

    std::cout << "planningResponse: " << std::endl << planningResponse.DebugString();
    google::protobuf::io::ZeroCopyOutputStream* raw_output = new google::protobuf::io::FileOutputStream(execMonitorSockFD);
    writeDelimitedTo(raw_output, planningResponse);
    delete raw_output;
  }

  // clean up
  close(execMonitorSockFD);
  close(plannerSockFD);

  // (optional) delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}
