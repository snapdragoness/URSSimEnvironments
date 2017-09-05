/* main.cc */

#include "reader.h"
#include "timer.h"
#include "planner.h"
#include <iostream>
#include <map>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>

#include "../../include/proto_generated/action.pb.h"
#include "../../include/protobuf_helper.h"

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
    urs_protobuf::Action* action = new urs_protobuf::Action;
    google::protobuf::io::ZeroCopyInputStream* raw_input = new google::protobuf::io::FileInputStream(execMonitorSockFD);
    if (!readDelimitedFrom(raw_input, action))
    {
      std::cerr << "Execution Monitor has disconnected" << std::endl;
      delete raw_input;
      delete action;
      break;
    }
    std::cout << "Message: " << action->DebugString();
    delete raw_input;
    delete action;
  }

//  system("./swi_script.sh");
//
//  Planner planner(&reader, &timer);
//
//  FILE* f = fopen("theory_names", "r");
//  // TODO: check open file
//  char filename[16];
//  while (!feof(f))
//  {
//    fscanf(f, "%s", filename);
//    if (freopen(filename, "r", stdin) == NULL)
//    {
//      std::cerr << "ERROR: Cannot open " << filename << std::endl;
//    }
//
//    reader.read();
//    planner.main();
//  }
//
//  for(int i = 0; i < planner.my_action_map.size(); i++)
//  {
//    for(map<string,int>::const_iterator it = planner.my_action_map.begin(); it != planner.my_action_map.end(); it++)
//    {
//      if(it->second == i)
//      {
//        std::cout << it->first << std::endl;
//      }
//    }
//  }

  // clean up
  close(execMonitorSockFD);
  close(plannerSockFD);

  // (optional) delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}
