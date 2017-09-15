#include "urs_wearable/pose.h"
#include "urs_wearable/controller.h"
#include "urs_wearable/navigator.h"

#include "protobuf_helper.h"
#include "proto_generated/planning.pb.h"
#include "proto_generated/state.pb.h"
#include "proto_generated/predicate.pb.h"
#include "proto_generated/action.pb.h"

#include <ros/ros.h>

#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

const unsigned int N_UAV = 4;

const char* CPA_PLUS_PATH_NAME = "/tmp/cpa_plus_socket";

const unsigned short PORT_EXEC_MONITOR = 8080;
const unsigned int BUFFER_SIZE = 1024;

int main(int argc, char **argv)
{
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  /**************************************/
  /* Initialize ROS, and sets up a node */
  /**************************************/
  ros::init(argc, argv, "exec_monitor");
  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

  if (N_UAV == 0) {
    ROS_ERROR("No UAV to control");
    exit(EXIT_FAILURE);
  }

  Controller controller[N_UAV];
  Navigator navigator[N_UAV];

  for (unsigned int i = 0; i < N_UAV; i++)
  {
    controller[i].setNamespace("/uav" + std::to_string(i));
    controller[i].start();
    navigator[i].setNamespace("/uav" + std::to_string(i));
  }

  /************************************************/
  /* Establish a client to connect to the planner */
  /************************************************/
  int plannerSockFD;
  struct sockaddr_un plannerAddress;

  // create a socket for the client
  if ((plannerSockFD = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
  {
    ROS_ERROR("Failed to create a planner socket");
    exit(EXIT_FAILURE);
  }

  // name the socket, as agreed with the server
  plannerAddress.sun_family = AF_UNIX;
  strcpy(plannerAddress.sun_path, CPA_PLUS_PATH_NAME);

  // now connect our socket to the server's socket
  if (connect(plannerSockFD, (struct sockaddr*)&plannerAddress, sizeof(struct sockaddr_un)) == -1)
  {
    ROS_ERROR("Failed to connect to the planner");
    exit(EXIT_FAILURE);
  }

  /*********************************************************/
  /* Establish a server for wearable devices to connect to */
  /*********************************************************/
  int opt = 1;
  int execMonitorSockFD;
  struct sockaddr_in execMonitorAddress;
  int execMonitorAddressLen = sizeof(execMonitorAddress);
  std::vector<int> wearableSockFDList;

  // create a master socket
  if ((execMonitorSockFD = socket(AF_INET, SOCK_STREAM, 0)) == -1)
  {
    ROS_ERROR("Failed to create an execution monitor socket");
    exit(EXIT_FAILURE);
  }

  // set master socket to allow multiple connections.
  // this is just a good habit, it will work without this
  if (setsockopt(execMonitorSockFD, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt)) == -1)
  {
    ROS_ERROR("Failed to set the socket option");
    exit(EXIT_FAILURE);
  }

  // type of the socket created
  execMonitorAddress.sin_family = AF_INET;
  execMonitorAddress.sin_addr.s_addr = INADDR_ANY;
  execMonitorAddress.sin_port = htons(PORT_EXEC_MONITOR);

  // bind the socket to localhost at PORT_WEARABLE
  if (bind(execMonitorSockFD, (struct sockaddr*)&execMonitorAddress, sizeof(struct sockaddr_in)) == -1)
  {
    ROS_ERROR("Failed to bind the socket");
    exit(EXIT_FAILURE);
  }

  // try to specify maximum of 5 pending connections for the master socket
  if (listen(execMonitorSockFD, 5) == -1)
  {
    ROS_ERROR("Failed to enable the socket to accept connections");
    exit(EXIT_FAILURE);
  }

  // accept incoming connections
  ROS_INFO("Waiting for connections...");

  while (true)
  {
    fd_set readFDSet;                       // create a set of socket descriptors
    FD_ZERO(&readFDSet);                    // clear the socket set
    FD_SET(execMonitorSockFD, &readFDSet);  // add master socket to set

    // add child sockets to set
    int maxSockFD = execMonitorSockFD;
    for (std::vector<int>::iterator it = wearableSockFDList.begin(); it != wearableSockFDList.end(); it++)
    {
      int wearableSockfd = *it;

      // if valid socket descriptor, then add to read list
      if (wearableSockfd > 0)
        FD_SET(wearableSockfd, &readFDSet);

      // highest file descriptor number, need it for the select function
      if (wearableSockfd > maxSockFD)
        maxSockFD = wearableSockfd;
    }

    // wait for an activity on one of the sockets, timeout is NULL, so wait indefinitely
    if (select(maxSockFD + 1, &readFDSet, NULL, NULL, NULL) == -1)
    {
      ROS_ERROR("Select error");
    }

    // if something happened on the master socket, then its an incoming connection
    if (FD_ISSET(execMonitorSockFD, &readFDSet))
    {
      int newSockFD;
      if ((newSockFD = accept(execMonitorSockFD, (struct sockaddr *)&execMonitorAddress, (socklen_t*)&execMonitorAddressLen)) == -1)
      {
        ROS_ERROR("Failed to accept a connection request");
        exit(EXIT_FAILURE);
      }

      // inform user of socket number - used in send and receive commands
      ROS_INFO("New connection [Socket fd: %d, IP: %s, Port: %d]", newSockFD, inet_ntoa(execMonitorAddress.sin_addr), ntohs(execMonitorAddress.sin_port));

      // add the new socket to the vector of sockets
      wearableSockFDList.push_back(newSockFD);
    }
    else    // else it is some IO operations on some other sockets
    {
      for (std::vector<int>::iterator it = wearableSockFDList.begin(); it != wearableSockFDList.end(); it++)
      {
        int wearableSockfd = *it;
        if (FD_ISSET(wearableSockfd, &readFDSet))
        {
          /*********************************************/
          /* Wait for a command from a wearable device */
          /*********************************************/
          int bytesRead;
          char buffer[BUFFER_SIZE + 1];

          // check if it was for closing, and also read the incoming message
          if ((bytesRead = read(wearableSockfd, buffer, BUFFER_SIZE)) == 0)
          {
            // some client has disconnected, get its details and print
            getpeername(wearableSockfd, (struct sockaddr*)&execMonitorAddress, (socklen_t*)&execMonitorAddressLen);
            ROS_INFO("Client disconnected [IP: %s, Port: %d]", inet_ntoa(execMonitorAddress.sin_addr), ntohs(execMonitorAddress.sin_port));

            // close the socket
            close(wearableSockfd);
            wearableSockFDList.erase(it);  // 'it' now points to the next element after the erased one

            if (it == wearableSockFDList.end())
              break;
            continue;
          }

          // echo back the message that came in
          buffer[bytesRead] = '\0';
          write(wearableSockfd, buffer, strlen(buffer));

          /**********************************************/
          /* Evaluate the command and query the planner */
          /**********************************************/
          // break the input into tokens
          std::string bufferString = std::string(buffer);
          boost::algorithm::trim(bufferString);
          std::vector<std::string> tokens;
          boost::char_separator<char> sep {" "};
          boost::tokenizer<boost::char_separator<char>> tok {bufferString, sep};
          for (const auto &token : tok)
          {
            tokens.push_back((std::string)token);
          }
          if (tokens[0].compare("goto") == 0 && tokens.size() == 5)
          {
            ROS_INFO("Received Command: GOTO");

            urs_wearable_pb::PlanningRequest planningRequest;
            urs_wearable_pb::State* initialState = planningRequest.mutable_initial();

            for (unsigned int i = 0; i < N_UAV; i++)
            {
              urs_wearable_pb::At* at = initialState->add_at();
              Pose pose = controller[i].getPose();
              at->set_uav_id(i);
              at->set_x(pose.x);
              at->set_y(pose.y);
              at->set_z(pose.z);
            }

            urs_wearable_pb::State* goalState = planningRequest.mutable_goal();
            urs_wearable_pb::At* at = goalState->add_at();
            at->set_uav_id(std::stoi(tokens[1]));
            at->set_x(std::stod(tokens[2]));
            at->set_y(std::stod(tokens[3]));
            at->set_z(std::stod(tokens[4]));

            google::protobuf::io::ZeroCopyOutputStream* raw_output = new google::protobuf::io::FileOutputStream(plannerSockFD);
            writeDelimitedTo(raw_output, planningRequest);
            delete raw_output;
          }
          else if (tokens[0].compare("move") == 0 && tokens.size() == 5)
          {
            ROS_INFO("Received Command: MOVE");

            urs_wearable_pb::PlanningRequest planningRequest;
            urs_wearable_pb::State* initialState = planningRequest.mutable_initial();

            for (unsigned int i = 0; i < N_UAV; i++)
            {
              urs_wearable_pb::At* at = initialState->add_at();
              Pose pose = controller[i].getPose();
              at->set_uav_id(i);
              at->set_x(pose.x);
              at->set_y(pose.y);
              at->set_z(pose.z);
            }

            urs_wearable_pb::State* goalState = planningRequest.mutable_goal();
            urs_wearable_pb::At* at = goalState->add_at();
            Pose pose = controller[std::stoi(tokens[1])].getPose();
            at->set_uav_id(std::stoi(tokens[1]));
            at->set_x(pose.x + std::stod(tokens[2]));
            at->set_y(pose.y + std::stod(tokens[3]));
            at->set_z(pose.z + std::stod(tokens[4]));

            google::protobuf::io::ZeroCopyOutputStream* raw_output = new google::protobuf::io::FileOutputStream(plannerSockFD);
            writeDelimitedTo(raw_output, planningRequest);
            delete raw_output;
          }
          else
          {
            ROS_ERROR("Invalid Command");
          }

          /*******************************/
          /* Retrieve a plan and execute */
          /*******************************/
          google::protobuf::io::ZeroCopyInputStream* raw_input = new google::protobuf::io::FileInputStream(plannerSockFD);
          urs_wearable_pb::PlanningResponse planningResponse;
          if (!readDelimitedFrom(raw_input, &planningResponse))
          {
            std::cerr << "Planner has disconnected" << std::endl;
            delete raw_input;
            break;
          }
          delete raw_input;

          ROS_INFO("Received a plan");
          std::cout << planningResponse.DebugString();

          for (int i = 0; i < planningResponse.actions_size(); i++)
          {
            const urs_wearable_pb::Action& action = planningResponse.actions(i);
            switch (action.type())
            {
              case urs_wearable_pb::Action_ActionType_GOTO:
              {
                const urs_wearable_pb::Goto& action_goto = action.goto_();
                Pose pose;
                pose.x = action_goto.x();
                pose.y = action_goto.y();
                pose.z = action_goto.z();
                controller[action_goto.uav_id()].setDest(pose);
                break;
              }
            }
          }
        }
      }
    }
  }

  // clean up
  close(execMonitorSockFD);
  close(plannerSockFD);

  // (optional) delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}
