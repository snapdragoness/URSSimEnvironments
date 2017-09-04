#include "urs_wearable/pose.h"
#include "urs_wearable/controller.h"
#include "urs_wearable/navigator.h"

#include "urs_wearable/command.pb.h"
#include "urs_wearable/action.pb.h"

#include <ros/ros.h>

#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

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

          /*********************/
          /* Query the planner */
          /*********************/
          urs_protobuf::Action action;
          action.set_action_type(urs_protobuf::Action_ActionType_GOTO);
          action.set_x(1);
          action.set_y(2);
          action.set_z(3);

          std::cout << "Size after serializing: " << action.ByteSize() << std::endl;
          int size = action.ByteSize() + 4;
          char *pkt = new char[size];
          google::protobuf::io::ArrayOutputStream aos(pkt, size);
          google::protobuf::io::CodedOutputStream *coded_output = new google::protobuf::io::CodedOutputStream(&aos);
          coded_output->WriteVarint32(action.ByteSize());
          action.SerializeToCodedStream(coded_output);

          int bytecount;
          if((bytecount = write(plannerSockFD, (void *)pkt, size)) == -1)
          {
            std::cerr << "Error sending data " << errno << std::endl;
            delete pkt;
            goto FINISH;
          }
          printf("Sent bytes %d\n", bytecount);
          delete pkt;

          /********************/
          /* Execute the plan */
          /********************/
          // TODO
        }
      }
    }
  }

FINISH:
  // clean up
  close(execMonitorSockFD);
  close(plannerSockFD);

  // (optional) delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}
