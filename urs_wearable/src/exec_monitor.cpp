#include "urs_wearable/pose.h"
#include "urs_wearable/controller.h"
#include "urs_wearable/navigator.h"

#include <ros/ros.h>

#include <vector>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

const unsigned int N_UAV = 4;

const unsigned short PORT_PLANNER = 8081;

const unsigned short PORT_WEARABLE = 8080;
const unsigned int BUFFER_SIZE = 1024;

int main(int argc, char **argv)
{
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

  /*********************************************************/
  /* Establish a server for wearable devices to connect to */
  /*********************************************************/
  int opt = 1;
  int master_socket;
  std::vector<int> client_socket;
  struct sockaddr_in address;
  int addrlen = sizeof(address);
  char buffer[BUFFER_SIZE + 1];

  // create a master socket
  if ((master_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    ROS_ERROR("Failed to create a master socket");
    exit(EXIT_FAILURE);
  }

  // set master socket to allow multiple connections.
  // this is just a good habit, it will work without this
  if (setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0)
  {
    ROS_ERROR("Failed to set the socket option");
    exit(EXIT_FAILURE);
  }

  // type of the socket created
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT_WEARABLE);

  // bind the socket to localhost at PORT_WEARABLE
  if (bind(master_socket, (struct sockaddr *)&address, sizeof(address)) < 0)
  {
    ROS_ERROR("Failed to bind the socket");
    exit(EXIT_FAILURE);
  }

  // try to specify maximum of 3 pending connections for the master socket
  if (listen(master_socket, 3) < 0)
  {
    ROS_ERROR("Failed to enable the socket socket to accept connections");
    exit(EXIT_FAILURE);
  }

  // accept the incoming connections
  ROS_INFO("Waiting for connections...");

  while (true)
  {
    fd_set readfds;                   // create a set of socket descriptors
    FD_ZERO(&readfds);                // clear the socket set
    FD_SET(master_socket, &readfds);  // add master socket to set

    // add child sockets to set
    int max_sd = master_socket;
    for (std::vector<int>::iterator it = client_socket.begin(); it != client_socket.end(); it++)
    {
      int sd = *it;

      // if valid socket descriptor, then add to read list
      if (sd > 0)
        FD_SET(sd, &readfds);

      // highest file descriptor number, need it for the select function
      if (sd > max_sd)
        max_sd = sd;
    }

    // wait for an activity on one of the sockets, timeout is NULL, so wait indefinitely
    if (select(max_sd + 1, &readfds, NULL, NULL, NULL) < 0)
    {
      ROS_ERROR("Select error");
    }

    // if something happened on the master socket, then its an incoming connection
    if (FD_ISSET(master_socket, &readfds))
    {
      int new_socket;
      if ((new_socket = accept(master_socket, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0)
      {
        ROS_ERROR("Failed to accept a connection request");
        exit(EXIT_FAILURE);
      }

      // inform user of socket number - used in send and receive commands
      ROS_INFO("New connection [Socket fd: %d, IP: %s, Port: %d]", new_socket, inet_ntoa(address.sin_addr), ntohs(address.sin_port));

      // add the new socket to the vector of sockets
      client_socket.push_back(new_socket);
    }
    else    // else it is some IO operations on some other sockets
    {
      for (std::vector<int>::iterator it = client_socket.begin(); it != client_socket.end(); it++)
      {
        int sd = *it;
        if (FD_ISSET(sd, &readfds))
        {
          int valread;

          // check if it was for closing, and also read the incoming message
          if ((valread = read(sd, buffer, BUFFER_SIZE)) == 0)
          {
            // some client has disconnected, get its details and print
            getpeername(sd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
            ROS_INFO("Client disconnected [IP: %s, Port: %d]", inet_ntoa(address.sin_addr), ntohs(address.sin_port));

            // close the socket
            close(sd);
            client_socket.erase(it);  // 'it' now points to the next element after the erased one

            if (it == client_socket.end())
              break;
            continue;
          }

          // echo back the message that came in
          buffer[valread] = '\0';
          write(sd, buffer, strlen(buffer));
        }
      }
    }
  }
}
