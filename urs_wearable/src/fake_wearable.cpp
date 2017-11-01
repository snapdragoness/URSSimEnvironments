// Can be used to substitute a wearable device to test the execution monitor

#include <ros/ros.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include "protobuf_helper.h"
#include "wearable.pb.h"

const unsigned short PORT_EXEC_MONITOR = 8080;

int main(int argc, char const *argv[])
{
  struct sockaddr_in address;
  struct sockaddr_in serv_addr;
  int execMonitorSockFD;

  if ((execMonitorSockFD = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    std::cerr << "Socket creation error" << std::endl;
    return -1;
  }

  memset(&serv_addr, '0', sizeof(serv_addr));

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT_EXEC_MONITOR);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
  {
    std::cerr << "Invalid address / Address not supported" << std::endl;
    return -1;
  }

  if (connect(execMonitorSockFD, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
  {
    std::cerr << "Connection Failed" << std::endl;
    return -1;
  }

  ROS_INFO("Successfully connected to Execution Monitor");

  /***************************/
  /* Put your test code here */
  /***************************/

  pb_wearable::GotoRequest gotoRequest;

  gotoRequest.set_uav_id(1);
  gotoRequest.set_x(0);
  gotoRequest.set_y(0);
  gotoRequest.set_z(5);

  google::protobuf::io::ZeroCopyOutputStream* raw_output = new google::protobuf::io::FileOutputStream(execMonitorSockFD);
  writeDelimitedTo(raw_output, gotoRequest);
  delete raw_output;

  google::protobuf::io::ZeroCopyInputStream* raw_input = new google::protobuf::io::FileInputStream(execMonitorSockFD);
  pb_wearable::GotoResponse gotoResponse;
  if (!readDelimitedFrom(raw_input, &gotoResponse))
  {
    std::cerr << "Something is wrong" << std::endl;
    close(execMonitorSockFD);
    return -1;
  }
  delete raw_input;

  std::cout << "gotoResponse: " << std::endl << gotoResponse.DebugString();

  close(execMonitorSockFD);

  return 0;
}
