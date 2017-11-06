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

  google::protobuf::io::ZeroCopyOutputStream* raw_output = new google::protobuf::io::FileOutputStream(execMonitorSockFD);
  google::protobuf::io::ZeroCopyInputStream* raw_input = new google::protobuf::io::FileInputStream(execMonitorSockFD);
  pb_wearable::WearableRequest wearableRequest;
  pb_wearable::WearableResponse wearableResponse;

  /***************************/
  /* Put your test code here */
  /***************************/

  // 1
  wearableRequest.set_type(wearableRequest.SET_DEST);

  pb_wearable::SetDestRepeated* setDestRepeated = wearableRequest.mutable_set_dest_repeated();
  pb_wearable::SetDestRepeated_SetDest* setDest = setDestRepeated->add_set_dest();
  setDest->set_uav_id(1);
  setDest->set_x(0);
  setDest->set_y(0);
  setDest->set_z(5);

  setDest = setDestRepeated->add_set_dest();
  setDest->set_uav_id(3);
  setDest->set_x(0);
  setDest->set_y(0);
  setDest->set_z(10);

  writeDelimitedTo(raw_output, wearableRequest);

  // 2
  wearableRequest.set_type(wearableRequest.GET_REGION);
  writeDelimitedTo(raw_output, wearableRequest);

  if (!readDelimitedFrom(raw_input, &wearableResponse))
  {
    std::cerr << "Something is wrong" << std::endl;
    goto CLEANUP;
  }

  std::cout << "wearableResponse: " << std::endl << wearableResponse.DebugString();

CLEANUP:
  delete raw_input;
  delete raw_output;

  close(execMonitorSockFD);

  return 0;
}
