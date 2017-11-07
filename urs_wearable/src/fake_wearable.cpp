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

#include <cstdlib>
#include <ctime>

const uint16_t PORT_EXEC_MONITOR = 8080;

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

  /* initialize random seed: */
  srand(time(NULL));

  /***************************/
  /* Put your test code here */
  /***************************/

  // 1
  pb_wearable::WearableRequest req1;
  req1.set_type(req1.SET_DEST_REPEATED);

  pb_wearable::SetDestRepeated* setDestRepeated = req1.mutable_set_dest_repeated();
  pb_wearable::SetDestRepeated_SetDest* setDest = setDestRepeated->add_set_dest();
  setDest->set_uav_id(1);
  setDest->set_x(rand() % 10 - 4);
  setDest->set_y(rand() % 10 - 4);
  setDest->set_z(rand() % 10 + 1);

  setDest = setDestRepeated->add_set_dest();
  setDest->set_uav_id(3);
  setDest->set_x(rand() % 11 - 4);
  setDest->set_y(rand() % 11 - 4);
  setDest->set_z(rand() % 10 + 1);

  std::cout << "SET_DEST_REPEATED - " << writeDelimitedToSockFD(execMonitorSockFD, req1) << std::endl;

  // 2
  pb_wearable::WearableRequest req2;
  req2.set_type(req2.GET_REGION);

  std::cout << "GET_REGION - " <<  writeDelimitedToSockFD(execMonitorSockFD, req2) << std::endl;

  pb_wearable::WearableResponse res1;
  if (!readDelimitedFromSockFD(execMonitorSockFD, res1))
  {
    std::cerr << "Something is wrong" << std::endl;
    return -1;
  }

  std::cout << "wearableResponse: " << std::endl << res1.DebugString();

CLEANUP:
  close(execMonitorSockFD);

  return 0;
}
