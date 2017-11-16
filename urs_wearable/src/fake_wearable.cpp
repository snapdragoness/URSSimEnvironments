// Can be used to substitute a wearable device to test the execution monitor

#include "urs_wearable/colormod.h"

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

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

const uint16_t PORT_EXEC_MONITOR = 8080;

Color::Modifier fg_blue(Color::FG_BLUE);
Color::Modifier fg_green(Color::FG_GREEN);
Color::Modifier fg_default(Color::FG_DEFAULT);

int expectedWearableResponseType = -1;
pb_wearable::WearableResponse expectedWearableResponse;

void wearableResponseListener(int);

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

  // 0
//  while (true) {
//    pb_wearable::WearableResponse wearableResponse;
//    if (!readDelimitedFromSockFD(execMonitorSockFD, wearableResponse))
//    {
//      std::cerr << "Something is wrong" << std::endl;
//      return -1;
//    }
//
//    std::cout << "Status: " << std::endl << wearableResponse.DebugString();
//  }

  boost::thread(wearableResponseListener, execMonitorSockFD);

  while (true)
  {
    // 1
    pb_wearable::WearableRequest req1;
    req1.set_type(pb_wearable::WearableRequest::SET_DEST_REPEATED);

    pb_wearable::WearableRequest_SetDestRepeated* setDestRepeated = req1.mutable_set_dest_repeated();
    pb_wearable::WearableRequest_SetDestRepeated_SetDest* setDest = setDestRepeated->add_set_dest();
    setDest->set_uav_id(rand() % 4);
    setDest->set_x(rand() % 10 - 4);
    setDest->set_y(rand() % 10 - 4);
    setDest->set_z(rand() % 10 + 1);

    setDest = setDestRepeated->add_set_dest();
    setDest->set_uav_id(rand() % 4);
    setDest->set_x(rand() % 11 - 4);
    setDest->set_y(rand() % 11 - 4);
    setDest->set_z(rand() % 10 + 1);

    std::cout << fg_blue << "WearableRequest::SET_DEST_REPEATED sent - " << fg_default
        << writeDelimitedToSockFD(execMonitorSockFD, req1) << std::endl;

    // 2
    pb_wearable::WearableRequest req2;
    req2.set_type(pb_wearable::WearableRequest::GET_REGION);
    expectedWearableResponseType = pb_wearable::WearableRequest::GET_REGION;
    std::cout << fg_blue << "WearableRequest::GET_REGION sent - " << fg_default
        << writeDelimitedToSockFD(execMonitorSockFD, req2) << std::endl;

    while (expectedWearableResponseType != -1)
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }
    std::cout << fg_green << "-> Received WearableResponse" << std::endl << expectedWearableResponse.DebugString() << fg_default;

    boost::this_thread::sleep_for(boost::chrono::milliseconds(10000));
  }

  close(execMonitorSockFD);
  return 0;
}

void wearableResponseListener(int execMonitorSockFD)
{
  while (true)
  {
    try
    {
      pb_wearable::WearableResponse wearableResponse;
      if (!readDelimitedFromSockFD(execMonitorSockFD, wearableResponse))
      {
        std::cerr << "Something is wrong" << std::endl;
        return;
      }

      if (wearableResponse.type() == pb_wearable::WearableResponse::PERIODIC_STATUS)
      {
        std::cout << "-> Received WearableResponse::PERIODIC_STATUS" << std::endl;
      }
      else if (wearableResponse.type() == expectedWearableResponseType)
      {
        expectedWearableResponse = wearableResponse;
        expectedWearableResponseType = -1;
      }
    }
    catch (boost::thread_interrupted&)
    {
      break;
    }
  }
}
