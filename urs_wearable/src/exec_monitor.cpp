#include "urs_wearable/pose.h"
#include "urs_wearable/controller.h"
#include "urs_wearable/navigator.h"

#include "protobuf_helper.h"
#include "planning.pb.h"
#include "state.pb.h"
#include "predicate.pb.h"
#include "action.pb.h"
#include "wearable.pb.h"

#include <ros/ros.h>

const unsigned int N_UAV = 4;

/*** Socket communication ***/
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>

const char* CPA_PLUS_PATH_NAME = "/tmp/cpa_plus_socket";
const char* MADAGASCAR_PATH_NAME = "/tmp/madagascar_socket";
const char* PLANNER_PATH_NAME = CPA_PLUS_PATH_NAME;

const uint16_t PORT_EXEC_MONITOR = 8080;
/*** Socket communication [end] ***/

/*** Waypoint ***/
typedef struct Waypoint
{
  Pose pose;
  bool rotate;
} Waypoint;

const int WP_TABLE_SIZE = 100;
Waypoint wpTable[WP_TABLE_SIZE];
std::vector<int> wpUnusedIdList;

void wpInit();
int newWpId(std::vector<int>&);
void retrieveWpId(std::vector<int>&);
/*** Waypoint [end] ***/

/*** Region ***/
const double REGION_X0 = -50;
const double REGION_Y0 = -50;
const double REGION_X1 = 50;
const double REGION_Y1 = 50;
/*** Region [end] ***/

void initPlanningRequest(std::vector<int>&, pb_urs::State*, Controller*);

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

  wpInit();

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
  strcpy(plannerAddress.sun_path, PLANNER_PATH_NAME);

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
  ROS_INFO("Waiting for connections from wearable devices...");

  while (true)
  {
    fd_set readFDSet;                       // create a set of socket descriptors
    FD_ZERO(&readFDSet);                    // clear the socket set
    FD_SET(execMonitorSockFD, &readFDSet);  // add master socket to set

    // add child sockets to set
    int maxSockFD = execMonitorSockFD;
    for (std::vector<int>::iterator it = wearableSockFDList.begin(); it != wearableSockFDList.end(); it++)
    {
      int wearableSockFD = *it;

      // if valid socket descriptor, then add to read list
      if (wearableSockFD > 0)
        FD_SET(wearableSockFD, &readFDSet);

      // highest file descriptor number, need it for the select function
      if (wearableSockFD > maxSockFD)
        maxSockFD = wearableSockFD;
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
        int wearableSockFD = *it;
        if (FD_ISSET(wearableSockFD, &readFDSet))
        {
          /*********************************************/
          /* Wait for a command from a wearable device */
          /*********************************************/
          pb_wearable::WearableRequest wearableRequest;

          // check if it was for closing, and also read the incoming message
          if (!readDelimitedFromSockFD(wearableSockFD, wearableRequest))
          {
            // some client has disconnected, get its details and print
            getpeername(wearableSockFD, (struct sockaddr*)&execMonitorAddress, (socklen_t*)&execMonitorAddressLen);
            ROS_INFO("Client disconnected [IP: %s, Port: %d]", inet_ntoa(execMonitorAddress.sin_addr), ntohs(execMonitorAddress.sin_port));

            // close the socket
            close(wearableSockFD);
            wearableSockFDList.erase(it);  // 'it' now points to the next element after the erased one

            if (it == wearableSockFDList.end())
              break;
            continue;
          }

          ROS_INFO("Received Wearable Request");
          std::cout << wearableRequest.DebugString();

          /******************************/
          /* Evaluate the input request */
          /******************************/
          // Depending on the request, it may
          // - call the planner
          // - respond to the wearable
          // - just perform some actions
          // - a mix of the above

          std::vector<int> allocatedWpList;
          switch (wearableRequest.type())
          {
            // 'break' is used when a call to the planner is required
            // 'continue' is used when no calling to the planner is required

            case wearableRequest.GET_POSE_REPEATED:
            {
              pb_wearable::WearableResponse wearableResponse;
              wearableResponse.set_type(wearableResponse.POSE_REPEATED);

              const pb_wearable::GetPoseRepeated& getPoseRepeated = wearableRequest.get_pose_repeated();
              for (int i = 0; i < getPoseRepeated.get_pose_size(); i++)
              {
                int uav_id = getPoseRepeated.get_pose(i).uav_id();
                Pose pose = controller[uav_id].getPose();

                pb_wearable::PoseRepeated_Pose* poseRepeated_pose = wearableResponse.mutable_pose_repeated()->add_pose();
                poseRepeated_pose->set_uav_id(uav_id);
                poseRepeated_pose->set_x(pose.x);
                poseRepeated_pose->set_y(pose.y);
                poseRepeated_pose->set_z(pose.z);
                poseRepeated_pose->set_yaw(pose.yaw);
              }

              writeDelimitedToSockFD(wearableSockFD, wearableResponse);
              continue;
            }
            case wearableRequest.SET_DEST_REPEATED:
            {
              // Construct Planning Request
              pb_urs::PlanningRequest planningRequest;
              initPlanningRequest(allocatedWpList, planningRequest.mutable_initial(), controller);

              pb_urs::State* goalState = planningRequest.mutable_goal();
              const pb_wearable::SetDestRepeated& setDestRepeated = wearableRequest.set_dest_repeated();
              for (int i = 0; i < setDestRepeated.set_dest_size(); i++)
              {
                int wpId = newWpId(allocatedWpList);
                wpTable[wpId].pose.x = setDestRepeated.set_dest(i).x();
                wpTable[wpId].pose.y = setDestRepeated.set_dest(i).y();
                wpTable[wpId].pose.z = setDestRepeated.set_dest(i).z();
                if (setDestRepeated.set_dest(i).has_yaw())
                {
                  wpTable[wpId].pose.yaw = setDestRepeated.set_dest(i).yaw();
                  wpTable[wpId].rotate = true;
                }
                else
                {
                  wpTable[wpId].rotate = false;
                }

                pb_urs::At* at = goalState->add_at();
                at->set_uav_id(setDestRepeated.set_dest(i).uav_id());
                at->set_wp_id(wpId);
              }

              writeDelimitedToSockFD(plannerSockFD, planningRequest);
              break;
            }
            case wearableRequest.GET_REGION:
            {
              pb_wearable::WearableResponse wearableResponse;
              wearableResponse.set_type(wearableResponse.REGION);

              pb_wearable::Region* wearableResponse_region = wearableResponse.mutable_region();
              wearableResponse_region->set_x0(REGION_X0);
              wearableResponse_region->set_y0(REGION_Y0);
              wearableResponse_region->set_x1(REGION_X1);
              wearableResponse_region->set_y1(REGION_Y1);

              writeDelimitedToSockFD(wearableSockFD, wearableResponse);
              continue;
            }
          }

          /*******************************/
          /* Retrieve a plan and execute */
          /*******************************/
          pb_urs::PlanningResponse planningResponse;
          if (!readDelimitedFromSockFD(plannerSockFD, planningResponse))
          {
            std::cerr << "Planner has disconnected" << std::endl;
            break;
          }

          ROS_INFO("Received Planning Response");
          std::cout << planningResponse.DebugString();

          for (int i = 0; i < planningResponse.actions_size(); i++)
          {
            const pb_urs::Action& action = planningResponse.actions(i);
            switch (action.type())
            {
              case action.GOTO:
              {
                const pb_urs::Goto& action_goto = action.goto_();
                int wpId = action_goto.wp_id();
                if (wpTable[wpId].rotate)
                {
                  controller[action_goto.uav_id()].setDest(wpTable[wpId].pose);
                }
                else
                {
                  controller[action_goto.uav_id()].setDest(wpTable[wpId].pose.x, wpTable[wpId].pose.y, wpTable[wpId].pose.z);
                }
                break;
              }
            }
          }

          retrieveWpId(allocatedWpList);
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

void wpInit()
{
  wpUnusedIdList.reserve(WP_TABLE_SIZE);
  for (int i = WP_TABLE_SIZE - 1; i >= 0; i--)
  {
    wpUnusedIdList.push_back(i);
  }
}

int newWpId(std::vector<int>& allocatedWpList)
{
  int wp = wpUnusedIdList.back();
  wpUnusedIdList.pop_back();
  allocatedWpList.push_back(wp);
  return wp;
}

void retrieveWpId(std::vector<int>& allocatedWpList)
{
  for (std::vector<int>::reverse_iterator rit = allocatedWpList.rbegin(); rit != allocatedWpList.rend(); rit++)
  {
    wpUnusedIdList.push_back(*rit);
  }
}

void initPlanningRequest(std::vector<int>& allocatedWpList, pb_urs::State* initialState, Controller* controller)
{
  for (unsigned int i = 0; i < N_UAV; i++)
  {
    int wpId = newWpId(allocatedWpList);
    wpTable[wpId].pose = controller[i].getPose();
    wpTable[wpId].rotate = false;

    pb_urs::At* at = initialState->add_at();
    at->set_uav_id(i);
    at->set_wp_id(wpId);
  }
}
