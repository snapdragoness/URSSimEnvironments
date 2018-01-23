#include "urs_wearable/controller.h"
#include "urs_wearable/navigator.h"
#include "urs_wearable/pool.h"
#include "urs_wearable/thread_manager.h"
#include "urs_wearable/colormod.h"

#include "protobuf_helper.h"
#include "planning.pb.h"
#include "state.pb.h"
#include "predicate.pb.h"
#include "action.pb.h"
#include "wearable.pb.h"

#include "urs_wearable/ActionsAction.h"

#include <ros/ros.h>
#include <actionlib/client/service_client.h>
#include <boost/chrono.hpp>

const unsigned int N_UAV = 4;

const int WP_POOL_SIZE = 100;
const int PLANNER_POOL_SIZE = 2;

const double REGION_X0 = -50;
const double REGION_Y0 = -50;
const double REGION_X1 = 50;
const double REGION_Y1 = 50;

const int DRONE_STATE_PUBLISH_DELAY_MS = 100;

/*** Socket communication ***/
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>

const char* PLANNER_PATH_NAME[PLANNER_POOL_SIZE] =
{
  "/tmp/cpa_plus_socket_1",
  "/tmp/cpa_plus_socket_2",
};

const uint16_t PORT_EXEC_MONITOR = 8080;
/*** Socket communication [end] ***/

Pool<Waypoint, WP_POOL_SIZE> wp_pool;
Pool<int, PLANNER_POOL_SIZE> planner_pool;
ThreadManager threadManager;

Controller controller[N_UAV];
Navigator navigator[N_UAV];
actionlib::SimpleActionClient<urs_wearable::ActionsAction>* action_client[N_UAV];

void initPlanningRequest(std::vector<int>&, pb_urs::State*);
void droneStatePublisher(int);
void wearableRequestHandler(int, const pb_wearable::WearableRequest);

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

  for (unsigned int i = 0; i < N_UAV; i++)
  {
    controller[i].init();
    controller[i].setNamespace("/uav" + std::to_string(i));
    controller[i].start();

    navigator[i].init();
    navigator[i].setNamespace("/uav" + std::to_string(i));

    action_client[i] = new (actionlib::SimpleActionClient<urs_wearable::ActionsAction>) ("action_server_" + std::to_string(i));
    action_client[i]->waitForServer();
  }

  /************************************************/
  /* Establish a client to connect to the planner */
  /************************************************/
  for (int i = 0; i < PLANNER_POOL_SIZE; i++)
  {
    int plannerSockFD;
    struct sockaddr_un plannerAddress;

    // create a socket for the client
    if ((plannerSockFD = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
      ROS_ERROR("Failed to create a planner socket for %s", PLANNER_PATH_NAME[i]);
      exit(EXIT_FAILURE);
    }

    // name the socket, as agreed with the server
    plannerAddress.sun_family = AF_UNIX;
    strcpy(plannerAddress.sun_path, PLANNER_PATH_NAME[i]);

    // now connect our socket to the server's socket
    if (connect(plannerSockFD, (struct sockaddr*)&plannerAddress, sizeof(struct sockaddr_un)) == -1)
    {
      ROS_ERROR("Failed to connect to %s", PLANNER_PATH_NAME[i]);
      exit(EXIT_FAILURE);
    }

    planner_pool.data[i] = plannerSockFD;
    ROS_INFO("Successfully connect to %s", PLANNER_PATH_NAME[i]);
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

      // create drone's state publisher thread
      threadManager.add(newSockFD, new boost::thread(droneStatePublisher, newSockFD));
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

            // remove any running threads from this wearable
            threadManager.remove(wearableSockFD);

            // close the socket
            close(wearableSockFD);
            wearableSockFDList.erase(it);  // 'it' now points to the next element after the erased one

            if (it == wearableSockFDList.end())
              break;
            continue;
          }

          std::cout << Color::fg_green << "-> Received WearableRequest" << Color::fg_default << std::endl << wearableRequest.DebugString();

          // create a wearableRequestHandler thread for the incoming request
          boost::thread(wearableRequestHandler, wearableSockFD, wearableRequest);
        }
      }
    }
  }

  // clean up
  for (int i = 0; i < N_UAV; i++)
  {
    delete action_client[i];
  }

  for (int i = 0; i < PLANNER_POOL_SIZE; i++)
  {
    close(planner_pool.data[i]);
  }
  close(execMonitorSockFD);

  // (optional) delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();
}

void initPlanningRequest(std::vector<int>& allocatedWpList, pb_urs::State* initialState)
{
  for (unsigned int i = 0; i < N_UAV; i++)
  {
    int wpId = wp_pool.newId(allocatedWpList);
    while (wpId == -1)
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
      wpId = wp_pool.newId(allocatedWpList);
    }

    wp_pool.data[wpId].pose = controller[i].getPose();
    wp_pool.data[wpId].rotate = false;

    pb_urs::At* at = initialState->add_at();
    at->set_uav_id(i);
    at->set_wp_id(wpId);
  }
}

void droneStatePublisher(int wearableSockFD)
{
  while(true)
  {
    try
    {
      pb_wearable::WearableResponse wearableResponse;
      wearableResponse.set_type(pb_wearable::WearableResponse::PERIODIC_STATUS);

      for (unsigned int uav_id = 0; uav_id < N_UAV; uav_id++)
      {
        Pose pose = controller[uav_id].getPose();

        pb_wearable::WearableResponse_PeriodicStatus_Pose* _pose = wearableResponse.mutable_periodic_status()->add_pose();
        _pose->set_uav_id(uav_id);
        _pose->set_x(pose.x);
        _pose->set_y(pose.y);
        _pose->set_z(pose.z);
        _pose->set_yaw(pose.yaw);
      }

      writeDelimitedToSockFD(wearableSockFD, wearableResponse);

      boost::this_thread::sleep_for(boost::chrono::milliseconds(DRONE_STATE_PUBLISH_DELAY_MS));
    }
    catch (boost::thread_interrupted&)
    {
      break;
    }
  }
}

void wearableRequestHandler(int wearableSockFD, const pb_wearable::WearableRequest wearableRequest)
{
  /******************************/
  /* Evaluate the input request */
  /******************************/
  // Depending on the request, it may
  // - call the planner
  // - respond to the wearable
  // - just perform some actions
  // - a mix of the above

  std::vector<int> allocatedWpList;
  std::vector<int> allocatedPlannerList;
  pb_urs::PlanningRequest planningRequest;

  switch (wearableRequest.type())
  {
    case pb_wearable::WearableRequest::GET_POSE_REPEATED:
    {
      pb_wearable::WearableResponse wearableResponse;
      wearableResponse.set_type(pb_wearable::WearableResponse::POSE_REPEATED);

      const pb_wearable::WearableRequest_GetPoseRepeated& getPoseRepeated = wearableRequest.get_pose_repeated();
      for (int i = 0; i < getPoseRepeated.get_pose_size(); i++)
      {
        int uav_id = getPoseRepeated.get_pose(i).uav_id();
        Pose pose = controller[uav_id].getPose();

        pb_wearable::WearableResponse_PoseRepeated_Pose* _pose = wearableResponse.mutable_pose_repeated()->add_pose();
        _pose->set_uav_id(uav_id);
        _pose->set_x(pose.x);
        _pose->set_y(pose.y);
        _pose->set_z(pose.z);
        _pose->set_yaw(pose.yaw);
      }

      writeDelimitedToSockFD(wearableSockFD, wearableResponse);
      return;
    }
    case pb_wearable::WearableRequest::SET_DEST_REPEATED:
    {
      // Construct Planning Request
      initPlanningRequest(allocatedWpList, planningRequest.mutable_initial());

      pb_urs::State* goalState = planningRequest.mutable_goal();
      const pb_wearable::WearableRequest_SetDestRepeated& setDestRepeated = wearableRequest.set_dest_repeated();
      for (int i = 0; i < setDestRepeated.set_dest_size(); i++)
      {
        int wpId = wp_pool.newId(allocatedWpList);
        while (wpId == -1)
        {
          boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
          wpId = wp_pool.newId(allocatedWpList);
        }

        wp_pool.data[wpId].pose.x = setDestRepeated.set_dest(i).x();
        wp_pool.data[wpId].pose.y = setDestRepeated.set_dest(i).y();
        wp_pool.data[wpId].pose.z = setDestRepeated.set_dest(i).z();
        if (setDestRepeated.set_dest(i).has_yaw())
        {
          wp_pool.data[wpId].pose.yaw = setDestRepeated.set_dest(i).yaw();
          wp_pool.data[wpId].rotate = true;
        }
        else
        {
          wp_pool.data[wpId].rotate = false;
        }

        pb_urs::At* at = goalState->add_at();
        at->set_uav_id(setDestRepeated.set_dest(i).uav_id());
        at->set_wp_id(wpId);
      }

      break;
    }
    case pb_wearable::WearableRequest::GET_REGION:
    {
      pb_wearable::WearableResponse wearableResponse;
      wearableResponse.set_type(pb_wearable::WearableResponse::REGION);

      pb_wearable::WearableResponse_Region* region = wearableResponse.mutable_region();
      region->set_x0(REGION_X0);
      region->set_y0(REGION_Y0);
      region->set_x1(REGION_X1);
      region->set_y1(REGION_Y1);

      writeDelimitedToSockFD(wearableSockFD, wearableResponse);
      return;
    }
  }

  // Send a planning request
  int plannerId = planner_pool.newId(allocatedPlannerList);
  while (plannerId == -1)
  {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    plannerId = planner_pool.newId(allocatedPlannerList);
  }
  int plannerSockFD = planner_pool.data[plannerId];

  writeDelimitedToSockFD(plannerSockFD, planningRequest);

  /*********************************/
  /* Retrieve the plan and execute */
  /*********************************/
  pb_urs::PlanningResponse planningResponse;
  if (!readDelimitedFromSockFD(plannerSockFD, planningResponse))
  {
    std::cerr << "Planner has disconnected" << std::endl;
    return;
  }
  planner_pool.retrieveId(allocatedPlannerList);

  std::cout << Color::fg_green << "-> Received PlanningResponse" << Color::fg_default << std::endl << planningResponse.DebugString();

  for (int i = 0; i < planningResponse.actions_size(); i++)
  {
    const pb_urs::Action& action = planningResponse.actions(i);
    switch (action.type())
    {
      case action.GOTO:
      {
        const pb_urs::Goto& action_goto = action.goto_();
        int wpId = action_goto.wp_id();
        urs_wearable::ActionsGoal goal;

        if (wp_pool.data[wpId].rotate)
        {
          goal.action_type = 1;
        }
        else
        {
          goal.action_type = 2;
        }

        goal.pose.x = wp_pool.data[wpId].pose.x;
        goal.pose.y = wp_pool.data[wpId].pose.y;
        goal.pose.z = wp_pool.data[wpId].pose.z;
        goal.pose.yaw = wp_pool.data[wpId].pose.yaw;

        action_client[action_goto.uav_id()]->sendGoal(goal);
//        action_client[action_goto.uav_id()]->waitForResult(ros::Duration(5.0));

        ros::Rate r(100);
        while (action_client[action_goto.uav_id()]->getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
          r.sleep();
        }

        std::cout << Color::fg_blue << action_client[action_goto.uav_id()]->getState().toString() << Color::fg_default << std::endl;
        break;
      }
    }
  }

  wp_pool.retrieveId(allocatedWpList);
}
