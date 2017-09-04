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

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "generated_proto/action.pb.h"

#define VERSION "1.2"
#define DEBUG

#ifndef BUILT_DATE
#define BUILT_DATE "-"
#endif

const char* CPA_PLUS_PATH_NAME = "/tmp/cpa_plus_socket";

Reader reader;
Timer timer;

google::protobuf::uint32 readHdr(char *buf)
{
  google::protobuf::uint32 size;
  google::protobuf::io::ArrayInputStream ais(buf, 4);
  google::protobuf::io::CodedInputStream coded_input(&ais);
  coded_input.ReadVarint32(&size);  // decode the HDR and get the size
  std::cout << "size of action: " << size << std::endl;
  return size;
}

void readBody(int csock, google::protobuf::uint32 size)
{
  int bytecount;
  urs_protobuf::Action action;
  char buffer[size + 4];  //size of the payload and hdr
  // read the entire buffer including the hdr
  if ((bytecount = recv(csock, (void *)buffer, size + 4, MSG_WAITALL)) == -1)
  {
    std::cerr << "Error receiving data %d" << errno << std::endl;
    return;
  }
  std::cout << "Second read byte count: " << bytecount << std::endl;

  // assign ArrayInputStream with enough memory
  google::protobuf::io::ArrayInputStream ais(buffer, size + 4);
  google::protobuf::io::CodedInputStream coded_input(&ais);

  // read an unsigned integer with Varint encoding, truncating to 32 bits.
  coded_input.ReadVarint32(&size);

  // after the message's length is read, PushLimit() is used to prevent the CodedInputStream
  // from reading beyond that length.Limits are used when parsing length-delimited embedded messages
  google::protobuf::io::CodedInputStream::Limit msgLimit = coded_input.PushLimit(size);

  // de-serialize
  action.ParseFromCodedStream(&coded_input);

  // once the embedded message has been parsed, PopLimit() is called to undo the limit
  coded_input.PopLimit(msgLimit);

  // print the message
  std::cout << "Message: " << action.DebugString();

}

void* SocketHandler(void* lp)
{
  int *csock = (int*)lp;

  char buffer[4];
  int bytecount = 0;

  memset(buffer, '\0', 4);

  while(true)
  {
    //Peek into the socket and get the packet size
    if ((bytecount = recv(*csock, buffer, 4, MSG_PEEK)) == -1)
    {
      usleep(10);
      continue;
    }
    else if (bytecount == 0)
    {
      break;
    }
    cout << "First read byte count is " << bytecount << endl;
    readBody(*csock, readHdr(buffer));
  }

  delete csock;
  return 0;
}

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

//  std::cout << "Waiting for connections..." << std::endl;

  // accept a connection
  int execMonitorSockFD;
  struct sockaddr_un execMonitorAddress;
  socklen_t execMonitorAddressLen = sizeof(sockaddr_un);
//  if ((execMonitorSockFD = accept(plannerSockFD, (struct sockaddr*)&execMonitorAddress, &execMonitorAddressLen)) == -1)
//  {
//    std::cerr << "Failed to accept a connection request" << std::endl;
//    exit(EXIT_FAILURE);
//  }

  pthread_t thread_id=0;
  while (true)
  {
    std::cout << "Waiting for connections..." << std::endl;
    int* csock = (int*)malloc(sizeof(int));
    if((*csock = accept(plannerSockFD, (sockaddr*)&execMonitorAddress, &execMonitorAddressLen)) != -1)
    {
      std::cout << "Received connection" << std::endl;
      pthread_create(&thread_id, 0, &SocketHandler, (void*)csock);
      pthread_detach(thread_id);
    }
    else
    {
      std::cerr << "Error accepting %d" << errno << std::endl;
    }
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
