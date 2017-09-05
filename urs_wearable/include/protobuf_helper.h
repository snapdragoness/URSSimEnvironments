#ifndef URS_WEARABLE_INCLUDE_PROTOBUF_HELPER_H_
#define URS_WEARABLE_INCLUDE_PROTOBUF_HELPER_H_

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message_lite.h>

bool readDelimitedFrom(google::protobuf::io::ZeroCopyInputStream* rawInput, google::protobuf::MessageLite* message);
bool writeDelimitedTo(google::protobuf::io::ZeroCopyOutputStream* rawOutput, const google::protobuf::MessageLite& message);

#endif /* URS_WEARABLE_INCLUDE_PROTOBUF_HELPER_H_ */
