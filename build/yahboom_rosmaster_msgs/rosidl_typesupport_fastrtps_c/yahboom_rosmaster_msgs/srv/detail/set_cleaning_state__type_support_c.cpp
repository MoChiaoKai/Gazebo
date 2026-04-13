// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from yahboom_rosmaster_msgs:srv/SetCleaningState.idl
// generated code does not contain a copyright notice
#include "yahboom_rosmaster_msgs/srv/detail/set_cleaning_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "yahboom_rosmaster_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "yahboom_rosmaster_msgs/srv/detail/set_cleaning_state__struct.h"
#include "yahboom_rosmaster_msgs/srv/detail/set_cleaning_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SetCleaningState_Request__ros_msg_type = yahboom_rosmaster_msgs__srv__SetCleaningState_Request;

static bool _SetCleaningState_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetCleaningState_Request__ros_msg_type * ros_message = static_cast<const _SetCleaningState_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: desired_cleaning_state
  {
    cdr << (ros_message->desired_cleaning_state ? true : false);
  }

  return true;
}

static bool _SetCleaningState_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetCleaningState_Request__ros_msg_type * ros_message = static_cast<_SetCleaningState_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: desired_cleaning_state
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->desired_cleaning_state = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yahboom_rosmaster_msgs
size_t get_serialized_size_yahboom_rosmaster_msgs__srv__SetCleaningState_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetCleaningState_Request__ros_msg_type * ros_message = static_cast<const _SetCleaningState_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name desired_cleaning_state
  {
    size_t item_size = sizeof(ros_message->desired_cleaning_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SetCleaningState_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_yahboom_rosmaster_msgs__srv__SetCleaningState_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yahboom_rosmaster_msgs
size_t max_serialized_size_yahboom_rosmaster_msgs__srv__SetCleaningState_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: desired_cleaning_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = yahboom_rosmaster_msgs__srv__SetCleaningState_Request;
    is_plain =
      (
      offsetof(DataType, desired_cleaning_state) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SetCleaningState_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_yahboom_rosmaster_msgs__srv__SetCleaningState_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetCleaningState_Request = {
  "yahboom_rosmaster_msgs::srv",
  "SetCleaningState_Request",
  _SetCleaningState_Request__cdr_serialize,
  _SetCleaningState_Request__cdr_deserialize,
  _SetCleaningState_Request__get_serialized_size,
  _SetCleaningState_Request__max_serialized_size
};

static rosidl_message_type_support_t _SetCleaningState_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetCleaningState_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yahboom_rosmaster_msgs, srv, SetCleaningState_Request)() {
  return &_SetCleaningState_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "yahboom_rosmaster_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "yahboom_rosmaster_msgs/srv/detail/set_cleaning_state__struct.h"
// already included above
// #include "yahboom_rosmaster_msgs/srv/detail/set_cleaning_state__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // message
#include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions


using _SetCleaningState_Response__ros_msg_type = yahboom_rosmaster_msgs__srv__SetCleaningState_Response;

static bool _SetCleaningState_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetCleaningState_Response__ros_msg_type * ros_message = static_cast<const _SetCleaningState_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _SetCleaningState_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetCleaningState_Response__ros_msg_type * ros_message = static_cast<_SetCleaningState_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yahboom_rosmaster_msgs
size_t get_serialized_size_yahboom_rosmaster_msgs__srv__SetCleaningState_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetCleaningState_Response__ros_msg_type * ros_message = static_cast<const _SetCleaningState_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _SetCleaningState_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_yahboom_rosmaster_msgs__srv__SetCleaningState_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yahboom_rosmaster_msgs
size_t max_serialized_size_yahboom_rosmaster_msgs__srv__SetCleaningState_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = yahboom_rosmaster_msgs__srv__SetCleaningState_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SetCleaningState_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_yahboom_rosmaster_msgs__srv__SetCleaningState_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetCleaningState_Response = {
  "yahboom_rosmaster_msgs::srv",
  "SetCleaningState_Response",
  _SetCleaningState_Response__cdr_serialize,
  _SetCleaningState_Response__cdr_deserialize,
  _SetCleaningState_Response__get_serialized_size,
  _SetCleaningState_Response__max_serialized_size
};

static rosidl_message_type_support_t _SetCleaningState_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetCleaningState_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yahboom_rosmaster_msgs, srv, SetCleaningState_Response)() {
  return &_SetCleaningState_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "yahboom_rosmaster_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "yahboom_rosmaster_msgs/srv/set_cleaning_state.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SetCleaningState__callbacks = {
  "yahboom_rosmaster_msgs::srv",
  "SetCleaningState",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yahboom_rosmaster_msgs, srv, SetCleaningState_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yahboom_rosmaster_msgs, srv, SetCleaningState_Response)(),
};

static rosidl_service_type_support_t SetCleaningState__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SetCleaningState__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yahboom_rosmaster_msgs, srv, SetCleaningState)() {
  return &SetCleaningState__handle;
}

#if defined(__cplusplus)
}
#endif
