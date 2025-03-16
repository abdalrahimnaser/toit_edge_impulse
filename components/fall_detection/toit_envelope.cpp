#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <toit/toit.h>
#include <cstring>
#include "fall_detection.h" // Include the ML code header

// A struct to hold the context of the external service.
typedef struct {
  toit_msg_context_t* msg_context;
} echo_service_t;




extern "C" {
static toit_err_t on_created(void* user_data, toit_msg_context_t* context) {
  echo_service_t* echo_service = static_cast<echo_service_t*>(user_data);
  echo_service->msg_context = context;
  setup();
  return TOIT_OK;
}





static toit_err_t on_message(void* user_data, int sender, uint8_t* data, int length) {
  echo_service_t* echo_service = static_cast<echo_service_t*>(user_data);
  toit_msg_context_t* context = echo_service->msg_context;
  
  if (toit_msg_notify(context, sender, data, length, true) != TOIT_OK) {
    std::printf("unable to send\n");
  }
  return TOIT_OK;
}


static toit_err_t on_rpc_request(void* user_data, int sender, int function, toit_msg_request_handle_t handle, uint8_t* data, int length) {
  // Check if the message is "fetch result"
  const char* expected_msg = "fetch result";
  const int expected_length = strlen(expected_msg);
  
  // Only proceed with inference if the message matches
  if (length == expected_length && memcmp(data, expected_msg, expected_length) == 0) {
    int l = 4;
    uint8_t *temp = (uint8_t*)toit_malloc(l * sizeof(uint8_t));
    float outcome;

    outcome = inference();
    memcpy(temp, &outcome, l);

    if (toit_msg_request_reply(handle, temp, l, true) != TOIT_OK) {
      std::printf("unable to send\n");
    }
  
  }
  return TOIT_OK;
}


static toit_err_t on_removed(void* user_data) {
  std::free(user_data);
  return TOIT_OK;
}




namespace {
  void __attribute__((constructor)) init() {
  echo_service_t* echo_service = static_cast<echo_service_t*>(std::malloc(sizeof(echo_service_t)));
  echo_service->msg_context = nullptr;
  toit_msg_cbs_t cbs = TOIT_MSG_EMPTY_CBS();
  cbs.on_created = on_created;
  cbs.on_message = on_message;
  cbs.on_rpc_request = on_rpc_request;
  cbs.on_removed = on_removed;
  toit_msg_add_handler("toitlang.org/demo-echo", echo_service, cbs);
} }

} // extern "C"

