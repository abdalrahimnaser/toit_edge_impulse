// #include <cstddef>
// #include <cstdio>
// #include <cstdlib>
// #include <cstdint>
// #include <toit/toit.h>
// #include <Fall_Detection__inferencing.h>
// #include <LSM6DS3.h>
// #include <stdio.h> 

// // #ifndef FREERTOS_H
// // printf("Well, FreeRTOS wasn't defined already\n");
// // #endif

// typedef struct{
//     const char *name;
//     float *value;
//     uint8_t (*poll_sensor)(void);
//     bool (*init_sensor)(void);
//     int8_t status;  // -1 not used 0 used(unitialized) 1 used(initalized) 2 data sampled
// } eiSensors;

// #define CONVERT_G_TO_MS2    9.80665f
// #define MAX_ACCEPTED_RANGE  2.0f        
// #define N_SENSORS     7

// float ei_get_sign(float number);
// static bool ei_connect_fusion_list(const char *input_list);
// bool init_IMU(void);
// bool init_ADC(void);
// uint8_t poll_IMU(void);
// uint8_t poll_ADC(void);

// /* Private variables ------------------------------------------------------- */
// static const bool debug_nn = true; // Set this to true to see e.g. features generated from the raw signal
// static float data[N_SENSORS];
// static int8_t fusion_sensors[N_SENSORS];
// static int fusion_ix = 0;

// LSM6DS3 lis(I2C_MODE, 0x6B);    // the address here is 0x6B not 0x6A as was in the example
// uint16_t errorsAndWarnings = 0; // part of the imu driver example code; may not be used; for debugging purposes


// eiSensors sensors[] =
// {
//     "accX", &data[0], &poll_IMU, &init_IMU, -1,
//     "accY", &data[1], &poll_IMU, &init_IMU, -1,
//     "accZ", &data[2], &poll_IMU, &init_IMU, -1,
//     "adc", &data[6], &poll_ADC, &init_ADC, -1,
// };


// void setup()
// {

//     /* Connect used sensors */
//     if(ei_connect_fusion_list(EI_CLASSIFIER_FUSION_AXES_STRING) == false) {
//         printf("ERR: Errors in sensor list detected\r\n");
//         return;
//     }

//     /* Init & start sensors */
//     for(int i = 0; i < fusion_ix; i++) {
//         if (sensors[fusion_sensors[i]].status == 0) {
//             sensors[fusion_sensors[i]].status = sensors[fusion_sensors[i]].init_sensor();
//             if (!sensors[fusion_sensors[i]].status) {
//               printf("%s axis sensor initialization failed.\r\n", sensors[fusion_sensors[i]].name);
//             }
//             else {
//               printf("%s axis sensor initialization successful.\r\n", sensors[fusion_sensors[i]].name);
//             }
//         }
//     }
// }


// float inference()
// {
//     // printf("\nStarting inferencing in 2 seconds...\r\n");  // for some reason printf isn't working
//     //                                                     // that's why this is the only print there

//     // vTaskDelay(2);

//     if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != fusion_ix) {
//         printf("ERR: Sensors don't match the sensors required in the model\r\n"
//         "Following sensors are required: %s\r\n", EI_CLASSIFIER_FUSION_AXES_STRING);
//         return -1;
//     }

//     printf("Sampling...\r\n");

//     // Allocate a buffer here for the values we'll read from the sensor
//     float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

//     for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
//         // Determine the next tick (and then sleep later)
//         int64_t next_tick = (int64_t)micros() + ((int64_t)EI_CLASSIFIER_INTERVAL_MS * 1000);

//         for(int i = 0; i < fusion_ix; i++) {
//             if (sensors[fusion_sensors[i]].status == 1) {
//                 sensors[fusion_sensors[i]].poll_sensor();
//                 sensors[fusion_sensors[i]].status = 2;
//             }
//             if (sensors[fusion_sensors[i]].status == 2) {
//                 buffer[ix + i] = *sensors[fusion_sensors[i]].value;
//                 // printf("%d %f\n", fusion_sensors[i], buffer[ix + i]);
//                 sensors[fusion_sensors[i]].status = 1;
//             }
//         }

//         int64_t wait_time = next_tick - (int64_t)micros();

//         if(wait_time > 0) {
//             delayMicroseconds(wait_time);
//         }
//     }

//     // Turn the raw buffer in a signal which we can the classify
//     signal_t signal;
//     int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
//     if (err != 0) {
//         printf("ERR:(%d)\r\n", err);
//         return -1;
//     }

//     // Run the classifier
//     ei_impulse_result_t result = { 0 };

//     err = run_classifier(&signal, &result, debug_nn);
//     if (err != EI_IMPULSE_OK) {
//         printf("ERR:(%d)\r\n", err);
//         return -1;
//     }

//     // // print the predictions
//     // printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\r\n",
//     //     result.timing.dsp, result.timing.classification, result.timing.anomaly);
//     // for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//     //     printf("%s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);
//     // }

//     // result.classification[0] -> fall; grab the result with .value, so result.classification[0].value

//     // printf(" %.5f \n",result.classification[0].value); 

// // #if EI_CLASSIFIER_HAS_ANOMALY == 1   // understnad what this means and use it perhaps to enhance model output
// //     printf("    anomaly score: %.3f\r\n", result.anomaly);
// // #endif
//     return result.classification[0].value;

// }

// #if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER)
// #error "Invalid model for current sensor"
// #endif


// /**
//  * @brief Go through sensor list to find matching axis name
//  *
//  * @param axis_name
//  * @return int8_t index in sensor list, -1 if axis name is not found
//  */
// static int8_t ei_find_axis(char *axis_name)
// {
//     int ix;
//     for(ix = 0; ix < N_SENSORS; ix++) {
//         if(strstr(axis_name, sensors[ix].name)) {
//             return ix;
//         }
//     }
//     return -1;
// }

// /**
//  * @brief Check if requested input list is valid sensor fusion, create sensor buffer
//  *
//  * @param[in]  input_list      Axes list to sample (ie. "accX + gyrY + magZ")
//  * @retval  false if invalid sensor_list
//  */
// static bool ei_connect_fusion_list(const char *input_list)
// {
//     char *buff;
//     bool is_fusion = false;

//     /* Copy const string in heap mem */
//     char *input_string = (char *)ei_malloc(strlen(input_list) + 1);
//     if (input_string == NULL) {
//         return false;
//     }
//     memset(input_string, 0, strlen(input_list) + 1);
//     strncpy(input_string, input_list, strlen(input_list));

//     /* Clear fusion sensor list */
//     memset(fusion_sensors, 0, N_SENSORS);
//     fusion_ix = 0;

//     buff = strtok(input_string, "+");

//     while (buff != NULL) { /* Run through buffer */
//         int8_t found_axis = 0;

//         is_fusion = false;
//         found_axis = ei_find_axis(buff);

//         if(found_axis >= 0) {
//             if(fusion_ix < N_SENSORS) {
//                 fusion_sensors[fusion_ix++] = found_axis;
//                 sensors[found_axis].status = 0;
//             }
//             is_fusion = true;
//         }

//         buff = strtok(NULL, "+ ");
//     }

//     ei_free(input_string);

//     return is_fusion;
// }

// /**
//  * @brief Return the sign of the number
//  *
//  * @param number
//  * @return int 1 if positive (or 0) -1 if negative
//  */
// float ei_get_sign(float number) {
//     return (number >= 0.0) ? 1.0 : -1.0;
// }

// bool init_IMU(void) {
//   static bool init_status = false;
//   if (!init_status) {
//     // Wire.begin(0,1); 
//     // lis.begin(Wire, LIS3DHTR_ADDRESS_UPDATED);
//     init_status = ~lis.begin();

//     if(init_status == false) {
//         printf("Failed to connect to Inertial sensor!\n");
//         return false;
//     }

//     ei_sleep(100);
//     // lis.setFullScaleRange(LIS3DHTR_RANGE_2G);
//     // lis.setOutputDataRate(LIS3DHTR_DATARATE_100HZ);
   
//     uint8_t dataToWrite = 0;  //Temporary variable

//     //Setup the accelerometer******************************
//     dataToWrite = 0; //Start Fresh!
//     // dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
//     dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
//     dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

//     //Now, write the patched together data
//     errorsAndWarnings += lis.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

//     //Set the ODR bit
//     errorsAndWarnings += lis.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
//     dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
//   }
//   return init_status;
// }

// bool init_ADC(void) {
//   static bool init_status = false;
//   if (!init_status) {
//     init_status = true;
//   }
//   return init_status;
// }

// uint8_t poll_IMU(void) {

//     data[0] = lis.readFloatAccelX();
//     data[1] = lis.readFloatAccelY();
//     data[2] = lis.readFloatAccelZ();

//     // lis.getAcceleration(&data[0], &data[1], &data[2]);

//     for (int i = 0; i < 3; i++) {
//         if (fabs(data[i]) > MAX_ACCEPTED_RANGE) {
//             data[i] = ei_get_sign(data[i]) * MAX_ACCEPTED_RANGE;
//         }
//     }

//     // data[0] *= CONVERT_G_TO_MS2;
//     // data[1] *= CONVERT_G_TO_MS2;
//     // data[2] *= CONVERT_G_TO_MS2;

//     return 0;
// }

// uint8_t poll_ADC(void) {
//     // change to another pin if necessary
//     data[6] = analogRead(A0);
//     return 0;
// }


// // extern "C" void app_main(){
// //     setup();
// //     while(1){
// //         loop();
// //         vTaskDelay(1);

      
// //     }
// // } // note for your learning diary -> including this here results in multiple app_main definrtion error,
// // indicating that the toit funcs below (within the OOP) define this somewhere.



// // A struct to hold the context of the external service.
// typedef struct {
//   toit_msg_context_t* msg_context;
// } echo_service_t;




// extern "C" {
// static toit_err_t on_created(void* user_data, toit_msg_context_t* context) {
//   echo_service_t* echo_service = static_cast<echo_service_t*>(user_data);
//   echo_service->msg_context = context;
//   setup();
//   return TOIT_OK;
// }





// static toit_err_t on_message(void* user_data, int sender, uint8_t* data, int length) {
//   echo_service_t* echo_service = static_cast<echo_service_t*>(user_data);
//   toit_msg_context_t* context = echo_service->msg_context;
  
//   if (toit_msg_notify(context, sender, data, length, true) != TOIT_OK) {
//     std::printf("unable to send\n");
//   }

//   // for(int i = 0; i<4 ;i++){
//   //   temp[i] = (uint8_t)'h';
//   // }

//   // if (toit_msg_notify(context, sender, temp, l, false) != TOIT_OK) {
//   //   std::printf("unable to send\n");
//   // }

//     // int l = 4;
//     // uint8_t *temp = (uint8_t*)toit_malloc(l * sizeof(uint8_t));
//     // float outcome;


//     // outcome = inference();
//     // memcpy(temp, &outcome, l);

//     // // printf("outcome is %f", outcome);
    
//     // // for(int i = 0; i < l; i++){
//     // //   printf("current item is: %d", temp[i]);
//     // //   temp[i] = (uint8_t) ((uint32_t)outcome & (0xFF << i*8));  
//     // // }
  
//     // if (toit_msg_notify(context, 3, temp, l, true) != TOIT_OK) {
//     //   std::printf("unable to send\n");
//     // }
    

  
//   // free(temp);
//   return TOIT_OK;
// }


// static toit_err_t on_rpc_request(void* user_data, int sender, int function, toit_msg_request_handle_t handle, uint8_t* data, int length) {
//   // Check if the message is "fetch result"
//   const char* expected_msg = "fetch result";
//   const int expected_length = strlen(expected_msg);
  
//   // Only proceed with inference if the message matches
//   if (length == expected_length && memcmp(data, expected_msg, expected_length) == 0) {
//     int l = 4;
//     uint8_t *temp = (uint8_t*)toit_malloc(l * sizeof(uint8_t));
//     float outcome;

//     outcome = inference();
//     memcpy(temp, &outcome, l);

//     if (toit_msg_request_reply(handle, temp, l, true) != TOIT_OK) {
//       std::printf("unable to send\n");
//     }
  
//   }
//   return TOIT_OK;
// }

// static toit_err_t on_removed(void* user_data) {
//   std::free(user_data);
//   return TOIT_OK;
// }



  
// // if (temp == NULL) {
// //     // Handle memory allocation failure
// // }

// // void* u_data;
// // echo_service_t* echo_service_p = static_cast<echo_service_t*>(u_data);
// // echo_service_p->msg_context = nullptr;



// namespace {
//   void __attribute__((constructor)) init() {
//   echo_service_t* echo_service = static_cast<echo_service_t*>(std::malloc(sizeof(echo_service_t)));
//   echo_service->msg_context = nullptr;
//   toit_msg_cbs_t cbs = TOIT_MSG_EMPTY_CBS();
//   cbs.on_created = on_created;
//   cbs.on_message = on_message;
//   cbs.on_rpc_request = on_rpc_request;
//   cbs.on_removed = on_removed;
//   toit_msg_add_handler("toitlang.org/demo-echo", echo_service, cbs);





// } }

// } // extern "C"

