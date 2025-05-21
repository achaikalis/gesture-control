#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../bsxlite_interface/bsxlite_interface.h"
#include "../bsxlite_interface/examples/file_operations.h"

int main() {
  
  char* input_file_name  = "example/shortLog_bsxlite.txt";
  char* output_file_name = "example/out_bsxlite.txt";

  /** Write Header into the Output File **/
  data_log_write_header(output_file_name);

  vector_3d_t accel_in, gyro_in;

  memset(&accel_in, 0x00, sizeof(accel_in));
  memset(&gyro_in, 0x00, sizeof(gyro_in));

  int32_t w_time_stamp = 0U;
  
  bsxlite_out_t bsxlite_fusion_out;
  memset(&bsxlite_fusion_out, 0x00, sizeof(bsxlite_fusion_out));

  bsxlite_return_t result;

  bsxlite_instance_t instance = 0x00;
  result = bsxlite_init(&instance);

  if (result != BSXLITE_OK) return EXIT_FAILURE;

  /** Read Input Log and Iterate over doSteps **/
  FILE* input_file = fopen(input_file_name, "r");

  if (input_file != NULL) {
    while (fscanf(input_file, "%d\t%f\t%f\t%f\t%d\t%f\t%f\t%f\t\n",
                  &w_time_stamp, &accel_in.x, &accel_in.y, &accel_in.z,
                  &w_time_stamp, &gyro_in.x, &gyro_in.y, &gyro_in.z) != EOF) {
      
      result = bsxlite_do_step(&instance, w_time_stamp, &accel_in, &gyro_in,
                               &(bsxlite_fusion_out));

      if (result != BSXLITE_OK) {
        
        switch (result) {
          
          case (BSXLITE_E_DOSTEPS_TSINTRADIFFOUTOFRANGE): {
            printf(
                "Error: Subsequent time stamps in input data were found to be "
                "out of range from the expected sample rate!!!\n");
            break;
          }
          
          case (BSXLITE_E_FATAL): {
            printf("Fatal Error: Process terminating!!!\n");
            return EXIT_FAILURE;
          }
          
          case (BSXLITE_I_DOSTEPS_NOOUTPUTSRETURNABLE): {
            printf(
                "Info: Sufficient memory not allocated for output,  all "
                "outputs cannot be returned because no memory provided!!!\n");
            break;
          }
          
          default: {
            printf("Info: Unknown return \n");
            break;
          }
        }
      }

      /** Log the Output Data into File */
      data_log_write_output(output_file_name, &accel_in, &gyro_in, w_time_stamp,
                            &(bsxlite_fusion_out.rotation_vector),
                            &(bsxlite_fusion_out.orientation),
                            &(bsxlite_fusion_out.accel_calibration_status),
                            &(bsxlite_fusion_out.gyro_calibration_status));
    }
    
    fclose(input_file);

  }

  result = bsxlite_set_to_default(&instance);
  if (result != BSXLITE_OK) return EXIT_FAILURE;

  printf("Success! \n");

  fflush(stdout);  // lint !e534
  
  return EXIT_SUCCESS;
}
/*! @}*/