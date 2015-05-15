#pragma once
#ifdef __cplusplus
extern "C" {
#endif  

typedef struct {
  double x;
  double y;
  double theta;
} carmen_point_t, *carmen_point_p;

int carmen_base_direct_sonar_on(void);

int carmen_base_direct_sonar_off(void);

int carmen_base_direct_reset(void);

int carmen_base_direct_initialize_robot(const char *model, const char *dev);

int carmen_base_direct_shutdown_robot(void);

int carmen_base_direct_set_acceleration(double acceleration);

int carmen_base_direct_set_deceleration(double deceleration);

int carmen_base_direct_set_velocity(double tv, double rv);

int carmen_base_direct_update_status(double *update_timestamp);

int carmen_base_direct_get_state(double *displacement, double *rotation, double *tv, double *rv);

int carmen_base_direct_get_integrated_state(double *x, double *y, double *theta, double *tv, double *rv);

int carmen_base_direct_send_binary_data(unsigned char *data, int size);

int carmen_base_direct_get_binary_data(unsigned char **data, int *size);

int carmen_base_direct_get_bumpers(unsigned char *state, int num_bumpers);

void carmen_base_direct_arm_get(double servos[], int num_servos, double *currents, int *gripper);

void carmen_base_direct_arm_set(double servos[], int num_servos);

int carmen_base_direct_get_sonars(double *ranges, carmen_point_t *positions, int num_sonars);

#ifdef __cplusplus
}
#endif

