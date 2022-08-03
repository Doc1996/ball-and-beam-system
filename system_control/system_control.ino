#include <EEPROM.h>
#include <BasicLinearAlgebra.h>
#include <NewPing.h>

#define ENCODER_ENCA_PIN 2
#define ENCODER_ENCB_PIN 3

#define MOTOR_ENA_PIN 5
#define MOTOR_IN1_PIN 6
#define MOTOR_IN2_PIN 7

#define ULTRASOUND_ECHO_PIN 8
#define ULTRASOUND_TRIG_PIN 9

#define SERIAL_BAUD_RATE 115200
#define SERIAL_SCALING_FACTOR 1000.0
#define WORKING_STATE_ADDRESS 0

#define LED_BLINKING_TIME_IN_SEC 0.15
#define SAMPLE_TIME_IN_SEC 0.012
#define REF_COMPLETE_CYCLE_TIME_IN_SEC 5.0

#define STARTING_DELAY_TIME_IN_SEC 1.0
#define MIN_WARMUP_TIME_IN_SEC 0.5
#define MAX_WORKING_TIME_IN_SEC (MIN_WARMUP_TIME_IN_SEC + 600.0)

#define MICROSEC_TO_MILLISEC_FACTOR (1.0 / 1000.0)
#define MILLISEC_TO_SEC_FACTOR (1.0 / 1000.0)
#define SEC_TO_MIN_FACTOR (1.0 / 60.0)

#define MILLIMETER_TO_METER_FACTOR (1.0 / 1000.0)
#define ROT_PER_MIN_TO_ROT_PER_SEC_FACTOR (1.0 / 60.0)
#define ROT_TO_RAD_FACTOR (2 * PI_NUMBER)
#define DEG_TO_RAD_FACTOR (2 * PI_NUMBER / 180.0)

#define PI_NUMBER 3.14159
#define EULER_NUMBER 2.71828
#define SOUND_VEL_IN_METERS_PER_SEC 346.0

#define ENCODER_COUNTS_RATIO 17.0
#define MOTOR_GEAR_RATIO 18.75
#define BEAM_GEAR_RATIO 3.0

#define LIMIT_BEAM_ROTATION_DEG (20.0 * 1000.0)
#define LIMIT_MOTOR_ROTATION_RAD (LIMIT_BEAM_ROTATION_DEG * BEAM_GEAR_RATIO * DEG_TO_RAD_FACTOR)
#define MAX_ROTATION_VEL (330.0 * SEC_TO_MIN_FACTOR * ROT_TO_RAD_FACTOR)

#define THRESHOLD_PWM_SIGNAL 5.0
#define MIN_PWM_SIGNAL 120.0
#define MAX_PWM_SIGNAL 255.0

#define HALF_BEAM_LENGTH_IN_METERS 0.220
#define INITIAL_DISTANCE_IN_METERS 0.008
#define BALL_RADIUS_IN_METERS 0.020

#define CENTER_DISTANCE_IN_METERS (HALF_BEAM_LENGTH_IN_METERS + INITIAL_DISTANCE_IN_METERS - BALL_RADIUS_IN_METERS)
#define MAX_DISTANCE_IN_METERS (CENTER_DISTANCE_IN_METERS + HALF_BEAM_LENGTH_IN_METERS)

#define U_ORDER 1
#define N_ORDER 4
#define Y_ORDER 2




enum controller_enum {ControllerStopped, ControllerRotVelocityNone, ControllerRotationPD, ControllerOffsetCascadePD, ControllerOffsetStateSpace};
enum estimator_enum {EstimatorUndefined, EstimatorFiltering, EstimatorKalman};
enum tracking_enum {TrackingUndefined, TrackingKalmanNone, TrackingKalmanOffset};

controller_enum controller_type;
estimator_enum estimator_type;
tracking_enum tracking_type;

NewPing ultrasound_sensor(ULTRASOUND_TRIG_PIN, ULTRASOUND_ECHO_PIN, (int) (MAX_DISTANCE_IN_METERS / MILLIMETER_TO_METER_FACTOR));

int prev_working_state_num;
int working_state_num;

float time_in_millisec;
float time_in_sec;
float prev_time_in_sec = 0.0;
float delta_time_in_sec;

float ref_cycling_time_in_sec = 0.0;
int ref_cycling_state_num = 0;

int PRBS_counter = 0;

// variables used in interrupts must be declared volatile
volatile float readed_increments = 0.0;
volatile float readed_distance = 0.0;

float raw_rotation;
float prev_raw_rotation = 0.0;
float rotation;
float prev_rotation = 0.0;

float raw_rotation_vel;
float rotation_vel;
float prev_rotation_vel = 0.0;

float raw_offset;
float prev_raw_offset = 0.0;
float offset;
float prev_offset = 0.0;

float raw_offset_vel;
float offset_vel;
float prev_offset_vel = 0.0;

float raw_error;
float prev_raw_error = 0.0;
float error;
float prev_error = 0.0;

float raw_error_vel;
float error_vel;
float prev_error_vel = 0.0;

float raw_inner_error;
float inner_error;
float prev_inner_error = 0.0;

float raw_inner_error_vel;
float inner_error_vel;
float prev_inner_error_vel = 0.0;

float ref_rotation;
float ref_rotation_vel;
float ref_offset;

float pwm_model_value;
float pwm_real_value;



BLA::Matrix<U_ORDER> u_k = 0.0;

BLA::Matrix<N_ORDER> x_hat_k = {0.0, 0.0, 0.0, 0.0};
BLA::Matrix<N_ORDER> x_hat_m_k;

BLA::Matrix<N_ORDER> x_dev_k = {0.0, 0.0, 0.0, 0.0};

BLA::Matrix<Y_ORDER> y_k;

BLA::Matrix<N_ORDER, N_ORDER> Phi_k;
BLA::Matrix<Y_ORDER, N_ORDER> H_k;

BLA::Matrix<N_ORDER, 1> f_hat_k;
BLA::Matrix<N_ORDER, U_ORDER> g_hat_k;

BLA::Matrix<U_ORDER, N_ORDER> K_control_k;

BLA::Matrix<N_ORDER, Y_ORDER> K_kalman_k;

BLA::Matrix<N_ORDER, N_ORDER> I_k = {1.0, 0.0, 0.0, 0.0,
                                     0.0, 1.0, 0.0, 0.0,
                                     0.0, 0.0, 1.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0
                                    };

BLA::Matrix<N_ORDER, N_ORDER> P_k = {x_dev_k(0),        0.0,        0.0,        0.0,
                                            0.0, x_dev_k(1),        0.0,        0.0,
                                            0.0,        0.0, x_dev_k(2),        0.0,
                                            0.0,        0.0,        0.0, x_dev_k(3)
                                    };
BLA::Matrix<N_ORDER, N_ORDER> P_m_k;

BLA::Matrix<N_ORDER, N_ORDER> Q_k = {0.001, 0.0,   0.0, 0.0,
                                       0.0, 0.2,   0.0, 0.0,
                                       0.0, 0.0, 0.001, 0.0,
                                       0.0, 0.0,   0.0, 8.0
                                   };

BLA::Matrix<Y_ORDER, Y_ORDER> R_k = {0.01,  0.0,
                                      0.0, 0.01
                                    };



float F_PREV_ERROR = 0.85;
float F_RAW_ERROR = (1.0 - F_PREV_ERROR);
float F_PREV_ERROR_VEL = 0.9;
float F_RAW_ERROR_VEL = (1.0 - F_PREV_ERROR_VEL);

float F_PREV_INNER_ERROR = 0.35;
float F_RAW_INNER_ERROR = (1.0 - F_PREV_INNER_ERROR);
float F_PREV_INNER_ERR_VEL = 0.45;
float F_RAW_INNER_ERR_VEL = (1.0 - F_PREV_INNER_ERR_VEL);

float K_CRIT_ERROR = -4.0;
float T_CRIT_ERROR = 2.5;
float K_PROP_ERROR = (0.8 * K_CRIT_ERROR);
float K_DER_ERROR = (0.15 * K_CRIT_ERROR * T_CRIT_ERROR);

float K_CRIT_INNER_ERROR = 230.0;
float T_CRIT_INNER_ERROR = 0.3;
float K_PROP_INNER_ERROR = (0.8 * K_CRIT_INNER_ERROR);
float K_DER_INNER_ERROR = (0.15 * K_CRIT_INNER_ERROR * T_CRIT_INNER_ERROR);

float F_PREV_OFFSET = 0.85;
float F_RAW_OFFSET = (1.0 - F_PREV_OFFSET);
float F_PREV_OFFSET_VEL = 0.9;
float F_RAW_OFFSET_VEL = (1.0 - F_PREV_OFFSET_VEL);

float F_PREV_ROTATION = 0.35;
float F_RAW_ROTATION = (1.0 - F_PREV_ROTATION);
float F_PREV_ROT_VEL = 0.45;
float F_RAW_ROT_VEL = (1.0 - F_PREV_ROT_VEL);

bool plot_time_data_bool = true;
bool inverted_belt_bool = true;




void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(ENCODER_ENCA_PIN, INPUT);
  pinMode(ENCODER_ENCB_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTOR_ENA_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  pinMode(ULTRASOUND_ECHO_PIN, INPUT);
  pinMode(ULTRASOUND_TRIG_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_ENCB_PIN), read_encoder_inc_change, RISING);

  stop_motor();


  prev_working_state_num = EEPROM.read(WORKING_STATE_ADDRESS); 

  if (prev_working_state_num < 4) {
    working_state_num = prev_working_state_num + 1;
  }
  else {
    working_state_num = 0;
  }

  EEPROM.write(WORKING_STATE_ADDRESS, working_state_num);

  if (working_state_num == 0) {
    controller_type = ControllerStopped;
    estimator_type = EstimatorUndefined;
    tracking_type = TrackingUndefined;
  }
  else if (working_state_num == 1) {
    controller_type = ControllerOffsetCascadePD;
    estimator_type = EstimatorUndefined;
    tracking_type = TrackingUndefined;
  }
  else if (working_state_num == 2) {
    controller_type = ControllerOffsetStateSpace;
    estimator_type = EstimatorFiltering;
    tracking_type = TrackingUndefined;
  }
  else if (working_state_num == 3) {
    controller_type = ControllerOffsetStateSpace;
    estimator_type = EstimatorKalman;
    tracking_type = TrackingKalmanNone;
  }
  else if (working_state_num == 4) {
    controller_type = ControllerOffsetStateSpace;
    estimator_type = EstimatorKalman;
    tracking_type = TrackingKalmanOffset;
  }


  blink_LED();
  blink_LED();

  delay((int) (2 * LED_BLINKING_TIME_IN_SEC / MILLISEC_TO_SEC_FACTOR));

  for (int counter = 0; counter < working_state_num; counter++) {
    blink_LED();
  }
  
  delay((int) (STARTING_DELAY_TIME_IN_SEC / MILLISEC_TO_SEC_FACTOR));
}




void loop() {
  time_in_millisec = (float) micros() * MICROSEC_TO_MILLISEC_FACTOR;
  time_in_sec = time_in_millisec * MILLISEC_TO_SEC_FACTOR;


  if (time_in_sec - prev_time_in_sec >= SAMPLE_TIME_IN_SEC) {

    delta_time_in_sec = time_in_sec - prev_time_in_sec;
    prev_time_in_sec = time_in_sec;


    if (time_in_sec <= MAX_WORKING_TIME_IN_SEC) {

      noInterrupts();  // disable interrupts before accessing raw increment change
      raw_rotation = readed_increments * ROT_TO_RAD_FACTOR / (ENCODER_COUNTS_RATIO * MOTOR_GEAR_RATIO);
      interrupts();  // enable interrupts after accessing raw increment change

      if (inverted_belt_bool == true) {
        raw_rotation = - raw_rotation;
      }


      ultrasound_sensor.ping_timer(read_ultrasound_distance);
      raw_offset = readed_distance - CENTER_DISTANCE_IN_METERS;

      if (raw_offset >= HALF_BEAM_LENGTH_IN_METERS) {
        raw_offset = prev_raw_offset;
      }

      prev_raw_offset = raw_offset;


      if (time_in_sec >= MIN_WARMUP_TIME_IN_SEC) {

        if ((raw_rotation >= - LIMIT_MOTOR_ROTATION_RAD) && (raw_rotation <= LIMIT_MOTOR_ROTATION_RAD)) {

          if (controller_type == ControllerStopped) {
      	    pwm_model_value = 0.0;
      	    pwm_real_value = pwm_model_value;
	        }


          else if (controller_type == ControllerRotVelocityNone) {
            ref_rotation_vel = 0.0;

            raw_rotation_vel = (raw_rotation - prev_raw_rotation) / delta_time_in_sec;
            prev_raw_rotation = raw_rotation;

            pwm_model_value = ref_rotation_vel / MAX_ROTATION_VEL * MAX_PWM_SIGNAL;
            pwm_real_value = pwm_model_value;
          }


          else if (controller_type == ControllerRotationPD) {
            ref_rotation = 0.0;
            raw_error = ref_rotation - raw_rotation;

            raw_error_vel = (raw_error - prev_raw_error) / delta_time_in_sec;
            prev_raw_error = raw_error;

            pwm_model_value = K_PROP_INNER_ERROR * raw_error + K_DER_INNER_ERROR * raw_error_vel;
            pwm_real_value = get_dead_zone_compensated_value(pwm_model_value);
          }


          else if (controller_type == ControllerOffsetCascadePD) {
            ref_offset = 0.0;
            raw_error = ref_offset - raw_offset;


            error = F_PREV_ERROR * prev_error + F_RAW_ERROR * raw_error;
            raw_error_vel = (error - prev_error) / delta_time_in_sec;
            prev_error = error;

            error_vel = F_PREV_ERROR_VEL * prev_error_vel + F_RAW_ERROR_VEL * raw_error_vel;
            prev_error_vel = error_vel;

            ref_rotation = K_PROP_ERROR * error + K_DER_ERROR * error_vel;
            raw_inner_error = ref_rotation - raw_rotation;


            inner_error = F_PREV_INNER_ERROR * prev_inner_error + F_RAW_INNER_ERROR * raw_inner_error;
            raw_inner_error_vel = (inner_error - prev_inner_error) / delta_time_in_sec;
            prev_inner_error = inner_error;

            inner_error_vel = F_PREV_INNER_ERR_VEL * prev_inner_error_vel + F_RAW_INNER_ERR_VEL * raw_inner_error_vel;
            prev_inner_error_vel = inner_error_vel;

            pwm_model_value = K_PROP_INNER_ERROR * inner_error + K_DER_INNER_ERROR * inner_error_vel;
            pwm_real_value = get_dead_zone_compensated_value(pwm_model_value);
          }


          else if (controller_type == ControllerOffsetStateSpace) {
            if (tracking_type == TrackingKalmanNone) {
              ref_offset = 0.0;

              y_k = {raw_offset, raw_rotation};
            }

            else if (tracking_type == TrackingKalmanOffset) {
              if (ref_cycling_time_in_sec < REF_COMPLETE_CYCLE_TIME_IN_SEC) {
                ref_cycling_time_in_sec = ref_cycling_time_in_sec + delta_time_in_sec;
              }
              else {
                ref_cycling_time_in_sec = 0.0;
                
                if (ref_cycling_state_num < 3) {
                  ref_cycling_state_num = ref_cycling_state_num + 1;
                }
                else {
                  ref_cycling_state_num = 0;
                }
              }

              if (ref_cycling_state_num == 0) {
                ref_offset = 0.0;
              }
              else if (ref_cycling_state_num == 1) {
                ref_offset = 0.1;
              }
              else if (ref_cycling_state_num == 2) {
                ref_offset = 0.0;
              }
              else if (ref_cycling_state_num == 3) {
                ref_offset = -0.1;
              }

              y_k = {raw_offset - ref_offset, raw_rotation};
            }


            if (estimator_type == EstimatorFiltering) {
              offset = F_PREV_ERROR * prev_offset + F_RAW_ERROR * raw_offset;
              raw_offset_vel = (offset - prev_offset) / delta_time_in_sec;
              prev_offset = offset;

              offset_vel = F_PREV_ERROR_VEL * prev_offset_vel + F_RAW_ERROR_VEL * raw_offset_vel;
              prev_offset_vel = offset_vel;


              rotation = F_PREV_INNER_ERROR * prev_rotation + F_RAW_INNER_ERROR * raw_rotation;
              raw_rotation_vel = (rotation - prev_rotation) / delta_time_in_sec;
              prev_rotation = rotation;

              rotation_vel = F_PREV_INNER_ERR_VEL * prev_rotation_vel + F_RAW_INNER_ERR_VEL * raw_rotation_vel;
              prev_rotation_vel = rotation_vel;


              x_hat_k = {offset, offset_vel, rotation, rotation_vel};

              K_control_k = {-700.0, -450.0, 250.0, 10.0};
            }


            else if (estimator_type == EstimatorKalman) {
              Phi_k = {1.0, 0.012, 0.0,   0.0,
                       0.0,   1.0, 0.0,   0.0,
                       0.0,   0.0, 1.0, 0.012,
                       0.0,   0.0, 0.0,   1.0
                      };

              H_k = {1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0
                    };

              f_hat_k = {x_hat_k(1),
                                0.0,
                         x_hat_k(3),
                                0.0
                        };

              g_hat_k = {   0.0,
                            0.0,
                            0.0,
                         1.0933
                        };

              x_hat_m_k = x_hat_k + (f_hat_k + g_hat_k * u_k) * delta_time_in_sec;

              P_m_k = Phi_k * P_k * ~Phi_k + Q_k;
              K_kalman_k = P_m_k * ~H_k * BLA::Inverse((H_k * P_m_k * ~H_k + R_k));
              P_k = (I_k - K_kalman_k * H_k) * P_m_k * ~(I_k - K_kalman_k * H_k) + K_kalman_k * R_k * ~K_kalman_k;

              x_hat_k = x_hat_m_k + K_kalman_k * (y_k - H_k * x_hat_m_k);

              K_control_k = {-700.0, -400.0, 250.0, 15.0};
            }


            u_k = - K_control_k * x_hat_k;

            pwm_model_value = u_k(0);
            pwm_real_value = get_dead_zone_compensated_value(pwm_model_value);
          }


          if (pwm_real_value <= - MAX_PWM_SIGNAL) {
            pwm_real_value = - MAX_PWM_SIGNAL;
          }
          else if (pwm_real_value >= MAX_PWM_SIGNAL) {
            pwm_real_value = MAX_PWM_SIGNAL;
          }

          if (inverted_belt_bool == true) {
            pwm_real_value = - pwm_real_value;
          }

          drive_motor(pwm_real_value);
        }

        else {
          stop_motor();
        }


        if (plot_time_data_bool == true) {
          print_scaled_to_serial(time_in_millisec / SERIAL_SCALING_FACTOR);
        }


        if (controller_type == ControllerRotVelocityNone) {
          print_scaled_to_serial(ref_rotation_vel);

          print_scaled_to_serial(raw_rotation_vel);
        }

        else if (controller_type == ControllerRotationPD) {
          print_scaled_to_serial(ref_rotation);
          
          print_scaled_to_serial(raw_error);
          print_scaled_to_serial(raw_error_vel);
        }

        else if (controller_type == ControllerOffsetCascadePD) {
          print_scaled_to_serial(ref_offset);

          print_scaled_to_serial(raw_error);
          print_scaled_to_serial(error);

          print_scaled_to_serial(raw_error_vel);
          print_scaled_to_serial(error_vel);

          print_scaled_to_serial(raw_inner_error);
          print_scaled_to_serial(inner_error);

          print_scaled_to_serial(raw_inner_error_vel);
          print_scaled_to_serial(inner_error_vel);
        }

        else if (controller_type == ControllerOffsetStateSpace) {
          print_scaled_to_serial(ref_offset);

          print_scaled_to_serial(raw_offset);
          print_scaled_to_serial(x_hat_k(0) + ref_offset);

          print_scaled_to_serial(x_hat_k(1));

          print_scaled_to_serial(raw_rotation);
          print_scaled_to_serial(x_hat_k(2));

          print_scaled_to_serial(x_hat_k(3));
        }

        print_scaled_to_serial(pwm_model_value / SERIAL_SCALING_FACTOR);
        print_scaled_to_serial(pwm_real_value / SERIAL_SCALING_FACTOR);

        Serial.println();
      }
    }

    else {
      stop_motor();
    }
  }
}




float get_sine_value(float sine_base, float sine_amplitude, float sine_period_in_sec, float time) {
  float sine_value = sine_base + sine_amplitude * sin(time / sine_period_in_sec * 2 * PI_NUMBER);

  return sine_value;
}



void get_PRBS_64_value(float PRBS_amplitude) {
  int starting_PRBS_counter = 2;
  int shifting_PRBS_counter = 63;

  int first_PRBS_bit_for_xor = 6;
  int second_PRBS_bit_for_xor = 5;

  if (PRBS_counter == 0) {
    PRBS_counter = starting_PRBS_counter;
  }

  int most_left_PRBS_bit = ((PRBS_counter >> first_PRBS_bit_for_xor - 1) ^ (PRBS_counter >> second_PRBS_bit_for_xor - 1)) & 1;
  PRBS_counter = ((PRBS_counter << 1) | most_left_PRBS_bit) & shifting_PRBS_counter;

  float PRBS_value;
  if (PRBS_counter % 2 == 0) {
    PRBS_value = - PRBS_amplitude;
  }
  else {
    PRBS_value = PRBS_amplitude;
  }

  return PRBS_value;
}



float get_dead_zone_compensated_value(float input_pwm_signal) {
  float output_pwm_signal;

  if (input_pwm_signal < - THRESHOLD_PWM_SIGNAL) {
    output_pwm_signal = (input_pwm_signal + THRESHOLD_PWM_SIGNAL) * (MAX_PWM_SIGNAL - MIN_PWM_SIGNAL) / (MAX_PWM_SIGNAL - THRESHOLD_PWM_SIGNAL) - MIN_PWM_SIGNAL;
  }
  else if (input_pwm_signal > THRESHOLD_PWM_SIGNAL) {
    output_pwm_signal = (input_pwm_signal - THRESHOLD_PWM_SIGNAL) * (MAX_PWM_SIGNAL - MIN_PWM_SIGNAL) / (MAX_PWM_SIGNAL - THRESHOLD_PWM_SIGNAL) + MIN_PWM_SIGNAL;
  }
  else {
    output_pwm_signal = 0;
  }

  return output_pwm_signal;
}



void drive_motor(float pwm_real_value) {
  if (pwm_real_value >= 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }
  else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  }

  analogWrite(MOTOR_ENA_PIN, (int) fabs(pwm_real_value));
}



void stop_motor() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}



// read encoder A signal when encoder B signal rises
void read_encoder_inc_change() {
  int readed_increment_change = digitalRead(ENCODER_ENCA_PIN);

  // increment position if encoder A signal is high and vice versa
  if (readed_increment_change == 1) {
    readed_increments = readed_increments + 1.0;
  }
  else {
    readed_increments = readed_increments - 1.0;
  }
}



 void read_ultrasound_distance() {
   if (ultrasound_sensor.check_timer()) {
     readed_distance = ultrasound_sensor.ping_result *  MICROSEC_TO_MILLISEC_FACTOR * MILLISEC_TO_SEC_FACTOR * SOUND_VEL_IN_METERS_PER_SEC / 2 ;
   }
 }



void blink_LED() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay((int) (LED_BLINKING_TIME_IN_SEC / MILLISEC_TO_SEC_FACTOR));
  digitalWrite(LED_BUILTIN, LOW);
  delay((int) (LED_BLINKING_TIME_IN_SEC / MILLISEC_TO_SEC_FACTOR));
}



void print_scaled_to_serial(float printing_value) {
  Serial.print(printing_value * SERIAL_SCALING_FACTOR);
  Serial.print(" ");
}
