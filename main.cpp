#include "DigitalOut.h"
#include "Encoder.hpp"
#include "MD.hpp"
#include "PID.hpp"
#include "UnbufferedSerial.h"
#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#define ENCODER_A_CHANNEL A0
#define ENCODER_B_CHANNEL A1
#define MD_PWM D2
#define MD_DIR D3

UnbufferedSerial serial(USBTX, USBRX, 9600);
DigitalOut acknowledge(LED1);
Encoder encoder(ENCODER_A_CHANNEL, ENCODER_B_CHANNEL, 58);
MD md(MD_PWM, MD_DIR);
PID::ctrl_param_t pid_param = {0.01, 0, 0, 0};
PID::ctrl_variable_t pid_vel = {0, 0, 0};
PID pid(&pid_vel, &pid_param);
Timer timer;

//  time
#define STATUS_DURATION 500
#define CONTROLE_DURATION 10
#define TARGET_UPDATER_DURATION 100
int current_time_ms;
int last_status_send_at = 0;
int last_controle_at = 0;
int last_target_update_at = 0;

//  speed target
#define TARGET_SPEED_COMPLETE_LIMIT 30
int _process_target_rpm_speed = 0;
int final_target_rpm_speed = 0;
int current_target_rpm_speed = 0;
float target_speed_gain = 0.085;

//  command
char rx_command_parts[5][10];
//  receiver
char rx_buffer[30] = {0};
int rx_buffer_index = 0;
char rx_char;
bool rx_is_started = false;

//  status
enum class Status : uint8_t { RUNNING, STOP, EMERGENCY_STOP, ERROR };
Status status = Status::STOP;

char status_message[30];
char status_code[10];

//  error count
#define MAX_ERROR_COUNT 20
int error_count = 0;

//  prototype definition
int split_by_space(char message[30], char parts[5][10]);
void rx_callback();
void parse();
inline void toggleAcknowledge();

int main() {
  timer.start();

  serial.attach(&rx_callback, SerialBase::RxIrq);

  while (true) {
    current_time_ms = timer.read_ms();

    if (status != Status::EMERGENCY_STOP && status != Status::ERROR) {
      //  PID
      if (current_time_ms - last_controle_at > CONTROLE_DURATION) {
        pid_vel.feedback =
            encoder.get_rps((current_time_ms - last_controle_at) / 1000.0);
        pid_vel.target = current_target_rpm_speed / 60.0;
        //  step
        pid.step((current_time_ms - last_controle_at) / 1000.0);
        //  drive
        md.drive(pid_vel.output);

        //  error check
        if (pid_vel.target * pid_vel.feedback < 0) {
          error_count++;
        } else {
          error_count = 0;
        }

        if (error_count > MAX_ERROR_COUNT) {
          status = Status::ERROR;
        }

        //  update
        last_controle_at = current_time_ms;
      }

      //  Target Speed
      if (current_time_ms - last_target_update_at > TARGET_UPDATER_DURATION) {
        _process_target_rpm_speed =
            status == Status::RUNNING ? final_target_rpm_speed : 0;
        current_target_rpm_speed =
            target_speed_gain *
            (_process_target_rpm_speed - current_target_rpm_speed);

        //  LIMIT
        if (abs(_process_target_rpm_speed - current_target_rpm_speed) <
            TARGET_SPEED_COMPLETE_LIMIT) {
          current_target_rpm_speed = _process_target_rpm_speed;
        }

        //  update
        last_target_update_at = current_time_ms;
      }

    } else {
      //  cut output
      md.drive(0.0);

      //  set zero
      pid_vel.target = 0;
    }

    if (current_time_ms - last_status_send_at > STATUS_DURATION) {
      //  fill 0
      memset(status_message, 0, 30);

      switch (status) {
      case Status::RUNNING:
        sprintf(status_code, "RUN");
        break;
      case Status::STOP:
        sprintf(status_code, "STOP");
        break;
      case Status::EMERGENCY_STOP:
        sprintf(status_code, "EMERG");
        break;
      case Status::ERROR:
        sprintf(status_code, "ERROR");
        break;
      }

      printf(";%s %c %d %d %d\n", status_code,
             final_target_rpm_speed >= 0.0 ? 'F' : 'R', final_target_rpm_speed,
             (int)(pid_vel.feedback * 60), (int)(pid_vel.output * 100));

      // printf("t:%.2f c:%.2f\n", pid_vel.target, pid_vel.feedback);
      //  update
      last_status_send_at = current_time_ms;
    }
  }
}

int split_by_space(char message[30], char parts[5][10]) {
  int parts_num = 0, part_index = 0;

  for (int i = 0; 30 > i; i++) {
    if (message[i] == '\0') {
      break;
    }

    if (message[i] == ' ') {
      parts[parts_num][part_index] = '\0';

      parts_num++;
      part_index = 0;
    } else {
      parts[parts_num][part_index] = message[i];

      part_index++;
    }
  }

  return parts_num + 1;
}

void rx_callback() {
  while (serial.readable()) {
    if (serial.read(&rx_char, 1) <= 0) {
      continue;
    }

    if (rx_is_started) {
      //  push
      rx_buffer[rx_buffer_index] = rx_char;

      //  increment
      rx_buffer_index++;

      if (rx_char == '\n') {
        rx_buffer[rx_buffer_index - 1] = '\0';
        //  restart
        rx_is_started = false;

        //  parse
        parse();
      }
    } else {
      if (rx_char == ';') {
        //  start
        rx_is_started = true;
        rx_buffer_index = 0;
      }
    }
  }
}

void parse() {
  //  memset
  for (int i = 0; i < 5; i++) {
    memset(rx_command_parts[i], 0, 10);
  }
  //  split
  int counts = split_by_space(rx_buffer, rx_command_parts);

  //  check if parts counts
  if (counts < 1) {
    return;
  }

  //  command
  if (strcmp(rx_command_parts[0], "TARGET") == 0) { //  TARGET
    //  argument check
    if (counts != 3) {
      return;
    }

    //  speed
    final_target_rpm_speed = atoi(rx_command_parts[2]);

    //  direction
    if (rx_command_parts[1][0] == 'R') {
      final_target_rpm_speed *= -1;
    }

    toggleAcknowledge();
  } else if (strcmp(rx_command_parts[0], "START") == 0) { //  START
    //  start spindle
    if (status != Status::RUNNING) {
      //  reset
      encoder.reset();
      pid.reset();
      error_count = 0;
    }

    status = Status::RUNNING;

    toggleAcknowledge();
  } else if (strcmp(rx_command_parts[0], "STOP") == 0) { //  STOP
    //  stop spindle
    status = Status::STOP;

    toggleAcknowledge();
  } else if (strcmp(rx_command_parts[0], "EMERG") == 0) { //  EMERG
    //  emergency stop
    status = Status::EMERGENCY_STOP;

    toggleAcknowledge();
  }
}

inline void toggleAcknowledge() { acknowledge = !acknowledge; }