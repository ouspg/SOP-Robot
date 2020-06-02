#include <ros.h>
#include <xl430move/XL430Move.h>
#include <xl430move/XL430Moves.h>
#include <xl430move/XL430Positions.h>
#include <std_msgs/String.h>
#include <SoftwareSerial.h>

#include "XL430.h"

ros::NodeHandle nh;
SoftwareSerial soft_serial(8, 9);   // Rx, Tx
long publisher_timer;

// move single servo
void servo_action(const xl430move::XL430Move& msg) {
    int s_pos = convert_to_servo(msg.id, msg.pos);

    xl430_change_torque_enable(msg.id, 1);
    xl430_change_servo_speed(msg.id, msg.spd);
    xl430_change_goal_position(msg.id, s_pos);
}

// move all servos
void servos_action(const xl430move::XL430Moves& msg) {
    int s_pos_right = convert_to_servo(SERVO_RIGHT, msg.pos_right);
    int s_pos_front = convert_to_servo(SERVO_FRONT, msg.pos_front);
    int s_pos_left = convert_to_servo(SERVO_LEFT, msg.pos_left);
    int s_pos_turn = convert_to_servo(SERVO_TURN, msg.pos_turn);

    xl430_change_torque_enable(SERVO_RIGHT, 1);
    xl430_change_torque_enable(SERVO_FRONT, 1);
    xl430_change_torque_enable(SERVO_LEFT, 1);
    xl430_change_torque_enable(SERVO_TURN, 1);

    xl430_change_servo_speed(SERVO_RIGHT, msg.spd_right);
    xl430_change_goal_position(SERVO_RIGHT, s_pos_right);

    xl430_change_servo_speed(SERVO_FRONT, msg.spd_front);
    xl430_change_goal_position(SERVO_FRONT, s_pos_front);

    xl430_change_servo_speed(SERVO_LEFT, msg.spd_left);
    xl430_change_goal_position(SERVO_LEFT, s_pos_left);

    xl430_change_servo_speed(SERVO_TURN, msg.spd_turn);
    xl430_change_goal_position(SERVO_TURN, s_pos_turn);
}

// nod "yes"
void nod_action() {
    xl430_move(60, 0, 0, 0, 0);
    xl430_wait_for_moves();
    xl430_move(60, 0, 500, 0, 0);
    xl430_wait_for_moves();
    delay(500);
    xl430_move(60, 0, 0, 0, 0);
    xl430_wait_for_moves();
}

// tilt head to listen
void listen_action() {
    xl430_move(40, -300, 0, 700, 0);
    xl430_wait_for_moves();
}

// shake head "no"
void shake_action() {
    xl430_move(60, 0, 0, 0, 250);
    xl430_wait_for_moves();
    xl430_move(60, 0, 0, 0, -250);
    xl430_wait_for_moves();
    xl430_move(60, 0, 0, 0, 250);
    xl430_wait_for_moves();
    xl430_move(60, 0, 0, 0, 0);
    xl430_wait_for_moves();
}

// handle incoming gesture calls
void gesture_action(const std_msgs::String& msg) {
    if (msg.data == "nod") {
        nod_action();
    }
    else if (msg.data == "listen") {
        listen_action();
    }
    else if (msg.data == "shake") {
        shake_action();
    }
}

xl430move::XL430Positions servo_pos;

ros::Subscriber<xl430move::XL430Move> move_servo("move_servo", &servo_action);
ros::Subscriber<xl430move::XL430Moves> move_servos("move_servos", &servos_action);
ros::Subscriber<std_msgs::String> gesture("gesture", &gesture_action);

ros::Publisher read_servos("read_servos", &servo_pos);

void setup() {
    soft_serial.begin(BAUD_RATE);
    
    // init rosserial
    nh.initNode();

    // init subscribers
    nh.subscribe(move_servo);
    nh.subscribe(move_servos);
    nh.subscribe(gesture);

    // init publishers
    nh.advertise(read_servos);
    
    xl430_change_torque_enable(SERVO_RIGHT, 0);
    xl430_change_torque_enable(SERVO_FRONT, 0);
    xl430_change_torque_enable(SERVO_LEFT, 0);
    xl430_change_torque_enable(SERVO_TURN, 0);
    /*
     * EEPROM changes here, if needed
     */
    xl430_change_torque_enable(SERVO_RIGHT, 1);
    xl430_change_torque_enable(SERVO_FRONT, 1);
    xl430_change_torque_enable(SERVO_LEFT, 1);
    xl430_change_torque_enable(SERVO_TURN, 1);
}

void loop() {
    
    ReadData r;
    if (millis() > publisher_timer) {
        r = xl430_read(SERVO_RIGHT, XL_PRESENT_POSITION, 4);
        servo_pos.pos_right = (r.params[1] << 8 | r.params[0]);
        r = xl430_read(SERVO_FRONT, XL_PRESENT_POSITION, 4);
        servo_pos.pos_front = (r.params[1] << 8 | r.params[0]);
        r = xl430_read(SERVO_LEFT, XL_PRESENT_POSITION, 4);
        servo_pos.pos_left = (r.params[1] << 8 | r.params[0]);
        r = xl430_read(SERVO_TURN, XL_PRESENT_POSITION, 4);
        servo_pos.pos_turn = (r.params[1] << 8 | r.params[0]);
        read_servos.publish(&servo_pos);

        publisher_timer = millis() + 1000;
    }
    
    nh.spinOnce();
    delay(10);
}
