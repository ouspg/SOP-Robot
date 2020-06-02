#include "XL430.h"

uint16_t update_crc(uint16_t crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    uint16_t i, j;
    
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ pgm_read_word_near(crc_table + i);
    }

    return crc_accum;
}

// send PING command
void xl430_cmd_PING(unsigned char id) {
    unsigned char instruction = 0x01;
    unsigned char data[10] = {0};
    
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFD;
    data[3] = 0x00;  
    data[4] = id;
    data[5] = 3 & 0xFF;
    data[6] = (3 >> 8) & 0xFF;
    data[7] = instruction;

    uint16_t crc = update_crc(0, data, 8);
    data[8] = crc & 0x00FF;
    data[9] = (crc >> 8) & 0x00FF;

    soft_serial.write(data, 10);
}

void xl430_cmd_REBOOT(unsigned char id) {
    unsigned char instruction = 0x08;
    unsigned char data[10] = {0};

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFD;
    data[3] = 0x00;
    data[4] = id;
    data[5] = 3 & 0xFF;
    data[6] = (3 >> 8) & 0xFF;
    data[7] = instruction;

    uint16_t crc = update_crc(0, data, 8);
    data[8] = crc & 0x00FF;
    data[9] = (crc >> 8) & 0x00FF;

    soft_serial.write(data, 10);
}

// send WRITE command
void xl430_cmd_WRITE(unsigned char id, int parameter_len, ...) {
    unsigned char instruction = 0x03;
    unsigned char data[parameter_len + 10] = {0};
    
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFD;
    data[3] = 0x00;    
    data[4] = id;
    data[5] = (parameter_len + 3) & 0xFF;
    data[6] = ((parameter_len + 3) >> 8) & 0xFF;
    data[7] = instruction;

    va_list args;
    va_start(args, parameter_len);

    for (int i = 0; i < parameter_len; i++) {
        unsigned char arg = va_arg(args, int);
        data[8 + i] = arg;
    }

    uint16_t crc = update_crc(0, data, parameter_len + 8);
    data[8 + parameter_len] = crc & 0x00FF;
    data[9 + parameter_len] = (crc >> 8) & 0x00FF;
    va_end(args);
    soft_serial.write(data, parameter_len + 10);
}

// send READ command
void xl430_cmd_READ(unsigned char id, ...) {
    unsigned char instruction = 0x02;
    unsigned char len = 14;
    unsigned char data[len] = {0};

    // headers
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFD;
    data[3] = 0x00;

    // target
    data[4] = id;
    // data length
    data[5] = (len - 7) & 0xFF;
    data[6] = ((len - 7) >> 8) & 0xFF;
    // instruction
    data[7] = instruction;

    va_list args;
    va_start(args, id);

    // data
    for (int i = 0; i < 4; i++) {
        unsigned char arg = va_arg(args, int);
        data[8 + i] = arg;
    }

    uint16_t crc = update_crc(0, data, 12);
    data[12] = crc & 0x00FF;
    data[13] = (crc >> 8) & 0x00FF;
    va_end(args);

    soft_serial.write(data, 14);
}

// read response to command
struct ReadData xl430_read_response(unsigned char len) {
    unsigned char response[len] = {0};
    unsigned char resp_char;
    while (soft_serial.available() < len) {
        delay(50);
    }
    for (int i = 0; i < len; i++) {
        response[i] = soft_serial.read();
    }
    ReadData r = {response[8], {0}};
    if (response[5] > 4) {
        for (int i = 0; i < response[5] - 4; i++) {
            r.params[i] = response[i + 9];
        }
    }
    return r;
}

// check that the response doesn't contain errors
void xl430_check_response(unsigned char id, unsigned char len) {
    ReadData resp = xl430_read_response(len);
    if (resp.err != 0) {
        nh.logerror("ERR in response");
    }
}

/*
 * ACTUAL COMMANDS HERE *
 */

// Read a memory address
// Use like ReadData resp = xl430_read(XL_LEFT, XL_MOVING, 1);
ReadData xl430_read(unsigned char id, unsigned int address, unsigned char datalen) {
    unsigned char address_low = address & 0xFF;
    unsigned char address_high = (address >> 8) & 0xFF;
    xl430_cmd_READ(id, address_low, address_high, datalen, 0);
    ReadData resp = xl430_read_response(datalen + 11);
    return resp;
}

void xl430_change_id(unsigned char id, unsigned char new_id) {
    unsigned char address_low = XL_ID & 0xFF;
    unsigned char address_high = (XL_ID >> 8) & 0xFF;
    xl430_cmd_WRITE(id, 3, address_low, address_high, new_id);
    xl430_check_response(id, 11);
}


void xl430_change_baud(unsigned char id, unsigned char baud) {
    unsigned char address_low = XL_BAUD_RATE & 0xFF;
    unsigned char address_high = (XL_BAUD_RATE >> 8) & 0xFF;
    xl430_cmd_WRITE(id, 3, address_low, address_high, baud);
    xl430_check_response(id, 11);
}

void xl430_change_servo_speed(unsigned char id, unsigned char s_speed) {
    unsigned char address_low = XL_PROFILE_VELOCITY & 0xFF;
    unsigned char address_high = (XL_PROFILE_VELOCITY >> 8) & 0xFF;
    unsigned char speed_low = s_speed & 0xFF;
    unsigned char speed_high = (s_speed >> 8) & 0xFF;
    xl430_cmd_WRITE(id, 6, address_low, address_high, speed_low, speed_high, 0, 0);
    xl430_check_response(id, 11);
    delay(50); // delay needed, otherwise servo might not spin on next command
}

void xl430_change_led(unsigned char id, unsigned char enabled) {
    unsigned char address_low = XL_LED & 0xFF;
    unsigned char address_high = (XL_LED >> 8) & 0xFF;
    xl430_cmd_WRITE(id, 3, address_low, address_high, enabled);
    xl430_check_response(id, 11);
}

void xl430_change_goal_position(unsigned char id, unsigned int pos) {
    unsigned char address_low = XL_GOAL_POSITION & 0xFF;
    unsigned char address_high = (XL_GOAL_POSITION >> 8) & 0xFF;
    unsigned char pos_1 = pos & 0xFF;
    unsigned char pos_2 = (pos >> 8) & 0xFF;
    xl430_cmd_WRITE(id, 6, address_low, address_high, pos_1, pos_2, 0, 0);
    xl430_check_response(id, 11);
}

void xl430_change_goal_velocity(unsigned char id, unsigned int spd) {
    unsigned char address_low = XL_GOAL_VELOCITY & 0xFF;
    unsigned char address_high = (XL_GOAL_VELOCITY >> 8) & 0xFF;
    unsigned char spd_low = spd & 0xFF;
    unsigned char spd_high = (spd >> 8) & 0xFF;
    xl430_cmd_WRITE(id, 3, address_low, address_high, spd_low, spd_high, 0, 0);
    xl430_check_response(id, 11);
    delay(50); // delay needed, otherwise servo might not spin on next command
}

void xl430_change_torque_enable(unsigned char id, unsigned char enable) {
    unsigned char address_low = XL_TORQUE_ENABLE & 0xFF;
    unsigned char address_high = (XL_TORQUE_ENABLE >> 8) & 0xFF;
    xl430_cmd_WRITE(id, 3, address_low, address_high, enable);
    xl430_check_response(id, 11);
}

void xl430_ping(unsigned char id) {
    xl430_cmd_PING(id);
    xl430_read_response(14);
}

void xl430_reboot(unsigned char id) {
    xl430_cmd_REBOOT(id);
    xl430_read_response(11);
}

void xl430_wait_for_move(unsigned char id) {
    while (true) {
        ReadData r = xl430_read(id, XL_MOVING, 1);
        if (r.params[0] == 0) {
            break;
        } else {
            delay(50);

        }
    }
}

void xl430_wait_for_moves() {
    xl430_wait_for_move(SERVO_RIGHT);
    xl430_wait_for_move(SERVO_FRONT);        
    xl430_wait_for_move(SERVO_LEFT);
    xl430_wait_for_move(SERVO_TURN);
}

void xl430_move(unsigned char spd, int offset_right, int offset_front, int offset_left, int offset_turn) {
    xl430_change_servo_speed(SERVO_RIGHT, spd);
    xl430_change_goal_position(SERVO_RIGHT, SERVO_RIGHT_CENTER + offset_right);
    xl430_change_servo_speed(SERVO_FRONT, spd);
    xl430_change_goal_position(SERVO_FRONT, SERVO_FRONT_CENTER + offset_front);
    xl430_change_servo_speed(SERVO_LEFT, spd);
    xl430_change_goal_position(SERVO_LEFT, SERVO_LEFT_CENTER + offset_left);
    xl430_change_servo_speed(SERVO_TURN, spd);
    xl430_change_goal_position(SERVO_TURN, SERVO_TURN_CENTER + offset_left);
}

int convert_to_servo(char id, int pos) {
    int servo;
    if (id == SERVO_RIGHT) {
        float kk = (SERVO_RIGHT_TOP - SERVO_RIGHT_BOTTOM) / 100;
        servo = SERVO_RIGHT_BOTTOM + kk * pos;
    }
    else if (id == SERVO_FRONT) {
        float kk = (SERVO_FRONT_TOP - SERVO_FRONT_BOTTOM) / 100;
        servo = SERVO_FRONT_BOTTOM + kk * pos;
    }
    else if (id == SERVO_LEFT) {
        float kk = (SERVO_LEFT_TOP - SERVO_LEFT_BOTTOM) / 100;
        servo = SERVO_LEFT_BOTTOM + kk * pos;
    }
    else if (id == SERVO_TURN) {
        float kk = (SERVO_TURN_TOP - SERVO_TURN_BOTTOM) / 100;
        servo = SERVO_TURN_BOTTOM + kk * pos;
    }
    return servo;
}
