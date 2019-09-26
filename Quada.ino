#include "FlexiTimer2.h" //to set a timer to manage all servos
#include "GoBLE.h"
#include <EEPROM.h>
#include <Servo.h> //to define and control servos
#include <math.h>
#include <setjmp.h>

int inter(int y_1, int y_2, int x_min, int x_max, int x);
uint8_t set_leg(uint8_t leg, uint8_t angle_0, uint8_t angle_1, uint8_t angle_2);
void update(int time);
/* Servos
 * --------------------------s------------------------------------------*/
// define 12 servos for 4 legs
Servo servo[4][3];
// define servos' ports
const int servo_pin[4][3] = {{2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13}};
int time = 0;
float angle = 0;
float x = 0;
float angle_rad = 0;
int ratio = 300;

int8_t servo_offset[4][3] = {
    {0, 0, 10},  // Leg 0
    {0, 0, 0},   // Leg 1
    {0, 0, -20}, // Leg 2
    {5, -10, 5}, // Leg 3
};
int servo_scale[4][3] = {
    // percent
    {100, 100, 100}, // Leg 0
    {100, 100, 100}, // Leg 1
    {100, 100, 100}, // Leg 2
    {100, 100, 100}, // Leg 3
};
uint8_t startup[3] = {90, 90, 90};
int8_t gait_sequence[4] = {1, 4, 3, 2};

/*
FORWARD : G1 4 3 2
BACKWARD: G-1 -2 -3 -4
F0 0 10 0 0 0 0 0 -20 5 -10 5

RIGHT : G2 3 -1 -4
LEFT  : G-3 -2 4 1
F0 0 20 -10 -5 0 0 0 -20 5 -10 -10

CCW: G1 -3 -4 2
CW: G-1 -2 4 3
*/

char *read_intp(char *buf, int *out) {
    char negative = 0;
    int val = 0;
    if (*buf == '-') {
        negative = 1;
        buf++;
    }
    while (*buf >= '0' && *buf <= '9') {
        val *= 10;
        val += *buf - '0';
        buf++;
    }
    *out = negative ? -val : val;
    return buf;
}
int read_int(char *buf) {
    int temp;
    read_intp(buf, &temp);
    return temp;
}

const int t_swing_down = 10000;
const int t_swing_up1 = t_swing_down / 10;
const int t_swing_up2 = t_swing_down / 4;
const int period_gait = t_swing_down;

const int angle_0_cen = 50;
const int angle_0_u = angle_0_cen - 20;
const int angle_0_d = angle_0_cen + 20;

const int angle_1_center = 75;

const uint8_t leg_sequence[4] = {0, 3, 2, 1};

void update(int time) {
    for (int gait_order = 0; gait_order < 4; gait_order++) {
        int time_leg =
            (time - gait_order * t_swing_up2 + period_gait) % period_gait;

        char gait_inverse = gait_sequence[gait_order] < 0;
        uint8_t leg_servo = abs(gait_sequence[gait_order]) - 1;

        if (time_leg < 0 || time_leg > t_swing_down) {
            // Leg idle
        } else {

            /*
             * Angle2
             *          .''-.
             *        .'     '-.
             * _____.'          '-.______
             *      ^    ^         ^
             *      0  t_up2    t_down
             *
             * Angle0    t_up2
             *            v
             *        /'''|
             *       /    |
             * _____/     |_____________
             *      ^ ^             ^
             *      0 t_up1       t_down
             *
             */
            int angle_2 =
                inter(50, 110, 0, t_swing_up2, time_leg) +
                inter(110, 50, t_swing_up2 + 1, t_swing_down, time_leg);
            if (gait_inverse)
                angle_2 = 160 - angle_2;
            int angle_0 =
                inter(angle_0_d, angle_0_u, 0, t_swing_up1, time_leg) +
                inter(angle_0_u, angle_0_u, t_swing_up1 + 1, t_swing_up2,
                      time_leg) +
                inter(angle_0_d, angle_0_d, t_swing_up2 + 1, t_swing_down,
                      time_leg);
            int angle_1 = angle_1_center;
            set_leg(leg_servo, angle_0, angle_1, angle_2);
        }
    }
}

void loop() {}

uint8_t set_leg(uint8_t leg, uint8_t angle_0, uint8_t angle_1,
                uint8_t angle_2) {
    angle_0 = (angle_0 - 90) * (long)servo_scale[leg][0] / 100 + 90;
    angle_1 = (angle_1 - 90) * (long)servo_scale[leg][1] / 100 + 90;
    angle_2 = (angle_2 - 90) * (long)servo_scale[leg][2] / 100 + 90;
    angle_0 += servo_offset[leg][0];
    angle_1 += servo_offset[leg][1];
    angle_2 += servo_offset[leg][2];
    if (leg == 1) {
        angle_0 = 180 - angle_0;
        angle_1 = 180 - angle_1;
    } else if (leg == 2) {
        angle_0 = 180 - angle_0;
        angle_2 = 160 - angle_2;
        angle_1 = 180 - angle_1;
    } else if (leg == 3) {
        angle_2 = 160 - angle_2;
    }
    servo[leg][2].write(angle_2);
    servo[leg][0].write(angle_0);
    servo[leg][1].write(angle_1);
}

// Serial.print(inter(20, 160, 0, 300, time));

int inter(int y_1, int y_2, int x_min, int x_max, int x) {
    if (x >= x_min && x <= x_max)
        return (y_2 - y_1) * (long)(x - x_min) / (x_max - x_min) + y_1;
    return 0;
}

/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup() {
    // start serial for debug
    Serial.begin(115200); // baud rate chosen for bluetooth compatability
    //
    Serial.println("Robot starts initialization");
    // initialize default parameter
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            servo[i][j].attach(servo_pin[i][j]);
            // servo[i][j].write(startup[j] + servo_offset[i][j]);
        }
        set_leg(i, startup[0], startup[1], startup[2]);
    }

    if (1) {
        char buffer[80], buffer_end, running, *bufp;
        int param, leg = 0, joint = 0, k;
        uint16_t rate = 1;
        Serial.println("Test Console");
        while (1) {
            Serial.print(">>> ");
            buffer_end = -1;
            do {
                while (Serial.available() == 0)
                    ;
                buffer[++buffer_end] = Serial.read();
                Serial.print(buffer[buffer_end]);
            } while (buffer[buffer_end] != '\n');
            if (buffer_end == 0)
                Serial.println("Unknown command");
            else {
                switch (buffer[0]) {
                case 'a':
                    param = read_int(&buffer[1]);
                    Serial.print("Set startup angle_0 to ");
                    Serial.println(param);
                    startup[0] = param;
                    for (int i = 0; i < 4; i++)
                        set_leg(i, startup[0], startup[1], startup[2]);
                    break;
                case 'b':
                    param = read_int(&buffer[1]);
                    Serial.print("Set startup angle_1 to ");
                    Serial.println(param);
                    startup[1] = param;
                    for (int i = 0; i < 4; i++)
                        set_leg(i, startup[0], startup[1], startup[2]);
                    break;
                case 'c':
                    param = read_int(&buffer[1]);
                    Serial.print("Set startup angle_2 to ");
                    Serial.println(param);
                    startup[2] = param;
                    for (int i = 0; i < 4; i++)
                        set_leg(i, startup[0], startup[1], startup[2]);
                    break;
                case 'u':
                    Serial.println("Startup pose");
                    for (int i = 0; i < 4; i++)
                        set_leg(i, startup[0], startup[1], startup[2]);
                    break;
                case 't':
                    param = read_int(&buffer[1]);
                    update(param);
                    break;
                case 'l':
                    param = read_int(&buffer[1]);
                    Serial.print("Select leg ");
                    Serial.println(param);
                    leg = param;
                    break;
                case 'j':
                    param = read_int(&buffer[1]);
                    Serial.print("Select joint ");
                    Serial.println(param);
                    joint = param;
                    break;
                case 'g':
                    param = read_int(&buffer[1]);
                    gait_sequence[leg] = param;
                    Serial.print("Gait order ");
                    for (int i = 0; i < 4; i++) {
                        Serial.print(gait_sequence[i]);
                        Serial.print(' ');
                    }
                    Serial.println();
                    break;
                case 'G':
                    bufp = &buffer[1];
                    for (k = 0; k < 4; k++) {
                        bufp = read_intp(bufp, &param);
                        gait_sequence[k] = param;
                        if (*(bufp++) != ' ')
                            break;
                    }
                    Serial.print("Gait order ");
                    for (int i = 0; i < 4; i++) {
                        Serial.print(gait_sequence[i]);
                        Serial.print(' ');
                    }
                    Serial.println();
                    break;
                case 'o':
                    param = read_int(&buffer[1]);
                    Serial.print("Offset leg ");
                    Serial.print(leg);
                    Serial.print(" joint ");
                    Serial.print(joint);
                    Serial.print(" to ");
                    Serial.println(param);
                    servo_offset[leg][joint] = param;
                    for (int i = 0; i < 4; i++)
                        set_leg(i, startup[0], startup[1], startup[2]);
                    break;
                case 'F':
                    bufp = &buffer[1];
                    for (k = 0; k < 4 * 3; k++) {
                        bufp = read_intp(bufp, &param);
                        servo_offset[k / 3][k % 3] = param;
                        if (*(bufp++) != ' ')
                            break;
                    }
                    for (int i = 0; i < 4; i++)
                        set_leg(i, startup[0], startup[1], startup[2]);
                case 'O':
                    Serial.println("=== Offset ===");
                    for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 3; j++) {
                            Serial.print(servo_offset[i][j]);
                            Serial.print(' ');
                        }
                        Serial.println();
                    }
                    break;
                case 's':
                    param = read_int(&buffer[1]);
                    Serial.print("Scale leg ");
                    Serial.print(leg);
                    Serial.print(" joint ");
                    Serial.print(joint);
                    Serial.print(" to ");
                    Serial.println(param);
                    servo_scale[leg][joint] = param;
                    for (int i = 0; i < 4; i++)
                        set_leg(i, startup[0], startup[1], startup[2]);
                    break;
                case 'S':
                    Serial.println("=== Scale ===");
                    for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 3; j++) {
                            Serial.print(servo_scale[i][j]);
                            Serial.print(' ');
                        }
                        Serial.println();
                    }
                    break;
                case 'R':
                    param = read_int(&buffer[1]);
                    rate = param;
                    Serial.print("Rate = ");
                    Serial.println(rate);
                    break;
                case 'r':
                    Serial.println("Run gait. press x to exit");
                    running = 1;
                    buffer_end = -1;
                    long toffset = -millis();
                    while (running) {
                        time = ((millis() + toffset) * rate) % period_gait;
                        while (Serial.available() > 0) {
                            buffer[++buffer_end] = Serial.read();
                            Serial.print(buffer[buffer_end]);
                            if (buffer[buffer_end] == '\n') {
                                if (buffer[0] == 'x') {
                                    running = 0;
                                    break;
                                } else if (buffer[0] == 'R') {
                                    rate = read_int(&buffer[1]);
                                    Serial.print("Rate = ");
                                    Serial.println(rate);
                                    toffset = time / rate - millis();
                                }
                                buffer_end = -1;
                            }
                        }
                        update(time);
                    }
                    break;
                default:
                    Serial.println("Unknown command");
                }
            }
        }
        Serial.readStringUntil('\n');
    }
}