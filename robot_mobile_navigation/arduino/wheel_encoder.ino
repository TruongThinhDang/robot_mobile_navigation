#include <ros.h>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <PID_v1.h>

ros::NodeHandle nh;
double speed_act_left = 0;
double speed_act_right = 0;
float demandx, demandz;

float tick1 = 100;
float tick2 = 100;
float rad = 0.0935;

void velCallback( const geometry_msgs::Twist& vel){
    demandx = vel.linear.x;
    demandz = vel.angular.z;

    demandx = constrain(demandx, -0.5, 0.5);
    demandz = constrain(demandz, -0.5, 0.5);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);

double P1 = 1;
double I1 = 15;
double D1 = 0.15;

double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, P1, I1, D1, DIRECT);

double P2 = 1;
double I2 = 15;
double D2 = 0.15;

double Setpoint2, Input2, Output2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, P2, I2, D2, DIRECT);

float demand1, demand2;

unsigned long currentMillis;
unsigned long previousMillis;

#define encoder1PinA 2
#define encoder1PinB 3

#define encoder2PinA 18
#define encoder2PinB 19

volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;

float encoder1Driff;
float encoder1Prev;
float encoder1Error;

float encoder2Driff;
float encoder2Prev;
float encoder2Error;

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(speed_pub);

    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);

    pinMode(encoder1PinA, INPUT_PULLUP);
    pinMode(encoder1PinB, INPUT_PULLUP);
    pinMode(encoder2PinA, INPUT_PULLUP);
    pinMode(encoder2PinB, INPUT_PULLUP);

    attachInterrupt(0, doEncoderA, CHANGE);
    attachInterrupt(1, doEncoderB, CHANGE);

    attachInterrupt(4, doEncoderC, CHANGE);
    attachInterrupt(5, doEncoderD, CHANGE);

    PID1.SetMode(AUTOMATIC);
    PID1.SetOutputLimits(-250, 250);
    PID1.SetSampleTime(10);

    PID2.SetMode(AUTOMATIC);
    PID2.SetOutputLimits(-250, 250);
    PID2.SetSampleTime(10);

    Serial.begin(57600);
}

void loop() {
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10){
        previousMillis = currentMillis;

        demand1 = demandx - (demandz*rad);
        demand2 = demandx + (demandz*rad);

        encoder1Driff = encoder1Pos - encoder1Prev;
        encoder2Driff = encoder2Pos - encoder2Prev;

        encoder1Error = (demand1*tick1) - encoder1Driff;
        encoder2Error = (demand2*tick2) - encoder2Driff;

        encoder1Prev = encoder1Pos;
        encoder2Prev = encoder2Pos;

        Setpoint1 = demand1*tick1;
        Input1 = encoder1Driff;
        PID1.Compute();

        Setpoint2 = demand2*tick2;
        Input2 = encoder2Driff;
        PID2.Compute();

        speed_act_left = encoder1Driff/tick1;
        speed_act_right = encoder2Driff/tick2;

        if (Output1 > 0){
            Output1a = abs(Output1);
            analogWrite(6, Output1a);
            analogWrite(7, 0);
        }

        else if (Output1 < 0){
            Output1a = abs(Output1);
            analogWrite(7, Output1a);
            analogWrite(6, 0);
        }

        else {
            analogWrite(7, 0);
            analogWrite(6, 0);
        }

        if (Output2 > 0){
            Output2a = abs(Output2);
            analogWrite(5, Output2a);
            analogWrite(4, 0);
        }

        else if (Output2 < 0){
            Output2a = abs(Output2);
            analogWrite(4, Output2a);
            analogWrite(5, 0);
        }
        else {
            analogWrite(4, 0);
            analogWrite(5, 0);
        }
        publishSpeed(10);
    }
}

void publishSpeed(double time) {
    speed_msg.header.stamp = nh.now();
    speed_msg.vector.x = speed_act_left;
    speed_msg.vector.y = speed_act_right;
    speed_msg.vector.z = time/1000;
    speed_pub.publish(&speed_msg);
    nh.spinOnce();
}

void doEncoderA(){
    if (digitalRead(encoder1PinA) == HIGH){
        if (digitalRead(encoder1PinB) == LOW){
            encoder1Pos = encoder1Pos + 1;
        }
        else {
            encoder1Pos = encoder1Pos - 1;
        }
    }
    else{
        if (digitalRead(encoder1PinB) == HIGH){
        encoder1Pos = encoder1Pos + 1;
        }
        else {
            encoder1Pos = encoder1Pos - 1;
        }
    }
}

void doEncoderB(){
    if (digitalRead(encoder1PinB) == HIGH){
        if (digitalRead(encoder1PinA) == HIGH){
            encoder1Pos = encoder1Pos + 1;
        }
        else {
            encoder1Pos = encoder1Pos - 1;
        }
        }
        else{
        if (digitalRead(encoder1PinA) == LOW){
            encoder1Pos = encoder1Pos + 1;
        }
        else {
            encoder1Pos = encoder1Pos - 1;
        }
    }
}

void doEncoderC(){
    if (digitalRead(encoder2PinA) == HIGH){
        if (digitalRead(encoder2PinB) == LOW){
            encoder2Pos = encoder2Pos - 1;
        }
        else {
            encoder2Pos = encoder2Pos + 1;
        }
        }
        else{
        if (digitalRead(encoder2PinB) == HIGH){
            encoder2Pos = encoder2Pos - 1;
        }
        else {
            encoder2Pos = encoder2Pos + 1;
        }
    }
}

void doEncoderD(){
    if (digitalRead(encoder2PinB) == HIGH){
        if (digitalRead(encoder2PinA) == HIGH){
            encoder2Pos = encoder2Pos - 1;
        }
        else {
            encoder2Pos = encoder2Pos + 1;
        }
        }
        else{
        if (digitalRead(encoder2PinA) == LOW){
            encoder2Pos = encoder2Pos - 1;
        }
        else {
            encoder2Pos = encoder2Pos + 1;
        }
    }
}