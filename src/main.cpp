
#include <SPI.h>
#include "Arduino.h"
#include "RF24.h"
#include "printf.h"

#define Interval_T_Get_start_value 5  // second
void Get_Start_Value_Controller (void);
void Get_MinMax_Value_Controller (void);

// instantiate an object for the nRF24L01 transceiver
RF24 radio(2, 4);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber =1;  
// 0 uses address[0] to transmit, 1 uses address[1] to transmit

void RX_RF_Init(void);
void MODE_TX2RX(void);
void MODE_RX2TX(void);
void NRF_Send_Data(void);
void NRF_Get_Data(void);

float payload_RX[5];
float payload_TX[5] = {9, 8, 7, 6, 5};
float map_payload_RX[5];
float start_controll_value[4];
float min_controll_value[4]={1023,1023,1023,1023};
float max_controll_value[4]={0,0,0,0};
float min_controll_angle = -30;
float max_controll_angle =30;
uint32_t start_time = millis();
void setup() {
    Serial.begin(115200);
    RX_RF_Init();
    Get_Start_Value_Controller();
    Get_MinMax_Value_Controller();
}  // setup

void RX_RF_Init(void) {
    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {
        }  // hold in infinite loop
    }
    radio.setChannel(23);
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
    radio.setPayloadSize(
        sizeof(payload_RX));  // float datatype occupies 4 bytes
    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0
    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
    // additional setup specific to the node's role
    radio.startListening();
}

void loop() {
    // MODE_TX2RX();
    NRF_Get_Data();
    Func_map_payload_RX();
    // MODE_RX2TX();
    // NRF_Send_Data();
}  // loop

void NRF_Get_Data(void) {
    uint8_t pipe;
    if (radio.available(&pipe)) {
        uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
        radio.read(&payload_RX, bytes);          // fetch payload from FIFO
        // Serial.print(F("Received "));
        // Serial.print(bytes);  // print the size of the payload
        // Serial.print(F(" bytes on pipe "));
        // Serial.print(pipe);  // print the pipe number
        // Serial.print(F(": "));
        // // MODE_RX2TX();
        // Serial.print(millis());
        // Serial.print(",");
        // Serial.print(payload_RX[0]);
        // Serial.print(",");
        // Serial.print(payload_RX[1]);
        // Serial.print(",");
        // Serial.print(payload_RX[2]);
        // Serial.print(",");
        // Serial.print(payload_RX[3]);
        // Serial.print(",");
        // Serial.print(payload_RX[4]);
        // Serial.println();
        // // }
    }
    // else Serial.println("No signal");
}

void NRF_Send_Data(void) {
    unsigned long start_timer = micros();  // start the timer
    bool report = radio.write(
        &payload_TX, sizeof(payload_TX));  // transmit & save the report
    unsigned long end_timer = micros();    // end the timer

    if (report) {
        // MODE_TX2RX();
        Serial.print(F("Transmission successful! "));  // payload was delivered
        Serial.print(F("Time to transmit = "));
        Serial.print(end_timer - start_timer);  // print the timer result
        Serial.print(F(" us. Sent: "));
        Serial.println(payload_TX[0]);  // print payload sent
    } else {
        Serial.println(F(
            "Transmission failed or timed out"));  // payload was not delivered
    }
}

void MODE_RX2TX(void) {
    radio.stopListening();  // put radio in RX mode
    delay(1);
}

void MODE_TX2RX(void) {
    radio.startListening();  // put radio in RX mode
    delay(1);
}

void Get_Start_Value_Controller (void){
    delay(5000);
    NRF_Get_Data();

    start_controll_value[0] = payload_RX[0];
    start_controll_value[1] = payload_RX[1];
    start_controll_value[2] = payload_RX[2];
    start_controll_value[3] = payload_RX[3];
    Serial.print("start value 0:");
    Serial.print(start_controll_value[0]);
    Serial.print(",    1:");
    Serial.print(start_controll_value[1]);
    Serial.print(",    2:");
    Serial.print(start_controll_value[2]);
    Serial.print(",    3:");
    Serial.println(start_controll_value[3]);
    delay(5000);
}
void Get_MinMax_Value_Controller (void){
  Serial.println("Start Get Min Max Value of controller: ");
  Serial.print("running:");
  uint32_t start_time = millis();
  while (millis()-start_time<10000){
    NRF_Get_Data();
    if(payload_RX[0]>max_controll_value[0])max_controll_value[0]=payload_RX[0];
    if(payload_RX[1]>max_controll_value[1])max_controll_value[1]=payload_RX[1];
    if(payload_RX[2]>max_controll_value[2])max_controll_value[2]=payload_RX[2];
    if(payload_RX[3]>max_controll_value[3])max_controll_value[3]=payload_RX[3];

    if(payload_RX[0]<min_controll_value[0])min_controll_value[0]=payload_RX[0];
    if(payload_RX[1]<min_controll_value[1])min_controll_value[1]=payload_RX[1];
    if(payload_RX[2]<min_controll_value[2])min_controll_value[2]=payload_RX[2];
    if(payload_RX[3]<min_controll_value[3])min_controll_value[3]=payload_RX[3];
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Min  0: ");Serial.print(min_controll_value[0]);
  Serial.print(",  1: ");Serial.print(min_controll_value[1]);
  Serial.print(",  2: ");Serial.print(min_controll_value[2]);
  Serial.print(",  3: ");Serial.println(min_controll_value[3]);


  Serial.print("Max  0: ");Serial.print(max_controll_value[0]);
  Serial.print(",  1: ");Serial.print(max_controll_value[1]);
  Serial.print(",  2: ");Serial.print(max_controll_value[2]);
  Serial.print(",  3: ");Serial.println(max_controll_value[3]);
  Serial.println("END MIN MAX!");
}

void Func_map_payload_RX (void){
  for (int i=0;i<4;i++){
    if (payload_RX[i]<=start_controll_value[i])
    {
      map_payload_RX[i]= map(payload_RX[i],min_controll_value[i],start_controll_value[i],min_controll_angle,0);
    }else {
      map_payload_RX[i]= map(payload_RX[i],start_controll_value[i],max_controll_value[i],0,max_controll_angle);
    }
  }

  Serial.print(map_payload_RX[0]);Serial.print(", ");
  Serial.print(map_payload_RX[1]);Serial.print(", ");
  Serial.print(map_payload_RX[2]);Serial.print(", ");
  Serial.print(map_payload_RX[3]);Serial.println("");

}