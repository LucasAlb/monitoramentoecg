#include "BluetoothSerial.h"

#define LED_BUILTIN 2 //pin with LED to turn on when BT connected
#define LO_POS 40    // LO+ pin
#define LO_NEG 41    // LO- pin
#define ADC_PIN A0   // ADC pin
#define FILTER_SIZE 5   // filter size

BluetoothSerial ESP_BT; 

// global variables
boolean BT_cnx = false;
int adc_val = 0;
float filtered_val = 0;
float filter[FILTER_SIZE] = {0, 0, 0, 0, 0};

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    digitalWrite(LED_BUILTIN, HIGH);
    BT_cnx = true;
  }
 
  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
    digitalWrite(LED_BUILTIN, LOW);
    BT_cnx = false;
    ESP.restart();
  }
}

void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize serial communication:
  Serial.begin(115200);
  Serial.println(); 
  pinMode(LO_POS, INPUT);
  pinMode(LO_NEG, INPUT);
  pinMode(ADC_PIN, INPUT);
  // initialize Bluetooth serial communication:
  ESP_BT.register_callback(callback);
  if(!ESP_BT.begin("ESP32_ECG")){
    Serial.println("An error occurred initializing Bluetooth");
  }else{
    Serial.println("Bluetooth initialized... Bluetooth Device is Ready to Pair...");
  }
}

void loop() {
  int lo_pos_val = digitalRead(LO_POS);
  int lo_neg_val = digitalRead(LO_NEG);
  
  // check if there is noise in the ECG signal
  if(lo_pos_val == 1 || lo_neg_val == 1){
    Serial.println("ECG signal noise detected!");
    ESP_BT.println("!"); // send signal noise flag via Bluetooth
  }else{
    // read ADC value and apply digital filter
    adc_val = analogRead(ADC_PIN);
    filtered_val = 0;
    for(int i = FILTER_SIZE-1; i >= 1; i--){
      filter[i] = filter[i-1];
      filtered_val += filter[i];
    }
    filter[0] = adc_val;
    filtered_val = (filtered_val + filter[0]) / FILTER_SIZE;
    // send filtered value via Bluetooth
    if(BT_cnx){
      ESP_BT.print('E'); //App Bluetooth Graphics
      ESP_BT.println(filtered_val);
    } 
  }
  //delay to avoid data saturation
  delay(1);
}

