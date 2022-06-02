// Neuron with an activation function based on leaky integrate-and-fire model

#include <ESP8266WiFi.h>
#include <espnow.h>

const int DIODE = 5;

// Set the lenght of spike increase and decrease - allows us to make different spike shapes (e.g. slow increase and rapid decrease)
int spike_up_step = 1; // the lenght of a step of increase of the brightness
//int spike_activation_time = 100; 
int spike_down_step = 1; // the lenght of a step of decrease of the brightness
int spike_duration = 1; 

int refr_period = 700; 
int resting_V = 207;
float leak = 0.5;
int dt = 10; // time step 
int tau_m = 50; // membrane time constant; set to 1 to skip
int threshold = 450;

float memb_V_prev = resting_V; //set initial value of the membrane potential
float memb_V = memb_V_prev;
float dV;
float input;
float last_input;

const int window_n = 100;
float avarage_array[window_n] = {0};

// Structure example to receive data
// Must match the sender structure
typedef struct data_struct {
    int tau_m;
    int refracion_period;
    int threshold;
    int spike_duration;
} data_struct;

// Create a struct_message called myData
data_struct myData;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  tau_m = myData.tau_m; 
  refr_period = myData.refracion_period; 
  threshold = myData.threshold;
  spike_duration = myData.spike_duration;
//  Serial.println(myData.tau_m);
//  Serial.println(myData.refracion_period);
//  Serial.println(myData.threshold);
//  Serial.println(myData.spike_duration);
  
}
 


void setup() {
   // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
   
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  
  // put your setup code here, to run once:
  pinMode(DIODE, OUTPUT);
  pinMode(A0, INPUT);
//  Serial.begin(115200);


  for (int n=0; n < window_n; n++){
    avarage_array[n] = analogRead(A0) / 3;
    delay(dt);
  }

}


int avarage_n = 0;
float avarage;


void loop() {
  // print out constant values to check
    Serial.print("Received \n");
    Serial.println(tau_m);
    Serial.println(refr_period);
    Serial.println(threshold);
    Serial.println(spike_duration);
  
  delay(dt);
  last_input = input;
  input = analogRead(A0) / 3;

  avarage_array[avarage_n] = input;
  avarage_n = ++avarage_n % window_n;
  avarage = calc_avarage(avarage_array, window_n);
  
  dV = (-(memb_V_prev - resting_V)  + input/leak) * (dt/tau_m); // nor sure if dt here is necessary
  memb_V = memb_V + dV;
  memb_V_prev = memb_V; 
  if (memb_V > threshold) {
    spike(spike_up_step, spike_down_step, refr_period);
    memb_V = 1200;
    //Serial.println(memb_V);
    memb_V = resting_V;
  }
     //the first variable for plotting
  //Serial.print(memb_V);
  //Serial.print(",");       
//  Serial.print(avarage);
//  Serial.print(",");         
//  Serial.println(input);  
}

void spike(int spike_up_step, int spike_down_step, int refr_period) {
  for (int i = 0; i < 255; i++) {
    analogWrite(DIODE, i);
    delay(spike_up_step);
   }

  //delay(spike_activation_time);
  
  for (int i = 255; i >= 0; i--) {
    analogWrite(DIODE, i);
    delay(spike_down_step);
  }
  delay(refr_period);
}

float calc_avarage(float arr[], int n){

  float sum = 0;
  for (int i=0; i<n; i++){
    sum += arr[i];
  }  
  return sum / n;
  
  
}
