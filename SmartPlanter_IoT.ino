#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MCP23X17.h>
#include "Wire.h"
#include "RTClib.h"
#include "Credentials.h"

#define MAX_BUFFER        255

/* Defines for Delays */
#define INTER_DELAY       100
#define LOOP_DELAY        500
/*-------------------------------------------------------------*/
/* Defines for Robot functions */
#define PUMP_START_DELAY  1000
#define NUMBER_OF_PLANTS  4
#define AIR_VALUE         870
#define WATER_VALUE       430
/*-------------------------------------------------------------*/
/* Status LED Pins -> On MCP23S17 */
#define WIFI_INITIALIZED_LED  15  // White
#define API_CONNECTED_LED     14  // Blue
#define UPDATING_API_LED      13  // Yellow
#define WATERING_GARDEN_LED   12  // Green
#define ROBOT_SLEEPING_LED    11  // Red
/*-------------------------------------------------------------*/
/* SPI pins */
#define TIMER_INTERRUPT_PIN   15
#define MCP3008_CS_PIN        0
#define MCP23S17_CS_PIN       16
/*-------------------------------------------------------------*/
/* Pump / Solenoid */
#define WATER_PUMP_ON HIGH
#define WATER_PUMP_OFF LOW
#define SOLENOID_OPEN HIGH
#define SOLENOID_CLOSED LOW
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
ADC_MODE(ADC_VCC);
/*-------------------------------------------------------------*/

/* Peripherals */
Adafruit_MCP23X17 IO_EXPANDER;
Adafruit_MCP3008  ADC_EXPANDER;
RTC_PCF8523 RTC_CLOCK;
/*-------------------------------------------------------------*/

/* Serial Buffers */
char RX_BUFFER[MAX_BUFFER];
char TX_BUFFER[MAX_BUFFER];
/*-------------------------------------------------------------*/

/* Status Boolean to Control LED's */
bool WIFI_INIT = false;
bool API_INIT = false;
bool UPDATING = false;
bool WATERING = false;
bool SLEEPING = false;
/*-------------------------------------------------------------*/

/* Robot Data & Wifi Initialization */
String SSID = C_SSID;
String PASSWORD = C_PASSWORD;
char UNIQUE_ID[] = "SMRTPLNTR_PROTOTYPE";
char NAME[] = "default";
char LOCATION[] = "default";
char TIME_ZONE[] = "Eastern Standard Time";
char ESP_VCC[10];
DateTime NOW;
static DateTime TIME_CHECK;
/*-------------------------------------------------------------*/

/* API Initialization */
WiFiClient wifiClient;
HTTPClient _HTTP;
const char* apiGardenEndpoint = "http://your.api.server.com/api/garden"; // Replace
const char* apiCommandEndpoint = "http://your.api.server.com/api/command"; // Replace
bool API_CONNECT = false;
/*-------------------------------------------------------------*/

/* Define API Commands */
char getName_Command_API[] = "get-planter-name";
char getLocation_Command_API[] = "get-planter-location";
char getTimeZone_Command_API[] = "get-planter-timezone";
char getTime_Command_API[] = "get-planter-time";
char getWaterAllPlants_Command_API[] = "get-water-all-plants";
char getWaterPlant_Command_API[] = "get-water-plant";
char getPlantName_Command_API[] = "get-plant-name";
char getMoistureThreshold_Command_API[] = "get-plant-moisture-threshold";
char getDaysBetweenWatering_Command_API[] = "get-plant-days-between-watering";
char getTimeWatering_Command_API[] = "get-time-watering";
/*-------------------------------------------------------------*/

/* Plant Object/Linked List */
struct Plant{
  int index;
  char name[MAX_BUFFER];
  int daysToWater;
  int moistureThreshold;
  int currentMoisture;
  int timeWatering;
  DateTime lastTimeWatered;
  int solenoid_gpio;
  int adc_channel;
  Plant* next;
};
Plant* head = NULL;
/*-------------------------------------------------------------*/

/* Robot Funtions */
void getSystemBuffer(char * robotBuffer);
void substrBuffer(const char * buffer, char * newbuffer, int startPos, int endPos);
void initExpander();
void initClock();
/*-------------------------------------------------------------*/

/* Plant Data Structure */
Plant defaultPlant();
void initPlants(int numberOfPlants);
Plant* getPlant(int searchIndex);
int indexOf(const char * buffer, char c);
int indexOf(const char * buffer, char c, int startPos);
void appendPlant(int index, Plant plant);
void appendPlant(int index);
void clearPlants();
/*-------------------------------------------------------------*/

/* Plant Data for API Data */
int getPlantData_NoIndex(int searchIndex, char * plantBuffer);
int getPlantData_fromAPI(int index, Plant* plantData);
int getPlantUpdate_NoIndex(int searchIndex, char * plantBuffer);
/*-------------------------------------------------------------*/

/* Gardening Commands */
int getMoistureSensorValue(int adcChannel);
void UpdateGarden();
void WaterPlant(Plant * plant);
void WaterGarden();
void WaterAll();
void Water(Plant * plant);
/*-------------------------------------------------------------*/

/* Robot Wifi Initializers */
void setupWifi();
void setupWifi(const char * ssid, const char * password);
/*-------------------------------------------------------------*/

/* Robot Initialize to API */
bool ConnectToSmartPlanter_API();
int AddSmartPlanter_API();
int translatePlantData(int index, Plant* plantData);
/*-------------------------------------------------------------*/

/* Update to API */
int updatePlants();
int initRobot();
/*-------------------------------------------------------------*/

/*  Robot data -> API */
int UpdateSmartPlanter_API();
int UpdateSmartPlanterName_API();
int UpdateSmartPlanterTimeZone_API();
int UpdateSmartPlanterLocation_API();
int UpdateSmartPlanterESPVoltage_API();
int UpdateSmartPlanterNumberOfPlants_API();
int UpdateSmartPlanterPlantName_API(int plantIndex, const char * plantName);
int UpdateSmartPlanterPlantSoilMoisture_API(int plantIndex, int currMoisture);
int UpdateSmartPlanterPlantMoistureThreshold_API(int plantIndex, int moistureThresh);
int UpdateSmartPlanterPlantDaysBetweenWatering_API(int plantIndex, int dbw);
int UpdateSmartPlanterPlantTimeWatering_API(int plantIndex, int timeWater);
int UpdateSmartPlanterPlantLastTimeWatered_API(int plantIndex, const char * ltw);
/*-------------------------------------------------------------*/

/* API data -> robot */
bool GetSmartPlanterName_API();
bool GetSmartPlanterLocation_API();
bool GetSmartPlanterTime_API();
bool GetSmartPlanterTimeZone_API();
bool GetSmartPlanterPlantName_API(int plantIndex, char * plantNamePointer);
bool GetSmartPlanterPlantMoistureThreshold_API(int plantIndex, int * plantMoistureThreshPointer);
bool GetSmartPlanterPlantDaysBetweenWatering_API(int plantIndex, int * plantDbwPointer);
bool GetSmartPlanterPlantTimeWatering_API(int plantIndex, int * plantTimeWateringPointer);
bool GetSmartPlanterPlantLastTimeWatered_API(int plantIndex, char * plantLastTimeWateredPointer);
/*-------------------------------------------------------------*/

/* API Commands -> robot */
void GetCommand();
bool GetSmartPlanterCommand_API(char * command);
void AcknowledgeSmartPlanterCommand_API(int commandId, bool success);
bool initCommand(const char * command, const char * commandData);
bool setRTCTime(const char* timeString);
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*
  Main Program Functionality
*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  ADC_EXPANDER.begin(MCP3008_CS_PIN);
  initClock();
  initExpander();
  initPlants(NUMBER_OF_PLANTS);
  setupWifi();
  //API_INIT = ConnectToSmartPlanter_API();
  //int initializeRobotFromApi;
  //if(API_INIT == true){
    //int initData;
    //initData = initRobot();
    //if(initData < 0){ while(1){ Serial.println("Cannot initialize data to/from API"); } }
  //} else { while(1){ Serial.println("Cannot initialize data to/from API"); } }
  
}

void loop() {
  NOW = RTC_CLOCK.now();
  sprintf(ESP_VCC,"%d",ESP.getVcc());

  //static DateTime hourCheck = RTC_CLOCK.now();
  static DateTime fiveMinCheck = RTC_CLOCK.now();

  if(WIFI_INIT == true){ IO_EXPANDER.digitalWrite(WIFI_INITIALIZED_LED, HIGH); }
  //if(API_INIT == true){ IO_EXPANDER.digitalWrite(API_CONNECTED_LED, HIGH); }

  // (hasTimePassed(fiveMinCheck, 300)) // Actual 5 minutes
  if(hasTimePassed(fiveMinCheck, 30)) { // Do every 5 minutes // Testing every 2.5 minutes
    //API_INIT = ConnectToSmartPlanter_API();
    //WaterAll();
  }

  // (hasTimePassed(hourCheck, 3600))  // Actual hour
  //if(hasTimePassed(hourCheck, 300)){  // Do each hour  // Testing every 5 minutes
    //UpdateGarden();
    //delay(INTER_DELAY);
    //WaterGarden();
    //if(ConnectToSmartPlanter_API() == true){
      //int update = updatePlants();
      //if(update < 0){ Serial.println("Cannot Update Plant Database"); }
      //delay(LOOP_DELAY);
      //bool updateRobot = UpdateSmartPlanterESPVoltage_API();
      //if(updateRobot != true){ Serial.println("Cannot Update Robot Database"); }
    //}
  //}
  delay(LOOP_DELAY);
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*
          Functions after Main Program to keep readability
*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
void substrBuffer(const char * buffer, char * newbuffer, int startPos, int endPos){
  int length = endPos - startPos - 1;
  strncpy(newbuffer, &buffer[startPos + 1], length);
  newbuffer[length] = '\0';
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void getSystemBuffer(char * robotBuffer){
  sprintf(robotBuffer, "%s:%s:%s:%d", NAME, LOCATION, TIME_ZONE, ESP_VCC);
  return;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void initExpander(){
  IO_EXPANDER.begin_SPI(MCP23S17_CS_PIN);
  IO_EXPANDER.pinMode(WIFI_INITIALIZED_LED, OUTPUT);
  IO_EXPANDER.pinMode(API_CONNECTED_LED, OUTPUT);
  IO_EXPANDER.pinMode(UPDATING_API_LED, OUTPUT);
  IO_EXPANDER.pinMode(WATERING_GARDEN_LED, OUTPUT);
  IO_EXPANDER.pinMode(ROBOT_SLEEPING_LED, OUTPUT);
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void blinkLED(int gpio) {
  bool read = digitalRead(gpio);
  digitalWrite(gpio, !read);
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
bool hasTimePassed(DateTime &lastRecordedTime, int intervalInSeconds) {
  DateTime currentTime = RTC_CLOCK.now();
  long timeDifference = currentTime.unixtime() - lastRecordedTime.unixtime();
  if (timeDifference >= intervalInSeconds) {
    lastRecordedTime = currentTime;
    return true;
  }
  return false;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void initExpander_gpio(){
  IO_EXPANDER.pinMode(NUMBER_OF_PLANTS, OUTPUT);  // top most gpio pin is pump output
  for(int i = 0; i < NUMBER_OF_PLANTS; i++){
    IO_EXPANDER.pinMode(i, OUTPUT); // for 0 -> Number of plants - 1 equals expander output to solenoid
    delay(INTER_DELAY);
  }
  return;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
bool setRTCTime(const char* timeString) {
  int year, month, day, hour, min, sec;
  if (sscanf(timeString, "%d:%d:%d:%d:%d:%d", &year, &month, &day, &hour, &min, &sec) == 6) {
    RTC_CLOCK.adjust(DateTime(year, month, day, hour, min, sec));
    return true;
  } else {
    Serial.println("Failed to parse time string");
    return false;
  }
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void initClock(){
  RTC_CLOCK.begin();
  RTC_CLOCK.deconfigureAllTimers();
  RTC_CLOCK.adjust(DateTime(F(__DATE__), F(__TIME__))); // <-- Remove once completed
  RTC_CLOCK.start();
  /*
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // Following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  */
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*
  Garden functions
*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
int getMoistureSensorValue(int adcChannel){
  return map(ADC_EXPANDER.readADC(adcChannel), AIR_VALUE, WATER_VALUE, 0, 100);
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void UpdateGarden(){
  IO_EXPANDER.digitalWrite(UPDATING, HIGH);
  for (int i = 0; i < NUMBER_OF_PLANTS; i++){
    Plant* plant = getPlant(i);
    plant -> currentMoisture = getMoistureSensorValue(i);
    delay(INTER_DELAY);
  }
  IO_EXPANDER.digitalWrite(UPDATING, LOW);
  delay(LOOP_DELAY);
  return;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void WaterGarden(){
  IO_EXPANDER.digitalWrite(WATERING_GARDEN_LED, HIGH);
  for (int i = 0; i < NUMBER_OF_PLANTS; i++) {
    Plant* plant = getPlant(i);
    WaterPlant(plant);
    delay(INTER_DELAY);
  }
  IO_EXPANDER.digitalWrite(UPDATING, LOW);
  delay(LOOP_DELAY);
  return;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void WaterPlant(Plant * plant){
  if (plant == nullptr) {
    return;
  }
  DateTime lastWatered = plant->lastTimeWatered;
  unsigned long timeSinceLastWatering = NOW.unixtime() - lastWatered.unixtime();
  unsigned long wateringInterval = plant->daysToWater * 86400;  // Convert days to seconds
  if (timeSinceLastWatering >= wateringInterval && plant->currentMoisture < plant->moistureThreshold) {
    IO_EXPANDER.digitalWrite(NUMBER_OF_PLANTS, WATER_PUMP_ON);
    delay(PUMP_START_DELAY);
    yield();
    IO_EXPANDER.digitalWrite(plant->solenoid_gpio, SOLENOID_OPEN);
    delay(plant->timeWatering);
    yield();
    IO_EXPANDER.digitalWrite(plant->solenoid_gpio, SOLENOID_CLOSED);
    IO_EXPANDER.digitalWrite(NUMBER_OF_PLANTS, WATER_PUMP_OFF);
    plant->lastTimeWatered = NOW;
  }
  return;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void WaterAll(){
  IO_EXPANDER.digitalWrite(WATERING_GARDEN_LED, HIGH);
  for (int i = 0; i < NUMBER_OF_PLANTS; i++) {
    Plant* plant = getPlant(i);
    Water(plant);
    delay(INTER_DELAY);
  }
  IO_EXPANDER.digitalWrite(UPDATING, LOW);
  delay(LOOP_DELAY);
  return;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void Water(Plant * plant){
  if (plant == nullptr) {
    Serial.println("Error: plant is null");
    return;
  }
    IO_EXPANDER.digitalWrite(NUMBER_OF_PLANTS, WATER_PUMP_ON);
    delay(PUMP_START_DELAY);
    yield();
    IO_EXPANDER.digitalWrite(plant->solenoid_gpio, SOLENOID_OPEN);
    delay(plant->timeWatering);
    yield();
    IO_EXPANDER.digitalWrite(plant->solenoid_gpio, SOLENOID_CLOSED);
    IO_EXPANDER.digitalWrite(NUMBER_OF_PLANTS, WATER_PUMP_OFF);
    plant->lastTimeWatered = NOW;
  return;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*
                Plant Data Structure functions
*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
void initPlants(int numberOfPlants){
  IO_EXPANDER.pinMode(numberOfPlants, OUTPUT);  // top most gpio pin is pump output
  for(int i = 0; i < numberOfPlants; i++){
    char buffer[40];
    Plant plant = defaultPlant();
    appendPlant(i, plant);  // for 0 -> Number of plants - 1 indexed to the linked list
    IO_EXPANDER.pinMode(i, OUTPUT); // for 0 -> Number of plants - 1 equals expander output to solenoid
    delay(500);
  }
  return;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
Plant defaultPlant(){
  Plant p;
  strncpy(p.name, "default", sizeof(p.name));
  p.daysToWater = 1;
  p.moistureThreshold = 50;
  p.currentMoisture = 100;
  p.timeWatering = 5000;
  p.lastTimeWatered = NOW;
  return p;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void appendPlant(int index, Plant plant){
  Plant* newPlant = new Plant;
  newPlant->index = index;
  strncpy(newPlant->name, plant.name, sizeof(newPlant->name) - 1);
  newPlant->daysToWater = plant.daysToWater;
  newPlant->currentMoisture = plant.currentMoisture;
  newPlant->moistureThreshold = plant.moistureThreshold;
  newPlant->timeWatering = plant.timeWatering;
  newPlant->lastTimeWatered = plant.lastTimeWatered;
  newPlant->solenoid_gpio = index;
  newPlant->adc_channel = index;
  newPlant->next = NULL;

  if (head == NULL) {
    head = newPlant;
  } else {
    Plant* temp = head;
    while (temp->next != NULL) {
      temp = temp->next;
    }
    temp->next = newPlant;
  }
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void appendPlant(int index) {
  Plant* newPlant = new Plant;
  newPlant->index = index;
  newPlant->next = NULL;

  if (head == NULL) {
    head = newPlant;
  } else {
    Plant* temp = head;
    while (temp->next != NULL) {
      temp = temp->next;
    }
    temp->next = newPlant;
  }
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
Plant* getPlant(int searchIndex) {
    Plant* temp = head;
    while(temp != NULL) {
        if(temp->index == searchIndex) {
            return temp;  // Return pointer to plant
        }
        temp = temp->next;
    }
    return NULL; // plant not found
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
int getPlantData(int searchIndex, char * plantBuffer){
  Plant* plant = getPlant(searchIndex);
  if(plant){
    sprintf(plantBuffer, "%d:%s:%d:%d:%d:%d:%s", 
      plant->index, plant->name, plant->daysToWater, plant->currentMoisture, plant->moistureThreshold, plant->timeWatering, plant->lastTimeWatered);
    return 0;
  }
  return -1;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
int getPlantData_NoIndex(int searchIndex, char * plantBuffer){
  Plant* plant = getPlant(searchIndex);
  if(plant){
    sprintf(plantBuffer, "%s,%d,%d,%d,%d,%s", 
      plant->name, plant->daysToWater, plant->currentMoisture, plant->moistureThreshold, plant->timeWatering, plant->lastTimeWatered);
    return 0;
  }
  return -1;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
int getPlantUpdate_NoIndex(int searchIndex, char * plantBuffer){
  Plant* plant = getPlant(searchIndex);
  if(plant){
    sprintf(plantBuffer, "%d,%s", plant->currentMoisture, plant->lastTimeWatered);
    return 0;
  }
  return -1;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void clearPlants() {
  while (head != NULL) {
    Plant* temp = head;
    head = head->next;
    delete temp;
  }
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
int indexOf(const char * buffer, char c){
  char *p = strchr(buffer, c);
  if (p) {
    return p - buffer;
  } else {
    return -1;
  }
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
int indexOf(const char * buffer, char c, int startPos){
  char *startPtr = RX_BUFFER + startPos;
  char *p = strchr(buffer, c);
  if (p) {
    return p - buffer;
  } else {
    return -1;
  }
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*
                    Wifi and API Functions
*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*
                      Wifi Functionality
*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
void setupWifi(const char * ssid, const char * password) {
  WIFI_INIT = false;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  WIFI_INIT = true;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void setupWifi() {
  WIFI_INIT = false;
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  WIFI_INIT = true;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*
                    API Functionality
*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*              API Initialization Functionality               */
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
int initRobot(){
  int state = AddSmartPlanter_API();
  delay(INTER_DELAY);
  if(state < 0){ return state; }
  if(state == 0){
    int uploadPlanter = UpdateSmartPlanter_API();
    delay(INTER_DELAY);
    if(uploadPlanter < 0){
      return uploadPlanter;
    }
    for(int i = 0; i < NUMBER_OF_PLANTS; i++){
      int uploadPlant = UpdateSmartPlanterPlant_API(i);
      delay(INTER_DELAY);
      if(uploadPlant < 0){
        return uploadPlant; 
      }
    }
  }
  if(state == 1){
    int downloadPlanter = GetApiPlanterData();
    delay(INTER_DELAY);
    if(downloadPlanter < 0){
      return downloadPlanter;
    }
  for(int i = 0; i < NUMBER_OF_PLANTS; i++){
      Plant * plant = getPlant(i);
      if (plant == nullptr) {
        return -3999;
      }
      int downloadPlant = getPlantData_fromAPI(i, plant);
      delay(INTER_DELAY);
      if(downloadPlant < 0){
        return downloadPlant;
      }
    }
  }
  delay(LOOP_DELAY);
  return 0;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
int updatePlants() {
  for (int i = 0; i < NUMBER_OF_PLANTS; i++) {
    int uploadPlant = UpdateSmartPlanterPlant_API(i);
    delay(INTER_DELAY);
    if (uploadPlant < 0) {
      return uploadPlant;
    }
  }
  return 0;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-plant/{smartPlanterUniqueId}/{plantIndex}
int IntermittentPlantUpdate(int plantIndex){
  int retVal = -600;
  char url[MAX_BUFFER];
  char plantDataBuffer[MAX_BUFFER];
  int successPlant = getPlantUpdate_NoIndex(plantIndex, plantDataBuffer);
  if(successPlant < 0){ return retVal; }
  sprintf(url, "%s/update/smartplanter/set-plant-name/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(plantDataBuffer);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// connect/smartplanter
bool ConnectToSmartPlanter_API() {
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/connect/smartplanter", apiGardenEndpoint);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// add/smartplanter/{smartPlanterUniqueId}
int AddSmartPlanter_API() {
  int retVal = -1;
  char url[MAX_BUFFER];
  sprintf(url, "%s/add/smartplanter/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(UNIQUE_ID);  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();  // Get the response payload
    if(payload == "0"){ // SmartPlanter created in database
      return 0;
    }
    if(payload == "1"){ // SmartPlanter exists in database
      return 1;
    }
    retVal = true;
  } else {
    retVal = -1;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*                     Update API from ROBOT                   */
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-planter/{smartPlanterUniqueId}
int UpdateSmartPlanter_API(){
  int retVal = -100;
  char url[MAX_BUFFER];
  char robotBuffer[MAX_BUFFER];
  getSystemBuffer(robotBuffer);
  sprintf(url, "%s/update/smartplanter/set-planter", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.addHeader("Content-Type", "text/plain");
  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.POST(robotBuffer);  // Send the request
  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-name/{smartPlanterUniqueId}/{planterName}
int UpdateSmartPlanterName_API(){
  int retVal = -100;
  char url[MAX_BUFFER];
  sprintf(url, "%s/update/smartplanter/set-name/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(NAME);  // Send the request
  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-timezone/{smartPlanterUniqueId}/{planterTimezone}
int UpdateSmartPlanterTimeZone_API(){
  int retVal = -200;
  char url[MAX_BUFFER];
  sprintf(url, "%s/update/smartplanter/set-time/%s/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(TIME_ZONE);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-location/{smartPlanterUniqueId}/{location}
int UpdateSmartPlanterLocation_API(){
  int retVal = -300;
  char url[MAX_BUFFER];
  sprintf(url, "%s/update/smartplanter/set-location/%s/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(LOCATION);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-esp-voltage/{smartPlanterUniqueId}/{espVoltage}
int UpdateSmartPlanterESPVoltage_API(){
  int retVal = -400;
  char url[MAX_BUFFER];
  sprintf(url, "%s/update/smartplanter/set-esp-voltage/%s/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(ESP_VCC);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-number-of-plants/{smartPlanterUniqueId}/{numPlants}
int UpdateSmartPlanterNumberOfPlants_API(){
  int retVal = -500;
  char url[MAX_BUFFER];
  char intBuffer[10];
  sprintf(url, "%s/update/smartplanter/set-number-of-plants/%s/%s", apiGardenEndpoint, UNIQUE_ID);
  sprintf(intBuffer, "%d", NUMBER_OF_PLANTS);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(intBuffer);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/update-plant/{smartPlanterUniqueId}/{plantIndex}
int UpdateSmartPlanterPlant_API(int plantIndex){
  int retVal = -5000;
  char url[MAX_BUFFER];
  char plantDataBuffer[MAX_BUFFER];
  int successPlant = getPlantData_NoIndex(plantIndex, plantDataBuffer);
  if(successPlant < 0){ return retVal; }
  sprintf(url, "%s/update/smartplanter/set-plant-name/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(plantDataBuffer);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-plant-name/{smartPlanterUniqueId}/{plantIndex}/{plantName}
int UpdateSmartPlanterPlantName_API(int plantIndex, const char * plantName){
  int retVal = -600;
  char url[MAX_BUFFER];
  sprintf(url, "%s/update/smartplanter/set-plant-name/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(plantName);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-soil-moisture/{smartPlanterUniqueId}/{plantIndex}/{soilMoisture}
int UpdateSmartPlanterPlantSoilMoisture_API(int plantIndex, int currMoisture){
  int retVal = -700;
  char url[MAX_BUFFER];
  char intBuffer[10];
  sprintf(url, "%s/update/smartplanter/set-soil-moisture/%s/%d/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);
  sprintf(intBuffer, "%d", currMoisture);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(intBuffer);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-moisuture-threshold/{smartPlanterUniqueId}/{plantIndex}/{moistureThreshold}
int UpdateSmartPlanterPlantMoistureThreshold_API(int plantIndex, int moistureThresh){
  int retVal = -800;
  char url[MAX_BUFFER];
  char intBuffer[10];
  sprintf(url, "%s/update/smartplanter/set-moisuture-threshold/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);
  sprintf(intBuffer, "%d", moistureThresh);


  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(intBuffer);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-days-between-watering/{smartPlanterUniqueId}/{plantIndex}/{daysBetweenWatering}
int UpdateSmartPlanterPlantDaysBetweenWatering_API(int plantIndex, int dbw){
  int retVal = -900;
  char url[MAX_BUFFER];
  char intBuffer[10];
  sprintf(url, "%s/update/smartplanter/set-days-between-watering/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);
  sprintf(intBuffer, "%d", dbw);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(intBuffer);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-time-watering/{smartPlanterUniqueId}/{plantIndex}/{timeWatering}
int UpdateSmartPlanterPlantTimeWatering_API(int plantIndex, int timeWater){
  int retVal = -1000;
  char url[MAX_BUFFER];
  char intBuffer[10];
  sprintf(url, "%s/update/smartplanter/set-time-watering/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);
  sprintf(intBuffer, "%d", timeWater);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(intBuffer);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// update/smartplanter/set-last-time-watered/{smartPlanterUniqueId}/{plantIndex}/{lastTimeWatered}
int UpdateSmartPlanterPlantLastTimeWatered_API(int plantIndex, const char * ltw){
  int retVal = -1100;
  char url[MAX_BUFFER];
  char intBuffer[10];
  sprintf(url, "%s/update/smartplanter/set-last-time-watered/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);
  sprintf(intBuffer, "%d", ltw);

  _HTTP.begin(wifiClient, url);
  _HTTP.addHeader("Content-Type", "text/plain");
  int httpCode = _HTTP.POST(intBuffer);  // Send the request

  if (httpCode > 0) {
    retVal = abs(retVal);
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*                     Update Robot from API                   */
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-planter/{smartPlanterUniqueId}
bool GetSmartPlanter_API(char * plantData){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-planter/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(plantData, "%s",payload.c_str());
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
int GetApiPlanterData(){
  int retVal = -2000;
  char planterDataBuffer[MAX_BUFFER];
  bool success = GetSmartPlanter_API(planterDataBuffer);
  if(success == false){ return retVal; }

  // Check the UNIQUE_ID to the Id in the database
  char uniqueIdBuffer[50];
  size_t first_colon = indexOf(planterDataBuffer,':');
  substrBuffer(planterDataBuffer, uniqueIdBuffer, 0, first_colon - 1);
  if(strcmp(uniqueIdBuffer,UNIQUE_ID)!=0){ return retVal - 100; }

  // If the Id's are equal parse the rest of the string
  char nameBuffer[50];
  char locationBuffer[50];
  char timeZoneBuffer[50];
  size_t second_colon = indexOf(planterDataBuffer,':',first_colon + 1);
  size_t third_colon = indexOf(planterDataBuffer,':',second_colon + 1);
  substrBuffer(planterDataBuffer, nameBuffer, first_colon + 1, second_colon - 1);
  substrBuffer(planterDataBuffer, locationBuffer, second_colon + 1, third_colon - 1);
  substrBuffer(planterDataBuffer, timeZoneBuffer, third_colon + 1, strlen(planterDataBuffer));

  if(second_colon < 0 || third_colon < 0){ return retVal - 200; }  // Could not find the other colons
  strcpy(NAME, nameBuffer);
  strcpy(LOCATION, locationBuffer);
  strcpy(TIME_ZONE, timeZoneBuffer);

  return abs(retVal);
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-planter-name/{smartPlanterUniqueId}
bool GetSmartPlanterName_API(){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-planter-name/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(NAME, "%s",payload.c_str());
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-planter-location/{smartPlanterUniqueId}
bool GetSmartPlanterLocation_API(){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-planter-location/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(LOCATION, "%s",payload.c_str());
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-planter-timezone/{smartPlanterUniqueId}
bool GetSmartPlanterTimeZone_API(){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-planter-timezone/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(TIME_ZONE, "%s",payload.c_str());
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-planter-time/{smartPlanterUniqueId}
bool GetSmartPlanterTime_API(){
  bool retVal = false;
  char url[MAX_BUFFER];
  char timeBuffer[100];
  sprintf(url, "%s/get/smartplanter/get-planter-time/%s", apiGardenEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(timeBuffer, "%s",payload.c_str());
    if(setRTCTime(timeBuffer) == true){
      retVal = true;
    }
    return false;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-plant/{smartPlanterUniqueId}/{plantIndex}
bool GetSmartPlanterPlant_API(int plantIndex, char * plantDataBuffer){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-plant-name/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(plantDataBuffer, "%s",payload.c_str());
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-plant/{smartPlanterUniqueId}/{plantIndex}
int getPlantData_fromAPI(int index, Plant* plantData){
  int retVal = -3000;
  char plantDataBuffer[MAX_BUFFER];
  bool success = GetSmartPlanter_API(plantDataBuffer);
  if(success == false){ return retVal; }

  // Compare the Index to the index being called in the database
  char indexBuffer[50];
  size_t first_colon = indexOf(plantDataBuffer,':');
  substrBuffer(plantDataBuffer, indexBuffer, 0, first_colon - 1);
  int convertedIndex = atoi(indexBuffer);
  if(convertedIndex != index){ return retVal - 100; }

  // If the index's are equal parse the rest of the string
  char nameBuffer[50];
  char moistureThresholdBuffer[10];
  char daysBetweenWateringBuffer[10];
  char timeWateringBuffer[10];
  char lastTimeWateredBuffer[10];

  size_t second_colon = indexOf (plantDataBuffer, ':', first_colon   + 1);
  size_t third_colon = indexOf  (plantDataBuffer, ':', second_colon  + 1);
  size_t fourth_colon = indexOf (plantDataBuffer, ':', third_colon   + 1);
  size_t fifth_colon = indexOf  (plantDataBuffer, ':', fourth_colon  + 1);

  substrBuffer(plantDataBuffer, nameBuffer, first_colon + 1, second_colon - 1);
  substrBuffer(plantDataBuffer, moistureThresholdBuffer, second_colon + 1, third_colon - 1);
  substrBuffer(plantDataBuffer, daysBetweenWateringBuffer, third_colon + 1, fourth_colon - 1);
  substrBuffer(plantDataBuffer, timeWateringBuffer, fourth_colon + 1, fifth_colon - 1);
  substrBuffer(plantDataBuffer, lastTimeWateredBuffer, fifth_colon + 1, strlen(plantDataBuffer));

  if(second_colon < 0 || third_colon < 0 || fourth_colon < 0 || fifth_colon < 0 ){ return retVal - 200; }  // Could not find the other colons

  strcpy(plantData -> name, nameBuffer);
  plantData -> moistureThreshold = atoi(moistureThresholdBuffer);
  plantData -> daysToWater = atoi(daysBetweenWateringBuffer);
  plantData -> timeWatering = atoi(timeWateringBuffer);
  DateTime time = parseDateTime(lastTimeWateredBuffer);
  if(time.year() == 0){ return retVal - 300; }
  plantData -> lastTimeWatered = parseDateTime(lastTimeWateredBuffer);
  return abs(retVal);
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
DateTime parseDateTime(const char* timeString){
  int year, month, day, hour, min, sec;
  if (sscanf(timeString, "%d:%d:%d:%d:%d:%d", &year, &month, &day, &hour, &min, &sec) == 6) {
    return DateTime(year, month, day, hour, min, sec);
  } else {
    return DateTime(0,0,0,0,0,0);
  }
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-plant-name/{smartPlanterUniqueId}/{plantIndex}
bool GetSmartPlanterPlantName_API(int plantIndex, char * plantNamePointer){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-plant-name/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(plantNamePointer, "%s",payload.c_str());
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-moisuture-threshold/{smartPlanterUniqueId}/{plantIndex}
bool GetSmartPlanterPlantMoistureThreshold_API(int plantIndex, int * plantMoistureThreshPointer){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-moisuture-threshold/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    *plantMoistureThreshPointer = payload.toInt();
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-days-between-watering/{smartPlanterUniqueId}/{plantIndex}
bool GetSmartPlanterPlantDaysBetweenWatering_API(int plantIndex, int * plantDbwPointer){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-moisuture-threshold/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    *plantDbwPointer = payload.toInt();
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-time-watering/{smartPlanterUniqueId}/{plantIndex}
bool GetSmartPlanterPlantTimeWatering_API(int plantIndex, int * plantTimeWateringPointer){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-moisuture-threshold/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    *plantTimeWateringPointer = payload.toInt();
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get/smartplanter/get-last-time-watered/{smartPlanterUniqueId}/{plantIndex}
bool GetSmartPlanterPlantLastTimeWatered_API(int plantIndex, char * plantLastTimeWateredPointer){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get/smartplanter/get-moisuture-threshold/%s/%d", apiGardenEndpoint, UNIQUE_ID, plantIndex);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(plantLastTimeWateredPointer, "%s",payload.c_str());
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/


/*-------------------------------------------------------------*/
/*                       Robot Command API                     */
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
void GetCommand(){
  char commandBuffer[MAX_BUFFER];
  if(GetSmartPlanterCommand_API(commandBuffer) == true){
    char commandId[10];
    char command[50];
    char commandData[50];
    size_t first_colon = indexOf(commandBuffer,':');
    size_t second_colon = indexOf(commandBuffer,':',first_colon + 1);
    substrBuffer(commandBuffer, commandId, 0, first_colon - 1);
    substrBuffer(commandBuffer, command, first_colon + 1, second_colon-1);
    substrBuffer(commandBuffer, commandData, second_colon+1, strlen(commandBuffer));

    int success = initCommand(command, commandData);
    if(success < 0){
      AcknowledgeSmartPlanterCommand_API(atoi(commandId), false);
    } else {
      AcknowledgeSmartPlanterCommand_API(atoi(commandId), true);
    }
    return;
  }
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// get-command/{SmartPlanterUniqueId} 
bool GetSmartPlanterCommand_API(char * command){
  bool retVal = false;
  char url[MAX_BUFFER];
  sprintf(url, "%s/get-command/%s", apiCommandEndpoint, UNIQUE_ID);

  _HTTP.begin(wifiClient, url);
  int httpCode = _HTTP.GET();  // Send the request

  if (httpCode > 0) {
    String payload = _HTTP.getString();
    sprintf(command, "%s",payload.c_str());
    retVal = true;
  } else {
    retVal = false;
  }
  _HTTP.end();  // Close connection
  return retVal;
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// set-planter-acknowledge-command/{smartPlanterUniqueId}/{commandId}
void AcknowledgeSmartPlanterCommand_API(int commandId, bool success){
  char url[MAX_BUFFER];
  char response[MAX_BUFFER];
  if(success == true){
    sprintf(response, "success");
  } else {
    sprintf(response, "error");
  }
  sprintf(url, "%s/set-planter-acknowledge-command/%s/%d?acknowledge=%s", apiCommandEndpoint, UNIQUE_ID, commandId,response);
  _HTTP.begin(wifiClient, url);
  _HTTP.end();
}
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
// State Machine for Commanding from API
bool initCommand(const char * command, const char * commandData){
  if(strcmp(getName_Command_API,command)==0){
    strcpy(NAME, commandData);
    return 0;
  }
  else if(strcmp(getLocation_Command_API,command)==0){
    strcpy(LOCATION, commandData);
    return 0;
  }
  else if(strcmp(getTimeZone_Command_API,command)==0){
    strcpy(TIME_ZONE, commandData);
    return 0;
  }
  else if(strcmp(getTime_Command_API,command)==0){
    if( setRTCTime(commandData) != true){
      return -1;
    }
    return 0;
  }
  else if(strcmp(getWaterAllPlants_Command_API,command)==0){
    return 0;
  }
  else if(strcmp(getWaterPlant_Command_API,command)==0){
    return 0;
  }
  else if(strcmp(getPlantName_Command_API,command)==0){
    return 0;
  }
  else if(strcmp(getMoistureThreshold_Command_API,command)==0){
    return 0;
  }
  else if(strcmp(getDaysBetweenWatering_Command_API,command)==0){
    return 0;
  }
  else if(strcmp(getTimeWatering_Command_API,command)==0){
    return 0;
  }
  return -1;
}
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
//  Dead Code
/*-------------------------------------------------------------*/
/*
void appendPlant(int index, const char* name, int daysToWater, int currentMoisture, int moistureThreshold, int timeWatering, DateTime lastTimeWatered, int solenoid_gpio, int adc_channel);
void updatePlant(Plant* plant, const char* name, int daysToWater, int currentMoisture, int moistureThreshold, int timeWatering, DateTime lastTimeWatered);
void updatePlant(Plant* plant, Plant update);

void appendPlant(int index, const char* name, int daysToWater, int currentMoisture, int moistureThreshold, int timeWatering, DateTime lastTimeWatered, int solenoid_gpio, int adc_channel) {
  Plant* newPlant = new Plant;
  newPlant->index = index;
  strncpy(newPlant->name, name, sizeof(newPlant->name) - 1);
  newPlant->daysToWater = daysToWater;
  newPlant->currentMoisture = currentMoisture;
  newPlant->moistureThreshold = moistureThreshold;
  newPlant->timeWatering = timeWatering;
  newPlant->lastTimeWatered = lastTimeWatered;
  newPlant->solenoid_gpio = solenoid_gpio;
  newPlant->adc_channel = adc_channel;
  newPlant->next = NULL;

  if (head == NULL) {
    head = newPlant;
  } else {
    Plant* temp = head;
    while (temp->next != NULL) {
      temp = temp->next;
    }
    temp->next = newPlant;
  }
}
void updatePlant(Plant* plant, const char* name, int daysToWater, int currentMoisture, int moistureThreshold, int timeWatering, DateTime lastTimeWatered) {
  strncpy(plant->name, name, sizeof(plant->name) - 1);
  plant->daysToWater = daysToWater;
  plant->currentMoisture = currentMoisture;
  plant->moistureThreshold = moistureThreshold;
  plant->timeWatering = timeWatering;
  plant->lastTimeWatered = lastTimeWatered;
}
void updatePlant(Plant* plant, Plant update) {
  strncpy(plant->name, update.name, sizeof(plant->name) - 1);
  plant->daysToWater = update.daysToWater;
  plant->currentMoisture = update.currentMoisture;
  plant->moistureThreshold = update.moistureThreshold;
  plant->timeWatering = update.timeWatering;
  plant->lastTimeWatered = update.lastTimeWatered;
}
*/
