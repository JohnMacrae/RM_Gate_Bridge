#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include <Wire.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#define DEFAULT_POWER_MODE false
//#define ESP_RMAKER_DEF_POWER_NAME  GARAGE

static int gpio_gates = 12;
bool gates_state = false;

/*
    Pins
*/
//GPIO for push button reset
static int gpio_0 = 0;
static Device my_gates("Gates", "my.device.gate" , &gpio_gates);

/*
   Provisioning Service Name
*/
const char *service_name = "PROV_1234";
const char *pop = "abcd1234";

#include <AsyncMqttClient.h>
#define MQTT_HOST IPAddress(192, 168, 199, 8)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();
  //Note: These parameters are there just to give a starting point if you wanted to make this into a switching thermometer
  Serial.printf("Received value = %d for %s - param: %s\n", val.val.b, device_name, param_name);

  if (strcmp(param_name, "Power") == 0) {
    Serial.printf("Received value = %d for %s - param: %s\n", val.val.b, device_name, param_name);
    gates_state = val.val.b;
    if (gates_state)
    {
      digitalWrite(gpio_gates, HIGH);
      mqttClient.publish("gates_state", 1, true, "true");
      vTaskDelay(1000);
      digitalWrite(gpio_gates, LOW);
      mqttClient.publish("gates_state", 1, true, "false");
    } else
    {
      digitalWrite(gpio_gates, LOW);
      mqttClient.publish("gates_state", 1, true, "false");
    }
  }
}

bool strToBool(char* str)
{
  bool res = false;
  if (strcmp( "ON", str) == 0) {
    res = true;
  }
  return res;
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("stat/ts_gate_state/POWER", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  packetIdSub = mqttClient.subscribe("stat/ts_gate_state/RESULT", 2);

  mqttClient.publish("temperature", 0, true, "tempMsg");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  //WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  len: ");
  Serial.println(len);
  if (strcmp(topic, "stat/ts_gate_state/POWER") == 0) {
    Serial.println("Matched");
    char msg[50] = "";
    strncpy ( msg, payload, len) ;
    msg[sizeof(payload)] = '\0';
    Serial.printf ("rx_msg: %s\n", msg);
   
    my_gates.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, strToBool(msg));
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(gpio_0, INPUT);
  pinMode(gpio_gates, OUTPUT);
  digitalWrite(gpio_gates, LOW);
  delay (1000);
  digitalWrite(gpio_gates, HIGH);
  delay (1000);
  digitalWrite(gpio_gates, LOW);
  /*
       Create the Node
  */
  Node my_node;
  my_node = RMaker.initNode("Gate Opener");
  /*
    Add custom gates device to the node
  */
  my_node.addDevice(my_gates);
  /*
     Now point at the callback code so we know what to do when something happens
  */
  my_gates.addCb(write_callback);

  my_gates.addNameParam();
  my_gates.addPowerParam(DEFAULT_POWER_MODE);
  my_gates.assignPrimaryParam(my_gates.getParamByName(ESP_RMAKER_DEF_POWER_NAME));

  /*Param my_mode("Mode", "esp.param.mode", value("closed"), PROP_FLAG_READ | PROP_FLAG_WRITE);
  static const char *valid_strs[] = {"Auto", "Turbo", "Sleep"};
  my_mode.addUIType(ESP_RMAKER_UI_DROPDOWN);
  my_gates.addParam(my_mode);
  */

  /*
      Start the Rainmaker Service
  */

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials("jramacrae", "jram7757");

  RMaker.start();

  /*
    Provisioning via BLE or WiFI
  */
  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

  mqttClient.publish("temperature", 0, true, "tempMsg");

}

void loop()
{

  if (digitalRead(gpio_0) == LOW) { //Push button pressed

    // Key debounce handling
    vTaskDelay(100);
    Serial.printf("Pressed");
    int startTime = millis();
    while (digitalRead(gpio_0) == LOW) delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000) {
      // If key pressed for more than 10secs, reset all
      Serial.printf("Reset to factory.\n");
      RMakerFactoryReset(2);
    } else if ((endTime - startTime) > 3000) {
      Serial.printf("Reset Wi - Fi.\n");
      // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
      RMakerWiFiReset(2);
    }
  }
}
