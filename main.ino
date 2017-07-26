#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char *ssid = "*******";
const char *password = "********";

const uint8_t MAX_LX_DEVICES = 16;
uint8_t LX_DEVICES = 0;

const uint8_t SIZE_OF_MAC = 6;

byte lxDevices[MAX_LX_DEVICES][SIZE_OF_MAC];
IPAddress lxDevicesAddr[MAX_LX_DEVICES];

int lxPort = 56700;

IPAddress bcastAddr(192, 168, 0, 255);
IPAddress ip(192, 168, 0, 73);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiUDP UDP;

uint8_t buttonPushed = 0;

#pragma pack(push, 1)
typedef struct {
  /* frame */
  uint16_t size;
  uint16_t protocol : 12;
  uint8_t addressable : 1;
  uint8_t tagged : 1;
  uint8_t origin : 2;
  uint32_t source;
  /* frame address */
  uint8_t target[8];
  uint8_t reserved[6];
  uint8_t res_required : 1;
  uint8_t ack_required : 1;
  uint8_t : 6;
  uint8_t sequence;
  /* protocol header */
  uint64_t : 64;
  uint16_t type;
  uint16_t : 16;
  /* variable length payload follows */
} lx_protocol_header_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  /* set power */
  uint16_t level;
  uint32_t duration;
} lx_set_power_t;
#pragma pack(pop)

// Device::StatePower Payload
#pragma pack(push, 1)
typedef struct {
  uint16_t level;
} lx_state_power_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  uint8_t reserved;
  uint16_t hue;
  uint16_t saturation;
  uint16_t brightness;
  uint16_t kelvin;
  uint32_t duration;
} lx_set_color_t;
#pragma pack(pop)


// Payload types
#define LIFX_DEVICE_GETPOWER 20
#define LIFX_DEVICE_SETPOWER 21
#define LIFX_DEVICE_STATEPOWER 22
#define LIFX_DEVICE_SETCOLOR 102

unsigned long timeoutInterval = 500;
byte packetBuffer[128];


void lxMakeFrame(lx_protocol_header_t *lxHead, uint8_t extraSize, uint8_t tagged, uint8_t *target, uint16_t message) {
  /* frame */
  lxHead->size = (uint8_t)36 + extraSize;
  lxHead->protocol = (uint16_t)1024;
  lxHead->addressable = (uint8_t)1;
  lxHead->tagged = tagged;
  lxHead->origin = (uint8_t)0;
  lxHead->source = (uint32_t)3549;

  /* frame address */
  uint8_t i = 0;
  for (i = 0; i < 8; i++) {
    lxHead->target[i] = (uint8_t)target[i];
  }
  lxHead->res_required = (uint8_t)0;
  lxHead->ack_required = (uint8_t)0;
  lxHead->sequence = (uint8_t)0;

  /* protocol header */
  lxHead->type = message;
}


void lxDiscovery() {
  /* Build lxDiscovery payload */
  byte target_addr[8] = {0};
  lx_protocol_header_t *lxHead;
  lxHead = (lx_protocol_header_t *)calloc(1, sizeof(lx_protocol_header_t));
  lxMakeFrame(lxHead, 0, 1, target_addr, 2);

  /* Start listening for responses */
  delay(500);
  /* Send a couple of discovery packets out to the network*/
  for (int i = 0; i < 1; i++) {
    byte *b = (byte *)lxHead;
    UDP.beginPacket(bcastAddr, lxPort);
    UDP.write(b, sizeof(lx_protocol_header_t));
    UDP.endPacket();

    delay(500);
  }

  free(lxHead);

  for (int j = 0; j < 20; j++) {

    int sizePacket = UDP.parsePacket();
    if (sizePacket) {
      UDP.read(packetBuffer, sizePacket);
      byte target_addr[SIZE_OF_MAC];
      memcpy(target_addr, packetBuffer + 8, SIZE_OF_MAC);
      // Print MAC
      for (int i = 1; i <= sizeof(target_addr); i++) {
        Serial.print(target_addr[i - 1], HEX);
      }
      Serial.println();

      // Check if this device is new
      uint8_t previouslyKnownDevice = 0;
      for (int i = 0; i < LX_DEVICES; i++) {
        if (!memcmp(lxDevices[i], target_addr, SIZE_OF_MAC)) {
          Serial.println("Previously seen target");
          previouslyKnownDevice = 1;
          break;
        }
      }
      // Store new devices
      if (!previouslyKnownDevice) {

        lxDevicesAddr[LX_DEVICES] = (uint32_t)UDP.remoteIP();

        Serial.println(UDP.remoteIP());
        memcpy(lxDevices[LX_DEVICES++], target_addr, SIZE_OF_MAC);
        Serial.print("Storing device as LX_DEVICE ");
        Serial.println(LX_DEVICES);
      }
    }
    delay(20);
  }
}

void SetColor(uint8_t *target_addr) {

    /* Build payload */
    uint32_t duration = 500;

    lx_protocol_header_t *lxHead;
    lxHead = (lx_protocol_header_t *)calloc(1, sizeof(lx_protocol_header_t));
    lxMakeFrame(lxHead, sizeof(lx_set_color_t), 0, target_addr, LIFX_DEVICE_SETCOLOR);

    lx_set_color_t *lxSetColor;
    lxSetColor = (lx_set_color_t *)calloc(1, sizeof(lx_set_color_t));
    lxSetColor->duration = 500;
    lxSetColor->hue = 0;
    lxSetColor->saturation = 0;
    lxSetColor->brightness = 65535;
    lxSetColor->kelvin = 2500;

    UDP.beginPacket(bcastAddr, lxPort);
    byte *b = (byte *)lxHead;
    UDP.write(b, sizeof(lx_protocol_header_t));
    b = (byte *)lxSetColor;
    UDP.write(b, sizeof(lx_set_color_t));
    UDP.endPacket();

    free(lxSetColor);
    free(lxHead);
}

void SetPower(uint8_t *target_addr, uint16_t level) {

    /* Build payload */
    uint32_t duration = 500;

    lx_protocol_header_t *lxHead;
    lxHead = (lx_protocol_header_t *)calloc(1, sizeof(lx_protocol_header_t));
    lxMakeFrame(lxHead, sizeof(lx_set_power_t), 0, target_addr, LIFX_DEVICE_SETPOWER);

    lx_set_power_t *lxSetPower;
    lxSetPower = (lx_set_power_t *)calloc(1, sizeof(lx_set_power_t));
    lxSetPower->duration = 500;
    lxSetPower->level = level;

    UDP.beginPacket(bcastAddr, lxPort);
    byte *b = (byte *)lxHead;
    UDP.write(b, sizeof(lx_protocol_header_t));
    b = (byte *)lxSetPower;
    UDP.write(b, sizeof(lx_set_power_t));
    UDP.endPacket();

    free(lxSetPower);
    free(lxHead);
}

uint16_t GetPower(uint8_t *target_addr) {

  uint16_t power = 0;

  lx_protocol_header_t *lxHead;
  lxHead = (lx_protocol_header_t *)calloc(1, sizeof(lx_protocol_header_t));
  lxMakeFrame(lxHead, 0, 0, target_addr, LIFX_DEVICE_GETPOWER);

  UDP.beginPacket(bcastAddr, lxPort);
  byte *b = (byte *)lxHead;
  UDP.write(b, sizeof(lx_protocol_header_t));
  UDP.endPacket();

  free(lxHead);

  unsigned long started = millis();
  while (millis() - started < timeoutInterval) {
    int packetLen = UDP.parsePacket();
    if (packetLen) {
      UDP.read(packetBuffer, sizeof(packetBuffer));

      if (((lx_protocol_header_t *)packetBuffer)->type == LIFX_DEVICE_STATEPOWER) {
        power = ((lx_state_power_t *)(packetBuffer + sizeof(lx_protocol_header_t)))->level;
        return power;
      } else {
        Serial.print("Unexpected Packet type: ");
        Serial.println(((lx_protocol_header_t *)packetBuffer)->type);
      }
    }
  }
  return power;
}

void isr_p2() {
  buttonPushed = 1;
}

void startWifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.persistent(false);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Wifi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void startUDP() {
  UDP.begin(4097);
  Serial.println("Local port:\t");
  Serial.println(UDP.localPort());
  Serial.println();
}
void setup() {

  /* fill with zeros */
  uint8_t zeros[SIZE_OF_MAC] = {0};
  for (int i = 0; i < MAX_LX_DEVICES; i++) {
    memcpy(lxDevices[i], zeros, SIZE_OF_MAC);
  }

  Serial.begin(115200);

  startWifi();
  startUDP();

  //lxDiscovery();

  /* Setup Interupts */
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), isr_p2, FALLING);
}



void loop() {

  if (buttonPushed) {
    uint8_t dest[] = {0xd0, 0x73, 0xd5, 0x22, 0xc5, 0x89};
    buttonPushed = 0;

    if (GetPower(dest)) {
      SetPower(dest, 0);
    } else {
      SetColor(dest);
      SetPower(dest, 65534);
    }
  }
  delay(50);
}
