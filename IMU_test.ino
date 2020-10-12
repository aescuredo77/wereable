#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h> //IMU

float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;
const int BUFFER_SIZE = 64;
char msgprint[BUFFER_SIZE];


const char* uuid_service = "00001101-0000-1000-8000-00805f9b34fb";
const char* uuid_string = "00001143-0000-1000-8000-00805f9b34fb";
const char* uuid_char = "00001150-0000-1000-8000-00805f9b34fb";

BLEService customService(uuid_service);
BLECharacteristic Send_string(uuid_string, BLERead | BLENotify , BUFFER_SIZE, false);
BLECharCharacteristic led_control(uuid_char, BLERead | BLEWrite);// 0,1

void setup() {
  Serial.begin(9600); //to debug
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("IMU_test"); // name of Bluetooth
  BLE.setAdvertisedService(customService.uuid());
  customService.addCharacteristic(Send_string);
  customService.addCharacteristic(led_control);
  //// initial values////
  led_control.setValue(0);

  // Event to write
  led_control.setEventHandler(BLEWritten, led_Update);
  BLE.setEventHandler(BLEConnected, onBLEConnected);
  BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

  BLE.addService(customService);
  BLE.advertise();
  Serial.println(BLE.address());
  pinMode(13, OUTPUT);

}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    while (central.connected()) {
      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
        IMU.readAcceleration(Ax, Ay, Az);
        IMU.readGyroscope(Gx, Gy, Gz);
        IMU.readMagneticField(Mx, My, Mz);
        sprintf(msgprint, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz);
        Serial.println(msgprint);
        Send_string.writeValue(msgprint, sizeof(msgprint));
      
    }
  }
}
}

void led_Update(BLEDevice central, BLECharacteristic characteristic) {
  char aux = led_control.value();
  if (aux == '1') digitalWrite(13, HIGH); // '1' o 0x31
  else  digitalWrite(13, LOW);
}

void onBLEConnected(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void onBLEDisconnected(BLEDevice central) {
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}
