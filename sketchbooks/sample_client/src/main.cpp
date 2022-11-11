#define ROSSERIAL_ARDUINO_BLUETOOTH

#include <M5Stack.h>
#include <string.h>
#include <ros.h>
#include <std_msgs/String.h>

#define BUFFER_SIZE 256

void messageCb(const std_msgs::String& msg);

char default_str[] = "Hellow, world!";
char buffer[256];
uint8_t bt_address[6];
char bt_address_str[18] = {0};
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher publisher("output", &str_msg);
ros::Subscriber<std_msgs::String> subscriber("input", &messageCb);

const int POINT_X_FOR_BT_STATUS = 0;
const int POINT_Y_FOR_BT_STATUS = 40;
const int RANGE_X_FOR_BT_STATUS = 320;
const int RANGE_Y_FOR_BT_STATUS = 20;

const int POINT_X_FOR_BUFFER = 0;
const int POINT_Y_FOR_BUFFER = 60;
const int RANGE_X_FOR_BUFFER = 320;
const int RANGE_Y_FOR_BUFFER = 40;

void messageCb(const std_msgs::String& msg)
{
  snprintf(buffer, BUFFER_SIZE, "%s", msg.data);
}

void setup()
{
  strcpy(buffer, default_str);

  esp_read_mac(bt_address, ESP_MAC_BT);
  sprintf(bt_address_str,
          "%02X:%02X:%02X:%02X:%02X:%02X",
          bt_address[0],
          bt_address[1],
          bt_address[2],
          bt_address[3],
          bt_address[4],
          bt_address[5]);

  M5.begin();
  M5.Lcd.begin();

  nh.initNode("ROSSERIAL_BT_SAMPLE");
  nh.advertise(publisher);
  nh.subscribe(subscriber);

  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setTextSize(2);

  M5.Lcd.print("BT rosserial sample\n");
  M5.Lcd.printf("BT: %s\n", bt_address_str);
}

void clear_bt_status_print()
{
    M5.Lcd.fillRect(
            POINT_X_FOR_BT_STATUS,
            POINT_Y_FOR_BT_STATUS,
            RANGE_X_FOR_BT_STATUS,
            RANGE_Y_FOR_BT_STATUS,
            WHITE);
}

void update_buffer_print()
{
    M5.Lcd.fillRect(
            POINT_X_FOR_BUFFER,
            POINT_Y_FOR_BUFFER,
            RANGE_X_FOR_BUFFER,
            RANGE_Y_FOR_BUFFER,
            WHITE);
    M5.Lcd.setCursor(
            POINT_X_FOR_BUFFER,
            POINT_Y_FOR_BUFFER
            );
    M5.Lcd.setTextSize(1);
    M5.Lcd.printf("buffer: %s", buffer);
}

void loop()
{
  if (not nh.connected()) {
    M5.Lcd.setTextSize(2);
    clear_bt_status_print();
    while (not nh.connected()) {
      update_buffer_print();
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(
              POINT_X_FOR_BT_STATUS,
              POINT_Y_FOR_BT_STATUS
              );
      nh.spinOnce();
      delay(500);
      M5.Lcd.print(".");
      nh.spinOnce();
      delay(500);
      M5.Lcd.print(".");
      nh.spinOnce();
      delay(500);
      clear_bt_status_print();
    }
    M5.Lcd.setCursor(
            POINT_X_FOR_BT_STATUS,
            POINT_Y_FOR_BT_STATUS
            );
    M5.Lcd.print("Bluetooth Connected!");
  } else {
    update_buffer_print();
  }
  str_msg.data = buffer;
  publisher.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
