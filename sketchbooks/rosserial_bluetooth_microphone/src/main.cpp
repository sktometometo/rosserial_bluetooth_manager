#define ROSSERIAL_ARDUINO_BLUETOOTH

#include <M5Stack.h>
#include <driver/i2s.h>
#include <ros.h>
#include <audio_common_msgs/AudioData.h>
#include <audio_common_msgs/AudioInfo.h>


#define PIN_CLK  22
#define PIN_DATA 21

#define I2S_BUFFER_SIZE 512

const char sample_format[] = "S16LE";
const char coding_format[] = "WAVE";

int16_t i2s_samples[I2S_BUFFER_SIZE];

ros::NodeHandle nh;
audio_common_msgs::AudioData msg_audio_data;
audio_common_msgs::AudioInfo msg_audio_info;
ros::Publisher publisher_audio_data("~output/data", &msg_audio_data);
ros::Publisher publisher_audio_info("~output/info", &msg_audio_info);


void setup() {
  M5.begin(true, true, true, true);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.fillRect(0, 0, 320, 30, BLACK);
  M5.Lcd.setTextDatum(TC_DATUM);
  M5.Lcd.drawString("PDM Unit", 160, 3, 4); 

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
      .bck_io_num   = I2S_PIN_NO_CHANGE,
      .ws_io_num    = PIN_CLK,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num  = PIN_DATA,
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  nh.initNode("ROSSERIAL_BT_MIC");
  nh.advertise(publisher_audio_data);
  nh.advertise(publisher_audio_info);

  msg_audio_info.channels = 1;
  msg_audio_info.sample_rate = 16000;
  msg_audio_info.sample_format = sample_format;
  msg_audio_info.coding_format = coding_format;

  delay(1000);
}

void read_and_publish() {
    size_t bytes_read;
    i2s_read(I2S_NUM_0, (void *)i2s_samples, sizeof(i2s_samples), &bytes_read, portMAX_DELAY);
    msg_audio_data.data_length = bytes_read * 2;
    msg_audio_data.data = (uint8_t *)i2s_samples;
    publisher_audio_data.publish(&msg_audio_data);
    publisher_audio_info.publish(&msg_audio_info);
}

void loop() {
    read_and_publish();
}
