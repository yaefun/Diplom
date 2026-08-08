/*
  ESP32-CAM Object Detection System

  Board:
  - AI Thinker ESP32-CAM
  - OV2640 camera

  Sensor:
  - HC-SR04 ultrasonic distance sensor

  Storage:
  - MicroSD card using SD_MMC in 1-bit mode

  Logic:
  - Object farther than 50 cm -> system waits
  - Object closer than 50 cm -> object tracking starts
  - Object leaves before 5 seconds -> one photo is saved
  - Object remains for 5 seconds -> 25 JPEG frames are saved
  - System waits until the object leaves before detecting a new event

  HC-SR04:
  TRIG -> GPIO12
  ECHO -> GPIO13

  IMPORTANT:
  ESP32-CAM AI Thinker works with SD_MMC in 1-bit mode
  so GPIO12 and GPIO13 remain available for the ultrasonic sensor.
*/

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// ============================================================
// CAMERA PINS - AI THINKER ESP32-CAM
// ============================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============================================================
// HARDWARE PINS
// ============================================================

#define TRIG_PIN          12
#define ECHO_PIN          13

// Built-in flash LED on AI Thinker ESP32-CAM
#define FLASH_LED_PIN      4

// ============================================================
// SYSTEM SETTINGS
// ============================================================

const float OBJECT_DISTANCE_CM = 50.0;

const unsigned long VIDEO_START_TIME = 5000;
const unsigned long VIDEO_FRAME_INTERVAL = 200;

const int VIDEO_FRAME_COUNT = 25;

const unsigned long SENSOR_INTERVAL = 100;

// ============================================================
// STATE MACHINE
// ============================================================

enum SystemState
{
  WAITING_FOR_OBJECT,
  TRACKING_OBJECT,
  WAITING_FOR_OBJECT_TO_LEAVE
};

SystemState currentState = WAITING_FOR_OBJECT;

// ============================================================
// VARIABLES
// ============================================================

unsigned long objectStartTime = 0;
unsigned long lastSensorRead = 0;

bool videoCaptured = false;

int photoNumber = 1;
int videoNumber = 1;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

float readDistance();

bool initCamera();
bool initSDCard();

bool capturePhoto(const String &filename);
bool saveDistance(const String &filename, float distance);

String getNextPhotoFilename();
String getNextVideoFolder();

void createPhotoEvent(float distance);
void createVideoEvent();

void flashOn();
void flashOff();

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("ESP32-CAM Object Detection System");
  Serial.println("======================================");

  // Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  // Flash LED
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Initialize camera
  if (!initCamera())
  {
    Serial.println("ERROR: Camera initialization failed.");
    while (true)
    {
      delay(1000);
    }
  }

  Serial.println("Camera initialized.");

  // Initialize SD card
  if (!initSDCard())
  {
    Serial.println("ERROR: SD card initialization failed.");
    while (true)
    {
      delay(1000);
    }
  }

  Serial.println("SD card initialized.");

  Serial.println();
  Serial.println("System ready.");
  Serial.println("Waiting for an object...");
  Serial.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  unsigned long currentTime = millis();

  if (currentTime - lastSensorRead < SENSOR_INTERVAL)
  {
    return;
  }

  lastSensorRead = currentTime;

  float distance = readDistance();

  Serial.print("Distance: ");

  if (distance < 0)
  {
    Serial.println("Sensor error");
    return;
  }

  Serial.print(distance, 2);
  Serial.println(" cm");

  bool objectDetected = distance > 0 &&
                        distance <= OBJECT_DISTANCE_CM;

  // ==========================================================
  // WAITING FOR OBJECT
  // ==========================================================

  if (currentState == WAITING_FOR_OBJECT)
  {
    if (objectDetected)
    {
      objectStartTime = millis();
      videoCaptured = false;

      Serial.println("--------------------------------------");
      Serial.println("Object detected!");
      Serial.println("Tracking started...");
      Serial.println("--------------------------------------");

      currentState = TRACKING_OBJECT;
    }
  }

  // ==========================================================
  // TRACKING OBJECT
  // ==========================================================

  else if (currentState == TRACKING_OBJECT)
  {
    unsigned long objectTime = millis() - objectStartTime;

    // Object disappeared before 5 seconds
    if (!objectDetected)
    {
      Serial.println();
      Serial.println("Object disappeared.");

      if (objectTime < VIDEO_START_TIME)
      {
        Serial.println("Object stayed less than 5 seconds.");
        Serial.println("Taking a photo...");

        createPhotoEvent(distance);
      }

      Serial.println("Waiting for the next object...");

      currentState = WAITING_FOR_OBJECT_TO_LEAVE;
      return;
    }

    // Object stayed for 5 seconds
    if (!videoCaptured && objectTime >= VIDEO_START_TIME)
    {
      Serial.println();
      Serial.println("Object stayed for 5 seconds.");
      Serial.println("Starting frame sequence...");

      createVideoEvent();

      videoCaptured = true;

      Serial.println("Frame sequence completed.");
      Serial.println("Waiting for object to leave...");

      currentState = WAITING_FOR_OBJECT_TO_LEAVE;
    }
  }

  // ==========================================================
  // WAIT UNTIL OBJECT LEAVES
  // ==========================================================

  else if (currentState == WAITING_FOR_OBJECT_TO_LEAVE)
  {
    if (!objectDetected)
    {
      Serial.println();
      Serial.println("Object completely left.");
      Serial.println("System ready for a new detection.");
      Serial.println();

      currentState = WAITING_FOR_OBJECT;
    }
  }
}

// ============================================================
// CAMERA INITIALIZATION
// ============================================================

bool initCamera()
{
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t result = esp_camera_init(&config);

  if (result != ESP_OK)
  {
    Serial.printf(
      "Camera init failed with error 0x%x\n",
      result
    );

    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();

  if (sensor != nullptr)
  {
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 0);
    sensor->set_saturation(sensor, 0);
  }

  return true;
}

// ============================================================
// SD CARD INITIALIZATION
// ============================================================

bool initSDCard()
{
  /*
    1-bit mode is used because GPIO12 and GPIO13
    are required for the HC-SR04 sensor.
  */

  if (!SD_MMC.begin("/sdcard", true))
  {
    Serial.println("SD_MMC mount failed.");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card detected.");
    return false;
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);

  Serial.print("SD card size: ");
  Serial.print(cardSize);
  Serial.println(" MB");

  return true;
}

// ============================================================
// ULTRASONIC SENSOR
// ============================================================

float readDistance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
    pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0)
  {
    return -1;
  }

  float distance =
    (duration * 0.0343) / 2.0;

  return distance;
}

// ============================================================
// FLASH
// ============================================================

void flashOn()
{
  digitalWrite(FLASH_LED_PIN, HIGH);
}

void flashOff()
{
  digitalWrite(FLASH_LED_PIN, LOW);
}

// ============================================================
// CAPTURE PHOTO
// ============================================================

bool capturePhoto(const String &filename)
{
  Serial.print("Capturing: ");
  Serial.println(filename);

  flashOn();
  delay(100);

  camera_fb_t *fb = esp_camera_fb_get();

  flashOff();

  if (!fb)
  {
    Serial.println("Camera capture failed.");
    return false;
  }

  File file = SD_MMC.open(filename, FILE_WRITE);

  if (!file)
  {
    Serial.println("Failed to open file.");
    esp_camera_fb_return(fb);
    return false;
  }

  size_t written = file.write(
    fb->buf,
    fb->len
  );

  file.close();

  esp_camera_fb_return(fb);

  if (written != fb->len)
  {
    Serial.println("File write error.");
    return false;
  }

  Serial.print("Saved ");
  Serial.print(written);
  Serial.println(" bytes.");

  return true;
}

// ============================================================
// SAVE DISTANCE TO TXT
// ============================================================

bool saveDistance(
  const String &filename,
  float distance
)
{
  File file = SD_MMC.open(
    filename,
    FILE_WRITE
  );

  if (!file)
  {
    Serial.println(
      "Failed to create distance file."
    );

    return false;
  }

  file.println(
    "ESP32-CAM Object Detection"
  );

  file.println(
    "--------------------------"
  );

  file.print(
    "Distance: "
  );

  file.print(
    distance,
    2
  );

  file.println(
    " cm"
  );

  file.print(
    "Timestamp: "
  );

  file.print(
    millis()
  );

  file.println(
    " ms"
  );

  file.close();

  return true;
}

// ============================================================
// GET NEXT PHOTO NAME
// ============================================================

String getNextPhotoFilename()
{
  while (true)
  {
    String filename =
      "/photo_" +
      String(photoNumber) +
      ".jpg";

    if (!SD_MMC.exists(filename))
    {
      return filename;
    }

    photoNumber++;
  }
}

// ============================================================
// GET NEXT VIDEO FOLDER
// ============================================================

String getNextVideoFolder()
{
  while (true)
  {
    String folder =
      "/video_" +
      String(videoNumber);

    if (!SD_MMC.exists(folder))
    {
      return folder;
    }

    videoNumber++;
  }
}

// ============================================================
// CREATE PHOTO EVENT
// ============================================================

void createPhotoEvent(float distance)
{
  String photoFilename =
    getNextPhotoFilename();

  String txtFilename =
    photoFilename.substring(
      0,
      photoFilename.length() - 4
    ) + ".txt";

  bool photoOK =
    capturePhoto(photoFilename);

  if (photoOK)
  {
    saveDistance(
      txtFilename,
      distance
    );

    Serial.println(
      "Photo event completed."
    );

    Serial.print(
      "Photo: "
    );

    Serial.println(
      photoFilename
    );

    Serial.print(
      "Distance log: "
    );

    Serial.println(
      txtFilename
    );
  }

  photoNumber++;
}

// ============================================================
// CREATE VIDEO EVENT
// ============================================================

void createVideoEvent()
{
  String folder =
    getNextVideoFolder();

  if (!SD_MMC.mkdir(folder))
  {
    Serial.println(
      "Failed to create video folder."
    );

    return;
  }

  String distanceFile =
    folder + "/distance.txt";

  File logFile =
    SD_MMC.open(
      distanceFile,
      FILE_WRITE
    );

  if (logFile)
  {
    logFile.println(
      "ESP32-CAM Frame Sequence"
    );

    logFile.println(
      "Duration: approximately 5 seconds"
    );

    logFile.println(
      "Frames: 25"
    );

    logFile.println(
      "Frame interval: 200 ms"
    );

    logFile.close();
  }

  for (int i = 1;
       i <= VIDEO_FRAME_COUNT;
       i++)
  {
    String frameFilename =
      folder +
      "/frame_" +
      String(i) +
      ".jpg";

    Serial.print(
      "Frame "
    );

    Serial.print(
      i
    );

    Serial.print(
      "/"
    );

    Serial.println(
      VIDEO_FRAME_COUNT
    );

    flashOn();
    delay(50);

    camera_fb_t *fb =
      esp_camera_fb_get();

    flashOff();

    if (!fb)
    {
      Serial.println(
        "Frame capture failed."
      );

      delay(
        VIDEO_FRAME_INTERVAL
      );

      continue;
    }

    File file =
      SD_MMC.open(
        frameFilename,
        FILE_WRITE
      );

    if (file)
    {
      file.write(
        fb->buf,
        fb->len
      );

      file.close();
    }
    else
    {
      Serial.println(
        "Failed to save frame."
      );
    }

    esp_camera_fb_return(fb);

    delay(
      VIDEO_FRAME_INTERVAL
    );
  }

  Serial.print(
    "Frame sequence saved to: "
  );

  Serial.println(
    folder
  );

  videoNumber++;
}
