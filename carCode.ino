/*
 * DemoBot code
 */

// =================================================================
// ========================= I2C start =============================
// =================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128                             // data buffer length of test buffer
#define W_LENGTH 1                                  // data length for w, [0,DATA_LENGTH]
#define R_LENGTH 16                                 // data length for r, [0,DATA_LENGTH]

#define I2C_MASTER_SCL_IO (gpio_num_t)33            // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO (gpio_num_t)25            // gpio number for I2C master data
#define I2C_MASTER_NUM I2C_NUMBER(1)                // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 40000                    // I2C master clock frequency (Hz)
#define I2C_MASTER_TX_BUF_DISABLE 0                 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0                 // I2C master doesn't need buffer

#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS     // ESP32 slave address, you can set any 7bit value
#define WRITE_BIT I2C_MASTER_WRITE                  // I2C master write
#define READ_BIT I2C_MASTER_READ                    // I2C master read
#define ACK_CHECK_EN 0x1                            // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0                           // I2C master will not check ack from slave
#define ACK_VAL I2C_MASTER_ACK                      // I2C ack value
#define NACK_VAL I2C_MASTER_NACK                    // I2C nack value

const char* ssid = "woofwoof";
const char* password = "11111111";

// Variables and set up for Wifi
WiFiUDP UDPTestServer;
unsigned int udpTargetPort=2150;                  // Define the target port
IPAddress myIPaddress(192, 168, 1, 155);          // Define my IP Address
IPAddress targetIPaddress(192, 168, 1, 155);      // Define target IP Address

const int UDP_PACKET_SIZE = 48;                   // Define the packet size
char udpBuffer[UDP_PACKET_SIZE];                  // Initialize the send buffer
byte packetBuffer[UDP_PACKET_SIZE+1];             // Initialize the receive buffer

#define LEFT_PWM_PIN 26
#define RIGHT_PWM_PIN 27
#define LEFT_DIR_PIN 19
#define LEFT_REV_PIN 18
#define RIGHT_DIR_PIN 5
#define RIGHT_REV_PIN 17

#define TRIG_PIN 23
#define ECHO_ONE_PIN 16
#define ECHO_TWO_PIN 4
#define ECHO_THREE_PIN 15

#define SERVO_PWM_PIN 13
#define SERVO_DIR_PIN 22
#define SERVO_REV_PIN 21

#define SWITCH_PIN 26
#define VIVE_PIN 32

// Variables for switch
int color;


// Variables for vive
unsigned long previous_fall_time, fall_time, last_pulse, last_last_pulse, last_last_last_pulse;
float x_coordinate, y_coordinate;
float sync_pulse_time = 8.3;


// Define the settings for ledc (generating PWM)
int freq = 150;
int leftChannel = 0;
int rightChannel = 1;
int resolution = 8;

int ls = -10; // Variable for storing the value from the controller
/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t nsize)
{
    if (nsize == 0) 
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN); 
    if (nsize > 1) 
    {
        i2c_master_read(cmd, data_rd, nsize - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + nsize - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS); // send all queued commands
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t nsize)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, nsize, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) 
    {
        Serial.printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) 
        {
            Serial.printf("\n");
        }
    }
    Serial.printf("\n");
}

uint8_t data_wr[DATA_LENGTH];
uint8_t data_rd[DATA_LENGTH];

static void i2c_read_test()
{
    int ret;

    ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);

    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
        Serial.println("I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information read from I2C
        Serial.printf(" MASTER READ FROM SLAVE ******\n");
        disp_buf(data_rd, DATA_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, " %s: Master read slave error, IO not connected...\n",
            esp_err_to_name(ret));
    }
}

static void i2c_write_test()
{ 
    int ret;
                                                                             
    ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, W_LENGTH);
    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information being send over I2C
        Serial.printf(" MASTER WRITE TO SLAVE\n");
        disp_buf(data_wr, W_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, "%s: Master write slave error, IO not connected....\n",
            esp_err_to_name(ret));
    }
}
// =================================================================
// ========================== I2C end ==============================
// =================================================================


// =================================================================
// ====================== Interrupt start ==========================
// =================================================================
// Timer + Interrupt for reading I2C
hw_timer_t* timer = NULL;                               // initialize a timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;   // needed to sync between main loop and ISR when modifying shared variable
volatile bool readI2C = 0;                              // should we read data from I2C?

void IRAM_ATTR readI2COnTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    readI2C = 1;                        // need to read I2C next loop
    portEXIT_CRITICAL_ISR(&timerMux);
}
// =================================================================
// ======================= Interrupt end ===========================
// =================================================================


// =================================================================
// ========================= LED start =============================
// =================================================================
#include "FastLED.h"
FASTLED_USING_NAMESPACE

#define RED             0xFF0000    // color for the red team
#define BLUE            0x0000FF    // color for the blue team
#define HEALTHCOLOR     0x00FF00    // color for the health LEDs   

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

// ===== GAME VARIABLES =====
// change ROBOTNUM (1-4) and TEAMCOLOR (BLUE or RED) as necessary
#define ROBOTNUM    2               // robot number on meta team (1-4)
#define TEAMCOLOR   BLUE            // color for the robot team, either RED or BLUE
// ==========================

#define NEO_LED_PIN 12              // pin attached to LED ring
#define LED_TYPE    WS2812          // APA102
#define COLOR_ORDER GRB             // changes the order so we can use standard RGB for the values
#define NUM_LEDS    24              // number of LEDs in the ring
CRGB leds[NUM_LEDS];                // set value of LED, each LED is 24 bits

#define BRIGHTNESS          60      // lower the brightness a bit

// core to run FastLED.show()
#define FASTLED_SHOW_CORE 0

// task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;

/** show() for ESP32
 *  Call this function instead of FastLED.show(). It signals core 0 to issue a show, 
 *  then waits for a notification that it is done.
 */
void FastLEDshowESP32()
{
    if (userTaskHandle == 0) 
    {
        // -- Store the handle of the current task, so that the show task can
        //    notify it when it's done
        userTaskHandle = xTaskGetCurrentTaskHandle();

        // -- Trigger the show task
        xTaskNotifyGive(FastLEDshowTaskHandle);

        // -- Wait to be notified that it's done
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
        ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        userTaskHandle = 0;
    }
}

/** show Task
 *  This function runs on core 0 and just waits for requests to call FastLED.show()
 */
void FastLEDshowTask(void *pvParameters)
{
    // -- Run forever...
    for(;;) 
    {
        // -- Wait for the trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // -- Do the show (synchronously)
        FastLED.show();

        // -- Notify the calling task
        xTaskNotifyGive(userTaskHandle);
    }
}

void SetupFastLED(void)
{
    // tell FastLED about the LED strip configuration
    FastLED.addLeds<LED_TYPE,NEO_LED_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

    // set master brightness control
    FastLED.setBrightness(BRIGHTNESS);

    int core = xPortGetCoreID();
    Serial.print("FastLED: Main code running on core ");
    Serial.println(core);

    // -- Create the FastLED show task
    xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);
}

void ShowRobotNum(void)
{
    int robotLeds[] = {0,6,12,18};      // location of the LEDs used to display the robot number

    // change the LEDs based on the robot number
    leds[robotLeds[0]] = TEAMCOLOR;     // The first LED is always displayed with the robot number

    switch (ROBOTNUM)
    {
        case 1:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = 0;
            break;
        case 2:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = 0;
            break;
        case 3:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
        case 4:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
    }
}

void ShowHealth(int health)
{
  //value between 2-20//
  int i=0;
  int j=0;
  int val=0;
  val=health;
  val = map(val,0,10,0,24);
  for(i=0;i<24;i++)
  {if(i==0 ||i==6|| i==12||i==18)
  { } 
  else if(j<val)
    {leds[i] = 0x00FF00;
    j++;}
  else{leds[i] = 0x000000;}
 
 
  }

}

void clearLEDs(void)
{
    for(int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = 0;
    }
}

void ShowRespawnTimer(int respawnTime)
{
   int i=0,j=0,val=0;
  val=respawnTime;
  val = map(val,0,15,0,24);
  //Serial.println(val);
  for(i=0;i<val;i++)
  {
    leds[i]=0xFF0000;
    }   
   for(j=val;j<24;j++)
   {
    leds[j]=0x000000;
    } 
}

float checkUltrasonic(int echo_pin){
  digitalWrite(TRIG_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIG_PIN, LOW); 
  float duration = pulseIn(echo_pin, HIGH);
  float distance = (duration*.0343)/2; 
  return distance; 
}



void autonomous()
{

}

}
// =================================================================
// ========================== LED end ==============================
// =================================================================


// =====================================================================
// ============================= SETUP =================================
// =====================================================================
void setup()
{
    // Connect the WiFi
    Serial.begin(115200);
    pinMode(2,OUTPUT);
    // Initiate station mode
    WiFi.setSleep(false);
    WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    WiFi.begin(ssid, password);
    UDPTestServer.begin(udpTargetPort); // strange bug needs to come after WiFi.begin but before connect
  
    // Wait for the WiFi to connect before continuing 
    while(WiFi.status()!=WL_CONNECTED){
      delay(500);
      Serial.print(".");
    }
    Serial.print("Connected");
    
    // ========================= I2C start =============================
    ESP_ERROR_CHECK(i2c_master_init()); // initialize the i2c
    // ========================== I2C end ==============================

    // ===================== Interrupts start ==========================
    // default clock speed is 240MHz
    // 240MHz / 240 = 1MHz      1 000 000 timer increments per second
    // 1 000 000 / 20 = 50 000  timer value to count up to before calling interrupt (call 20 times per second)
    timer = timerBegin(0, 240, true);                       // initialize timer with pre-scaler of 240
    timerAttachInterrupt(timer, &readI2COnTimer, true);     // attach function to timer interrupt
    timerAlarmWrite(timer, 50000, true);                    // set count value before interrupt occurs
    timerAlarmEnable(timer);                                // start the timer
    // ====================== Interrupts end ===========================

    // ========================= LED start =============================
    SetupFastLED(); // set the LEDs
    // ========================== LED end ==============================

    // ================== Motor PWM LEDC start =========================
    pinMode(RIGHT_DIR_PIN,OUTPUT);
    pinMode(LEFT_DIR_PIN,OUTPUT);
    pinMode(RIGHT_REV_PIN,OUTPUT);
    pinMode(LEFT_REV_PIN,OUTPUT);
     pinMode(SERVO_DIR_PIN,OUTPUT);
    pinMode(SERVO_REV_PIN,OUTPUT);
    ledcSetup(leftChannel, freq, resolution);
    ledcAttachPin(27, leftChannel);
    ledcSetup(rightChannel, freq, resolution);
    ledcAttachPin(26, rightChannel);
    ledcSetup(servoChannel, freq, resolution); 
    ledcAttachPin(13, servoChannel);
      
}
// =====================================================================
// ========================== END OF SETUP =============================
// =====================================================================


// =====================================================================
// ============================== LOOP =================================
// =====================================================================
void loop()
{
    ReceiveNum(); // Receive messages 
    Serial.println(ls); // Print the recieved message
    // ========================= I2C start =============================
    // static variables
    // static variables only get initialized once, when the loop function is first called
    // values of static variables persist through function calls
    static bool gameStatus = 0;     // game on: 1, game off: 0
    static bool reset = 0;          // 1 for resetting
    static bool autoMode = 0;       // not autonomous mode: 0, is auto mode: 1
    static bool syncStatus = 0;     // 1 for sync

    static int health;              // robot's health

    static int respawnTimer;        // amount of time remaining on respawn
    int i = 0;
    float xavg, yavg;
    int xj = 1;
    int yj = 1;
    if (readI2C)
    {
        readI2C = 0;                // set readI2C to be false
        i2c_write_test();           // need this write for some reason in order to get read to work?
        i2c_read_test();            // read information from slave (top hat)  

        // read information
        gameStatus  = 1 & (data_rd[0] >> 0);
        reset       = 1 & (data_rd[0] >> 1);
        autoMode    = 1 & (data_rd[0] >> 2);
        syncStatus  = 1 & (data_rd[0] >> 3);
    
        if (data_rd[1] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            health = data_rd[1];
        }

        if (data_rd[2] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            respawnTimer = data_rd[2];
        }
    }
    // ========================== I2C end ==============================

    // ========================= LED start =============================
    ShowRobotNum();         // set the LEDs for the robot number
    ShowHealth(health);     // set the LEDs for the health
    
    if (health == 0) // Robot is dead, stop motors, show respawn timer
    {
        clearLEDs();
        ShowRespawnTimer(respawnTimer);
        ledcWrite(rightChannel, 0);
        ledcWrite(leftChannel, 0);
        ledcWrite(servoChannel, 0);
    }
    else if (gameStatus==0) // Game is off, stop motors
    {
      ledcWrite(rightChannel, 0);
      ledcWrite(leftChannel, 0);
      ledcWrite(servoChannel, 0);
    }
    else if(autoMode==0) // Remote control period, allow motors to receive
    {
      motorAdjust();   //includes both motor and servo control
    }
    else if(autoMode==1) // Autonomous period, check for obstacles and drive
    {
      int xfinal,yfinal;         // define target coordinates
      autonomous(xfinal,yfinal); // travel to the target
    }
      
    FastLEDshowESP32();
    // ========================== LED end ==============================


}
// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================

// Function to receive an integer over Wifi. Result is tored in global variable ls. 
void ReceiveNum() {
  int cb = UDPTestServer.parsePacket();                   
  if (cb) { // If the packet isn't empty
    digitalWrite(LED_BUILTIN, HIGH);                      // Turn on the LED so we know it's connected
    UDPTestServer.read(packetBuffer, UDP_PACKET_SIZE);    // Read the packet
    ls = packetBuffer[0];                                 // Store the value in a globl variable
  }
}

// Function to calculate the x,y position of the robot using the vive sensing circuit.
int calibrate()
{
  float x_coordinate;
  float y_coordinate;
  for (i; i < 40; i++){ // Observe 40 pulses before calculating the coordinates
    
    float pulse_width = pulseIn(VIVE_PIN, HIGH);                    // Determine the pulse width
    fall_time = millis();                                           // Get the system time
    float pulse_time = (fall_time - previous_fall_time)/1000.0;     // Determine the time between pulses

    if (pulse_width > 2){ // If its a real pulse
      if (pulse_width < 60 && last_pulse > 60 && last_last_pulse > 60 && last_last_last_pulse > 60) {
        xavg = xavg + 100.0 * (pulse_time/sync_pulse_time);         // Map the x pulse time to a coordinate based on its distance between two sync pulses
        xj = xj + 1;                                                
      }
      else if (pulse_width < 60 && last_pulse > 60) {
        yavg = yavg + 100.0 * (pulse_time/sync_pulse_time);         // Map the y pulse time to a coordinate based on its distance between two sync pulses
        yj = yj + 1;
      }

      // Update the last three pulse widths
      last_last_last_pulse = last_last_pulse;
      last_last_pulse = last_pulse;
      last_pulse = pulse_width;
      // Count that a pulse has been recorded
      i = i + 1;
    }
    
  }
  // Average the results
  x_coordinate = xavg/xj;                                      
  y_coordinate = yavg/yj; 
  return  x_coordinate, y_coordinate;
  }

// Function to adjust the motors based on control commands
void motorAdjust() {

  // Turn the motors off
  ledcWrite(rightChannel, 0);
  ledcWrite(leftChannel, 0);
  ledcWrite(servoChannel, 0);
  ReceiveNum();
  
  // Servo control
  if(ls==21)      // when button 1 is pressed, servo rotates anticlockwise
  {
    digitalWrite(SERVO_DIR_PIN,LOW);
    digitalWrite(SERVO_REV_PIN,HIGH);
    ledcWrite(servoChannel, int(180));
  }
  if(ls==22)  // when button 2 is pressed, servo rotates clockwise
  {
    digitalWrite(SERVO_DIR_PIN,HIGH);
    digitalWrite(SERVO_REV_PIN,LOW);
    ledcWrite(servoChannel, int(180));
  }   
  
  // Motor Control
  while(ls <= 255 && ls > 230){        // Left motors forward
    digitalWrite(LEFT_DIR_PIN,LOW);
    digitalWrite(LEFT_REV_PIN,HIGH);
    ledcWrite(leftChannel, int(180));
    Serial.println("a");
    ReceiveNum();
  }
  while (ls <= 220 && ls > 199){      // Left motors backward
    digitalWrite(LEFT_DIR_PIN,HIGH);
    digitalWrite(LEFT_REV_PIN,LOW);
    ledcWrite(leftChannel, int(180));
    Serial.println("b");
    ReceiveNum();
  }
  while (ls <= 50 && ls > 35){ .      // Right motors forward
    digitalWrite(RIGHT_DIR_PIN,HIGH);
    digitalWrite(RIGHT_REV_PIN,LOW);
    ledcWrite(rightChannel, int(180));
    Serial.println("c");
    ReceiveNum();
  }
  while (ls <= 20 && ls > 0){         // Right motors backward
    digitalWrite(RIGHT_DIR_PIN,LOW);
    digitalWrite(RIGHT_REV_PIN,HIGH);
    ledcWrite(rightChannel, int(180));
    Serial.println("d");
    ReceiveNum();
  }
}

// Function to determine the orientation of the robot and rotate towards the target
int autonomous_orientation(int x_init_diff, int y_init_diff)  
{
  float x,y,xfin,yfin;
  float ratio;
  float angle;
  float t;
  x,y=calibrate(); // Get the x,y coordinates
  
  // forward direction for 1 second
  ledcWrite(rightChannel, int(180));
  ledcWrite(leftChannel, int(180));
  digitalWrite(RIGHT_DIR_PIN,HIGH);
  digitalWrite(RIGHT_REV_PIN,LOW);
  digitalWrite(LEFT_DIR_PIN,HIGH);
  digitalWrite(LEFT_REV_PIN,LOW);
  delay(1000);

  // stop
  ledcWrite(rightChannel,0);
  ledcWrite(leftChannel, 0);
  digitalWrite(RIGHT_DIR_PIN,LOW);
  digitalWrite(RIGHT_REV_PIN,LOW);
  digitalWrite(LEFT_DIR_PIN,LOW);
  digitalWrite(LEFT_REV_PIN,LOW);

  xfin,yfin=calibrate();   // Get the x,y coordinates
  ratio=(xfin-x)/(yfin-y); 
  angle=atan(ratio);       // Calculate the angle
  
  if(angle < 0)            // Account for negative angles
  {angle = angle*-1;}

  //To make it come forward facing:   DC motor max speed=115 rpm  so ang velocity=2*3.14*115/60=12.0367
  if(xfin>x && yfin>y){
    ledcWrite(leftChannel, 255);
    digitalWrite(LEFT_DIR_PIN,HIGH);
    digitalWrite(LEFT_REV_PIN,LOW);
    t=(angle/12.0367)*1000;  //time=angle/angular velocity in seconds
    delay(t);
    ledcWrite(leftChannel, 0);
    digitalWrite(LEFT_DIR_PIN,LOW);
    digitalWrite(LEFT_REV_PIN,LOW);
  }
  
  else if(xfin<x && yfin>y){
    ledcWrite(rightChannel,255 );
    digitalWrite(RIGHT_DIR_PIN,HIGH);
    digitalWrite(RIGHT_REV_PIN,LOW);
    t=(angle/12.0367)*1000;   //time=angle/angular velocity in seconds
    delay(t);
    ledcWrite(rightChannel,0 );
    digitalWrite(RIGHT_DIR_PIN,LOW);
    digitalWrite(RIGHT_REV_PIN,LOW);
  }
  
  else if(xfin<x && yfin<y){
    ledcWrite(rightChannel,255 );
    digitalWrite(RIGHT_DIR_PIN,HIGH);
    digitalWrite(RIGHT_REV_PIN,LOW);
    angle=angle+1.57;   //   pi/2=1.57 adding it when it's in the 3rd quadrant
    t=(angle/12.0367)*1000;   //time=angle/angular velocity in seconds
    delay(t);
    ledcWrite(rightChannel,0 );
    digitalWrite(RIGHT_DIR_PIN,LOW);
    digitalWrite(RIGHT_REV_PIN,LOW);
  }
  
  else if(xfin>x && yfin<y){
    ledcWrite(leftChannel, 255);
    digitalWrite(LEFT_DIR_PIN,HIGH);
    digitalWrite(LEFT_REV_PIN,LOW);
    angle=angle+1.57;  // pi/2=1.57 adding it when it's in the 4th quadrant
    t=(angle/12.0367)*1000;     //time=angle/angular velocity in seconds
    delay(t);
    ledcWrite(leftChannel, 0);
    digitalWrite(LEFT_DIR_PIN,LOW);
    digitalWrite(LEFT_REV_PIN,LOW);
  }
}

// Function to drive the robot to the target
void autonomous(int xfin,int yfin)  
{
  float x,y,x_init_diff,y_init_diff;
  x,y=calibrate();
  x_init_diff=xfin-x;
  y_init_diff=yfin-y;
  autonomous_orientation(x_init_diff,y_init_diff);
  while(y_init_diff!=0) // Correct the y-coordinate
  {
    // Drive forward
    if(y_init_diff > 0){ 
      ledcWrite(rightChannel, int(180));
      ledcWrite(leftChannel, int(180));
      digitalWrite(RIGHT_DIR_PIN,HIGH);
      digitalWrite(RIGHT_REV_PIN,LOW);
      digitalWrite(LEFT_DIR_PIN,HIGH);
      digitalWrite(LEFT_REV_PIN,LOW);
      x,y=calibrate();
      x_init_diff=xfin-x;
      y_init_diff=yfin-y;
    }
    
    // Drive backwards
    else{
      ledcWrite(rightChannel, int(180));
      ledcWrite(leftChannel, int(180));
      digitalWrite(RIGHT_DIR_PIN,LOW);
      digitalWrite(RIGHT_REV_PIN,HIGH);
      digitalWrite(LEFT_DIR_PIN,LOW);
      digitalWrite(LEFT_REV_PIN,HIGH);
      x,y=calibrate();
      x_init_diff=xfin-x;
      y_init_diff=yfin-y;
    }    
  }
  
  while(x_init_diff!=0) // Correct the x-coordinate
  {
    // Drive forward
    if(x_init_diff>0){
      ledcWrite(rightChannel, int(180));
      ledcWrite(leftChannel, int(180));
      digitalWrite(RIGHT_DIR_PIN,HIGH);
      digitalWrite(RIGHT_REV_PIN,LOW);
      digitalWrite(LEFT_DIR_PIN,HIGH);
      digitalWrite(LEFT_REV_PIN,LOW);
      x,y=calibrate(1,1);
      x_init_diff=xfin-x;
      y_init_diff=yfin-y;
    }

    // Drive backwards
    else{
      ledcWrite(rightChannel, int(180));
      ledcWrite(leftChannel, int(180));
      digitalWrite(RIGHT_DIR_PIN,LOW);
      digitalWrite(RIGHT_REV_PIN,HIGH);
      digitalWrite(LEFT_DIR_PIN,LOW);
      digitalWrite(LEFT_REV_PIN,HIGH);
      x,y=calibrate(1,1);
      x_init_diff=xfin-x;
      y_init_diff=yfin-y;
    }   
  } 
}
