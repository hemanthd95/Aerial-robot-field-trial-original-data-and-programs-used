#include <mavlink.h>
//#include <SoftwareSerial.h>
#include <SDI12.h>
#include <ezButton.h>

//#define RXpin 0
//#define TXpin 1
#define DATA_PIN 11         /*!< The pin of the SDI-12 data bus */
#define POWER_PIN 22       /*!< The sensor power pin (or -1 if not switching power) */
#define SENSOR_ADDRESS 3

ezButton limitSwitch(5);  // create ezButton object that attach to pin 7;
/** Define the SDI-12 bus */
SDI12 mySDI12(DATA_PIN);
String sdiResponse = "";
String myCommand   = "";
int wave=0;
int smc_count=0;
int auto_count=0;
int count=0;
//SoftwareSerial SerialMAV(RXpin, TXpin); // sets up serial communication on pins 3 and 2
float lat[]={41.7658823,41.7658723,41.7658063,41.7658603,41.7658443,41.7658013,41.7658883,41.7658438,41.7658023,41.7658528,41.7658878,41.7657783,41.7658498,41.7658848};
float longi[]={-111.8121797,-111.8120107,-111.8121797,-111.8121301,-111.8119812,-111.8121354,-111.8121079,-111.8120409,-111.8120985,-111.8121837,-111.8120503,-111.8121797,-111.8120939,-111.8119812};
uint8_t _system_id = 1; // id of computer which is sending the command (ground control software has id of 255)
uint8_t _component_id = 100; // seems like it can be any # except the number of what Pixhawk sys_id is
uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
uint8_t _target_component = 1; // Target component, 0 = all (seems to work with 0 or 1
int mission_seq_count=0;
int miss_count=0;
int gps_count=0;
uint32_t gps_lat=0;
uint32_t gps_lon=0;
//int auto_count=0;
void setup() {
   Serial.begin(115200); //Main serial port for console output
  Serial1.begin(115200);//connection to myRIO
  Serial2.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
  request_datastream();
  setmode_Auto();
  
  
  while (!Serial)
    ;

  Serial.println("Opening SDI-12 bus...");
  mySDI12.begin();
  //setmode_poshold();//going to this mode to write the mission to flight controller
  //request_datastream();
  //setmode_Auto();
  //mission_count();    
  delay(500);  // allow things to settle

  // Power the sensors;
  if (POWER_PIN > 0) {
    Serial.println("Powering up sensors...");
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    delay(200);
  }
  
  request_datastream();
 // mission_count();
  
}

void loop() {
//delay(1000);
MavLink_receive();//receive gps data
 limitSwitch.loop(); // MUST call the loop() function first

  char nogo =
    Serial1.read();  // simply hit enter in the terminal window or press send and the
                    // characters get discarded but now the rest of the loop continues
                    //make seria1 when using with myRIO,getting signal from myRIO
   int meas = nogo-'0'; //initializing a variable to control the number of readings to three
   //Serial.println("the meas is");
   //Serial1.print(meas);
   if(meas==1){
    count=count+1;//this will count the number of soil moisture measurements
   }
   
   if(count%4==0&wave>0&meas==1){//I want to stop once three measurements are complete
    count=0;
   }
   
    if(meas==1&count!=0){
  //Serial1.println(count);
  // first command to take a measurement
  myCommand = String(SENSOR_ADDRESS) + "M!";
  mySDI12.sendCommand(myCommand);
  delay(30);  // wait a while for a response

  while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(10);  // 1 character ~ 7.5ms
    }
  }
  if (sdiResponse.length() > 1)
  mySDI12.clearBuffer();


  delay(1000);       // delay between taking reading and requesting data
  sdiResponse = "";  // clear the response string


  // next command to request data from last measurement
  myCommand = String(SENSOR_ADDRESS) + "D0!";
  Serial1.println(myCommand);
  
  mySDI12.sendCommand(myCommand);
  delay(30);  // wait a while for a response

  while (mySDI12.available()) {  // build string from response
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(10);  // 1 character ~ 7.5ms
    }
  }
  if (sdiResponse.length() > 1)
    Serial1.println(sdiResponse);//send data to myRIO
    wave=wave+1;//this will control the waveform
    auto_count=auto_count+1;//this will control the autonomous mission
    smc_count=smc_count + 1;//this will track the measurement number 
   Serial1.println(smc_count);
   delay(1000);       // delay between taking reading and requesting data
   
  sdiResponse = "";  // clear the response string
   if (wave%3==0){
     //int command_time_decimal = 0;
 myCommand = String(SENSOR_ADDRESS) + "M!";
  mySDI12.sendCommand(myCommand);
  delay(30);  // wait a while for a response

  while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(10);  // 1 character ~ 7.5ms
    }
  }
  if (sdiResponse.length() > 1)
    mySDI12.clearBuffer();
  delay(1000);       // delay between taking reading and requesting data
  sdiResponse = "";  // clear the response string

  myCommand = String(SENSOR_ADDRESS) + "D1!";
  Serial1.println(myCommand);
  mySDI12.sendCommand(myCommand);
  delay(30);  // wait a while for a response

  while (mySDI12.available()) {  // build string from response
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(10);  // 1 character ~ 7.5ms
    }
  }
  if (sdiResponse.length() > 1)
    Serial1.println(sdiResponse);//send data to myRIO
  delay(1000);       // delay between taking reading and requesting data
   sdiResponse = "";  // clear the response string
  Serial.println(meas);
    for (int t= 1995; t < 6001; t=t+15){
     //command_time_decimal = t/5;
  //String command_time_hex = String(command_time_decimal,HEX);
  //myCommand = String(SENSOR_ADDRESS) + "XA" + command_time_hex +"!";
    mySDI12.sendCommand(String(SENSOR_ADDRESS) + "XA" + String(t/5,HEX) +"!");
    delay(30);  // wait a while for a response
  
   while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(8);  // 1 character ~ 7.5ms
    }
  }
  
  if (sdiResponse.length() > 1)
    Serial1.println(sdiResponse); // write the response to the myRIO
    mySDI12.clearBuffer();
   sdiResponse = "";  // clear the response string
    meas=0;//this will limit the number of measurements per location to 3
    
  }
   Serial1.println(meas);
   //mission_count();
 }
 //mission_count();
 Serial1.flush();
 }

}


 //}

//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive()
  { 
  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial2.available())
  {
    uint8_t c= Serial2.read();

    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      //Handle new message from autopilot
      if(auto_count>8){
      switch(msg.msgid)
      {
      Serial.println("in switch case");
      // Step 2 uploading a new waypoint - Check for mission replies
      case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        if(gps_count<1){
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(&msg, &packet);
        gps_lat=packet.lat;
        gps_lon= packet.lon;
        float lat_rem= packet.lat%10000000;
        float lon_rem= packet.lon%10000000;
        //gps_lat=((packet.lat)/10000000)+(lat_rem/10000000);
        //gps_lon=((packet.lon)/10000000)+(lon_rem/10000000);
        Serial1.print("GPS Latitude: ");Serial1.println(packet.lat);
        Serial1.print("GPS Longitude: ");Serial1.println(packet.lon);
        setmode_Auto();
        }
       }
       //sending the mission waypoints
       mission_count(); 
       gps_count=gps_count+1;
       //Serial.print("GPS Count: ");
       //Serial.println(gps_count);
       break;

      case MAVLINK_MSG_ID_MISSION_REQUEST:
      {
       mavlink_mission_request_t missionreq;
       mavlink_msg_mission_request_decode(&msg, &missionreq);
               
        Serial1.print("\nMission Req Sequence: ");Serial1.println(missionreq.seq);
        Serial1.print("\SysID: ");Serial1.println(missionreq.target_system);
        Serial1.print("\Compid: ");Serial1.println(missionreq.target_component);

        if (missionreq.seq == 0) {
        create_home();
        Serial1.print("Sent Home: \n");
        }

        if (missionreq.seq == 1) {
        //create_waypoint();
        uav_takeoff();
        Serial1.print("Sent Waypoint: \n");
        }
        if (missionreq.seq == 2) {
        uav_land();
        Serial1.print("Sent Waypoint: \n");
        }
      }
      break; 

      case MAVLINK_MSG_ID_MISSION_ACK:
      // Step 4 uploading a new waypoint - Receive Mission Ack Message
      {
       mavlink_mission_ack_t missionack;
       mavlink_msg_mission_ack_decode(&msg, &missionack);
       
        Serial.print("\nMission Ack Sequence: ");Serial.println(missionack.type);
        Serial.print("\SysID: ");Serial.println(missionack.target_system);
        Serial.print("\CompID: ");Serial.println(missionack.target_component);

        if (missionack.type == 1) {
          //commandlong_Arm();
  
        Serial.print("\nMission upload FAILED: ");Serial.println(missionack.type);
        }

        if (missionack.type == 0) {
          
        Serial.print("\nMission upload SUCCESSFULL: ");Serial.println(missionack.type);
        }   
      }
      break;
      }
      }
    }
  }
}
    
void request_datastream() {
//Request Data from Pixhawk
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; //1 = start, 0 = stop
 
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
 
  Serial2.write(buf, len); //Write data to serial port
}

void setmode_Auto() {
  
  uint8_t _base_mode = 1;
  uint32_t _custom_mode = 16; //3 = auto mode
 // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message
  mavlink_msg_set_mode_pack(_system_id, _component_id, &msg, _target_system, _base_mode, _custom_mode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
  Serial2.write(buf, len); //Write data to serial port
  //mission_count();//informing the drone about number of waypoints
  //commandlong_Arm();
}


void mission_count() {
  if(miss_count<12){
  //Step #1 of uploading a new waypoint
  uint8_t _system_id = 1; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 100; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 1; // Pixhawk component id, 0 = all (seems to work fine)

  uint16_t count = 3; // How many items to upload (HOME coordinates are always the first way-point)

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_mission_count_pack(_system_id, _component_id, &msg, _target_system, _target_component, count);
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t count

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial2.write(buf, len);
  //Serial.print("mission count is: ");
  //Serial.println(miss_count);
  miss_count=miss_count+1;
  }
}

void create_home() {
  //Step 3 of uploading a new waypoint (send HOME coordinates)
  uint8_t _system_id = 1; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 100; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 1; // Pixhawk component id, 0 = all (seems to work fine)

  uint16_t seq = 0; // Sequence number
  uint8_t frame = 0; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for PX4
  uint8_t current = 0; // false:0, true:1 - When downloading, whether the item is the current mission item.
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 0; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle
  float x = 41.7419665; // Latitude - degrees
  float y = -111.8082145; // Longitude - degrees
  float z = 0; // Altitude - meters

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
  //uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial2.write(buf, len);
  
}

 void commandlong_Arm() {
  //Set message variables
  int state=limitSwitch.getState();
  if(auto_count>8){
  uint8_t _base_mode = 1;
  uint16_t _command= MAV_CMD_COMPONENT_ARM_DISARM;
  uint8_t _confirmation=0;
  float param1=1;
  float param2=21196;
  float a=0;
  float b=0;
  float c=0;
  float d=0;
  float e=0;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_command_long_pack(_system_id, _component_id,&msg, _target_system,_target_component,_command,_confirmation,param1,param2,a,b,c,d,e);
  //mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
  Serial2.write(buf, len); //Write data to serial port
  //Serial1.println("In the arming part");
  //mission_seq_count=mission_seq_count + 1;
  //auto_count=0;
  //gps_count=0;
  auto_count=0;
  gps_count=0;
 

  }
 }

void uav_takeoff() {
  uint16_t seq = 1; // Sequence number
  uint8_t frame = 6; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_TAKEOFF; // Specific command for PX4
  uint8_t current = 0; // false:0, true:1 - When downloading, whether the item is the current mission item.
  uint8_t autocontinue =0 ; // Always 0
  float p1 = 0; // Loiter time
  float p2 = 0; // Acceptable range from target - radius in meters
  float p3 = 0; // Pass through waypoint
  float p4 = 0; // Desired yaw angle
  //float x = 41.7419665; // Latitude - degrees
  //float y = -111.8082145; // Longitude - degrees
  float z = 15; // Altitude - meters

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  //Serial1.println("sending take off");
  // Pack the message
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, p1, p2, p3, p4,0,0,z);
  //uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  //Serial1.println("All the best");
  // Send the message (.write sends as bytes)
  Serial2.write(buf, len);
  //Serial1.println("In the takeoff part");
  
  //uav_land();
 }

void uav_land(){
   uint16_t seq = 2; // Sequence number
  uint8_t frame = 6; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_LAND; // Specific command for PX4
  uint8_t current = 0; // false:0, true:1 - When downloading, whether the item is the current mission item.
  uint8_t autocontinue = 0; // Always 0
  float p1 = 0; // Loiter time
  float p2 = 0; // Acceptable range from target - radius in meters
  float p3 = 0; // Pass through waypoint
  float p4 = 0; // Desired yaw angle
  //float x = 41.7419665; // Latitude - degrees
  //float y = -111.8082145; // Longitude - degrees
  float z = 0; // Altitude - meters

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  Serial.print("Mission count value: ");
  Serial.println(mission_seq_count);
  // Pack the message
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, p1, p2, p3, p4,lat[mission_seq_count],longi[mission_seq_count],z);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
  Serial2.write(buf, len);
   mission_seq_count=mission_seq_count+1;
  Serial1.print("New mission count value: ");
  Serial1.println(mission_seq_count);
  
    //auto_count=0;
   // gps_count=0;
  //uav_disarm();
  setmode_fly();
          //commandlong_Arm();
  
 }
 void setmode_fly() {
  
  uint8_t _base_mode = 1;
  uint32_t _custom_mode = 3; //3 = auto mode
 // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // Pack the message
  mavlink_msg_set_mode_pack(_system_id, _component_id, &msg, _target_system, _base_mode, _custom_mode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
  Serial2.write(buf, len); //Write data to serial port
  
  //mission_count();//informing the drone about number of waypoints
  commandlong_Arm();
}
