

#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 以A为高八位 B为低八位 合并为16位整形

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

//#define LOBOT_DEBUG 1  /*调试是用，打印调试数据*/

#define RX_EN 12
#define TX_EN 11

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  Serial.print("Sending move command");
  SerialX.write(buf, 10);
  Serial.print("Sent move command");
}

void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}

void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{

  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  SerialX.write(buf, 10);
}

void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}


int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available()) {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {
        
#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
          
#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
  Serial.print("No Data recieved");
  return -1;
}


int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  //digitalWrite(TX_EN,LOW); //turn into recieve mode
  //digitalWrite(RX_EN,LOW);
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  //delay(1);
  SerialX.write(buf, 6);
  SerialX.flush();
  digitalWrite(TX_EN,LOW); //turn into recieve mode
  digitalWrite(RX_EN,HIGH);

  /*while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -17;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;*/

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -17;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  
  return ret;
}
int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  Serial.println("here 1");
  SerialX.write(buf, 6);
  Serial.println("here 2");
  SerialX.flush();
  Serial.println("here 3");
  digitalWrite(TX_EN,LOW); //turn into recieve mode
  digitalWrite(RX_EN,HIGH);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

//读取转动范围
int retL;
int retH;
int LobotSerialServoReadAngleRange(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ANGLE_LIMIT_READ;
  buf[5] = LobotCheckSum(buf);
  
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  SerialX.write(buf, 6);
  SerialX.flush();
  digitalWrite(TX_EN,LOW); //turn into recieve mode
  digitalWrite(RX_EN,HIGH);
  
  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    {
      retL = (int16_t)BYTE_TO_HW(buf[2], buf[1]); 
      retH = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
    }
  else
    ret = -2048;
  return ret;
}

void LobotSerialWriteAngleRange(HardwareSerial &SerialX, uint8_t id, int16_t min, uint16_t max)
{
  byte buf[10];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_ANGLE_LIMIT_WRITE;
  buf[5] = GET_LOW_BYTE(min);
  buf[6] = GET_HIGH_BYTE(min);
  buf[7] = GET_LOW_BYTE(max);
  buf[8] = GET_HIGH_BYTE(max);
  buf[9] = LobotCheckSum(buf);
  Serial.print("Sending limit set");
  //SerialX.write(buf, 10);
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  SerialX.write(buf, 10);
  SerialX.flush();
  Serial.print("Sent limit set command");
}

#define ID1   1
#define ID2   2
#define setID 7

void setup() {
  // put your setup code here, to run once:
  Serial8.begin(115200);
  Serial.begin(9600);
  delay(4000);
  pinMode(TX_EN, OUTPUT);
  pinMode(RX_EN, OUTPUT);
  delay(10);
 // LobotSerialServoMove(Serial8, setID, 250, 2000);
 // delay(3000);
  //LobotSerialServoMove(Serial8, setID, 700, 80000);
}



void loop() {
  //LobotSerialServoSetID(Serial8, ID1, setID); NOT THIS
  //Serial.println(LobotSerialServoReadPosition(Serial8, setID));
  //delay(6000);
  //pos, time
  //delay(10);
  //LobotSerialWriteAngleRange(Serial8, setID, 0, 1000);
  Serial.print("Wrist: ");
  Serial.print(LobotSerialServoReadPosition(Serial8, 3));
  Serial.print(" Forearm: ");
  Serial.print(LobotSerialServoReadPosition(Serial8, 5)); 
  Serial.print(" Lower Shoulder: ");
  Serial.println(LobotSerialServoReadPosition(Serial8, 4)); 
  delay(300);

  /*
  Serial.println(LobotSerialServoReadPosition(Serial8, setID));
  delay(10000);
  Serial.println("AT 500");
  LobotSerialServoMove(Serial8, setID, 500, 4000);
  delay(100000);*/
 /* delay(1000);
  LobotSerialServoReadAngleRange(Serial8, setID);
  Serial.print("Lower:");
  Serial.print(retL);
  Serial.print("Upper:");
  Serial.print(retH);
  //delay(10);
  LobotSerialServoMove(Serial8, setID, 0, 4000);
  delay(7000);
  Serial.println(LobotSerialServoReadPosition(Serial8, setID));
  LobotSerialServoMove(Serial8, setID, 500, 4000);
  delay(7000);
  Serial.println("At pos 1:  pos:");
  Serial.println(LobotSerialServoReadPosition(Serial8, setID));*/
  
  //delay(2000000);
  /*
  LobotSerialServoMove(Serial8, setID, 800, 3000);
  Serial.println("800");
  delay(10000);*/
  /*Serial.println("Finsihed");
  Serial.println(LobotSerialServoReadPosition(Serial8, setID));
  delay(2000);
  delay(1);
  LobotSerialServoMove(Serial8, setID, 200, 3000);
  delay(3000);*/
  
  
  /*
  //LobotSerialServoSetID(Serial8, ID1, setID);
  // put your main code here, to run repeatedly:
  digitalWrite(TX_EN,HIGH); //turn into transmit mode
  digitalWrite(RX_EN,LOW);
  delay(1);
  LobotSerialServoMove(Serial8, setID, 100, 500);
  delay(3000);
  delay(1);
  Serial.println("At pos 1:  pos:");
  Serial.println(LobotSerialServoReadPosition(Serial8, setID));
  //Serial.println(LobotSerialServoReadVin(Serial8, setID));
  delay(2000);
  //Serial.println(LobotSerialServoReadVin(Serial8, ID1));
  //LobotSerialServoMove(Serial8, ID2, 500, 500);
  delay(1);
  LobotSerialServoMove(Serial8, setID, 500, 500);
  delay(2000);
  Serial.println("At pos 2:  pos:");
  Serial.println(LobotSerialServoReadPosition(Serial8, setID));
  delay(2000);
  
  
  delay(1000);
  Serial.println("pos 2:");
  Serial.println(LobotSerialServoReadPosition(Serial8, ID1));
  delay(2000);
  digitalWrite(TX_EN,LOW); //turn into transmit mode
  digitalWrite(RX_EN,HIGH);
  delay(2000);*/
  /*  Serial.println(LobotSerialServoReadPosition(Serial8, 1));
  //LobotSerialServoMove(Serial8, ID2, 600, 500);
  delay(1000);
  LobotSerialServoMove(Serial8, ID1, 900, 500);
  delay(50);
  Serial.println(LobotSerialServoReadPosition(Serial8, 1));
  //LobotSerialServoMove(Serial8, ID2, 700, 500);
  delay(1000);
  LobotSerialServoMove(Serial8, ID1, 500, 500);
  //LobotSerialServoMove(Serial8, ID2, 600, 500);*/
  
 // delay(1000000);
}
