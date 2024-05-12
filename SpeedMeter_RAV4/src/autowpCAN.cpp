#include <Arduino.h>
#include <SPI.h>
#include <MCP2515.h>
#include <Wire.h>

#define LED PC13
#define CLK_1 PA5
#define MOSI_1 PA7
#define CS_CAN_GW PA4
#define CAN_BAUDRATE (500000)

MCP2515 mcp2515(CS_CAN_GW);                                     // Creates MCP2515 object instance

unsigned long startMillis = 0;
unsigned long currentMillis = 0;
const unsigned long pollPeriod = 250;
unsigned int TimerTick = 0;

// put function declarations here:
void initPINs()
{
  pinMode(LED, OUTPUT);
  delay(100);  
}

void initSerial()
 {
  Serial.begin(9600, SERIAL_8N1);   
  delay(100);
 }

void initSPI()
{
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);                // According to CAN spec: CAN frmes are always big endan, msb first
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
}

void OBD2_request()
{
  /*Variable declatarions*/
  int CANID = 0x7DF;                        // General broadcast request (since I do not know which ECU will respond, I send it to everybody)
  char mode = 0x01;
  char dVehSpeed = 0x0D;                    // PID ID 0x0D = vehicle speed (1 byte lenght)
  char dMotRPM = 0x0C;                      // PID ID 0x0C = engine speed (2 byte lenght)
  
  delay(10);

  /*Send frame*/
//  mcp2515.beginPacket(CANID);  // Start Frame
//  mcp2515.write(0x02);         // numbers of following bytes (DLC)
//  mcp2515.write(mode);
//  mcp2515.write(dVehSpeed);    // data to request (PID)
//  mcp2515.write(0xCC);         // padding...
//  mcp2515.write(0xCC);
//  mcp2515.write(0xCC);
//  mcp2515.write(0xCC);
//  mcp2515.write(0xCC);
//  mcp2515.endPacket();        // End Frame

  //mcp2515.end();

  /*Do diagnostic stuff and return True to restart timer*/
    digitalWrite(LED, !digitalRead(LED));
  //Serial.println("OBD request sent...");  
}

void UDS_ReqDiagSession()
{
  /*Variable declatarions*/
  struct can_frame rqstDiagSession;

  Serial.println("Requesting Diagnostic session...");

  /*Send frame*/
  rqstDiagSession.can_id = 0x7DF;
  rqstDiagSession.can_dlc = 0x02;          // 0. DLC: 2 bytes will be transferred
  rqstDiagSession.data[0] = 0x10;          // 1. SID: Request diagnostic session (0x10)
  rqstDiagSession.data[1] = 0x01;          // 2. SUB-FUNCTION: Default session (0x01)
  rqstDiagSession.data[2] = 0xCC;          // 3. padding...
  rqstDiagSession.data[3] = 0xCC;          // 4. 
  rqstDiagSession.data[4] = 0xCC;          // 5.
  rqstDiagSession.data[5] = 0xCC;          // 6.
  rqstDiagSession.data[6] = 0XCC;          // 7.

  /*
  mcp2515.beginPacket(0x7E0);   // Start Frame (0x7DF)
  mcp2515.write(0x02);          // PCI lenght
  mcp2515.write(0x10);          // SID: Request diagnostic session (0x10)
  mcp2515.write(0x01);          // SUB-FUNCTION: Default session (0x01)
  mcp2515.write(0xCC);          // padding...
  mcp2515.write(0xCC);          
  mcp2515.write(0xCC);
  mcp2515.write(0xCC);
  mcp2515.write(0xCC);
  mcp2515.endPacket();          // End Frame
  */
}

void UDS_TesterPresent()
{
 /*Variable declatarions*/
  struct can_frame TstrPresent;
  int CANID = 0x7DF;                      // General broadcast request (since I do not know which ECU will respond, I send it to everybody)
  char PCI = 0x02;                        // PCI field_ 1st 4 bit = frame type (0x0 for Single Frame), 2nd 4 bit = lenght of request (0x3 or 3 byte sent)
  char SID = 0x3E;                        // SID = Service Identifier: 0x3E = Tester Present 
//  char DID_1 = ;                        // DID = Data identifier: 
//  char DID_2 = ;                        // DID = Data identifier: 

  Serial.println("Sending TESTER PRESENT");

  /*Send frame*/
  TstrPresent.can_id = 0x7DF;             // Start Frame
  TstrPresent.can_dlc = 0x02;             // 0. PCI lenght (2 byte)
  TstrPresent.data[0] = 0x3E;             // 1. SID: Tester present (0x3E)
  TstrPresent.data[1] = 0x80;             // 2. SUPPRESS POSITIVE RESPONSE 
  TstrPresent.data[2] = 0xCC;             // 3. padding...
  TstrPresent.data[3] = 0xCC;             // 4.
  TstrPresent.data[4] = 0xCC;             // 5. 
  TstrPresent.data[5] = 0xCC;             // 6.
  TstrPresent.data[6] = 0xCC;             // 7.
}

void UDS_requestData()
{
 /*Variable declatarions*/
  struct can_frame rqstSpeed;
  int CANID = 0x7DF;                        // General broadcast request (since I do not know which ECU will respond, I send it to everybody)
  char PCI = 0x03;                          // PCI field_ 1st 4 bit = frame type (0x0 for Single Frame), 2nd 4 bit = lenght of request (0x3 or 3 byte sent)
  char SID = 0x22;                          // SID = Service Identifier: 22 = Read Data by identifier
  char DID_1 = 0xF4;                        // DID = Data identifier: Vehicle speed = 0xF4D. This is the 1st byte
  char DID_2 = 0x0D;                        // DID = Data identifier: Vehicle speed = 0xF40D. This is the 2nd byte
  
  Serial.println("Requesting UDS data...");

  /*Send frame*/
  rqstSpeed.can_id = 0x7DF;                 // Start Frame
  rqstSpeed.can_dlc = 0x03;                 // 0. 3 bytes will be sent
  rqstSpeed.data[0] = 0x22;                 // 1. SID -> Read data by ID
  rqstSpeed.data[1] = 0xF4;                 // 2. DID -> Vehicle speed 1st byte
  rqstSpeed.data[2] = 0x0D;                 // 3. DID -> Vehicle speed 2nd byte
  rqstSpeed.data[3] = 0xCC;                 // 4. padding...
  rqstSpeed.data[4] = 0xCC;                 // 5.
  rqstSpeed.data[5] = 0xCC;                 // 6.
  rqstSpeed.data[6] = 0xCC;                 // 7.

  digitalWrite(LED, !digitalRead(LED));
}

void CAN_Listen()
{
  struct can_frame RXcanMsg;
  //Serial.println();
  //Serial.println("ID DLC DATA");

  if (mcp2515.readMessage(&RXcanMsg) == MCP2515::ERROR_OK) 
  {
    Serial.print(RXcanMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(RXcanMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<RXcanMsg.can_dlc; i++)  {  // print the data
      Serial.print(RXcanMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();  
  }
  else
  {
    Serial.println("No packet received...");
  }
} 

void WaitForResp()
{
  unsigned int attemptCounter = 0;
  while (attemptCounter < 20)
    {
      CAN_Listen();
      attemptCounter = attemptCounter +1;
    }
  Serial.println("Listening for response finished...");
}

void setup() 
{
  /*put your setup code here, to run once:*/
  // Init Pheripherials
    initPINs();
    initSPI();
    initSerial();

    delay(200);

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    //mcp2515.setLoopbackMode();
    mcp2515.setNormalMode();

    Serial.println("Pheripherial init finished...");
    delay(200);

    UDS_ReqDiagSession();
    WaitForResp();

    UDS_requestData();
    WaitForResp();
}

void loop() 
{
  //CAN_Listen();

  // put your main code here, to run repeatedly:
  /*
  currentMillis = millis();
  if (currentMillis - startMillis >= pollPeriod)
  {
    UDS_requestData();
    startMillis = currentMillis;
    TimerTick = TimerTick + 1;
    if (TimerTick == 8)
    {
      UDS_TesterPresent();
      TimerTick = 0;
    }
  }
  else
  {
    CAN_Listen();
  }
*/

  //initSPI();
  //PutChar(0x31, 0);
  //Serial.println("looping...");
}
