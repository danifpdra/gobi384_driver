
/**************************************************************************/
/*!
This is a demo for the Adafruit MCP9808 breakout
----> http://www.adafruit.com/products/1782
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!
*/
/**************************************************************************/

#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor2= Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor3 = Adafruit_MCP9808();


ros::NodeHandle nh;
std_msgs::Float32MultiArray Temp_sensors;
ros::Publisher Temp_pub("Temp_sensors", &Temp_sensors);
char dim0_label[] = "Temp_sensors";

void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();

  tempsensor1.begin(0x18);
  tempsensor2.begin(0x1F);
  tempsensor3.begin(0x1A);

  Temp_sensors.layout.dim = (std_msgs::MultiArrayDimension *)

  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);

  Temp_sensors.layout.dim[0].label = dim0_label;
  Temp_sensors.layout.dim[0].size = 3;
  Temp_sensors.layout.dim[0].stride = 1 * 3;
  Temp_sensors.layout.dim_length = 0; // tirar esta linha se der chatices
  Temp_sensors.layout.data_offset = 0;

  // Defenição do array de dados concreto
  Temp_sensors.data_length = 3;

  Temp_sensors.data = (float *)malloc(sizeof(float) * 3);

  nh.advertise(Temp_pub);
  
//  Serial.begin(9600);
//  while (!Serial); //waits for serial terminal to be open, necessary in newer arduino boards.
//  Serial.println("MCP9808 demo");
  
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example, also can be left in blank for default address use
  // Also there is a table with all addres possible for this sensor, you can connect multiple sensors
  // to the same i2c bus, just configure each sensor with a different address and define multiple objects for that
  //  A2 A1 A0 address
  //  0  0  0   0x18  this is the default address
  //  0  0  1   0x19
  //  0  1  0   0x1A
  //  0  1  1   0x1B
  //  1  0  0   0x1C
  //  1  0  1   0x1D
  //  1  1  0   0x1E
  //  1  1  1   0x1F

//  if (!tempsensor1.begin(0x18) || !tempsensor2.begin(0x1F) || !tempsensor3.begin(0x1A)) {
//    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
//    while (1);
//  }
//    
//   Serial.println("Found MCP9808!");

  tempsensor1.setResolution(1); // sets the resolution mode of reading, the modes are defined in the table bellow:
  tempsensor2.setResolution(1);
  tempsensor3.setResolution(1);
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms



  // A partir daqui mudar todos os 2 para o num de elem do array
  // Definição do cabeçalho para a mensagem do tipo Float32MultiArray


}

void loop() {
//  Serial.println("wake up MCP9808.... "); // wake up MCP9808 - power consumption ~200 mikro Ampere
  tempsensor1.wake();   // wake up, ready to read!
  tempsensor2.wake();   // wake up, ready to read!
  tempsensor3.wake();   // wake up, ready to read!

  float c1 = tempsensor1.readTempC();
//  Serial.print("Temp1: "); 
//  Serial.print(c1, 4); Serial.print("*C\t and "); 
//  Serial.print(f1, 4); Serial.println("*F.");

  float c2 = tempsensor2.readTempC();
//  Serial.print("Temp2: "); 
//  Serial.print(c2, 4); Serial.print("*C\t and "); 
//  Serial.print(f2, 4); Serial.println("*F.");

  float c3 = tempsensor3.readTempC();
//  Serial.print("Temp3: "); 
//  Serial.print(c3, 4); Serial.print("*C\t and "); 
//  Serial.print(f3, 4); Serial.println("*F.");

  
  Temp_sensors.data[0] = c1;
  Temp_sensors.data[1] = c2;
  Temp_sensors.data[2] = c3;

  Temp_pub.publish(&Temp_sensors);

  nh.spinOnce();
  // 50Hz - 20
  delay(20);
//  ROS_INFO("Spinned once");
//  Serial.println("Shutdown MCP9808.... ");
  tempsensor1.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
  tempsensor2.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
  tempsensor3.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling

//  Serial.println("");
//  delay(200);
}
