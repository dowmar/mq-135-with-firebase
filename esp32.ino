

//Include the library
#include <MQUnifiedsensor.h>
#include <FirebaseESP32.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiManager.h> 

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
String status_co2 = " ";
String status_co = " ";

#define FIREBASE_HOST ""
#define FIREBASE_AUTH ""

/************************Hardware Related Macros************************************/
#define         Board                   ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define         Pin                     (35) //check the esp32-wroom-32d.jpg image on ESP32 folder 
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-135") //MQ2 or other MQ Sensor, if change this verify your a and b values.
#define         Voltage_Resolution      (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         ADC_Bit_Resolution      (12) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         RatioMQ135CleanAir        (3.6) //RS / R0 = 9.83 ppm
/*****************************Globals***********************************************/
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
/*****************************Globals***********************************************/

FirebaseData firebaseData;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

void setup()
{

  //Init the serial port communication - to debug the library
  Serial.begin(115200); //Init serial port
  delay(10);

  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
//  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to to calculate H2 concentration
  
  //LCD
   // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();

  /*****************************  MQ Init ********************************************/ 
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/ 
  MQ135.init(); 
 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  //MQ2.serialDebug(true); uncomment if you want to print the table on the serial port
  

    // We start by connecting to a WiFi network
//
//    Serial.println();
//    Serial.println();
//    Serial.print("Connecting to ");
//    Serial.println(ssid);
//
//    WiFi.begin(ssid, password);
//
//    while (WiFi.status() != WL_CONNECTED) {
//        delay(500);
//        Serial.print(".");
//    }
 // WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    // it is a good practice to make sure your code sets wifi mode how you want it.

    // put your setup code here, to run once:
    Serial.begin(115200);
    
    //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
        Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.println("** Values from MQ-135 ****");
        Serial.println("|    CO   |  Alcohol |   CO2  |  Toluen  |  NH4  |  Aceton  |");  
    }
    
}

void loop()
{
//  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
//  //MQ2.serialDebug(); // Will print the table on the serial port
//  Serial.print(MQ135.readSensor()); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
//  Serial.println(" PPM");
//  delay(500); //Sampling frequency
 MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
  float Alcohol = MQ135.readSensor(); // SSensor will read   concentration using the model, a and b values set previously or from the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
  float Toluen = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
  float NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
  float Aceton = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  Serial.print("|   "); Serial.print(CO); 
  Serial.print("   |   "); Serial.print(Alcohol);
  // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29

  float CO2_final = CO2 + 400;

  Serial.print("   |   "); Serial.print(CO2_final); 
  Serial.print("   |   "); Serial.print(Toluen); 
  Serial.print("   |   "); Serial.print(NH4); 
  Serial.print("   |   "); Serial.print(Aceton);
  Serial.println("   |"); 

  if (CO2_final < 600){
    status_co2 = "NORMAL";
  }else if (CO2_final < 1000){
    status_co2 = "WASPADA";
  }else{
 status_co2 = "BAHAYA";
  }

  if (CO < 15){
    status_co = "NORMAL";
  }else if (CO < 50){
    status_co = "WASPADA";
  }else{
 status_co = "BAHAYA";
  }


//  Serial.print(status);
//  Firebase.setFloat(firebaseData, "/CO", CO);
//  Firebase.setFloat(firebaseData, "/CO2", CO2);

  Firebase.setFloat(firebaseData, "Polusi/CO_PPM", CO);
  Firebase.setFloat(firebaseData, "Polusi/CO2_PPM", CO2_final);
  Firebase.setString(firebaseData, "Polusi/status_co2", status_co2);
  Firebase.setString(firebaseData, "Polusi/status_co", status_co);
  


  
  lcd.clear();
    float x = CO2_final;
    float y = CO;
    lcd.setCursor(0, 0);
    lcd.print("CO2 = "); lcd.print(x,3); lcd.print("PPM");
    lcd.setCursor(0,1);
    lcd.print("CO = "); lcd.print(y,3); lcd.print("PPM");
    
  delay(500); //Sampling frequency
  
}
