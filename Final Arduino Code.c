/*
**************************************************************************************************************
 * Computergestützte Experimente und Signalauswertung
 * PHY.W04UB/UNT.038UB
 *
 * Anwendung: Messung einer Lichtquelle - Unterscheidung Kerzenlicht und LED
 * Sensoren: TMD4903 (Lichtintensität, Clear, Red, Green, Blue, IR), MLX90614 (IR)
 *
 * Gruppe: Brunke, Cas, Royer
**************************************************************************************************************
*/

// Include necessary libraries 
#include <Wire.h>  // Library für I2C Kommunikation 
#include <Servo.h> // Servo library integrieren 
#include <Adafruit_MLX90614.h> // Adafruit Library für den Infrarot Sensor MLX90614

#define uchar unsigned char // Kürzel definieren für unsigned character 

// I2C Device Address TMD4903 Sensor definieren 
#define DEV_ADDR 0x39

// Definieren einiger Variablen und Checkpoints
// Boolean (true/false) für Überprüfung Initialisierung TMD4903
bool didTMD4903InitWork = false; // im Setup auf false stellen, erst wenn funktioniert auf true (später)
bool RunServoProgram = false; // den boolean "RunProgram" als Checkpoint (später) zuerst als false festlegen
bool RunMeasurement = false;
bool RunMeasurementWithAverage = false;


// Schrittweite in ° als Konstante definieren (kann man dann unten wie Variable verwenden)
// Ebenso für Start- und Stoppwerte für horizontales und vertikales Rastern
#define STEPSIZE 10 
#define horizontalStepsize 10
#define verticalStepsize 5
#define horizontalRangeStart 0
#define verticalRangeStart 45
#define horizontalRangeEnd 190
#define verticalRangeEnd 140

Servo horizontalServo; // Objekt Servo erstellen -> horizontal und vertikal für die jeweiligen Servos 
Servo verticalServo; 

// Objekte von der Klasse des jeweiligen Sensors erstellen 
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); 
 
/////////////////////////////////////////////////////////////////////////////////////////////////
//Funktionen definieren 

// I2C Write function:
// requires:
// - Device Address
// - Register Address
// - Data which will be sent to the device
// - The Data is only one byte (unsigned char)
// returns if the transmission was successful or not.


// I2C Write Funktion 
bool I2C_Write( uchar IC_Addr , uchar Reg_Addr , uchar dataToI2C )
{
  Wire.begin(); // startet Kommunikation am I2C
  Wire.beginTransmission(IC_Addr); // ÜBertragung zu I2C device dieser Adresse starten
  Wire.write(Reg_Addr); // register address zum I2C device senden 
  Wire.write(dataToI2C); // Daten zum Register der definierten Adresse schicken
  if(Wire.endTransmission()==0) // Übertragung beenden, wenn Check erfolgreich wear
    return true;
  return false;
}

// I2C BlockRead function:
// requires:
// - Device Address
// - Register Address
// - How many bytes to read
// - Pointer to where the read data should be stored
// returns how many bytes have been received

uchar I2C_BlockRead( uchar IC_Addr , uchar Reg_Addr , uchar Bytes_requested , uchar DataFromI2C[] )
{
    unsigned char Bytes_received = 0;             //Bytes_received Zähler erstellen und auf 0 setzen
    Wire.beginTransmission(IC_Addr);              // Übertragung beginnen
    Wire.write(Reg_Addr);                         // Register Adresse
    Wire.endTransmission();                       // I2C Übertragung stoppen
    Wire.requestFrom(IC_Addr, Bytes_requested);   // Request the Amount of Bytes
    Wire.endTransmission();
    uchar remaining = Bytes_requested;
  
    while (Wire.available() && remaining--)
        {
            DataFromI2C[Bytes_received] = Wire.read();
            ++Bytes_received;
        }

  return Bytes_received;
}

// Funktion für den TMD4903
bool InitTMD4903(uchar DeviceAddress , uchar Gain , int IntTime_ms )
{
  delay(20);
  Serial.println("#Vor Write"); //Checkpoint, ob bis hier funktioniert hat
  delay(20);                    // # vor alle Checkpoint strings wegen Auslesen mit Python!
  bool status = false;
  
  // Integration time between 2.78 and 711 ms
  // Stellt sicher, dass integration time in diesem Intervall liegt 
  //- wenn größer als 711 ms wird auf 711 gestellt
  // wenn kleiner als 3, wird auf 3 gestellt
  if( IntTime_ms > 711 )
    IntTime_ms = 711;   // kann man ohne Klammern schreiben, weil nur eine Zeile 
  if( IntTime_ms < 3)
    IntTime_ms = 3;

// Calculate the ATime Value of the Register, for more information check the TMD4903 Datasheet
//  Integration time in den Wert lt. Datenblatt umrechnen, damit Sensor Wert verwenden kann 
  uchar ATimeValue = ((IntTime_ms-711.0)/(-2.78));

  Serial.println("#Vor Write");
  delay(20);
  status = I2C_Write(DEV_ADDR,0x80,0x01); // 0x80 = Enable Register, schreibt 0x01 ins 0x80 Register
  Serial.println("#Nach Write");
  I2C_Write(DEV_ADDR,0x81,ATimeValue);    // 0x81 = ATime Register, die berechnete ATime wird hier eingetragen
  I2C_Write(DEV_ADDR,0x90, Gain&0x03 );   // 0x90 = Configuration Register One, Gain & 0x03 -> Gain mit 0x03 maskiert 
  I2C_Write(DEV_ADDR,0x9F,0x00);          // 0x9F = Configuration Register Two
  I2C_Write(DEV_ADDR,0x80,0x03);          // 0x80 = Enable Register, Turn on the ALS Engine
  return status;                          // sagt ob erste Zeile status erfolgreich war
}

// einzelne Sensor-Read-Funktionen noch einmal nicht als library 
// damit ich dann aufrufen kann in der average Funktion 

// MLX (IR)
float readMLXObjectTemp()
{return mlx.readObjectTempC();}
float readMLXAmbientTemp()
{return mlx.readAmbientTempC();}


// Messung MLX Object T mit Average 
float getAverageMLXObject(int numReadings)
{
  float total = 0;                          // Zähler total erstellen 
  for (int i = 0; i < numReadings; i ++)    
  {
    total += mlx.readObjectTempC();        // bei jeder Messung Wert zu total addieren
    delay(10);
  }
  float average = total/numReadings;      // Summe aller Messungen durch Anzahl 
  return average;
}

// Messung MLX Ambient T mit Average 
// selbe Funktion wie oben nur mit der Ambient Temperature
float getAverageMLXAmbient(int numReadings)
{
  float total = 0;
  for (int i = 0; i < numReadings; i ++)
  {
    total += readMLXAmbientTemp();
    delay(10);
  }
  float average = total/numReadings;
  return average;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP FUNKTIONEN
void setup() {

  Serial.begin(57600);                     // Baudrate festlegen
  Serial.println("#SetupCheckpoint1");     // Checkpoint ob generell Setup geht 
  
  didTMD4903InitWork = InitTMD4903( 0x39 , 3 , 100 ); // funktioniert Initialisierung?

  Serial.println("#Partytime2");

  // Je nachdem, ob funktioniert hat, entsprechende message ausgeben (quasi Checkpoint 3)
  if( didTMD4903InitWork )
    Serial.println("#TMD4903 works, let's go!");
  else
    Serial.println("#Init of TMD4903 did not work!");
  
  Serial.println("#Partytime3");

  horizontalServo.attach(3); // der Variable "horizontalServo" wird der Digitalpin 3 zugeordnet (an dem der horizontale Servo hängt)
  verticalServo.attach(5); // wie oben nur für den vertikalen Servo

  Serial.println("#Partytime4");


  // Servos auf Ausgangsposition stellen (0°)
  horizontalServo.write(0); // beide Servos zuerst einmal auf 0° stellen (geht, weil die Servo-Library bei servo.write(Grad) unterstützt)
  verticalServo.write(0); // mit dem 0° Einstellen werden die Servos zentriert, zum Überprüfen am Start

  Serial.println("#Servos sind ready :)");

  // Initialisieren IR-Sensor (Checkpoint 4)
  while (!mlx.begin())
  { //mlx.begin() gibt boolean aus, ob der Sensor bereit zum Starten ist oder nicht
    Serial.println("#Error connecting to MLX sensor. Check wiring."); //wenn Sensor noch nicht bereit, wird diese Fehlermeldung ausgegebe 
    while(1); //endlose Schleife 
  };
  // wie oben, wenn geht, kommt Programm aus Schleife heraus und gibt positive Rückmeldung: 
  Serial.println("#MLX fertig");
  delay (1000);

  Serial.println("#Partytime"); // Alles funktioniert, auf geht's! 
}

// Unterprogramm erstellen für Auslesen von TMD4903
void ReadOutTMDWithCRGBIRData( unsigned int *inputValues )
{
  if( didTMD4903InitWork )    // nur ausführen, wenn Init step funktioniert hat
        {
          unsigned char data_buffer[8]; // buffer zum Speichern der Daten vom Sensor
          unsigned int value[4];        // int zum Speichern von Values
          char string[30];


          I2C_Write( DEV_ADDR , 0xab , 0x00 ); 
          I2C_Write( DEV_ADDR , 0x80 , 0x01 ); 
          I2C_Write( DEV_ADDR , 0x80 , 0x03 ); 
          delay(120);                          

          uchar read_reg = I2C_BlockRead( DEV_ADDR , 0x94 , 8 , data_buffer );
          if(read_reg==0)
            Serial.println("#no Reading possible");
          else
          {
            inputValues[0] = data_buffer[1]*256 + data_buffer[0];
            inputValues[1] = data_buffer[3]*256 + data_buffer[2];
            inputValues[2] = data_buffer[5]*256 + data_buffer[4];
            inputValues[3] = data_buffer[7]*256 + data_buffer[6];  
          }

          I2C_Write( DEV_ADDR , 0xab , 0x40 );
          I2C_Write( DEV_ADDR , 0x80 , 0x01 );
          I2C_Write( DEV_ADDR , 0x80 , 0x03 );
          
          delay(120);

          read_reg = I2C_BlockRead( DEV_ADDR , 0x94 , 8 , data_buffer );
          if(read_reg==0) // testen, ob reading Funktion erfolgreich war
            Serial.println("#no Reading possible");
          else
          {
            // Alle bytes kombinieren im ZwischenValue
            unsigned long ZwischenValue =((data_buffer[1]*256 + data_buffer[0]) + (data_buffer[3]*256 + data_buffer[2]) + data_buffer[5]*256 + data_buffer[4] + data_buffer[7]*256 + data_buffer[6])/4; 
            if(ZwischenValue>65535) // Filter für Übersteuern
              ZwischenValue = 65535;
            inputValues[4] = (unsigned int)ZwischenValue;
          }
        }          
}


// 
void Messung( float hDeg, float vDeg )      // Unterprogramm erstellen für Messung
{
        unsigned int TMD_CRGBIR_Data[5];    	          // Array erstellen mit 5 Einträgen
        float TObject = getAverageMLXObject(5);         // Wert IR Object Temp (average aus 5 Messungen)
        float TAmbient = getAverageMLXAmbient(5);       // Wert IR Ambient Temp (average aus 5 Messungen)
        ReadOutTMDWithCRGBIRData ( TMD_CRGBIR_Data );   // TMD4903 auslesen und Werte in den vorher erstellten Array reinschreiben
        String a = String(",");
        Serial.println( String(hDeg) + a + String(vDeg) + a + String(TObject) + a + String(TAmbient) + a + String(TMD_CRGBIR_Data[0]) + a + String(TMD_CRGBIR_Data[1]) + a + String(TMD_CRGBIR_Data[2]) + a + String(TMD_CRGBIR_Data[3]) + a + String(TMD_CRGBIR_Data[4]));
}     

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void loop: Main Program folgt hier: 
void loop() {

  // Buffer anschauen -> Serial.available => wie viele Briefe im Postkastl
  if(Serial.available()>0) // schaut ins "Postkastl", ob schon was da is (Zahl der "Briefe" > 0 )
    {
      char readValue = Serial.read();
       // macht jetzt "Brief" -> liest Info aus Input- Buffer ein 

    ////// Jetzt folgen die Unterprogramme, die ich mit einzelnen Buchstaben ansprechen kann

    // m: Programm TMD4903 Einmalmessung bei 0,0
    if( readValue == 'm') 
    {
      Messung( 0 , 0 );
      Serial.println("exit"); // exit damit man mit Python auslesen kann
    }

    // o: Programm Rastern mit allen Sensoren 
    if (readValue == 'o')
    {
      // Servos auf Startposition stellen 
      // langer Delay für Aufbau der Sensoren
      horizontalServo.write(0); 
      verticalServo.write(45);
      delay(5000);

      //Doppelschleife über den gesamten Bereich
      for (int hDegrees = horizontalRangeStart; hDegrees < horizontalRangeEnd; hDegrees += horizontalStepsize)
      {
        //zuerst horizontalen Servo in Position bringen 
        horizontalServo.write(hDegrees);
        delay(50);

        for (int vDegrees = verticalRangeStart; vDegrees < verticalRangeEnd; vDegrees += verticalStepsize) 
        {
          float readValue = Serial.read();
          if (readValue == 'x') 
          {
            return;
          }
          verticalServo.write(vDegrees);
          delay(50);

          Messung(hDegrees, vDegrees); // hier Messung durchführen (an jeder Position)
        }
     }
     Serial.println("exit");
    }

    // a: Programm MLX90614 Einmalmessung
    if( readValue =='a') // Programm IR Sensor
    {
        // Messung mit MLX90614
        float Ambient = readMLXAmbientTemp();
        float TObject = readMLXObjectTemp();
        Serial.println( String(Ambient) + String(",") + String(TObject));
        Serial.println("exit");
    }





