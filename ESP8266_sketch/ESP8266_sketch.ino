#define X        0
#define Y        1
#define Z        2

#define P        0
#define I        1
#define D        2

#define LF       0
#define LB       1
#define RF       2
#define RB       3

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define EEPROMESC 0
#define EEPROMMPU 1

//Библиотеки
#include <ESP8266WiFi.h>  //WiFi
#include <ArduinoJson.h>  //JSON
#include <Servo.h>        //Управление двигателями
#include <Wire.h>         //I2C
#include <EEPROM.h>       //Энергонезависимая память
#include "MadgwickAHRS.h" //Фильтр Маджвика

//Переменные
Servo ESC[4]; //Массив контроллеров оборотов двигателя

// Выбираем пины SDA и SCL для коммуникации по I2C
const uint8_t scl = D1;
const uint8_t sda = D2;

//Факторы чувствительности из технической документации MPU6050
const float AccelScaleFactor = 16384;
const float GyroScaleFactor  =   131;

// MPU6050 I2C адрес в режиме slave
const uint8_t MPU6050SlaveAddress = 0x68;

//Адреса регистров конфигурации MPU6050
const uint8_t MPU6050_REGISTER_SMPLRT_DIV         = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL          = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1         = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2         = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG             = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG        = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG       = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN            = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE         = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H       = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

Madgwick filter;

uint8_t NeedCalibMPU = 0; //Флаг необходимости калибровки IMU
uint8_t NeedCalibESC = 0; //Флаг калибровки ESC

float   AccelOffset[3];   //Коэффициенты отклонений акселлерометра
float   GyroOffset [3];   //Коэффициенты отклонений гироскопа

int16_t AccelMPU [3];     //Сырые значения акселлерометра и
int16_t GyroMPU  [3];     //гироскопа с MPU6050

float   Accel[3];         //Обработанные значения акселлерометра и
float   Gyro [3];         //гироскопа

float   Euler[3];         //Получаемые после фильтра углы Эйлера

unsigned long startMillis = 0; //Таймер для конвертации градусов/секунду
unsigned long deltaMillis;     //Дельта времени
float   fps = 100;             //Переменная для хранения частоты выборок фильтра

//Значения с контроллера
int   Controller[4];                          //Желаемые углы
float ControllerPID[3] = {0.235, 0.0, 0.0};   //Значения PID

int   PWM[4];                                 //Шим для ESC

float Desired[4] = {0, 0, 0, 1000};           //Ожидаемые значения углов и тяги

// ------------- Global variables used for PID automation --------------------
float errors[3];                     // Ошибки: [Yaw, Pitch, Roll]
float error_sum[3]      = {0, 0, 0}; // Сумма ошибок для интегральной части: [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Предыдущие ошибки для производной части: [Yaw, Pitch, Roll]
// ---------------------------------------------------------------------------

//Переменные для защиты двигателя
bool mot_activated      = false;
long activate_count     =     0;
long des_activate_count =     0;

WiFiServer server(8888);

void setup()
{
  //Читаем из энергонезависимой памяти флаги калибровки
  EEPROM.begin(512);
  int NeedCalibMPU = EEPROM.read(EEPROMMPU);
  int NeedCalibESC = EEPROM.read(EEPROMESC);

  //Выделяем пины для ESC
  ESC[LF].attach(D3);              //Левый передний двигатель
  ESC[LB].attach(D4);              //Левый задний двигатель
  ESC[RF].attach(D5);              //Правый передний двигатель 
  ESC[RB].attach(D6);              //Правый задний двигатель

  if (NeedCalibESC == 1) 
  {
    Serial.println("Настройка ESC.");
    Serial.println("Установлено максимальное значение.");
    ESC[LF].writeMicroseconds(1800); //Подаем максимальный сигнал
    ESC[LB].writeMicroseconds(1800);
    ESC[RF].writeMicroseconds(1800); 
    ESC[RB].writeMicroseconds(1800);
    delay(5000);
    Serial.println("Установлено минимальное значение.");
    ESC[LF].writeMicroseconds(1100); //Подаем минимальный сигнал
    ESC[LB].writeMicroseconds(1100);
    ESC[RF].writeMicroseconds(1100); 
    ESC[RB].writeMicroseconds(1100);
    delay(5000);
    Serial.println("Настройка ESC завершена.");
    EEPROM.write(EEPROMESC, 0);
    EEPROM.commit();
    delay(500);
    ESP.restart();
  }
  else 
  {
  ESC[LF].writeMicroseconds(1000); //Чтобы убедиться что ESC не войдут в режим настройки
  ESC[LB].writeMicroseconds(1000); //подаем на них минимальный сигнал
  ESC[RF].writeMicroseconds(1000); 
  ESC[RB].writeMicroseconds(1000);
  }
  
  //Запускаем WIFI сервер на локальном адресе 192.168.4.1
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Test", "12345678");
  server.begin();

  //Инициализируем MPU6050
  Wire.begin(sda, scl);
  MPU6050_Init();
  if (NeedCalibMPU == 1) MPU6050_Calibration(50);
  else 
  {
    AccelOffset[X] =   0.051536;
    AccelOffset[Y] =  -0.005274; 
    AccelOffset[Z] =  -0.044106;
    GyroOffset[X]  = -15.273678;
    GyroOffset[Y]  =   2.068484;
    GyroOffset[Z]  =  -0.68353f;
    startMillis    =   millis();
  }
}

WiFiClient client;

void loop()
{ 
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //Делим все параметры на фактор чувствительности и вычитаем ошибку датчика
  Accel[X] = (float)AccelMPU[X] / AccelScaleFactor - AccelOffset[X];
  Accel[Y] = (float)AccelMPU[Y] / AccelScaleFactor - AccelOffset[Y];
  Accel[Z] = (float)AccelMPU[Z] / AccelScaleFactor - AccelOffset[Z];
  Gyro[X]  = (float)GyroMPU[X]  / GyroScaleFactor  -  GyroOffset[X];
  Gyro[Y]  = (float)GyroMPU[Y]  / GyroScaleFactor  -  GyroOffset[Y]; //-
  Gyro[Z]  = (float)GyroMPU[Z]  / GyroScaleFactor  -  GyroOffset[Z];

  deltaMillis = millis() - startMillis;
  fps = 1000 / (deltaMillis); //Вычисляем частоту обработки фильтра через время обработки данных
  startMillis = millis();
  
  filter.begin(fps);
  filter.updateIMU(Gyro[X], Gyro[Y], Gyro[Z], Accel[X], Accel[Y], Accel[Z]);
 
  //Получение углов yaw, pitch и roll из фильтра
  Euler[YAW]   = filter.getYaw();
  Euler[PITCH] = filter.getPitch();
  Euler[ROLL]  = filter.getRoll();

  if (!client.connected())
  {
    client = server.available();
  }
  else
  {
    if (client.available() > 0)
    {
      String JSONMessage = client.readStringUntil('}');
      JSONMessage += '}';
      Serial.println("Получено необработанное сообщение:" + JSONMessage);
        
      StaticJsonBuffer<300> JSONBuffer;  //Пул памяти
      JsonObject& parsed = JSONBuffer.parseObject(JSONMessage);  //Парсим сообщение

      if (!parsed.success()) //Проверяем парсер на ошибки
      {   
        Serial.println("Не получилось распарсить сообщение.");
        //return;
      }
        
      //Получаем обработанные значения с контроллера
      bool Type = parsed["Type"];
      mot_activated = parsed["Enabled"];
      if (Type)
      {
        for (int i = 0; i < 4; i++)
        {
        Controller[i] = parsed["Controller"][i];
        }
      }
      else
      {
        for (int i = 0; i < 3; i++)
        {
          ControllerPID[i] = parsed["K"][i];
        }
        
        if (parsed["ESC"] == true)
        {
          EEPROM.write(EEPROMESC, 1);
        }
        else 
        {
          EEPROM.write(EEPROMESC, 0);
        }
        
        if (parsed["MPU"] == true)
        {
          EEPROM.write(EEPROMMPU, 1);
        }
        else 
        {
          EEPROM.write(EEPROMMPU, 0);
        }
        EEPROM.commit();                //Запись в EEPROM
      }
    }
  }

  
  /*///////////////////////////P I D///////////////////////////////////*/
  Desired[YAW]      = map(     Controller[YAW], 1000, 2000,    0,  360);
  Desired[ROLL]     = map(    Controller[ROLL], 1000, 2000,  -25,   25);
  Desired[PITCH]    = map(   Controller[PITCH], 1000, 2000,  -25,   25);
  Desired[THROTTLE] = map(Controller[THROTTLE], 1000, 2000, 1100, 1800);

  calculateErrors();
  automation();

  //Лог для тестирования
  Serial.print("T: ");
  Serial.print(Controller[THROTTLE]);
  Serial.print(" Y: ");
  Serial.print(Controller[YAW]);
  Serial.print(" P: ");
  Serial.print(Controller[PITCH]);
  Serial.print(" R: ");
  Serial.print(Controller[ROLL]);

  Serial.print(" | P: ");
  Serial.print(ControllerPID[P]*1000);
  Serial.print(" I: ");
  Serial.print(ControllerPID[I]*1000);
  Serial.print(" D: ");
  Serial.print(ControllerPID[D]*1000);

  Serial.print(" | LF: ");
  Serial.print(PWM[LF]);
  Serial.print(" LB: ");
  Serial.print(PWM[LB]);
  Serial.print(" RF: ");
  Serial.print(PWM[RF]);
  Serial.print(" RB: ");
  Serial.print(PWM[RB]);

  Serial.print(" | X: ");
  if (Euler[ROLL]>=0) Serial.print(" ");
  Serial.print(Euler[ROLL]);
  Serial.print(" Y: ");
  if (Euler[PITCH]>=0) Serial.print(" ");
  Serial.print(Euler[PITCH]);
  Serial.print(" Z: ");
  if (Euler[YAW]>=0) Serial.print(" ");  
  Serial.print(Euler[YAW]);
  Serial.println(" ");

  ESCWrite();         //Запись в ESC
}

void automation() {
  float EulerPID[3] = {0, 0, 0};                //Углы после обработки PID контроллером

  //Инициализируем двигатели с ожидаемой тягой 
  int pulseLengthEsc[4] = {Desired[THROTTLE], Desired[THROTTLE], Desired[THROTTLE], Desired[THROTTLE]};

  //Ничего не рассчитываем если тяга 0
  if ((Desired[THROTTLE] >= 1100) && (mot_activated)) 
  {
    //PID = e.Kp
    EulerPID[PITCH] = (errors[PITCH] * ControllerPID[P]);
    EulerPID[ROLL]  = (errors[ROLL]  * ControllerPID[P]);
    
    //Рассчитываем сумму ошибков для Интегральной части (I)
    //error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    //PID += ∫e.Ki
    EulerPID[PITCH] += (error_sum[PITCH] * ControllerPID[I] * deltaMillis);
    EulerPID[ROLL]  += (error_sum[ROLL]  * ControllerPID[I] * deltaMillis);

    //PID += (-)Δe.Kd
    EulerPID[PITCH] += ((errors[PITCH] - previous_error[PITCH]) * ControllerPID[D] / deltaMillis);
    EulerPID[ROLL]  += ((errors[ROLL]  - previous_error[ROLL])  * ControllerPID[D] / deltaMillis);

    //Сохраняем текущую ошибку как предыдущую
    //previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];

    //Рассчитываем значения ШИМ для ESC
    /*
    pulseLengthEsc[LF] = Desired[THROTTLE] - EulerPID[YAW] + EulerPID[PITCH] - EulerPID[ROLL];
    pulseLengthEsc[RF] = Desired[THROTTLE] + EulerPID[YAW] + EulerPID[PITCH] + EulerPID[ROLL];
    pulseLengthEsc[LB] = Desired[THROTTLE] + EulerPID[YAW] - EulerPID[PITCH] - EulerPID[ROLL];
    pulseLengthEsc[RB] = Desired[THROTTLE] - EulerPID[YAW] - EulerPID[PITCH] + EulerPID[ROLL];
    */
    pulseLengthEsc[LF] = minMax((Desired[THROTTLE]  + EulerPID[PITCH] - EulerPID[ROLL]),Desired[THROTTLE]-400,Desired[THROTTLE]+400);
    pulseLengthEsc[RF] = minMax((Desired[THROTTLE]  + EulerPID[PITCH] + EulerPID[ROLL]),Desired[THROTTLE]-400,Desired[THROTTLE]+400);
    pulseLengthEsc[LB] = minMax((Desired[THROTTLE]  - EulerPID[PITCH] - EulerPID[ROLL]),Desired[THROTTLE]-400,Desired[THROTTLE]+400);
    pulseLengthEsc[RB] = minMax((Desired[THROTTLE]  - EulerPID[PITCH] + EulerPID[ROLL]),Desired[THROTTLE]-400,Desired[THROTTLE]+400);
  }

  PWM[LF] = minMax(pulseLengthEsc[LF], 1100, 2000);
  PWM[RF] = minMax(pulseLengthEsc[RF], 1100, 2000);
  PWM[LB] = minMax(pulseLengthEsc[LB], 1100, 2000);
  PWM[RB] = minMax(pulseLengthEsc[RB], 1100, 2000);
}

void calculateErrors() {
    //errors[YAW] = Euler[YAW] - Desired[YAW];
    errors[PITCH] = Euler[PITCH] - Desired[PITCH];
    errors[ROLL]  = Euler[ROLL]  - Desired[ROLL];
}

//Запись частоты вращения двигателя в ESC
void ESCWrite()
{
  if(mot_activated)
  {
  ESC[LF].writeMicroseconds(PWM[LF]); 
  ESC[LB].writeMicroseconds(PWM[LB]);
  ESC[RF].writeMicroseconds(PWM[RF]); 
  ESC[RB].writeMicroseconds(PWM[RB]);
  }
  if(!mot_activated)
  {
    ESC[LF].writeMicroseconds(1000); 
    ESC[LB].writeMicroseconds(1000);
    ESC[RF].writeMicroseconds(1000); 
    ESC[RB].writeMicroseconds(1000);
  }
}

//Запись в датчик MPU6050
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

//Считываем все 14 регистров MPU6050
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelMPU[X] = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelMPU[Y] = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelMPU[Z] = (((int16_t)Wire.read()<<8) | Wire.read());
  Wire.read(); //Пропускаем 2 байта,
  Wire.read(); //так как значение с датчика температуры не требуется
  GyroMPU[X]  = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroMPU[Y]  = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroMPU[Z]  = (((int16_t)Wire.read()<<8) | Wire.read());
}

//Первоначальная настройка MPU6050
void MPU6050_Init()
{
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00); //Установлено +/-250 градусов/секунду
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00); //Установлено +/- 2g
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
  Serial.println("Инициализация MPU6050.");
}

//Калибровка гироскопа и акселерометра MPU6050
void MPU6050_Calibration(int accuracy)
{
  delay(150);
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  AccelOffset[X] = (float)AccelMPU[X] / AccelScaleFactor;
  AccelOffset[Y] = (float)AccelMPU[Y] / AccelScaleFactor;
  AccelOffset[Z] = (float)AccelMPU[Z] / AccelScaleFactor;
  GyroOffset[X]  = (float)GyroMPU[X]  / GyroScaleFactor;
  GyroOffset[Y]  = (float)GyroMPU[Y]  / GyroScaleFactor;
  GyroOffset[Z]  = (float)GyroMPU[Z]  / GyroScaleFactor;
  Serial.println("Идет калибровка, не трогайте датчик!");
  Serial.print("Калибровка.");
  delay(100);
  
  for(int i = accuracy; i > 0; i--)
  {
    
    Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
    AccelOffset[X] = ((float)AccelMPU[X] / AccelScaleFactor + AccelOffset[X]) / 2;
    AccelOffset[Y] = ((float)AccelMPU[Y] / AccelScaleFactor + AccelOffset[Y]) / 2;
    AccelOffset[Z] = ((float)AccelMPU[Z] / AccelScaleFactor + AccelOffset[Z]) / 2;
    GyroOffset[X]  = ((float)GyroMPU[X]  / GyroScaleFactor  + GyroOffset[X])  / 2;
    GyroOffset[Y]  = ((float)GyroMPU[Y]  / GyroScaleFactor  + GyroOffset[Y])  / 2;
    GyroOffset[Z]  = ((float)GyroMPU[Z]  / GyroScaleFactor  + GyroOffset[Z])  / 2;
    Serial.print(".");
  }
  AccelOffset[Z]-=1; //Делаем поправку на 1g для оси Z
  startMillis = millis();
}

float minMax(float value, float min_value, float max_value) 
{
  if (value > max_value) 
  {
    value = max_value;
  } 
  else if (value < min_value) 
  {
    value = min_value;
  }
  return value;
}
