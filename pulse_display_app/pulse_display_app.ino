#include <math.h> // (no semicolon)
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <stdio.h>
#include <SoftwareSerial.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
SoftwareSerial bluetooth(2,3); /* 블루투스 모듈 Rx,Tx*/

/* MPU-6050 sensor */
#define MPU6050_ACCEL_XOUT_H 0x3B // R
#define MPU6050_PWR_MGMT_1 0x6B // R/W
#define MPU6050_PWR_MGMT_2 0x6C // R/W
#define MPU6050_WHO_AM_I 0x75 // R
#define MPU6050_I2C_ADDRESS 0x68

int sensorValue_previous = 0;
int value;
int pulsePin = 0;
int blinkPin = 13;
unsigned long previousMillis = 0;
int State = LOW;
int persist = LOW;
/*센싱데이터 저장을 위한 문자열 선언*/
char AmbientTemp_buf[5] ="";
char ObjectTemp_buf[5] ="";
char Humidity_buf[3] ="";
char Average_buf[4] ="";
//센싱데이터를 하나의 버퍼에 모아주기위한 버퍼정
char temperature[23];
float temp;
int ad_conv(byte channel, byte num);
int calc_RH10(int adval);
int calc_TC10(int adval);
void display(int x);


volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

const int numReadings = 5;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;

/* Kalman filter */
struct GyroKalman{
  /* These variables represent our state matrix x */
  float x_angle, x_bias;

  /* Our error covariance matrix */
  float P_00, P_01, P_10, P_11;
  
  /*
  * Q is a 2x2 matrix of the covariance. Because we
  * assume the gyro and accelerometer noise to be independent
  * of each other, the covariances on the / diagonal are 0.
  * Covariance Q, the process noise, from the assumption
  * x = F x + B u + w
  * with w having a normal distribution with covariance Q.
  * (covariance = E[ (X - E[X])*(X - E[X])' ]
  * We assume is linear with dt
  */
  float Q_angle, Q_gyro;

  /*
  * Covariance R, our observation noise (from the accelerometer)
  * Also assumed to be linear with dt
  */
  float R_angle;
};

struct GyroKalman angX;
struct GyroKalman angY;
struct GyroKalman angZ;

/*
* R represents the measurement covariance noise. In this case,
* it is a 1x1 matrix that says that we expect 0.3 rad jitter
* from the accelerometer.
*/
static const float R_angle = 0.3;     //.3 default

/*
* Q is a 2x2 matrix that represents the process covariance noise.
* In this case, it indicates how much we trust the acceleromter
* relative to the gyros
*/
static const float Q_angle = 0.01;  //0.01 (Kalman)
static const float Q_gyro = 0.04; //0.04 (Kalman)

//These are the limits of the values I got out of the Nunchuk accelerometers (yours may vary).
const int lowX = -2150;
const int highX = 2210;
const int lowY = -2150;
const int highY = 2210;
const int lowZ = -2150;
const int highZ = 2550;


/* time */
unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;

typedef union accel_t_gyro_union
{
  struct
  {
  uint8_t x_accel_h;
  uint8_t x_accel_l;
  uint8_t y_accel_h;
  uint8_t y_accel_l;
  uint8_t z_accel_h;
  uint8_t z_accel_l;
  uint8_t t_h;
  uint8_t t_l;
  uint8_t x_gyro_h;
  uint8_t x_gyro_l;
  uint8_t y_gyro_h;
  uint8_t y_gyro_l;
  uint8_t z_gyro_h;
  uint8_t z_gyro_l;
  } reg;

  struct
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};

int xInit[5] = {0,0,0,0,0};
int yInit[5] = {0,0,0,0,0};
int zInit[5] = {0,0,0,0,0};
int initIndex = 0;
int initSize = 5;
int xCal = 0;
int yCal = 0;
int zCal = 1800;

void setup() {
  Serial.begin(9600);             // we agree to talk fast!
  interruptSetup();
  mlx.begin(); 
  Wire.begin();
  bluetooth.begin(9600); 
  int error;
  uint8_t c;

  initGyroKalman(&angX, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angY, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angZ, Q_angle, Q_gyro, R_angle);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
  
}



void loop() {
 
  unsigned long currentMillis = millis();
  if (State == HIGH )
  {
    previousMillis = currentMillis;
    State = LOW;
  }
  if (currentMillis - previousMillis <= 250) {
    persist = HIGH;
  }
  else
    persist = LOW;
// ATM1001 부분
int adval, RH10, TC10;


// MPU6050 부분
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  curSensoredTime = millis();

//  Serial.println(F(""));
//  Serial.println(F("MPU-6050"));
  
  // Read the raw values.
  // Read 14 bytes at once,
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
//  if(error != 0) {
//    Serial.print(F("Read accel, temp and gyro, error = "));
//    Serial.println(error,DEC);
//  }
  // Swap all high and low bytes.
  // After this, the registers values are swapped,
  // so the structure name like x_accel_l does no
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
  
  // ������ ���� �о�� ���ӵ� ��. �Ʒ� �ּ��� �����ϸ� Serial ��µ˴ϴ�.
  // Print the raw acceleration values
//  Serial.print(F("accel x,y,z: "));
//  Serial.print(accel_t_gyro.value.x_accel, DEC);
//  Serial.print(F(", "));
//  Serial.print(accel_t_gyro.value.y_accel, DEC);
//  Serial.print(F(", "));
//  Serial.print(accel_t_gyro.value.z_accel, DEC);
//  Serial.println(F(""));
  
  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet:
  // 340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412
//  Serial.print(F("temperature: "));
//  dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
//  Serial.print(dT, 3);
//  Serial.print(F(" degrees Celsius"));
//  Serial.println(F(""));
  
  // ������ ���� �о�� ���̷� ��. �Ʒ� �ּ��� �����ϸ� Serial ��µ˴ϴ�.
  // Print the raw gyro values.
//  Serial.print(F("gyro x,y,z : "));
//  Serial.print(accel_t_gyro.value.x_gyro, DEC);
//  Serial.print(F(", "));
//  Serial.print(accel_t_gyro.value.y_gyro, DEC);
//  Serial.print(F(", "));
//  Serial.print(accel_t_gyro.value.z_gyro, DEC);
//  Serial.println(F(""));


  /////////////////////////////////////////////////////////////////////////////
  //  Į������ �����ؼ� ������ ���� �����ϴ� ��ƾ
  /////////////////////////////////////////////////////////////////////////////
  
  if (QS == true) {
    //total = total - readings[readIndex];
    readings[readIndex] = BPM;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) {
      readIndex = 0;
      average = total / numReadings;                 //average, blutooth
      //Serial.println(average );

      //MPU부분
      if(prevSensoredTime > 0) {
    int gx1=0, gy1=0, gz1 = 0;
    float gx2=0, gy2=0, gz2 = 0;

    int loopTime = curSensoredTime - prevSensoredTime;

    gx2 = angleInDegrees(lowX, highX, accel_t_gyro.value.x_gyro);
    gy2 = angleInDegrees(lowY, highY, accel_t_gyro.value.y_gyro);
    gz2 = angleInDegrees(lowZ, highZ, accel_t_gyro.value.z_gyro);

    predict(&angX, gx2, loopTime);
    predict(&angY, gy2, loopTime);
    predict(&angZ, gz2, loopTime);

    gx1 = update(&angX, accel_t_gyro.value.x_accel) / 10;
    gy1 = update(&angY, accel_t_gyro.value.y_accel) / 10;
    gz1 = update(&angZ, accel_t_gyro.value.z_accel) / 10;

    /////////////////////////////////////////////////////////////////////////////
    //  ���� ����� �� �����Ǵ� n���� ���� ��� => ���� �����Ǵ� ���� ����
    /////////////////////////////////////////////////////////////////////////////
    if(initIndex < initSize) {
      xInit[initIndex] = gx1;
      yInit[initIndex] = gy1;
      zInit[initIndex] = gz1;
      if(initIndex == initSize - 1) {
        int sumX = 0; int sumY = 0; int sumZ = 0;
        for(int k=1; k <= initSize; k++) {
          sumX += xInit[k];
          sumY += yInit[k];
          sumZ += zInit[k];
        }

        xCal -= sumX/(initSize -1);
        yCal -= sumY/(initSize -1);
        zCal = (sumZ/(initSize -1) - zCal);
      }
      initIndex++;
    }
    
    /////////////////////////////////////////////////////////////////////////////
    //  �������� ���� �ʿ��� �۾��� ó���ϴ� ��ƾ
    /////////////////////////////////////////////////////////////////////////////
    else {
      // ������ ����
        gx1 += xCal;
        gy1 += yCal;
        //gz1 += zCal;
  
      // �������� ���� �ʿ��� ó���� ����
      // if(gz1 < 1400 && -250 < gy1 && gy1 < 250 && gx1 < 500) {
      //  Serial.print(F("Turn right"));
      //  Serial.println(F(""));
      //}

    }

    
//    Serial.print(gz1);
//    Serial.println("?");
      adval = ad_conv(1, 32); // 32 samples on Channel 0
      RH10 = calc_RH10(adval);
      RH10 = RH10/10;
      //온도 센서 데이터 저장
      float AmbientTemp = mlx.readAmbientTempC();
      float ObjectTemp = mlx.readObjectTempC();
      
      //각 센싱데이터를 문자열로 변환
      dtostrf(AmbientTemp,4,1,AmbientTemp_buf);
      dtostrf(ObjectTemp,4,1,ObjectTemp_buf);
      dtostrf(RH10,2,0,Humidity_buf);
      dtostrf(average,3,0,Average_buf);

      //하나의 버퍼에 센싱데이터를 모아줌
      sprintf(temperature,"%05d?%s?%s?%03d?%s?",gz1,AmbientTemp_buf,ObjectTemp_buf,average,Humidity_buf);
//      Serial.print(AmbientTemp);
//      Serial.print("      ");
//      Serial.print(ObjectTemp);
//      Serial.print("      ");
      
      Serial.println(temperature);
      bluetooth.print(temperature);
      total = 0;
    }
    //else{Serial.println(BPM+1000);}  //bluetooth
    }
    QS = false;
    prevSensoredTime = curSensoredTime;
    delay(500);
    
  }
   

}

/**************************************************
 * Sensor read/write
 **************************************************/
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;
  
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  
  n = Wire.write(start);
  if (n != 1)
    return (-10);
  
  n = Wire.endTransmission(false); // hold the I2C-bus
  if (n != 0)
    return (n);
  
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);
  return (0); // return : no error
}

int ad_conv(byte channel, byte num)
{
  long sum = 0;
  byte n;

  for (n = 0; n < num; n++)
  {
    sum = sum + analogRead(channel);
  }
  return (sum / num);
}

int calc_RH10(int adval)
{
  int RH10;

  RH10 = adval + 6 * adval / 10 + 3 * adval / 100; // 1.63 * adval

  return (RH10);
}



void display(int x)
{
  int whole, fract;

  whole = x / 10;
  fract = x % 10;

  Serial.print(whole, DEC);
  Serial.print(".");
  Serial.print(fract, DEC);

}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;
  
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  
  n = Wire.write(start); // write the start address
  if (n != 1)
    return (-20);
    
  n = Wire.write(pData, size); // write data bytes
  if (n != size)
    return (-21);
    
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);
  return (0); // return : no error
}

int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;
  error = MPU6050_write(reg, &data, 1);
  return (error);
}

/**************************************************
 * Raw data processing
 **************************************************/
float angleInDegrees(int lo, int hi, int measured) {
  float x = (hi - lo)/180.0;
  return (float)measured/x;
}

void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle) {
  kalman->Q_angle = Q_angle;
  kalman->Q_gyro = Q_gyro;
  kalman->R_angle = R_angle;
  
  kalman->P_00 = 0;
  kalman->P_01 = 0;
  kalman->P_10 = 0;
  kalman->P_11 = 0;
}

/*
* The kalman predict method.
* kalman    the kalman data structure
* dotAngle    Derivitive Of The (D O T) Angle. This is the change in the angle from the gyro.
*           This is the value from the Wii MotionPlus, scaled to fast/slow.
* dt        the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
*/
void predict(struct GyroKalman *kalman, float dotAngle, float dt) {
  kalman->x_angle += dt * (dotAngle - kalman->x_bias);
  kalman->P_00 += -1 * dt * (kalman->P_10 + kalman->P_01) + dt*dt * kalman->P_11 + kalman->Q_angle;
  kalman->P_01 += -1 * dt * kalman->P_11;
  kalman->P_10 += -1 * dt * kalman->P_11;
  kalman->P_11 += kalman->Q_gyro;
}

/*
* The kalman update method
* kalman  the kalman data structure
* angle_m   the angle observed from the Wii Nunchuk accelerometer, in radians
*/
float update(struct GyroKalman *kalman, float angle_m) {
  const float y = angle_m - kalman->x_angle;
  const float S = kalman->P_00 + kalman->R_angle;
  const float K_0 = kalman->P_00 / S;
  const float K_1 = kalman->P_10 / S;
  kalman->x_angle += K_0 * y;
  kalman->x_bias += K_1 * y;
  kalman->P_00 -= K_0 * kalman->P_00;
  kalman->P_01 -= K_0 * kalman->P_01;
  kalman->P_10 -= K_1 * kalman->P_00;
  kalman->P_11 -= K_1 * kalman->P_01;
  return kalman->x_angle;
}
