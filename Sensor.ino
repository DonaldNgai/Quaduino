//float gx_temp[GYRO_MAF_NR]={
//  0.0,0.0};
//float gy_temp[GYRO_MAF_NR]={
//  0.0,0.0};
//float gz_temp[GYRO_MAF_NR]={
//  0.0,0.0};

unsigned long tp;
unsigned long ts=millis();
unsigned long tf=micros();
int GXIndex,GYIndex,GZIndex;
int AXIndex, AYIndex, AZIndex;
float filteredGyroX,filteredGyroY,filteredGyroZ = 0.0;
float gx_old, gy_old, gz_old = 0.0;

void updateSensorVal(){
//  if((micros()-tf)>1300){
//    updateGyro();  //Update only per 1300us, (~800Hz update rate)
//    tf=micros();  
//  }
//  if((millis()-ts)>20){  //Update only once per 20ms (50Hz update rate)
//    
//    ts=millis();
//  }
  unsigned long t = millis();
  float dt = (float)(t-tp)/1000.0;
  
  float x2 = accx_temp*accx_temp;
  float y2 = accy_temp*accy_temp;
  float z2 = accz_temp*accz_temp;

  //Apply Trigonometry to get Pitch and Roll
  float accPitch = atan(accx_temp/sqrt(y2+z2));
  float accRoll = atan(accy_temp/sqrt(x2+z2));

  //Convert Radians to Degrees
  accPitch = accPitch * RadToDeg;
  accRoll = accRoll * RadToDeg;

  //Apply Low Pass Filter

  //Apply Complimentary Filter
  angles[0]=SPLIT*(-gy_aver*dt+angles[0])+(1.0-SPLIT)*accy;
  angles[1]=SPLIT*(gx_aver*dt+angles[1])+(1.0-SPLIT)*accx;

  if (abs(angles[0]) + abs(angles[1]) > CRAZY_ANGLE_THRESHOLD){
              failSafe = true;
              Serial.println("P: " + String(angles[0]) + " R: " + String(angles[1]));
              Serial.println(F("Crazy Angle"));
  }

  tp=t; 
}

void updateAcc(){
  int buffer[3]; //Axl buffer
  mpu.getAcceleration(&buffer[0],&buffer[1],&buffer[2]);

  accx_temp = digitalSmooth(int(buffer[0]/ACCEL_LSB_SENSITIVITY),accXSmoothArray,accXsortedArray,&AXIndex,accFilterSamples);
  accy_temp = digitalSmooth(int(buffer[1]/ACCEL_LSB_SENSITIVITY),accYSmoothArray,accYsortedArray,&AYIndex,accFilterSamples);
  accz_temp = digitalSmooth(int(buffer[2]/ACCEL_LSB_SENSITIVITY),accZSmoothArray,accZsortedArray,&AZIndex,accFilterSamples);

}

void updateGyroData(){
  
  mpu.getRotation(&gyroX,&gyroY,&gyroZ);

  gx_old=gx_aver;
  gy_old=gy_aver;
  gz_old=gz_aver;

  gx_aver = digitalSmooth(int(gyroX/GYRO_LSB_SENSITIVITY),gyroXSmoothArray,gyroXsortedArray,&GXIndex,gyroFilterSamples);
  gy_aver = digitalSmooth(int(gyroY/GYRO_LSB_SENSITIVITY),gyroYSmoothArray,gyroYsortedArray,&GYIndex,gyroFilterSamples);
  gz_aver = digitalSmooth(int(gyroZ/GYRO_LSB_SENSITIVITY),gyroZSmoothArray,gyroZsortedArray,&GZIndex,gyroFilterSamples);

}

void gyroHPF(){//High Pass filter
  #if GYRO_HPF_NR > 0
  
  float oneMinusAlpha = (1-ALPHA);

  filteredGyroX= oneMinusAlpha*filteredGyroX + oneMinusAlpha*(gx_aver - gx_old);
  filteredGyroY= oneMinusAlpha*filteredGyroY + oneMinusAlpha*(gy_aver - gy_old);
  filteredGyroZ= oneMinusAlpha*filteredGyroZ + oneMinusAlpha*(gz_aver - gz_old);
  
#endif
}
