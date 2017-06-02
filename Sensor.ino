unsigned long tp;
int GXIndex,GYIndex,GZIndex;
int AXIndex, AYIndex, AZIndex;
float filteredGyroX,filteredGyroY,filteredGyroZ = 0.0;
float filteredAccelPitch,filteredAccelRoll = 0.0;
float gx_old, gy_old, gz_old = 0.0;

void updateSensorVal(){

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
  filteredAccelPitch = (ONE_MINUS_ALPHA * accPitch) + (ALPHA * filteredAccelPitch);
  filteredAccelRoll = (ONE_MINUS_ALPHA * accRoll) + (ALPHA * filteredAccelRoll);

  //Apply Complimentary Filter
  angles[0] = ONE_MINUS_ALPHA * (angles[0] + (filteredGyroY * dt)) + (ALPHA * filteredAccelPitch);
  angles[1] = ONE_MINUS_ALPHA * (angles[1] + (filteredGyroX * dt)) + (ALPHA * filteredAccelRoll);

  if (abs(angles[0]) + abs(angles[1]) > CRAZY_ANGLE_THRESHOLD)
  {
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

  gyroHPF();
}

void gyroHPF(){//High Pass filter

  filteredGyroX= (ONE_MINUS_ALPHA*filteredGyroX) + (ONE_MINUS_ALPHA*(gx_aver - gx_old));
  filteredGyroY= (ONE_MINUS_ALPHA*filteredGyroY) + (ONE_MINUS_ALPHA*(gy_aver - gy_old));
  filteredGyroZ= (ONE_MINUS_ALPHA*filteredGyroZ) + (ONE_MINUS_ALPHA*(gz_aver - gz_old));
  
}
