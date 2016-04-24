package com.example.donald.orientationsensor;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.TextView;

import java.io.IOException;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private SensorManager mSensorManager;
    Sensor accelerometer;
    Sensor magnetometer;
    public float orientation[] = new float[3];
    public float[] calibratedPhoneOrientation = new float[3];
    public float cYaw,cRoll = 0;
    public float cPitch = -90;
    public float rawYaw,rawRoll,rawPitch = 0;



    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);    // Register the sensor listeners
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    }

    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);
    }

    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }

    public void calibratePhone(View v){
        cYaw = rawYaw;
        cPitch = rawPitch;
        cRoll = rawRoll;
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {  }

    float[] mGravity;
    float[] mGeomagnetic;
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
            mGravity = event.values;
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
            mGeomagnetic = event.values;
        if (mGravity != null && mGeomagnetic != null) {
            float Rm[] = new float[9];
            float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(Rm, I, mGravity, mGeomagnetic);
            if (success) {

                SensorManager.getOrientation(Rm, orientation);

                //Pitch is originally ranging from -pi/2 to pi/2 but this if function makes it go from
                //-pi to pi
                if(mGravity[2]<0)
                    orientation[1] = (float) (Math.PI - orientation[1]);

                rawYaw = Math.round(Math.toDegrees(orientation[0]));
                //move the pitch from -90 -> 270 to -180 -> 180
                rawPitch = Math.round(Math.toDegrees(orientation[1])-90);
                rawRoll = Math.round(Math.toDegrees(orientation[2]));
                //-180 - 0 - 180
//                calibratedPhoneOrientation[0] = ((yaw + (360 - (cYaw + 180))) % 360) - 180;
                calibratedPhoneOrientation[0] = Math.round((((rawYaw - cYaw) + 180)%360)-180);
                //-90 - 0 - 270
                calibratedPhoneOrientation[1] = Math.round((((rawPitch - cPitch) + 180)%360)-180);
                //-180 - 0 - 180
                calibratedPhoneOrientation[2] = Math.round((((rawRoll - cRoll) + 180)%360)-180);
                ((TextView)findViewById(R.id.origY)).setText("Yaw: "+ rawYaw);
                ((TextView)findViewById(R.id.origP)).setText("Pitch: "+ rawPitch);
                ((TextView)findViewById(R.id.origR)).setText("Roll: "+ rawRoll);
                ((TextView)findViewById(R.id.phone_yaw)).setText("Yaw: "+ calibratedPhoneOrientation[0]);
                ((TextView)findViewById(R.id.phone_pitch)).setText("Pitch: "+ calibratedPhoneOrientation[1]);
                ((TextView)findViewById(R.id.phone_roll)).setText("Roll: " + calibratedPhoneOrientation[2]);
   }
        }
    }
}
