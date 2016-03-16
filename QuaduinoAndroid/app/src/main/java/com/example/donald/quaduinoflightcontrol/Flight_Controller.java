package com.example.donald.quaduinoflightcontrol;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.text.method.ScrollingMovementMethod;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;

//TODO calibrate drone button
//TODO receive data
//TODO send data at frequency
/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 */
public class Flight_Controller extends AppCompatActivity {

    private View mControlsView;
    // SPP UUID service - this should work for most devices
    private static final UUID BTMODULEUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    final int handlerState = 0;                        //used to identify handler message
    private BluetoothAdapter btAdapter = null;
    private BluetoothSocket btSocket = null;
    private StringBuilder recDataString = new StringBuilder();

    private SensorThread mSensorThread;
    private ConnectedThread mConnectedThread;
    volatile boolean connectedThreadRunning = true;
    volatile boolean sensorThreadRunning = true;
    Handler bluetoothIn;
    Handler bluetoothOut;
    public final int DATA_TRANSMIT_FREQUENCY = 3000;

    public CheckBox beginCheck;
    public CheckBox armCheck;
    public CheckBox controlCheck;
    public TextView debugBox;
    public boolean doCalibrate;

    public float[] phoneOrientation = new float[3];
    public float[] calibratedPhoneOrientation = new float[3];
    //Values to help with calibration
    public float cYaw,cPitch,cRoll = 0;

    public double RP_P = 0;
    public double RP_I = 0;
    public double RP_D = 0;
    public double Y_P = 0;
    public double Y_I = 0;
    public double Y_D = 0;
    public final int THROTTLE_AMOUNT = 2;
    public final double RP_P_FINAL = 0.25;
    public final double RP_I_FINAL = 0;
    public final double RP_D_FINAL = 0;
    public final double Y_P_FINAL = 0.25;
    public final double Y_I_FINAL = 0;
    public final double Y_D_FINAL = 0;
    public final double RP_P_AMOUNT = 0.01;
    public final double RP_I_AMOUNT = 0.01;
    public final double RP_D_AMOUNT = 0.01;
    public final double Y_P_AMOUNT = 0.01;
    public final double Y_I_AMOUNT = 0.01;
    public final double Y_D_AMOUNT = 0.01;

    public SeekBar throttleBar;

    public Runnable transmitRunnable = new Runnable() {
        @Override
        public void run() {
//            Toast.makeText(getApplicationContext(), "INSIDE", Toast.LENGTH_SHORT).show();
            sendData();
            bluetoothOut.postDelayed(this,DATA_TRANSMIT_FREQUENCY);
        }
    };

    public void sendData(){
        //Data string is of the form
        //Debug,Calibrate,Control,Throttle,PhoneYaw,PhonePitch,PhoneRoll,RP_P,RP_I,RP_D,Y_P,Y_I,Y_D|
        //0,1,1,15,23.96,21.29,30.21,3.431,2.231,1.213,3.213,2.213,1.321|
        //has 63 characters
        StringBuilder stringBuilder = new StringBuilder(65);

        //Debug
        if (debugBox.getVisibility() == View.INVISIBLE) stringBuilder.append("0");
        else stringBuilder.append("1");
        stringBuilder.append(",");

        //Calibrate
        if (doCalibrate){ stringBuilder.append("1"); doCalibrate = false;}
        else {stringBuilder.append("0");}
        stringBuilder.append(",");

        //Control
        if (controlCheck.isChecked()) {
            stringBuilder.append("1");stringBuilder.append(",");
            stringBuilder.append(calibratedPhoneOrientation[0]);stringBuilder.append(",");
            stringBuilder.append(calibratedPhoneOrientation[1]);stringBuilder.append(",");
            stringBuilder.append(calibratedPhoneOrientation[2]);stringBuilder.append(",");
        }
        else {
            stringBuilder.append("0");stringBuilder.append(",");
            stringBuilder.append("0");stringBuilder.append(",");
            stringBuilder.append("0");stringBuilder.append(",");
            stringBuilder.append("0");stringBuilder.append(",");
        }

        //RP_PID
        stringBuilder.append(RP_P);stringBuilder.append(",");
        stringBuilder.append(RP_I);stringBuilder.append(",");
        stringBuilder.append(RP_D);stringBuilder.append(",");

        //Y_PID
        stringBuilder.append(Y_P);stringBuilder.append(",");
        stringBuilder.append(Y_I);stringBuilder.append(",");
        stringBuilder.append(Y_D);stringBuilder.append("|");

        //TODO UNCOMMENT THIS
//        mConnectedThread.write(stringBuilder.toString());
        mConnectedThread.write("x");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.flight_controller);
        throttleBar = (SeekBar)findViewById(R.id.throttle_bar);
        beginCheck = (CheckBox)findViewById(R.id.begin_checkbox);
        armCheck = (CheckBox)findViewById(R.id.arm_checkbox);
        controlCheck = (CheckBox)findViewById(R.id.control_checkbox);
        debugBox = (TextView)findViewById(R.id.debut_text);
        debugBox.setMovementMethod(new ScrollingMovementMethod());

        beginCheck.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

              @Override
              public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                  if (isChecked) {
                      bluetoothOut = new Handler();
                      bluetoothOut.postDelayed(transmitRunnable, DATA_TRANSMIT_FREQUENCY);
                  } else {
                      bluetoothOut.removeCallbacks(transmitRunnable);
                  }
              }
          }
        );

        mControlsView = getWindow().getDecorView();
        mControlsView.setOnSystemUiVisibilityChangeListener(new View.OnSystemUiVisibilityChangeListener() {
            @Override
            public void onSystemUiVisibilityChange(int visibility) {
                if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
                    delayedHide(1500);
                }
            }
        });

        throttleBar.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                return true;
            }
        });
        throttleBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub
            }

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                ((TextView) findViewById(R.id.throttle_label)).setText("Throttle: " + progress);
            }
        });

        resetPID(null);
        setButtonListeners();

        ///////////BLUETOOTH////////////

        btAdapter = BluetoothAdapter.getDefaultAdapter();

        bluetoothIn = new Handler() {
            public void handleMessage(android.os.Message msg)
            {
                if (msg.what == handlerState)
                {                                     //if message is what we want
                    String readMessage = (String) msg.obj;                                                                // msg.arg1 = bytes from connect thread
//                    Toast.makeText(getApplicationContext(), readMessage, Toast.LENGTH_SHORT).show();
                    debugBox.append(readMessage);
                    final int scrollAmount = debugBox.getLayout().getLineTop(debugBox.getLineCount()) - debugBox.getHeight();
                    // if there is no need to scroll, scrollAmount will be <=0
                    if (scrollAmount > 0)
                        debugBox.scrollTo(0, scrollAmount);
                    else
                        debugBox.scrollTo(0, 0);
//                    recDataString.append(readMessage);                                      //keep appending to string until ~
//                int endOfLineIndex = recDataString.indexOf("~");                    // determine the end-of-line
//                if (endOfLineIndex > 0) {                                           // make sure there data before ~
//                    String dataInPrint = recDataString.substring(0, endOfLineIndex);    // extract string
//                    txtString.setText("Data Received = " + dataInPrint);
//                    int dataLength = dataInPrint.length();                          //get length of data received
//                    txtStringLength.setText("String Length = " + String.valueOf(dataLength));
//
//                    if (recDataString.charAt(0) == '#')                             //if it starts with # we know it is what we are looking for
//                    {
//                        String sensor0 = recDataString.substring(1, 5);             //get sensor value from string between indices 1-5
//                        String sensor1 = recDataString.substring(6, 10);            //same again...
//                        String sensor2 = recDataString.substring(11, 15);
//                        String sensor3 = recDataString.substring(16, 20);
//
//                        sensorView0.setText(" Sensor 0 Voltage = " + sensor0 + "V");    //update the textviews with sensor values
//                        sensorView1.setText(" Sensor 1 Voltage = " + sensor1 + "V");
//                        sensorView2.setText(" Sensor 2 Voltage = " + sensor2 + "V");
//                        sensorView3.setText(" Sensor 3 Voltage = " + sensor3 + "V");
//                    }
//                    recDataString.delete(0, recDataString.length());                    //clear all string data
                    // strIncom =" ";
//                    dataInPrint = " ";
//                }
                }
            }
        };

    }

    @Override
    public void onResume() {
        super.onResume();

        //receive the address of the bluetooth device
        String address = null;
        Intent newint = getIntent();
        address = newint.getStringExtra(FullscreenActivity.EXTRA_ADDRESS);

//        //create device and set the MAC address
        BluetoothDevice device = btAdapter.getRemoteDevice(address);

        try {
            btSocket = createBluetoothSocket(device);
        } catch (IOException e) {
            Toast.makeText(getBaseContext(), "Socket creation failed", Toast.LENGTH_LONG).show();
        }
        // Establish the Bluetooth socket connection.
        try
        {
            btSocket.connect();
        } catch (IOException e) {
            try
            {
                btSocket.close();
            } catch (IOException e2)
            {
                //insert code to deal with this
            }
        }
        mConnectedThread = new ConnectedThread(btSocket);
        mConnectedThread.start();
        mSensorThread = new SensorThread();
        mSensorThread.start();

        //I send a character when resuming.beginning transmission to check device is connected
        //If it is not an exception will be thrown in the write method and finish() will be called
        //TODO ADD BACK IN!
//        mConnectedThread.write("x");
    }

    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);

        // Trigger the initial hide() shortly after the activity has been
        // created, to briefly hint to the user that UI controls
        // are available.
        delayedHide(100);
    }

    @Override
    public void onPause()
    {
        super.onPause();
        try
        {
            connectedThreadRunning = false;
            sensorThreadRunning = false;
            //Don't leave Bluetooth sockets open when leaving activity
            btSocket.close();
        } catch (IOException e2) {
            //insert code to deal with this
        }
    }

    private final Runnable mHideRunnable = new Runnable() {
        @SuppressLint("InlinedApi")
        @Override
        public void run() {
            // Delayed removal of status and navigation bar

            // Note that some of these constants are new as of API 16 (Jelly Bean)
            // and API 19 (KitKat). It is safe to use them, as they are inlined
            // at compile-time and do nothing on earlier devices.
            mControlsView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                    | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                    | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                    | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION // hide nav bar
                    | View.SYSTEM_UI_FLAG_FULLSCREEN // hide status bar
                    | View.SYSTEM_UI_FLAG_IMMERSIVE);
        }
    };

    private final Handler mHideHandler = new Handler();

    private void delayedHide(int delayMillis) {
        mHideHandler.removeCallbacks(mHideRunnable);
        mHideHandler.postDelayed(mHideRunnable, delayMillis);
    }


    ////////////////////////////BLUETOOTH//////////////////

    private BluetoothSocket createBluetoothSocket(BluetoothDevice device) throws IOException {

        return  device.createRfcommSocketToServiceRecord(BTMODULEUUID);
        //creates secure outgoing connecetion with BT device using UUID
    }


    //create new class for connect thread
    private class ConnectedThread extends Thread {
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        //creation of the connect thread
        public ConnectedThread(BluetoothSocket socket) {
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            try {
                //Create I/O streams for connection
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) { }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[256];
            int bytes;

            // Keep looping to listen for received messages
            while (connectedThreadRunning) {
                try {
                    bytes = mmInStream.read(buffer);            //read bytes from input buffer
                    String readMessage = new String(buffer, 0, bytes);
                    // Send the obtained bytes to the UI Activity via handler
                    bluetoothIn.obtainMessage(handlerState, bytes, -1, readMessage).sendToTarget();
                } catch (IOException e) {
                    break;
                }
            }
            return;
        }
        //write method
        public void write(String input) {
            byte[] msgBuffer = input.getBytes();           //converts entered String into bytes
            try {
                mmOutStream.write(msgBuffer);                //write bytes over BT connection via outstream
            } catch (IOException e) {
                //if you cannot write, close the application
                Toast.makeText(getBaseContext(), "Connection Failure", Toast.LENGTH_LONG).show();
                finish();

            }
        }
    }

    /////////////Sensor Class///////////

    //create new class for connect thread
    private class SensorThread extends Thread implements SensorEventListener{

        private SensorManager mSensorManager;
        private Sensor sensor;
        private static final int SENSOR_DELAY = 500 * 1000; // 500ms
        private static final int FROM_RADS_TO_DEGS = -57;

        public void run() {
            mSensorManager = (SensorManager) getSystemService(Activity.SENSOR_SERVICE);
            sensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
            mSensorManager.registerListener(this, sensor, SENSOR_DELAY);
            // Keep looping to listen for received messages
            while (sensorThreadRunning) {

            }
            return;
        }

        @Override
        public final void onAccuracyChanged(Sensor sensor, int accuracy) {
            // Do something here if sensor accuracy changes.
        }

        @Override
        public final void onSensorChanged(SensorEvent event) {
            if (event.sensor == sensor) {
                if (event.values.length > 4) {
                    float[] truncatedRotationVector = new float[4];
                    System.arraycopy(event.values, 0, truncatedRotationVector, 0, 4);
                    update(truncatedRotationVector);
                } else {
                    update(event.values);
                }
            }
        }

        private void update(float[] vectors) {
            //TODO MAKE SURE VALUES ARE CORRECT!
            float[] rotationMatrix = new float[9];
            SensorManager.getRotationMatrixFromVector(rotationMatrix, vectors);
            int worldAxisX = SensorManager.AXIS_X;
            int worldAxisZ = SensorManager.AXIS_Z;
            float[] adjustedRotationMatrix = new float[9];
            SensorManager.remapCoordinateSystem(rotationMatrix, worldAxisX, worldAxisZ, adjustedRotationMatrix);
            float[] orientation = new float[3];
            SensorManager.getOrientation(adjustedRotationMatrix, orientation);
            float yaw = phoneOrientation[0] = Math.round(orientation[0] * FROM_RADS_TO_DEGS);
            float pitch = phoneOrientation[1] = Math.round(orientation[1] * FROM_RADS_TO_DEGS);
            float roll = phoneOrientation[2] = Math.round(orientation[2] * FROM_RADS_TO_DEGS);
            calibratedPhoneOrientation[0] = ((yaw + (360 - (cYaw + 180))) % 360) - 180;
            calibratedPhoneOrientation[1] = cPitch - pitch;
            calibratedPhoneOrientation[2] = cRoll - roll;
            ((TextView)findViewById(R.id.phone_yaw)).setText("Yaw: "+ phoneOrientation[0]);
            ((TextView)findViewById(R.id.phone_pitch)).setText("Pitch: "+ calibratedPhoneOrientation[1]);
            ((TextView)findViewById(R.id.phone_roll)).setText("Roll: "+ calibratedPhoneOrientation[2]);
        }

    }

    /////////////Buttons////////////
    public void calibrateDrone(View v){ doCalibrate = true; }

    public void finish(View v){
        finish();
    }

    public void debug(View v){
        TextView debugWindow = (TextView)findViewById(R.id.debut_text);
        int visibility = debugWindow.getVisibility() == View.VISIBLE ? View.INVISIBLE : View.VISIBLE;
        debugWindow.setVisibility(visibility);
    }

    public void addThrottle(View v){
        throttleBar.setProgress(throttleBar.getProgress() + THROTTLE_AMOUNT);
    }

    public void subThrottle(View v){
        throttleBar.setProgress(throttleBar.getProgress() - THROTTLE_AMOUNT);
    }

    public void calibratePhone(View v){
        cYaw = phoneOrientation[0];
        cPitch = phoneOrientation[1];
        cRoll = phoneOrientation[2];
    }

    public void resetPID(View v){
        RP_P = RP_P_FINAL;
        RP_I = RP_I_FINAL;
        RP_D = RP_D_FINAL;
        Y_P = Y_P_FINAL;
        Y_I = Y_I_FINAL;
        Y_D = Y_D_FINAL;
        ((TextView) findViewById(R.id.RP_P_label)).setText(String.valueOf(RP_P));
        ((TextView) findViewById(R.id.RP_I_label)).setText(String.valueOf(RP_I));
        ((TextView) findViewById(R.id.RP_D_label)).setText(String.valueOf(RP_D));
        ((TextView) findViewById(R.id.Y_P_label)).setText(String.valueOf(Y_P));
        ((TextView) findViewById(R.id.Y_I_label)).setText(String.valueOf(Y_I));
        ((TextView) findViewById(R.id.Y_D_label)).setText(String.valueOf(Y_D));
    }

    public void setButtonListeners(){

        ///RP_D
        findViewById(R.id.add_RP_P).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                RP_P = Math.round((RP_P + RP_P_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.RP_P_label)).setText(String.valueOf(RP_P));
            }
        });

        findViewById(R.id.sub_RP_P).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                RP_P = Math.round((RP_P - RP_P_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.RP_P_label)).setText(String.valueOf(RP_P));
            }
        });

        //RP_I
        findViewById(R.id.add_RP_I).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                RP_I = Math.round((RP_I + RP_I_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.RP_I_label)).setText(String.valueOf(RP_I));
            }
        });

        findViewById(R.id.sub_RP_I).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                RP_I = Math.round((RP_I - RP_I_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.RP_I_label)).setText(String.valueOf(RP_I));
            }
        });

        //RP_D
        findViewById(R.id.add_RP_D).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                RP_D = Math.round((RP_D + RP_D_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.RP_D_label)).setText(String.valueOf(RP_D));
            }
        });

        findViewById(R.id.sub_RP_D).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                RP_D = Math.round((RP_D - RP_D_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.RP_D_label)).setText(String.valueOf(RP_D));
            }
        });

        ///Y_D
        findViewById(R.id.add_Y_P).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Y_P = Math.round((Y_P + Y_P_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.Y_P_label)).setText(String.valueOf(Y_P));
            }
        });

        findViewById(R.id.sub_Y_P).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Y_P = Math.round((Y_P - Y_P_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.Y_P_label)).setText(String.valueOf(Y_P));
            }
        });

        //Y_I
        findViewById(R.id.add_Y_I).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Y_I = Math.round((Y_I + Y_I_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.Y_I_label)).setText(String.valueOf(Y_I));
            }
        });

        findViewById(R.id.sub_Y_I).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Y_I = Math.round((Y_I - Y_I_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.Y_I_label)).setText(String.valueOf(Y_I));
            }
        });

        //Y_D
        findViewById(R.id.add_Y_D).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Y_D = Math.round((Y_D + Y_D_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.Y_D_label)).setText(String.valueOf(Y_D));
            }
        });

        findViewById(R.id.sub_Y_D).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Y_D = Math.round((Y_D - Y_D_AMOUNT)*1000.0)/1000.0;
                ((TextView) findViewById(R.id.Y_D_label)).setText(String.valueOf(Y_D));
            }
        });

    }
}
