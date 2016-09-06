package com.example.donald.quaduinoflightcontrol;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.text.method.ScrollingMovementMethod;
import android.view.View;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Calendar;
import java.util.UUID;

//TODO deal with edge cases for seekbar tap

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
    //50 times per second = 50Hz
    public final int DATA_TRANSMIT_FREQUENCY = 20;

    public String time;

    public CheckBox beginCheck;
    public CheckBox armCheck;
    public CheckBox controlCheck;
    public CheckBox rateCheck;
    public CheckBox scrollCheck;
    public CheckBox logCheck;
    public CheckBox fineCheck;
    public TextView debugBox;
    public TextView outputBox;
    public TextView logBox;
    public LinearLayout debugWindow;
    public TextView stableText;
    public boolean PIDChanged = false;
    public boolean writeFailed = false;
    public boolean clearFailsafe = false;

    public float orientation[] = new float[3];
    public float[] calibratedPhoneOrientation = new float[3];
    //Values to help with calibration
    public float cYaw,cRoll = 0;
    public float cPitch = -90;
    public float rawYaw,rawRoll,rawPitch = 0;

    public double tempMultiplier = 1;
    public double rateMultiplier = 1.5;

    public int originalProgress;

    public double RP_P = 0;
    public double RP_I = 0;
    public double RP_D = 0;
    public double Y_P = 0;
    public double Y_I = 0;
    public double Y_D = 0;
    public final int THROTTLE_AMOUNT = 2;
    //What the starting PID values should be
    public final double RP_P_FINAL = 2.0;
    public final double RP_I_FINAL = 0;
    public final double RP_D_FINAL = 0;
    public final double Y_P_FINAL = 0.027;
    public final double Y_I_FINAL = 0;
    public final double Y_D_FINAL = 0;
    //How much to increase values when changing
    public double RP_P_AMOUNT = 0.05;
    public double RP_I_AMOUNT = 0.05;
    public double RP_D_AMOUNT = 0.05;
    public double Y_P_AMOUNT = 0.5;
    public double Y_I_AMOUNT = 0.05;
    public double Y_D_AMOUNT = 0.05;

    public SeekBar throttleBar;

    public Runnable transmitRunnable = new Runnable() {
        @Override
        public void run() {
//            Toast.makeText(getApplicationContext(), "INSIDE", Toast.LENGTH_SHORT).show();
            sendData();
            bluetoothOut.postDelayed(this,DATA_TRANSMIT_FREQUENCY);
        }
    };

    public void stopDrone() {
        throttleBar.setProgress(0);
        armCheck.setChecked(false);
        sendData();
    }

    public void sendData(){
        //Data string is of the form
        //CheckSum,Failsafe,PID,Throttle,Control,PhoneYaw,PhonePitch,PhoneRoll,RP_P,RP_I,RP_D,Y_P,Y_I,Y_D|
        //0,1,1,15,23.96,21.29,30.21,3.431,2.231,1.213,3.213,2.213,1.321|
        StringBuilder stringBuilder = new StringBuilder(65);

        //Debug
//        if (debugWindow.getVisibility() == View.INVISIBLE) stringBuilder.append("0");
//        else stringBuilder.append("1");
//        stringBuilder.append(",");
//        Failsafe
        if (clearFailsafe){
            stringBuilder.append("1");
            clearFailsafe = false;
        }
        else{
            stringBuilder.append("0");
        }
        stringBuilder.append(",");

        //Calibrate
//        if (doCalibrate){ stringBuilder.append("1"); doCalibrate = false;}
//        else {stringBuilder.append("0");}
//        stringBuilder.append(",");

        //Change PID
        if (PIDChanged) { stringBuilder.append("1"); PIDChanged = false;}
        else {stringBuilder.append("0");}
        stringBuilder.append(",");

        //Throttle
        stringBuilder.append(throttleBar.getProgress());stringBuilder.append(",");

        //Control
        if (rateCheck.isChecked()) {
            stringBuilder.append("0");stringBuilder.append(",");
        }
        else {
            stringBuilder.append("1");stringBuilder.append(",");
        }

        if (controlCheck.isChecked()) {
            //always sending 0 for yaw
            tempMultiplier = rateCheck.isChecked() ? rateMultiplier : 1;
            stringBuilder.append(0*tempMultiplier);
            stringBuilder.append(",");
            stringBuilder.append(calibratedPhoneOrientation[1]*tempMultiplier);
            stringBuilder.append(",");
            stringBuilder.append(calibratedPhoneOrientation[2]*tempMultiplier);
            stringBuilder.append(",");
        }
        else{
            //always sending 0 for yaw
            stringBuilder.append(0);
            stringBuilder.append(",");
            stringBuilder.append(0);
            stringBuilder.append(",");
            stringBuilder.append(0);
            stringBuilder.append(",");
        }

        //RP_PID
        stringBuilder.append(RP_P);stringBuilder.append(",");
        stringBuilder.append(RP_I);stringBuilder.append(",");
        stringBuilder.append(RP_D);stringBuilder.append(",");

        //Y_PID
        stringBuilder.append(Y_P);stringBuilder.append(",");
        stringBuilder.append(Y_I);stringBuilder.append(",");
        stringBuilder.append(Y_D);stringBuilder.append("|");

        //Prepend Checksum
        String stringLength = Integer.toString(stringBuilder.length());
        //+2 for the C and semicolon and +2 for 10-99 length
        stringBuilder.insert(0,"C" + stringLength + ",");

        if(scrollCheck.isChecked()) {
            outputBox.append(stringBuilder.toString());
            outputBox.append("\n");
            if (outputBox.getLineCount() >= 500) {
                outputBox.getEditableText().delete(0, outputBox.getLineCount() / 2);
            }
            final int scrollAmount = outputBox.getLayout().getLineTop(outputBox.getLineCount()) - outputBox.getHeight();
            // if there is no need to scroll, scrollAmount will be <=0
            if (scrollAmount > 0)
                outputBox.scrollTo(0, scrollAmount);
            else
                outputBox.scrollTo(0, 0);
        }

        mConnectedThread.write(stringBuilder.toString());
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
        rateCheck = (CheckBox)findViewById(R.id.rate_checkbox);
        scrollCheck = (CheckBox)findViewById(R.id.autoScrollCheck);
        logCheck = (CheckBox)findViewById(R.id.logCheck);
        fineCheck = (CheckBox)findViewById(R.id.fineBox);
        debugWindow = (LinearLayout)findViewById(R.id.debug_window);
        debugBox = (TextView)findViewById(R.id.debug_text);
        logBox = (TextView)findViewById(R.id.logPath);
        outputBox = (TextView)findViewById(R.id.outputWindow);
        stableText = (TextView)findViewById(R.id.stableText);
        debugBox.setMovementMethod(new ScrollingMovementMethod());
        outputBox.setMovementMethod(new ScrollingMovementMethod());

        Calendar c = Calendar.getInstance();
        time = Integer.toString(c.get(Calendar.DAY_OF_MONTH)) + "_" + Integer.toString(c.get(Calendar.HOUR_OF_DAY)) + "_" + Integer.toString(c.get(Calendar.MINUTE));
        appendLog("RP_P,\tRP_I,\tRP_D,\tY_P,\tY_I,\tY_D");
        appendLog(String.format("%-5s, %-5s, %-5s, %-5s, %-5s, %-5s",RP_P_FINAL,RP_I_FINAL,RP_D_FINAL,Y_P_FINAL,Y_I_FINAL,Y_D_FINAL));

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

        fineCheck.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

                                                 @Override
                                                 public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                 double amount;
                 if (isChecked) {
                     amount = 0.1;
                 } else {
                    amount = 10;
                 }
                 RP_P_AMOUNT    = RP_P_AMOUNT * amount ;
                 RP_I_AMOUNT    = RP_I_AMOUNT * amount ;
                 RP_D_AMOUNT    = RP_D_AMOUNT * amount ;
                 Y_P_AMOUNT     = Y_P_AMOUNT  * amount ;
                 Y_I_AMOUNT     = Y_I_AMOUNT  * amount ;
                 Y_D_AMOUNT     = Y_D_AMOUNT  * amount ;
             }
         }
        );

        armCheck.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
              @Override
              public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                  if (isChecked) {
                      clearFailsafe = true;
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

//        throttleBar.setOnTouchListener(new View.OnTouchListener() {
//            @Override
//            public boolean onTouch(View view, MotionEvent motionEvent) {
//                return true;
//            }
//        });
        throttleBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // TODO Auto-generated method stub
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                originalProgress = throttleBar.getProgress();
            }

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                int realProgress = 0;
                logCheck.setChecked(false);
                    if (fromUser == true) {
                        // only allow changes by 1 up or down
                        if (((progress > (originalProgress + 10))
                                || (progress < (originalProgress - 10)))) {
                            throttleBar.setProgress(originalProgress);
                            realProgress = originalProgress;
                        } else if (armCheck.isChecked() && beginCheck.isChecked()){
                            originalProgress = progress;
                            realProgress = progress;
                        }
                    }
                    else
                    {
                        if (armCheck.isChecked() && beginCheck.isChecked()){
//                            originalProgress = progress;
                            realProgress = progress;
                        }
                        else{
                            throttleBar.setProgress(originalProgress);
                            realProgress = originalProgress;
                        }
                    }
                    ((TextView) findViewById(R.id.throttle_label)).setText("Throttle: " + realProgress);

//
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
                    String readMessage = (String) msg.obj;   // msg.arg1 = bytes from connect thread

                    if(scrollCheck.isChecked()) {
                        debugBox.append(readMessage);

                        if (debugBox.getLineCount() >= 500) {
                            if (logCheck.isChecked()) appendLog(debugBox.getText().toString());
                            debugBox.getEditableText().delete(0, debugBox.getLineCount() / 2);
                        }
                        if (readMessage.contains("*")) {
                            stableText.setText("MPU is stable!");
                        }
                        if (readMessage.contains("F:1")){
                            stopDrone();
                        }
                        final int scrollAmount = debugBox.getLayout().getLineTop(debugBox.getLineCount()) - debugBox.getHeight();
                        // if there is no need to scroll, scrollAmount will be <=0
                        if (scrollAmount > 0)
                            debugBox.scrollTo(0, scrollAmount);
                        else
                            debugBox.scrollTo(0, 0);
                    }
                }
            }
        };

    }

    public void appendLog(String text)
    {
//        File logFile = new File(getApplicationContext().getFilesDir(), time + ".log");
        File logFile = new File(getApplicationContext().getExternalFilesDir(null), time + ".log");

        if (!logFile.exists())
        {
            try
            {
                logFile.getParentFile().mkdirs();
                logFile.createNewFile();
                logBox.append(logFile.getName().toString());
            }
            catch (IOException e)
            {
                // TODO Auto-generated catch block
                  Toast.makeText(getBaseContext(), "Error Creating Log File", Toast.LENGTH_LONG).show();

                e.printStackTrace();
            }
        }
        try
        {
            //BufferedWriter for performance, true to set append to file flag
            BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, true));
            buf.append(text);
            buf.newLine();
            buf.close();
        }
        catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
            Toast.makeText(getBaseContext(), "Error Writing Log File", Toast.LENGTH_LONG).show();

        }
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
            stopDrone();
            connectedThreadRunning = false;
            sensorThreadRunning = false;
            if (bluetoothOut != null) {
                bluetoothOut.removeCallbacks(transmitRunnable);
            }
            if (logCheck.isChecked()) appendLog(debugBox.getText().toString());
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
            if (!writeFailed){
                try {
                    mmOutStream.write(msgBuffer);                //write bytes over BT connection via outstream
                } catch (IOException e) {
                    //if you cannot write, close the application
                    writeFailed = true;
                    Toast.makeText(getBaseContext(), "Connection Failure", Toast.LENGTH_LONG).show();
                // setResult(RESULT_OK,
    //                new Intent().putExtra("YawP", Y_P).putExtra("YawI", Y_I).putExtra("YawD",Y_D).putExtra("RPP", RP_P).putExtra("RPI", RP_P).putExtra("RPD",RP_D));
                // finish();

                }
            }
        }
    }

    /////////////Sensor Class///////////

    //create new class for connect thread
    private class SensorThread extends Thread implements SensorEventListener{

        private SensorManager mSensorManager;
        Sensor accelerometer;
        Sensor magnetometer;
        float[] mGravity;
        float[] mGeomagnetic;
//        private static final int SENSOR_DELAY = 500 * 1000; // 500ms
//        private static final int FROM_RADS_TO_DEGS = -57;

        public void run() {
            mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
            accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
            mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);
            // Keep looping to listen for received messages
            while (sensorThreadRunning) {

            }
            mSensorManager.unregisterListener(this);
            return;
        }

        @Override
        public final void onAccuracyChanged(Sensor sensor, int accuracy) {
            // Do something here if sensor accuracy changes.
        }

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
                    //made it negative to match orientation of the drone
                    calibratedPhoneOrientation[2] = -Math.round((((rawRoll - cRoll) + 180)%360)-180);
//                    ((TextView)findViewById(R.id.origY)).setText("Yaw: "+ rawYaw);
//                    ((TextView)findViewById(R.id.origP)).setText("Pitch: "+ rawPitch);
//                    ((TextView)findViewById(R.id.origR)).setText("Roll: "+ rawRoll);
                    ((TextView)findViewById(R.id.phone_yaw)).setText("Yaw: "+ calibratedPhoneOrientation[0]);
                    ((TextView)findViewById(R.id.phone_pitch)).setText("Pitch: "+ calibratedPhoneOrientation[1]);
                    ((TextView)findViewById(R.id.phone_roll)).setText("Roll: " + calibratedPhoneOrientation[2]);
                }
            }
        }

    }

    /////////////Buttons////////////
    public void finish(View v){
        setResult(RESULT_OK,
                new Intent().putExtra("YawP", Y_P).putExtra("YawI", Y_I).putExtra("YawD",Y_D).putExtra("RPP", RP_P).putExtra("RPI", RP_I).putExtra("RPD",RP_D));
        finish();
    }

    public void debug(View v){
//        L debugWindow = (TextView)findViewById(R.id.debug_text);
        int visibility = debugWindow.getVisibility() == View.VISIBLE ? View.INVISIBLE : View.VISIBLE;
        debugWindow.setVisibility(visibility);
        debugBox.setVisibility(visibility);
    }

    public void output(View v){
        int visibility = debugWindow.getVisibility() == View.VISIBLE ? View.INVISIBLE : View.VISIBLE;
        debugWindow.setVisibility(visibility);
        outputBox.setVisibility(visibility);
    }

    public void addThrottle(View v){
        originalProgress = throttleBar.getProgress();
        throttleBar.setProgress(throttleBar.getProgress() + THROTTLE_AMOUNT);
    }

    public void subThrottle(View v){
        originalProgress = throttleBar.getProgress();
        throttleBar.setProgress(throttleBar.getProgress() - THROTTLE_AMOUNT);
    }

    public void calibratePhone(View v){
        cYaw = rawYaw;
        cPitch = rawPitch;
        cRoll = rawRoll;
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
                PIDChanged = true;
                RP_P = Math.round((RP_P + RP_P_AMOUNT) * 100000.0) / 100000.0;
                ((TextView) findViewById(R.id.RP_P_label)).setText(String.valueOf(RP_P));
            }
        });

        findViewById(R.id.sub_RP_P).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                    PIDChanged = true;
                    RP_P = Math.round((RP_P - RP_P_AMOUNT) * 100000.0) / 100000.0;
                if (RP_P <= 0) {
                    RP_P = 0;
                }
                    ((TextView) findViewById(R.id.RP_P_label)).setText(String.valueOf(RP_P));
            }
        });

        //RP_I
        findViewById(R.id.add_RP_I).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                RP_I = Math.round((RP_I + RP_I_AMOUNT)*100000.0)/100000.0;
                ((TextView) findViewById(R.id.RP_I_label)).setText(String.valueOf(RP_I));
            }
        });

        findViewById(R.id.sub_RP_I).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                    PIDChanged = true;
                    RP_I = Math.round((RP_I - RP_I_AMOUNT) * 100000.0) / 100000.0;
                    if (RP_I <= 0) {
                        RP_I = 0;
                    }
                    ((TextView) findViewById(R.id.RP_I_label)).setText(String.valueOf(RP_I));

            }
        });

        //RP_D
        findViewById(R.id.add_RP_D).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                RP_D = Math.round((RP_D + RP_D_AMOUNT)*100000.0)/100000.0;
                ((TextView) findViewById(R.id.RP_D_label)).setText(String.valueOf(RP_D));
            }
        });

        findViewById(R.id.sub_RP_D).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                RP_D = Math.round((RP_D - RP_D_AMOUNT)*100000.0)/100000.0;
                if (RP_D <= 0) {
                    RP_D = 0;
                }
                ((TextView) findViewById(R.id.RP_D_label)).setText(String.valueOf(RP_D));
            }
        });

        ///Y_D
        findViewById(R.id.add_Y_P).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                Y_P = Math.round((Y_P + Y_P_AMOUNT)*100000.0)/100000.0;
                ((TextView) findViewById(R.id.Y_P_label)).setText(String.valueOf(Y_P));
            }
        });

        findViewById(R.id.sub_Y_P).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                Y_P = Math.round((Y_P - Y_P_AMOUNT)*100000.0)/100000.0;
                if (Y_P <= 0) {
                    Y_P = 0;
                }
                ((TextView) findViewById(R.id.Y_P_label)).setText(String.valueOf(Y_P));
            }
        });

        //Y_I
        findViewById(R.id.add_Y_I).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                Y_I = Math.round((Y_I + Y_I_AMOUNT)*100000.0)/100000.0;
                ((TextView) findViewById(R.id.Y_I_label)).setText(String.valueOf(Y_I));
            }
        });

        findViewById(R.id.sub_Y_I).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                Y_I = Math.round((Y_I - Y_I_AMOUNT)*100000.0)/100000.0;
                if (Y_I <= 0) {
                    Y_I = 0;
                }
                ((TextView) findViewById(R.id.Y_I_label)).setText(String.valueOf(Y_I));
            }
        });

        //Y_D
        findViewById(R.id.add_Y_D).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                Y_D = Math.round((Y_D + Y_D_AMOUNT)*100000.0)/100000.0;
                ((TextView) findViewById(R.id.Y_D_label)).setText(String.valueOf(Y_D));
            }
        });

        findViewById(R.id.sub_Y_D).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                PIDChanged = true;
                Y_D = Math.round((Y_D - Y_D_AMOUNT)*100000.0)/100000.0;
                if (Y_D <= 0) {
                    Y_D = 0;
                }
                ((TextView) findViewById(R.id.Y_D_label)).setText(String.valueOf(Y_D));
            }
        });

    }
}
