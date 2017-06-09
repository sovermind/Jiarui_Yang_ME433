package com.example.yangjiarui.final_project_android;
// libraries

import android.Manifest;
import android.app.Activity;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
//import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;

import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

//import org.w3c.dom.Text;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;


public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private TextureView mTextureView;
    private TextView RangeTextView;
    private TextView ThreshTextView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;
    private SeekBar RangeSeekBar;
    private SeekBar ThreshSeekBar;

    //USB CDC Variables
    private UsbManager manager;
    private UsbSerialPort sPort;
    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private SerialInputOutputManager mSerialIoManager;

    static long prevtime = 0; // for FPS calculation
    int thresh = 0; // comparison value
    int T_COM = 70;
    int R_COM = 40;
    float COM = 0;
    float[] COM_inRange = new float[100];
    int motor1_PWM = 0;
    int motor2_PWM = 0;
    float servo_PWM = 4850;
    float pre_servo_PWM = 4850;
    float prev_COM = 0;
    float servo_neutral_PWM = 4850;
    float motor1_neutral_PWM = 450;
    float motor2_neutral_PWM = 450;

    //PD control gains
    float Kp = 2.8f;
    float Kd = 3.3f;
    float Kp_m = 2.8f;
    float Kd_m = 5f;

    int first_click = 1;

    Button button;
    TextView myTextView2;
    ScrollView myScrollView;
    TextView myTextView3;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off

        mTextView = (TextView) findViewById(R.id.cameraStatus);
        RangeSeekBar = (SeekBar) findViewById(R.id.RangeBar);
        ThreshSeekBar = (SeekBar) findViewById(R.id.ThreshBar);
        RangeTextView = (TextView) findViewById(R.id.RangeValue);
        ThreshTextView = (TextView) findViewById(R.id.ThreshValue);
        myTextView2 = (TextView) findViewById(R.id.textView02);
        myScrollView = (ScrollView) findViewById(R.id.ScrollView01);
        myTextView3 = (TextView) findViewById(R.id.textView03);
        button = (Button) findViewById(R.id.button1);

        for (int i = 0;i<100;i++) {
            COM_inRange[i] = 0;
        }

        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (first_click ==1) {
                    myTextView2.setText("button first clicked ");
                    motor1_PWM = 4;
                    motor2_PWM = 4;
                    servo_PWM = 4850;
                    String sendString = String.valueOf((int)motor1_PWM) +' '+String.valueOf((int)motor2_PWM)+ ' '+String.valueOf((int)servo_PWM)+'\n';
                    try {
                        sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                    } catch (IOException e) { }
                    first_click = 0;
                } else {
                    myTextView2.setText("button clicked ");
                    motor1_PWM = 10*RangeSeekBar.getProgress();
                    motor2_PWM = 10*RangeSeekBar.getProgress();
                    servo_PWM = 70*ThreshSeekBar.getProgress();
                    String sendString = String.valueOf((int)motor1_PWM) +' '+String.valueOf((int)motor2_PWM)+ ' '+String.valueOf((int)servo_PWM)+'\n';
                    try {
                        sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                    } catch (IOException e) { }
                }

            }
        });

        // see if the app has permission to use the camera
        //ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            mSurfaceHolder = mSurfaceView.getHolder();

            mTextureView = (TextureView) findViewById(R.id.textureview);
            mTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            mTextView.setText("started camera");
        } else {
            mTextView.setText("no camera permissions");
        }
        setMyControlListener();

        manager = (UsbManager) getSystemService(Context.USB_SERVICE);

    }

    //USB CDC Functions
    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {
                @Override
                public void onRunError(Exception e) {

                }

                @Override
                public void onNewData(final byte[] data) {
                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            MainActivity.this.updateReceivedData(data);
                        }
                    });
                }
            };

    @Override
    protected void onPause(){
        super.onPause();
        stopIoManager();
        if(sPort != null){
            try{
                sPort.close();
            } catch (IOException e){ }
            sPort = null;
        }
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();

        ProbeTable customTable = new ProbeTable();
        customTable.addProduct(0x04D8,0x000A, CdcAcmSerialDriver.class);
        UsbSerialProber prober = new UsbSerialProber(customTable);

        final List<UsbSerialDriver> availableDrivers = prober.findAllDrivers(manager);

        if(availableDrivers.isEmpty()) {
            //check
            return;
        }

        UsbSerialDriver driver = availableDrivers.get(0);
        sPort = driver.getPorts().get(0);

        if (sPort == null){
            //check
        }else{
            final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
            UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
            if (connection == null){
                //check
                PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent("com.android.example.USB_PERMISSION"), 0);
                usbManager.requestPermission(driver.getDevice(), pi);
                return;
            }

            try {
                sPort.open(connection);
                sPort.setParameters(9600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            }catch (IOException e) {
                //check
                try{
                    sPort.close();
                } catch (IOException e1) { }
                sPort = null;
                return;
            }
        }
        onDeviceStateChange();
    }

    private void stopIoManager(){
        if(mSerialIoManager != null) {
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if(sPort != null){
            mSerialIoManager = new SerialInputOutputManager(sPort, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange(){
        stopIoManager();
        startIoManager();
    }

    private void updateReceivedData(byte[] data) {
        //do something with received data

        //for displaying:
        String rxString = null;
        try {
            rxString = new String(data, "UTF-8"); // put the data you got into a string
            myTextView3.append(rxString);
            myScrollView.fullScroll(View.FOCUS_DOWN);
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
    }



    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setAutoExposureLock(false); // keep the white balance constant
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    public boolean isGreen(int pix) {
        boolean isG = false;
        if ((green(pix) - red(pix)) > thresh * 2 / 3 && (green(pix) - blue(pix)) > thresh * 2 / 3) {
            isG = true;
        }

        return isG;
    }

    public boolean isGrey(int pix) {
        if (((green(pix) - red(pix)) > -R_COM) && ((green(pix) - red(pix)) < R_COM) && (green(pix) > T_COM) && (red(pix) > T_COM)) {
            if (((green(pix) - blue(pix)) > -R_COM) && ((green(pix) - blue(pix)) < R_COM)) {
                return true;
            } else {
                return false;
            }
        } else
            return false;

    }

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        mTextureView.getBitmap(bmp);

        final Canvas c = mSurfaceHolder.lockCanvas();
        float ave_COM = 0;
        int cc = 0;
        if (c != null) {
            for (int j = 0; j < bmp.getHeight(); j = j + 5) {
                int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
                int startY = j; // which row in the bitmap to analyze to read
                bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);

                // in the row, see if there is more green than red
                int sum_mr = 0;
                int sum_m = 0;
                for (int i = 0; i < bmp.getWidth(); i++) {
//                    if (isGreen(pixels[i])) {
//                        pixels[i] = rgb(0, 255, 0); // over write the pixel with pure green
//                    }
                    if (isGrey(pixels[i])) {
                        pixels[i] = rgb(1, 1, 1); // set the pixel to almost 100% black

                        sum_m = sum_m + green(pixels[i]) + red(pixels[i]) + blue(pixels[i]);
                        sum_mr = sum_mr + (green(pixels[i]) + red(pixels[i]) + blue(pixels[i])) * i;
                    }
                    if (sum_m > 5) {
                        COM = sum_mr / sum_m;
                    } else {
                        COM = 0;
                    }
                }

                if (j>=90 && j<=190) {
                    canvas.drawCircle((int) COM, j, 3, paint1);
                    if (COM<5 || COM >bmp.getWidth()-5) {
                        COM_inRange[cc] = 0;
                    }else
                        COM_inRange[cc] = COM;
                    cc++;
                }

                // update the row
                bmp.setPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);
            }
            float sum_COM = 0;
            int ccc = 0;
            for (int i = 0;i<COM_inRange.length;i++) {
                if (COM_inRange[i]!=0) {
                    sum_COM = sum_COM+COM_inRange[i];
                    ccc++;
                }
            }
            if (ccc<=4){
                ave_COM = 0;
            }else{
                ave_COM = sum_COM/ccc;
            }
        }

        float error = ave_COM - 320;
//        if (error <-150) {
//            motor1_PWM = 300;
//            motor2_PWM = 100;
//        }else if (error > 150){
//            motor2_PWM = 300;
//            motor1_PWM = 100;
//        }
//        else {
//            motor2_PWM = 330;
//            motor1_PWM = 330;
//        }
        //use servo to control the direction

        float error_vel =0;
        if (ave_COM !=0) {
            error_vel = ave_COM - prev_COM;
            prev_COM = ave_COM;
//            servo_PWM = servo_neutral_PWM + error * Kp - error_vel * Kd;
            motor1_PWM = (int) (motor1_neutral_PWM - error * Kp_m - error_vel * Kd_m);
            motor2_PWM = (int) (motor2_neutral_PWM + error * Kp_m + error_vel * Kd_m);
//            if (ave_COM == 0) {
//                servo_PWM = pre_servo_PWM;
//                motor1_PWM = 240;
//                motor2_PWM = 240;
//            }

//            if (motor1_PWM > motor2_PWM) {
//                double angle = Math.atan(0.909*(2*(motor1_PWM - motor2_PWM))/(motor1_PWM+motor2_PWM));
//                servo_PWM = 4850-(float)(angle *12000/3.1415);
//            }else {
//                double angle = Math.atan(0.909*(2*(motor2_PWM - motor1_PWM))/(motor1_PWM+motor2_PWM));
//                servo_PWM = 4850+(float)(angle *12000/3.1415);
//            }
//            servo_PWM = 4850 - (motor1_PWM-motor2_PWM)*1f;
            pre_servo_PWM = servo_PWM;
            if (servo_PWM > 6300) {
                servo_PWM = 6300;
            } else if (servo_PWM < 3000) {
                servo_PWM = 3000;
            }

            if (motor1_PWM > 580) {
                motor1_PWM = 580;
            } else if (motor1_PWM < 10) {
                motor1_PWM = 10;
            }

            if (motor2_PWM > 580) {
                motor2_PWM = 580;
            } else if (motor2_PWM < 10) {
                motor2_PWM = 10;
            }
            if (motor1_PWM == 10 && motor2_PWM != 10) {
                servo_PWM = 4850 + 800;
            }
            else if (motor2_PWM == 10 && motor1_PWM != 10) {
                servo_PWM = 4850 - 800;
            }else {
                servo_PWM = 4850;
            }
            //Now sent over the PWM and servo PWM
//        motor1_PWM = 600;
//        motor2_PWM = 600;
//        servo_PWM = 3000;
            if (first_click == 0) {
                String sendString = String.valueOf((int) motor1_PWM) + ' ' + String.valueOf((int) motor2_PWM) + ' ' + String.valueOf((int) servo_PWM) + '\n';
                try {
                    sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                } catch (IOException e) {
                }
            }
        }

        // draw a circle at some position
        int pos = 50;
        canvas.drawCircle(pos, 240, 5, paint1); // x position, y position, diameter, color

        // write the pos as text
        canvas.drawText("pos = " + ave_COM+","+error+","+servo_PWM + ","+ error_vel, 10, 200, paint1);
        c.drawBitmap(bmp, 0, 0, null);
        mSurfaceHolder.unlockCanvasAndPost(c);

        // calculate the FPS to see how fast the code is running
        long nowtime = System.currentTimeMillis();
        long diff = nowtime - prevtime;
        mTextView.setText("FPS " + 1000 / diff);
        prevtime = nowtime;
    }

    private void setMyControlListener() {
        RangeSeekBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
//                double pg = progress*128.0/100.0;
                R_COM = progress;
                RangeTextView.setText("Range Value: " + R_COM);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        ThreshSeekBar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
//                double pg = progress*128.0/100.0;
                T_COM = progress;
                ThreshTextView.setText("Thresh Value: " + T_COM);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }
}
