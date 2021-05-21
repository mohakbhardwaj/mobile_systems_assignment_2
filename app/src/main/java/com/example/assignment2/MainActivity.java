package com.example.assignment2;

import  com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.LegendRenderer;
import  com.jjoe64.graphview.series.LineGraphSeries;
import  com.jjoe64.graphview.series.DataPoint;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import androidx.appcompat.app.AppCompatActivity;
import android.hardware.SensorEventListener;
import android.widget.TextView;

import java.io.BufferedWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.ArrayList;
import java.text.DecimalFormat;
import java.util.Date;

public class MainActivity extends AppCompatActivity implements SensorEventListener{

    private final Handler mHandler = new Handler();

    private TextView xAccTV, yAccTV, zAccTV;
    private TextView xGyroTV, yGyroTV, zGyroTV;
    private TextView GyroPitchTV, GyroRollTV, AccPitchTV, AccRollTV, FiltPitchTV, FiltRollTV;
    private TextView mainText;

    private SensorManager SM;
    private Sensor accelerometer;
    private Sensor gyroscope;
    private Button startB;
    private Button stopB;
    private boolean startFlag;

    float[] currAcc = new float[3];
    float[] currAccUnit = new float[3];
    float[] currGyro = new float[3];
    float[] currQuat = new float[4];
    float[] currQuatAccBody = new float[4];
    float[] currQuatAccWorld = new float[4];
    float[] currQuatGyro = new float[4];
    float[] currDQuatGyro = new float[4];
    float[] currRotAxis = new float[3];
    float[] currEuler = new float[3];
    float[] currEulerGyro = new float[3];
    float[] accTiltAxis = new float[3];
    float[] accTiltQuat = new float[4];
    float currAngVel, currAccNorm, currDTheta, currTilt, currTiltAcc, currTiltGyro, pitchAcc, rollAcc;
    float[] accBias = new float[3];
    float[] accNoise = new float[3];
    float[] gyroBias = new float[3];
    float[] gyroNoise = new float[3];
    float[] globalQuat = new float[4];
    private int samplingRate = 200; //Hz
    private float samplingDtSec = (float) 1.0 / (float)200;
    private int samplingDtMicroSec = (int) (samplingDtSec * (float) 1e6);
    private int NSamples = 1000;
    private int currSamplesAcc = 0;
    private int currSamplesGyro = 0;
    float[][] accData=  new float[NSamples][3];
    float[][] gyroData=  new float[NSamples][3];
    ArrayList<Float> pitchGyroList, rollGyroList, pitchAccList, rollAccList, pitchFilterList, rollFilterList;
    float[] vertVec = new float[3];
    double alpha = 0.6; //0.7; //0.85;
    private LineGraphSeries<DataPoint> mSeries, mSeries2, mSeriesGyro, mSeries2Gyro, mSeriesAcc, mSeries2Acc;
    GraphView graph, graph2;
    DecimalFormat df;
    long startTime, currentTime;
    float timeElapsed;
    private BufferedWriter writer;
    private boolean saved;



//     Used to load the 'native-lib' library on application startup.
//    static {
//        System.loadLibrary("native-lib");
//    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        accBias[0] = (float) 0.12430972;
        accBias[1] = (float) 0.22201096;
        accBias[2] = (float) 0.0018010712;

        startFlag = false;
        saved = false;
        currQuat[0] = (float) 1.0;
        currQuatAccBody[0] = (float) 0.0;
//        currQuatAccWorld[0] = (float) 0.0;
        currQuatGyro[0] = (float) 1.0;
        accTiltQuat[0] = (float) 1.0;
        currTilt = (float) 0.0;
        currTiltAcc = (float) 0.0;
        currTiltGyro = (float) 0.0;
        vertVec[0] = (float) 0.0;
        vertVec[1] = (float) 0.0;
        vertVec[2] = (float) 1.0;
        // Create our sensor Manager
        SM = (SensorManager)getSystemService(SENSOR_SERVICE);

        // Accelerometer and gyroscopr sensor
        accelerometer = SM.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope = SM.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        // Register sensor listener
        SM.registerListener(this, accelerometer, samplingDtMicroSec);
        SM.registerListener(this, gyroscope, samplingDtMicroSec);
        df = new DecimalFormat("###.####");


        // Assign text view
        xAccTV = (TextView) findViewById(R.id.accX);
        yAccTV = (TextView) findViewById(R.id.accY);
        zAccTV = (TextView) findViewById(R.id.accZ);
        xGyroTV = (TextView) findViewById(R.id.gyroX);
        yGyroTV = (TextView) findViewById(R.id.gyroY);
        zGyroTV = (TextView) findViewById(R.id.gyroZ);
        GyroPitchTV = (TextView) findViewById(R.id.PitchGyro);
        GyroRollTV = (TextView) findViewById(R.id.RollGyro);
        AccPitchTV = (TextView) findViewById(R.id.PitchAcc);
        AccRollTV = (TextView) findViewById(R.id.RollAcc);
        FiltPitchTV = (TextView) findViewById(R.id.PitchFilt);
        FiltRollTV = (TextView) findViewById(R.id.RollFilt);
        mainText = (TextView) findViewById(R.id.text);
        graph = (GraphView) findViewById(R.id.graphPitch);
        mSeries = new LineGraphSeries<>();
        mSeriesGyro = new LineGraphSeries<>();
        mSeriesAcc = new LineGraphSeries<>();
//        graph.addSeries(mSeriesAcc);
//        graph.addSeries(mSeriesGyro);
        graph.addSeries(mSeries);
        graph.getViewport().setXAxisBoundsManual(true);
        graph.getViewport().setMinX(0);
        graph.getViewport().setMaxX(5);
//        graph.getViewport().setMaxX(310);
        graph.getViewport().setYAxisBoundsManual(true);
        graph.getViewport().setMinY(-3.1415);
        graph.getViewport().setMaxY(3.1415);
//        graph.getViewport().setMinY(-0.01);
//        graph.getViewport().setMaxY(0.01);
        mSeries.setTitle("Filtered Pitch");
        mSeriesGyro.setTitle("Gyro Pitch");
        mSeriesAcc.setTitle("Acc Pitch");
        mSeriesGyro.setColor(Color.RED);
        mSeriesAcc.setColor(Color.GREEN);
        graph.getLegendRenderer().setVisible(true);
        graph.getLegendRenderer().setAlign(LegendRenderer.LegendAlign.TOP);

        graph2 = (GraphView) findViewById(R.id.graphRoll);
        mSeries2 = new LineGraphSeries<>();
        mSeries2Gyro = new LineGraphSeries<>();
        mSeries2Acc = new LineGraphSeries<>();
//        graph2.addSeries(mSeries2Acc);
//        graph2.addSeries(mSeries2Gyro);
        graph2.addSeries(mSeries2);
        graph2.getViewport().setXAxisBoundsManual(true);
        graph2.getViewport().setMinX(0);
        graph2.getViewport().setMaxX(5);
//        graph2.getViewport().setMaxX(310);
        graph2.getViewport().setYAxisBoundsManual(true);
        graph2.getViewport().setMinY(-3.1415);
        graph2.getViewport().setMaxY(3.1415);
//        graph2.getViewport().setMinY(-0.02);
//        graph2.getViewport().setMaxY(0.02);
        mSeries2.setTitle("Filtered Roll");
        mSeries2Gyro.setTitle("Gyro Roll");
        mSeries2Acc.setTitle("Acc Roll");
        mSeries2Gyro.setColor(Color.RED);
        mSeries2Acc.setColor(Color.GREEN);
        graph2.getLegendRenderer().setVisible(true);
        graph2.getLegendRenderer().setAlign(LegendRenderer.LegendAlign.TOP);

        pitchGyroList = new ArrayList<Float>();
        rollGyroList = new ArrayList<Float>();
        pitchAccList = new ArrayList<Float>();
        rollAccList = new ArrayList<Float>();
        pitchFilterList = new ArrayList<Float>();
        rollFilterList = new ArrayList<Float>();

        //Setup buttons
        startB = (Button) findViewById(R.id.btnStart);
        stopB = (Button) findViewById(R.id.btnStop);
        stopB.setEnabled(false);
        startB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("btnStart", "Button pushed!");
                startTime = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();
                timeElapsed = (float) ((currentTime - startTime)/1000.0);

                startFlag = true;
                startB.setEnabled(false);
                stopB.setEnabled(true);
                mainText.setText("Measurements started...");
            }
        });

        stopB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("btnStop", "Button pushed!");
                startFlag = false;
                startB.setEnabled(true);
                stopB.setEnabled(false);
                mainText.setText("Press Start to start measuring...");
            }
        });
    }
    @Override
    protected void onResume() {
        super.onResume();
        SM.registerListener(this, accelerometer, samplingDtMicroSec);
        SM.registerListener(this, gyroscope, samplingDtMicroSec);
    }

    @Override
    protected void onPause() {
        super.onPause();
        SM.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (startFlag) {
            float x = event.values[0];
            float y = event.values[1];
            float z = event.values[2];

            if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                xAccTV.setText(Float.toString(x));
                yAccTV.setText(Float.toString(y));
                zAccTV.setText(Float.toString(z));

                currAcc[0] = x; //- accBias[0];
                currAcc[1] = y - accBias[1];
                currAcc[2] = z - accBias[2];

                currAccNorm = l2_norm(currAcc);
                currAccUnit[0] = currAcc[0] / currAccNorm;
                currAccUnit[1] = currAcc[1] / currAccNorm;
                currAccUnit[2] = currAcc[2] / currAccNorm;

                //accelerometer quaternion in body frame
                currQuatAccBody[0] = (float) 0.0;
                currQuatAccBody[1] = currAcc[0];
                currQuatAccBody[2] = currAcc[1];
                currQuatAccBody[3] = currAcc[2];
                float n = l2_norm(currQuatAccBody);

                //accelerometer quaternion in world frame
//                currQuatAccWorld =  rotateQuaternion(currQuatAccBody, currQuat);
//                currQuatAccWorld[0] /= n;
//                currQuatAccWorld[1] /= n;
//                currQuatAccWorld[2] /= n;
//                currQuatAccWorld[3] /= n;

                pitchAcc = (float) -Math.atan2(currAcc[0], currAcc[2]);
                double sgn = Math.signum(currAcc[2]);
                rollAcc = (float) -Math.atan2(-currAcc[1], sgn * Math.sqrt(currAcc[0] * currAcc[0] + currAcc[2] * currAcc[2]));
                AccPitchTV.setText(String.valueOf(df.format(pitchAcc)));
                AccRollTV.setText(String.valueOf(df.format(rollAcc)));

                //calculate accelerometer tilt angle and quaternion
                currTiltAcc = (float) Math.atan2(Math.sqrt(1.0 - (currQuatAccBody[3]*currQuatAccBody[3])/(n*n)), currQuatAccBody[3]/n);
                accTiltAxis[0] = currQuatAccBody[2];
                accTiltAxis[1] = -currQuatAccBody[1];
                accTiltAxis[2] = (float) 0.0;
                accTiltAxis[0] /= l2_norm(accTiltAxis);
                accTiltAxis[1] /= l2_norm(accTiltAxis);
                accTiltAxis[2] /= l2_norm(accTiltAxis);

                accTiltQuat = angleAxisToQuat((float) ((1.0 - alpha) * currTiltAcc), accTiltAxis);

                pitchAccList.add(Float.valueOf(df.format(pitchAcc)));
                rollAccList.add(Float.valueOf(df.format(rollAcc)));

//                if(currSamplesAcc < NSamples){
//                    accData[currSamplesAcc][0] = x;
//                    accData[currSamplesAcc][1] = y;
//                    accData[currSamplesAcc][2] = z;
//                    currSamplesAcc += 1;
//                }
//                else{
//                    accBias = getMean(accData);
//                    accNoise = getVariance(accData, accBias);
//                    Log.d("sensor", "acc bias " + String.valueOf(Math.abs(accBias[0])) + " " + String.valueOf(Math.abs(accBias[1])) + " " + String.valueOf(Math.abs(accBias[2])));
//                    Log.d("sensor", "acc noise " + String.valueOf(Math.abs(accNoise[0])) + " " + String.valueOf(Math.abs(accNoise[1])) + " " + String.valueOf(Math.abs(accNoise[2])));
//
//                }

            } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                xGyroTV.setText(Float.toString(x));
                yGyroTV.setText(Float.toString(y));
                zGyroTV.setText(Float.toString(z));
                currGyro[0] = x;
                currGyro[1] = y;
                currGyro[2] = z;
                currAngVel = l2_norm(currGyro);
                currRotAxis[0] = currGyro[0] / currAngVel;
                currRotAxis[1] = currGyro[1] / currAngVel;
                currRotAxis[2] = currGyro[2] / currAngVel;

                currDQuatGyro = angleAxisToQuat(currAngVel * samplingDtSec, currRotAxis);

                currQuatGyro = multiplyQuaternions(currQuatGyro, currDQuatGyro); //Dead-reckoning
                currEulerGyro = quaternionToEuler(currQuatGyro);

                GyroPitchTV.setText(String.valueOf(df.format(currEulerGyro[1])));
                GyroRollTV.setText(String.valueOf(df.format(currEulerGyro[2])));
                pitchGyroList.add(Float.valueOf(df.format(currEulerGyro[1])));
                rollGyroList.add(Float.valueOf(df.format(currEulerGyro[2])));

                //Complementary filter
                currQuat = multiplyQuaternions(accTiltQuat, currQuatGyro);
                //get euler
                currEuler = quaternionToEuler(currQuat);
                FiltPitchTV.setText(String.valueOf(df.format(currEuler[1])));
                FiltRollTV.setText(String.valueOf(df.format(currEuler[2])));
                pitchFilterList.add(Float.valueOf(df.format(currEuler[1])));
                rollFilterList.add(Float.valueOf(df.format(currEuler[2])));

                //Plot filtered roll and pitch
                currentTime = System.currentTimeMillis();
                timeElapsed = (float) ((currentTime - startTime)/1000.0);
//                Log.d("time", String.valueOf(timeElapsed));
                mSeries.appendData(new DataPoint(timeElapsed, currEuler[1]), true, 500);
                mSeries2.appendData(new DataPoint(timeElapsed, currEuler[2]), true, 500);


//                if(timeElapsed <= 300){
//                    mSeries.appendData(new DataPoint(timeElapsed, currEuler[1]), false, 60000);
//                    mSeriesGyro.appendData(new DataPoint(timeElapsed, currEulerGyro[1]), false, 60000);
//                    mSeriesAcc.appendData(new DataPoint(timeElapsed, pitchAcc), false, 60000);
//
//                    mSeries2.appendData(new DataPoint(timeElapsed, currEuler[2]), false, 60000);
//                    mSeries2Gyro.appendData(new DataPoint(timeElapsed, currEulerGyro[2]), false, 60000);
//                    mSeries2Acc.appendData(new DataPoint(timeElapsed, rollAcc), false, 60000);
//                }
                mHandler.postDelayed(new Runnable(){
                    @Override
                    public void run(){}
                }, 200);

            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    public static float l2_norm(float[] arr){
        float norm = (float) 0.0;

        for(int i=0; i < arr.length; ++i){
            norm += arr[i] * arr[i];
        }
        return (float) Math.sqrt(norm);
    }

    public float[] getMean(float[][] arr){
        float[] mean = new float[arr[0].length];
        float[] sum = new float[arr[0].length];
        Arrays.fill(sum, (float)0.0);

        for(int i=0; i < arr.length; ++i){
            for(int j=0; j < arr[0].length; ++j){
                sum[j] += arr[i][j];
            }
        }
        for(int i=0; i < arr[0].length; ++i){
            mean[i] = sum[i] / arr.length;
        }

        return mean;
    }

    public float[] getVariance(float[][] arr, float[] mean){
        float[] var = new float[arr[0].length];
        float[] sqsum = new float[arr[0].length];
        float[] sqsum_mean = new float[arr[0].length];

        for(int i=0; i < arr.length; ++i){
            for(int j=0; j < arr[0].length; ++j){
                sqsum[j] += arr[i][j] * arr[i][j];
            }
        }
        for(int i=0; i < arr[0].length; ++i){
            sqsum_mean[i] = sqsum[i] / arr.length;
        }
        for(int i=0; i < arr[0].length; ++i){
            var[i] = sqsum_mean[i] - (mean[i]*mean[i]);
        }

        return var;
    }

    float angBetVecs(float[] vec1, float[] vec2){
        return (float) Math.acos(vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]);
    }

    public static float normalizeAngle(float angle) {
        return (float) Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    public float[] angleAxisToQuat(float ang, float[] axis){
        //Assumes axis is normalized
        float[] quat = new float[4];
        quat[0] = (float) Math.cos(ang/2.0);
        float sth2 = (float) Math.sin(ang/2.0);
        quat[1] = axis[0] * sth2;
        quat[2] = axis[1] * sth2;
        quat[3] = axis[2] * sth2;

        return quat;
    }

    public float[] quatToAngleAxis(float[] quat){
        float[] angAxis = new float[4];
        angAxis[0] = (float) ((float) 2.0 * Math.acos(quat[0]));
        float s = (float) Math.sqrt(1-quat[0]*quat[0]);
        if (s < 0.001) {
            angAxis[1] = quat[1];
            angAxis[2] = quat[2];
            angAxis[3] = quat[3];
        } else {
            angAxis[1] = quat[1] / s;
            angAxis[2] = quat[2] / s;
            angAxis[3] = quat[3] / s;
        }
        return angAxis;
    }

    public static float[] eulerToQuaternion(float[] r){
        float yaw = r[0];
        float pitch = r[1];
        float roll = r[2];
        float qx = (float) (Math.sin(roll/2) * Math.cos(pitch/2) * Math.cos(yaw/2) - Math.cos(roll/2) * Math.sin(pitch/2) * Math.sin(yaw/2));
        float qy = (float) (Math.cos(roll/2) * Math.sin(pitch/2) * Math.cos(yaw/2) + Math.sin(roll/2) * Math.cos(pitch/2) * Math.sin(yaw/2));
        float qz = (float) (Math.cos(roll/2) * Math.cos(pitch/2) * Math.sin(yaw/2) - Math.sin(roll/2) * Math.sin(pitch/2) * Math.cos(yaw/2));
        float qw = (float) (Math.cos(roll/2) * Math.cos(pitch/2) * Math.cos(yaw/2) + Math.sin(roll/2) * Math.sin(pitch/2) * Math.sin(yaw/2));
        float[] quat = new float[4];
        quat[0] = qw;
        quat[1] = qx;
        quat[2] = qy;
        quat[3] = qz;
        return  quat;
    }
//            (yaw, pitch, roll) = (r[0], r[1], r[2])

    public static float[] quaternionToEuler(float[] q){
        float[] r = new float[3];
        float w = q[0];
        float x = q[1];
        float y = q[2];
        float z = q[3];
        float t0 = (float) (+2.0 * (w * x + y * z));
        float t1 = (float) (+1.0 - 2.0 * (x * x + y * y));
        float roll = (float) Math.atan2(t0, t1);
        float t2 = (float) (+2.0 * (w * y - z * x));
        if(t2 > +1.0){
            t2 = (float) +1.0;
        }
        if(t2 < -1.0){
            t2 = (float) -1.0;
        }
        float pitch = (float) Math.asin(t2);
        float t3 = (float) (+2.0 * (w * z + x * y));
        float t4 = (float) (+1.0 - 2.0 * (y * y + z * z));
        float yaw = (float) Math.atan2(t3, t4);

        r[0] = yaw;
        r[1] = pitch;
        r[2] = roll;
        return r;
    }

    public static float[] addQuaternions(float[] q1, float[] q2){
        float w=  q1[0] + q2[0];
        float x = q1[1] + q2[1];
        float y = q1[2] + q2[2];
        float z = q1[3] + q2[3];
        return  new float[]{w, x, y, z};
    }

    public static float[] multiplyQuaternions(float[] q1, float[] q2) {
        float w1 = q1[0]; float w2 = q2[0];
        float x1 = q1[1]; float x2 = q2[1];
        float y1 = q1[2]; float y2 = q2[2];
        float z1 = q1[3]; float z2 = q2[3];


        float w=  w1*w2 - x1*x2 - y1*y2 - z1*z2;
        float x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
        float y = w1*y2 - x1*z2 + y1*w2 + z1*x2 ;
        float z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
        return  new float[]{w, x, y, z};

    }

    public static  float[] quaternionConjugate(float[] q1){
        return new float[]{q1[0], -q1[1], -q1[2], -q1[3]};
    }

    public static  float[] quaternionInverse(float[] q1){
        float q1_norm = l2_norm(q1);
        float q1_norm_sq = q1_norm * q1_norm;

        return new float[]{q1[0]/q1_norm_sq, -q1[1]/q1_norm_sq, -q1[2]/q1_norm_sq, -q1[3]/q1_norm_sq};
    }

    public static float[] rotateQuaternion(float[] q1, float[] q2) {
        float[] q2_inv = quaternionInverse(q2);
        return multiplyQuaternions(q2, multiplyQuaternions(q1, q2_inv));
    }

}
