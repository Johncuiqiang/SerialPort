package com.ling.jiboserialport;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;

public class MainActivity extends AppCompatActivity implements View.OnClickListener {

    private Button btnUp;
    private Button btnDown;
    private Button btnLeft;
    private Button btnRight;
    private Button btnStop;
    private Button btnBarrier;
    private Button btnHand;
    private Button btnOpen;
    private Button btnClose;
    private Button btnSpeedLeft;
    private Button btnSpeedRight;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initView();
    }

    private void initView() {
        btnUp = (Button) findViewById(R.id.up);
        btnUp.setOnClickListener(MainActivity.this);

        btnDown = (Button) findViewById(R.id.down);
        btnDown.setOnClickListener(MainActivity.this);

        btnLeft = (Button) findViewById(R.id.left);
        btnLeft.setOnClickListener(MainActivity.this);

        btnRight = (Button) findViewById(R.id.right);
        btnRight.setOnClickListener(MainActivity.this);

        btnStop = (Button) findViewById(R.id.stop);
        btnStop.setOnClickListener(MainActivity.this);

        btnBarrier = (Button) findViewById(R.id.moveBarrier);
        btnBarrier.setOnClickListener(MainActivity.this);

        btnHand = (Button) findViewById(R.id.moveHand);
        btnHand.setOnClickListener(MainActivity.this);

        btnOpen = (Button) findViewById(R.id.openlight);
        btnOpen.setOnClickListener(MainActivity.this);

        btnClose = (Button) findViewById(R.id.closelight);
        btnClose.setOnClickListener(MainActivity.this);

        btnSpeedLeft = (Button) findViewById(R.id.speedLeft);
        btnSpeedLeft.setOnClickListener(MainActivity.this);

        btnSpeedRight = (Button) findViewById(R.id.speedRight);
        btnSpeedRight.setOnClickListener(MainActivity.this);

    }

    public void onClick(View v) {
        switch (v.getId()){
            case R.id.up:
                SerialPortUtil.forward();
                break;
            case R.id.left:
                SerialPortUtil.turnLeft();
                break;
            case R.id.right:
                SerialPortUtil.turnRight();
                break;
            case R.id.down:
                SerialPortUtil.goback();
                break;
            case R.id.stop:
                SerialPortUtil.stop();
                break;
            case R.id.moveHand:
                SerialPortUtil.moveHand();
                break;
            case R.id.moveBarrier:
                SerialPortUtil.moveBarrier();
                break;
            case R.id.openlight:
                SerialPortUtil.openLight();
                break;
            case R.id.closelight:
                SerialPortUtil.closeLight();
                break;
            case R.id.speedLeft:
                SerialPortUtil.setSpeedLeft(0x09);
                break;
            case R.id.speedRight:
                SerialPortUtil.setSpeedRight(0x09);
                break;
        }
    }
}
