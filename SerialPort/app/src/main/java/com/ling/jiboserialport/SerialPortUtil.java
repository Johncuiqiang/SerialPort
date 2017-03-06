package com.ling.jiboserialport;

/**
 * Created by cuiqiang on 2017/1/3.
 */

public class SerialPortUtil {

    public static native String getSerialNumber();

    public static native boolean setSerialNumber(String serialNumber);

    public static native void turnLeft();

    public static native void turnRight();

    public static native void forward();

    public static native void goback();

    public static native void stop();

    public static native void moveHand();

    public static native void moveBarrier();

    public static native void openLight();

    public static native void closeLight();

    public static native void setSpeedLeft(int speed);

    public static native void setSpeedRight(int speed);

    static {
        System.loadLibrary("bobby");
    }
}
