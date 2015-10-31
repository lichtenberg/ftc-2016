package com.qualcomm.ftcrobotcontroller.opmodes;


//
// Class: ModernGyroReader
//
// This is a helper class to make it easier to deal with the Modern Robotics gyroscope, at least for
// compass heading functions.
//
// To use this class, initialize a copy of it using the "I2cDevice" object that you got
// from the hardware map that corresponds to your gyroscope.
//
// In your autonomous or teleop main loop, call the "checkGyro()" method to
// keep ths current heading up to date.
//
// When you want to know your current heading, use the "getHeading()" method.   The returned
// value is a compass heading, from 0 to 359 degrees.
//



import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

class I2CDeviceReader {

    private final I2cDevice device;
    private boolean transaction_complete;
    private boolean buffer_read_complete;
    private byte[] device_data;

    public I2CDeviceReader(I2cDevice i2cDevice, int i2cAddress, int memAddress, int num_bytes) {
        this.device = i2cDevice;
        device_data = null;
        transaction_complete = false;
        buffer_read_complete = false;
        i2cDevice.enableI2cReadMode(i2cAddress, memAddress, num_bytes);
        i2cDevice.setI2cPortActionFlag();
        i2cDevice.writeI2cCacheToController();
        i2cDevice.registerForI2cPortReadyCallback(new I2cController.I2cPortReadyCallback() {
            public void portIsReady(int port) {
                I2CDeviceReader.this.portDone();
            }
        });
    }

    public boolean isDone() {
        return this.transaction_complete && buffer_read_complete;
    }

    private void portDone() {
        if (!transaction_complete && device.isI2cPortReady()) {
            transaction_complete = true;
            device.readI2cCacheFromController();
        }
        else if (transaction_complete) {
            device_data = this.device.getCopyOfReadBuffer();
            device.deregisterForPortReadyCallback();
            buffer_read_complete = true;
        }
    }

    public byte[] getReadBuffer() {
        return device_data;
    }
}


/**
 * Created by mpl on 10/31/15.
 */
public class ModernGyroReader {

    private I2CDeviceReader gyroReader = null;
    int curHeading;
    I2cDevice gyro;

    public ModernGyroReader(I2cDevice theGyro)
    {
        gyro = theGyro;
        gyroReader = null;
    }

    public int getHeading()
    {
        return curHeading;
    }

    public double getHeadingAsDouble() {
        return (double) curHeading;
    }

    public void checkGyro()
    {
        if (gyroReader == null) {
            // Use our helper class to push through the I2C device driver's state machine.
            // Read 2 bytes starting at offset 0x04 from our gyro, which has
            // a fixed address of 0x20.
            gyroReader = new I2CDeviceReader(gyro, 0x20, 0x04, 2);
        } else {
            if (gyroReader.isDone()) {
                byte[] buffer = gyroReader.getReadBuffer();
                int msb = (buffer[1] & 0xFF);
                int lsb = (buffer[0] & 0xFF);
                curHeading = (msb << 8) | lsb;
                gyroReader = null;
            }
        }
    }
}
