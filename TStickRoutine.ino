boolean readTouch() {
    boolean changed = 0;
    byte temp[2] = {0, 0}; int i = 0;
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(BUTTON_STAT);
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADDR, 2);
    while (Wire.available()) { // slave may send less than requested
//    byte c = Wire.read();
//    temp[i] = c; // receive a byte as character
        temp[i] = Wire.read();
        i++;
    }
    Wire.endTransmission();

    for (int t = 0; t < 2; t++) {
        if (temp[t] != touch[t]) {
            changed = 1;
            touch[t] = temp[t];
        }
    }
    return changed;
}


void TStickRoutine() {
    mapper_timetag_t tt = MAPPER_NOW;
    mapper_device_poll(dev, 0);

    if (millis() - lastRead > touchInterval) {
        lastRead = millis();
        int rawCapsenseData[2] = { (int) touch[0] & touchMask[0], (int) touch[1] & touchMask[1] };
        mapper_signal_update(sigRawCapsense, rawCapsenseData, 2, tt);
    }

    lsm.read();  /* ask it to read in the data */

    /* Get a new sensor event */
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    outAccel[0] = a.acceleration.x / 9.80665F;
    outAccel[1] = a.acceleration.y / 9.80665F;
    outAccel[2] = a.acceleration.z / 9.80665F;

    mapper_signal_update(sigRawAccelX, &outAccel[0], 1, tt);
    mapper_signal_update(sigRawAccelY, &outAccel[1], 1, tt);
    mapper_signal_update(sigRawAccelZ, &outAccel[2], 1, tt);

    outMag[0] = m.magnetic.x;
    outMag[1] = m.magnetic.y;
    outMag[2] = m.magnetic.z;

    mapper_signal_update(sigRawMagX, &outMag[0], 1, tt);
    mapper_signal_update(sigRawMagY, &outMag[1], 1, tt);
    mapper_signal_update(sigRawMagZ, &outMag[2], 1, tt);

    outGyro[0] = g.gyro.x / 180.0f;
    outGyro[1] = g.gyro.y / 180.0f;
    outGyro[2] = g.gyro.z / 180.0f;

    mapper_signal_update(sigRawGyroX, &outGyro[0], 1, tt);
    mapper_signal_update(sigRawGyroY, &outGyro[1], 1, tt);
    mapper_signal_update(sigRawGyroZ, &outGyro[2], 1, tt);

    float sigMagGyroOut = sqrt(outGyro[0] * outGyro[0] + outGyro[1] * outGyro[1] + outGyro[2] * outGyro[2]);
    float sigMagAccelOut = sqrt(outAccel[0] * outAccel[0] + outAccel[1] * outAccel[1] + outAccel[2] * outAccel[2]);
    float sigMagMagOut = sqrt(outMag[0] * outMag[0] + outMag[1] * outMag[1] + outMag[2] * outMag[2]);

    mapper_signal_update(sigMagGyro, &sigMagGyroOut, 1, tt);
    mapper_signal_update(sigMagAccel, &sigMagAccelOut, 1, tt);
    mapper_signal_update(sigMagMag, &sigMagMagOut, 1, tt);

    int pressure = analogRead(pressurePin);
    if (calibrate == 1) {
        pressure = map(pressure, calibrationData[0], calibrationData[1], 0, 1024);
        if (pressure < 0) {pressure = 0;}
        // pressure = constrain(pressure, 0, 4095);
    }

    mapper_signal_update(sigRawPressure, &pressure, 1, tt);

    unsigned int piezo = analogRead(piezoPin);
//    if (calibrate == 1) {
//      calibrationData[0] = constrain(min(calibrationData[0], piezo), 0, 4095);
//      calibrationData[1] = constrain(max(calibrationData[1], piezo), 0, 4095);
//    }
    piezo = constrain(map(piezo, calibrationData[0], calibrationData[1], 0, 4095), 0, 4095);;

    mapper_signal_update(sigRawPiezo, &piezo, 1, tt);

    deltaTransferRate = millis();

    // quaternion update and coordinate rotation
    NowQuat = micros();
    deltat = ((NowQuat - lastUpdateQuat) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdateQuat = NowQuat;
    MadgwickQuaternionUpdate(outAccel[0], outAccel[1], outAccel[2], outGyro[0]*PI / 180.0f, outGyro[1]*PI / 180.0f, outGyro[2]*PI / 180.0f, outMag[0], outMag[1], outMag[2]);

    mapper_signal_update(sigOrientationQ1, &q[0], 1, tt);
    mapper_signal_update(sigOrientationQ2, &q[1], 1, tt);
    mapper_signal_update(sigOrientationQ3, &q[2], 1, tt);
    mapper_signal_update(sigOrientationQ4, &q[3], 1, tt);

    ledBlink();
    then = now;
}
