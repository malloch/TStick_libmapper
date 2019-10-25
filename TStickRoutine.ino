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

    mapper_signal_update(sigRawAccel, outAccel, 3, tt);

    outMag[0] = m.magnetic.x;
    outMag[1] = m.magnetic.y;
    outMag[2] = m.magnetic.z;

    mapper_signal_update(sigRawMag, outMag, 3, tt);

    outGyro[0] = g.gyro.x;
    outGyro[1] = g.gyro.y;
    outGyro[2] = g.gyro.z;

    mapper_signal_update(sigRawGyro, outGyro, 3, tt);

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

    mapper_signal_update(sigOrientation, q, 4, tt);

    ledBlink();
    then = now;
}
