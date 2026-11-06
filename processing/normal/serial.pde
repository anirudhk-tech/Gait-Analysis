import processing.serial.*;

String PORT_NAME = "COM7";
int baudrate = 115200;
Serial port;
String[] items = new String[7];

void setupSerial() {
    printArray(Serial.list());
    port = new Serial(this, PORT_NAME, baudrate);
    port.bufferUntil('\n');
}

void serialEvent(Serial p) {
    String line = p.readStringUntil('\n');
    if (line == null) return;
    line = trim(line);
    println("serial: " + line);

    items = splitTokens(line, ", ");
    if (items.length < 7) return;

    float[] raw = new float[4];
    for (int i = 0; i < 4; i++) raw[i] = float(items[i]);

    // While calibrating, accumulate resting values per sensor instead of plotting.
    if (calibrating) {
        for (int i = 0; i < 4; i++) calibSum[i] += raw[i];
        calibCount++;
        if (millis() - calibStartMs >= CALIB_MS && calibCount > 0) {
            for (int i = 0; i < 4; i++) baseline[i] = calibSum[i] / calibCount;
            calibrating = false;
        }
        return;
    }

    // Subtract each sensor's own baseline, then ignore anything inside the deadband.
    for (int i = 0; i < 4; i++) {
        float v = raw[i] - baseline[i];
        fsr[i] = v > DEADBAND ? v : 0;
    }
    onSample(float(items[4]), float(items[5]), float(items[6]));
    sendLeds();
    logRow();
}

// Map each FSR force to an LED brightness (0-255) and send to the Arduino.
void sendLeds() {
    String msg = "L";
    for (int i = 0; i < 4; i++) msg += "," + int(map(constrain(fsr[i], 0, YMAX), 0, YMAX, 0, 255));
    port.write(msg + "\n");
}
