import processing.serial.*;

String PORT_NAME = "/dev/cu.usbmodem123456781";
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

    for (int i = 0; i < 4; i++) fsr[i] = float(items[i]);
    onSample(float(items[4]), float(items[5]), float(items[6]));
}
