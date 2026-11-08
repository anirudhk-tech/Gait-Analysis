// Sensors: 0 = medial forefoot, 1 = lateral forefoot, 2 = medial mid-foot, 3 = heel.

String[] names = {"MF", "LF", "MM", "Heel"};
color[] lineColors = {#E53935, #1E88E5, #43A047, #FB8C00};
float[] bubbleX = {160, 220, 100, 160};
float[] bubbleY = {300, 440, 440, 580};
int N = 300;
float[] fsr = new float[4];
float[][] history = new float[4][N];
int THRESHOLD = 300;

float[] baseline = new float[4];
int DEADBAND = 60;
boolean calibrating = false;
int calibStartMs = 0;
int CALIB_MS = 1000;
float[] calibSum = new float[4];
int calibCount = 0;
int YMAX = 1500;
float[] force = new float[4];

boolean onGround = false;
int strideCount = 0;
int firstStrideMs = 0;
int lastStrideMs = 0;
int previousStrideMs = 0;
float[] stanceSum = new float[4];
float mfp = 0;
String gait = "Walk to see";

float activity = 0;
int lastMotionMs = 0;
int periodStartMs = 0;
int activeMs = 0;
int lastUpdateMs = 0;
boolean moving = false;

String stepLenStr = "70";
final int TARGET_CADENCE_MIN = 170;
final int TARGET_CADENCE_MAX = 180;
float impactG = 0;
float strikePeakG = 0;
int strikeWindowEndMs = 0;

void setup() {
    size(1000, 720);
    setupSerial();
    startCalibration();
}

void startCalibration() {
    calibrating = true;
    calibStartMs = millis();
    calibCount = 0;
    for (int i = 0; i < 4; i++) calibSum[i] = 0;
}

void keyPressed() {
    if (key == 'c' || key == 'C') startCalibration();
    if (key == 'k' || key == 'K') startRhythmCalib();
    if (key == 'l' || key == 'L') toggleLog();
    if (key == 'r' || key == 'R') {
        strideCount = 0;
        firstStrideMs = 0;
        lastStrideMs = 0;
        previousStrideMs = 0;
    }
    if (key >= '0' && key <= '9' && stepLenStr.length() < 3) stepLenStr += key;
    if (key == BACKSPACE && stepLenStr.length() > 0) stepLenStr = stepLenStr.substring(0, stepLenStr.length() - 1);
}

void draw() {
    background(255);
    fill(0);
    textSize(14);

    int steps = strideCount * 2;
    float cadence = currentCadence();
    color cadenceColor = cadenceColor(cadence);
    int stepLen = stepLenStr.length() > 0 ? int(stepLenStr) : 0;
    float period = moving ? (millis() - periodStartMs) / 1000.0 : 0;
    float speed = stepLen * cadence / 6000.0;

    if (calibrating) {
        fill(#FB8C00);
        text("Calibrating baseline... keep foot still (press c to redo)", 300, 25);
        fill(0);
    }

    text("Gait: " + gait + "   MFP: " + nf(mfp, 0, 1) + " %", 20, 25);
    text("Status: " + (moving ? "In Motion" : "Standing Still"), 20, 50);
    text("Period of activity: " + nf(period, 0, 1) + " s   Total active: " + nf(activeMs / 1000.0, 0, 1) + " s", 20, 75);
    text("Step count: " + steps + "   (r = reset)", 20, 100);
    fill(cadenceColor);
    text("Cadence: " + nf(cadence, 0, 1) + " steps/min", 20, 125);
    text("Target: " + TARGET_CADENCE_MIN + "-" + TARGET_CADENCE_MAX + " steps/min - " + cadenceStatus(cadence), 300, 125);
    fill(0);
    text("Step length: " + stepLenStr + " cm (type to change)   Stride length: " + (stepLen * 2) + " cm   Speed: " + nf(speed, 0, 2) + " m/s", 20, 150);
    text("Activity: " + activityType(cadence), 20, 175);
    text("Last foot-strike impact: " + nf(impactG, 0, 2) + " g", 300, 175);
    if (logging) { fill(#E53935); text("REC (l = stop)", 800, 175); fill(0); }

    noFill();
    stroke(0);
    ellipse(160, 440, 230, 500);
    textAlign(CENTER, CENTER);
    for (int i = 0; i < 4; i++) {
        float f = constrain(fsr[i] / YMAX, 0, 1);
        fill(lerpColor(color(0, 0, 255), color(255, 0, 0), f));
        ellipse(bubbleX[i], bubbleY[i], 30 + f * 50, 30 + f * 50);
        fill(0);
        text(names[i] + " " + int(fsr[i]), bubbleX[i], bubbleY[i] + 55);
    }
    textAlign(LEFT, BASELINE);

    for (int s = 0; s < 4; s++) {
        drawGraph(s, 360, 200 + s * 130, 600, 105);
    }
    fill(0);
    text("Time (last " + N + " samples)", 580, 710);
}

void drawGraph(int s, float x, float y, float w, float h) {
    fill(0);
    stroke(0);
    text(names[s], x, y - 6);
    text("0", x - 25, y + h);
    text(YMAX, x - 40, y + 10);
    text("FSR", x - 40, y + h / 2);
    noFill();
    rect(x, y, w, h);

    stroke(lineColors[s]);
    noFill();
    beginShape();
    for (int i = 0; i < N; i++) {
        vertex(x + map(i, 0, N - 1, 0, w), y + h - map(constrain(history[s][i], 0, YMAX), 0, YMAX, 0, h));
    }
    endShape();
}

void onSample(float ax, float ay, float az) {
    int now = millis();
    updateRhythmCalib();
    float accelMagnitudeG = sqrt(ax * ax + ay * ay + az * az) / 16384.0;
    float dynamicG = abs(accelMagnitudeG - 1.0);

    if (strikeWindowEndMs > 0) {
        if (now <= strikeWindowEndMs) strikePeakG = max(strikePeakG, dynamicG);
        else {
            impactG = strikePeakG;
            strikeWindowEndMs = 0;
        }
    }

    float total = 0;
    for (int i = 0; i < 4; i++) {
        force[i] = fsr[i] > THRESHOLD ? fsr[i] : 0;
        total += force[i];
        arrayCopy(history[i], 1, history[i], 0, N - 1);
        history[i][N - 1] = force[i];
    }

    if (!onGround && total > 0 && now - lastStrideMs > stepGap) {
        onGround = true;
        if (strideCount == 0) firstStrideMs = now;
        previousStrideMs = lastStrideMs;
        strideCount++;
        onFootfall(now);
        lastStrideMs = now;
        strikePeakG = dynamicG;
        strikeWindowEndMs = now + 150;
        stanceSum = new float[4];
    } else if (onGround && total == 0) {
        onGround = false;
        findGait();
    }
    if (onGround) {
        for (int i = 0; i < 4; i++) stanceSum[i] += force[i];
    }

    activity = 0.9 * activity + 0.1 * dynamicG;
    if (activity > 0.08) lastMotionMs = now;
    boolean nowMoving = now - lastMotionMs < 1000;
    if (nowMoving && !moving) periodStartMs = now;
    if (moving) activeMs += now - lastUpdateMs;
    lastUpdateMs = now;
    moving = nowMoving;
}

float currentCadence() {
    if (strideCount < 2 || previousStrideMs <= 0 || lastStrideMs <= previousStrideMs) return 0;
    if (millis() - lastStrideMs > 2500) return 0;
    return 120000.0 / (lastStrideMs - previousStrideMs);
}

String activityType(float cadence) {
    if (!moving) return "Waiting";
    if (cadence <= 0) return "Detecting";
    if (cadence < 120) return "Walking";
    if (cadence < TARGET_CADENCE_MIN) return "Jogging";
    return "Running";
}

String cadenceStatus(float cadence) {
    if (cadence <= 0) return "waiting for steps";
    if (cadence < TARGET_CADENCE_MIN) return "too slow";
    if (cadence <= TARGET_CADENCE_MAX) return "on target";
    return "too fast";
}

color cadenceColor(float cadence) {
    if (cadence <= 0) return color(120);
    if (cadence < TARGET_CADENCE_MIN || cadence > TARGET_CADENCE_MAX) return #E53935;
    return #43A047;
}

void findGait() {
    float all = stanceSum[0] + stanceSum[1] + stanceSum[2] + stanceSum[3] + 0.001;
    mfp = (stanceSum[2] + stanceSum[0]) * 100 / all;
    float heelPct = stanceSum[3] * 100 / all;

    if (heelPct < 10) gait = "Tip-toeing";
    else if (heelPct > 60) gait = "Heel walking";
    else if (mfp > 60) gait = "Out-toeing";
    else if (mfp < 40) gait = "In-toeing";
    else gait = "Normal gait";
}
