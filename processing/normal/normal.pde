String[] names = {"MF", "LF", "MM", "Heel"};
color[] lineColors = {#E53935, #1E88E5, #43A047, #FB8C00};
float[] bubbleX = {110, 210, 110, 160};
float[] bubbleY = {240, 240, 380, 520};

int N = 300;
float[] fsr = new float[4];
float[][] history = new float[4][N];

boolean onGround = false;
int strideCount = 0;
int firstStrideMs = 0;
int lastStrideMs = 0;
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

void setup() {
    size(1000, 700);
    setupSerial();
}

void keyPressed() {
    if (key == 'r') strideCount = 0;
    if (key >= '0' && key <= '9' && stepLenStr.length() < 3) stepLenStr += key;
    if (key == BACKSPACE && stepLenStr.length() > 0) stepLenStr = stepLenStr.substring(0, stepLenStr.length() - 1);
}

void draw() {
    background(255);
    fill(0);
    textSize(14);

    int steps = strideCount * 2;
    float cadence = 0;
    if (strideCount >= 2) cadence = (strideCount - 1) * 2 * 60000.0 / (lastStrideMs - firstStrideMs);
    int stepLen = stepLenStr.length() > 0 ? int(stepLenStr) : 0;
    float period = moving ? (millis() - periodStartMs) / 1000.0 : 0;

    text("Gait: " + gait + "   MFP: " + nf(mfp, 0, 1) + " %", 20, 25);
    text("Status: " + (moving ? "In Motion" : "Standing Still"), 20, 50);
    text("Period of activity: " + nf(period, 0, 1) + " s   Total active: " + nf(activeMs / 1000.0, 0, 1) + " s", 20, 75);
    text("Step count: " + steps + "   Cadence: " + nf(cadence, 0, 1) + " steps/min   (r = reset)", 20, 100);
    text("Step length: " + stepLenStr + " cm (type to change)   Stride length: " + (stepLen * 2) + " cm", 20, 125);

    noFill();
    stroke(0);
    ellipse(160, 380, 230, 500);
    textAlign(CENTER, CENTER);
    for (int i = 0; i < 4; i++) {
        float f = constrain(fsr[i] / 1023.0, 0, 1);
        fill(lerpColor(color(0, 0, 255), color(255, 0, 0), f));
        ellipse(bubbleX[i], bubbleY[i], 30 + f * 50, 30 + f * 50);
        fill(0);
        text(names[i] + " " + int(fsr[i]), bubbleX[i], bubbleY[i] + 55);
    }
    textAlign(LEFT, BASELINE);

    float x = 360, y = 160, w = 600, h = 450;
    noFill();
    stroke(0);
    rect(x, y, w, h);
    for (int s = 0; s < 4; s++) {
        stroke(lineColors[s]);
        beginShape();
        for (int i = 0; i < N; i++) {
            vertex(x + map(i, 0, N - 1, 0, w), y + h - map(history[s][i], 0, 1023, 0, h));
        }
        endShape();
        fill(lineColors[s]);
        text(names[s], x + 10 + s * 60, y - 10);
    }
    fill(0);
    text("0", x - 25, y + h);
    text("1023", x - 40, y + 10);
    text("Time (last " + N + " samples)", x + w / 2 - 80, y + h + 30);
    pushMatrix();
    translate(x - 50, y + h / 2);
    rotate(-HALF_PI);
    text("FSR reading", -30, 0);
    popMatrix();
}

void onSample(float ax, float ay, float az) {
    for (int i = 0; i < 4; i++) {
        arrayCopy(history[i], 1, history[i], 0, N - 1);
        history[i][N - 1] = fsr[i];
    }

    int now = millis();
    float total = fsr[0] + fsr[1] + fsr[2] + fsr[3];

    if (!onGround && total > 300 && now - lastStrideMs > 300) {
        onGround = true;
        if (strideCount == 0) firstStrideMs = now;
        strideCount++;
        lastStrideMs = now;
        stanceSum = new float[4];
    } else if (onGround && total < 150) {
        onGround = false;
        findGait();
    }
    if (onGround) {
        for (int i = 0; i < 4; i++) stanceSum[i] += fsr[i];
    }

    float g = sqrt(ax * ax + ay * ay + az * az) / 16384.0;
    activity = 0.9 * activity + 0.1 * abs(g - 1);
    if (activity > 0.08) lastMotionMs = now;
    boolean nowMoving = now - lastMotionMs < 1000;
    if (nowMoving && !moving) periodStartMs = now;
    if (moving) activeMs += now - lastUpdateMs;
    lastUpdateMs = now;
    moving = nowMoving;
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
