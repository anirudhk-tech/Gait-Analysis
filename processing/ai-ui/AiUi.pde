// ==========================================================================
//  ai-ui  ·  Smart-Shoe Insole Gait Analysis  ·  BME/CS 479 Lab 4
// --------------------------------------------------------------------------
//  A single-file, modern Processing UI for the Gait-Analysis smart insole.
//  It consumes the EXACT same serial stream the Arduino sketch emits:
//
//        fsr0, fsr1, fsr2, fsr3, ax, ay, az \n     (115200 baud)
//
//  Sensor index -> location  (matches Fig.4 of the lab PDF):
//        0 = MF   Medial  Forefoot
//        1 = LF   Lateral Forefoot
//        2 = MM   Medial  Mid-foot
//        3 = Heel
//
//  Features (mapped to the Lab-4 grading rubric):
//    • Live heat-map painted ON the foot outline (hot spot under each sensor).
//    • Bubble heat-map markers that grow + change color with force.
//    • Numerical line graphs per sensor, all axes labeled.
//    • Gait analysis of all 5 profiles: Normal / In-toeing / Out-toeing /
//      Tip-toeing / Heel-walking   (via Medial Force Percentage + heel share).
//    • "In Motion" / "Standing Still" indicator from the accelerometer.
//    • Period-of-activity + total-active timers from the accelerometer.
//    • Step count, cadence, step length, stride length, speed.
//    • Activity label, impact estimate, and target-cadence feedback.
//    • Baseline auto-calibration + deadband for clean, noise-free data.
//    • CSV logging of every profile.
//
//  Keys:  C recalibrate   K rhythm-calib   L log on/off   R reset steps
//         0-9 / BACKSPACE edit step length (cm)
// ==========================================================================

import processing.serial.*;

// ----- serial ------------------------------------------------------------
String PORT_NAME = "COM7";          // <-- change to your port if needed
int    BAUD      = 115200;
Serial port;

// ----- sensors -----------------------------------------------------------
final int NS = 4;
String[] NAME  = { "MF", "LF", "MM", "Heel" };
String[] LONG  = { "Medial Forefoot", "Lateral Forefoot", "Medial Mid-foot", "Heel" };
color[]  TINT  = { color(0xFF, 0x6B, 0x6B), color(0x4F, 0xC3, 0xF7),
                   color(0x81, 0xC7, 0x84), color(0xFF, 0xD5, 0x4F) };

float[] raw     = new float[NS];    // most recent raw ADC value
float[] fsr     = new float[NS];    // baseline-subtracted, deadbanded force
float[] force   = new float[NS];    // thresholded force used for gait math
float[] baseline= new float[NS];

int   YMAX      = 3000;             // full-scale for graphs / heat
int   THRESHOLD = 300;             // on-ground force gate
int   DEADBAND  = 60;              // ignore readings within this of baseline

// ----- history for line graphs ------------------------------------------
int N = 320;
float[][] history = new float[NS][N];

// ----- calibration -------------------------------------------------------
boolean calibrating = false;
int   calibStartMs  = 0;
int   CALIB_MS      = 1200;
float[] calibSum    = new float[NS];
int   calibCount    = 0;

// ----- stride / gait -----------------------------------------------------
boolean onGround   = false;
int   strideCount  = 0;
int   firstStrideMs= 0;
int   lastStrideMs = 0;
float[] stanceSum  = new float[NS];
float mfp          = 0;
String gait        = "Walk to begin";
color  gaitColor   = color(0x9E, 0x9E, 0x9E);
String stepLenStr  = "70";

// rhythm calibration (adaptive step gap)
int   stepGap      = 300;
boolean rhythmCalib= false;
int   rhythmStartMs= 0, firstFootMs = 0, lastFootMs = 0, footCount = 0;
int   previousStrideMs = 0;
final int TARGET_CADENCE_MIN = 170;
final int TARGET_CADENCE_MAX = 180;

// ----- motion / activity -------------------------------------------------
float activity     = 0;
int   lastMotionMs = 0, periodStartMs = 0, activeMs = 0, lastUpdateMs = 0;
boolean moving     = false;
float impactG      = 0, strikePeakG = 0;
int   strikeWindowEndMs = 0;
final float IMPACT_FULL = 3.0;   // g at which the impact meter reads full
final float LED_FULL    = 1200;  // force at which an LED is full brightness (lower = brighter)

// ----- logging -----------------------------------------------------------
PrintWriter logFile;
boolean logging = false;

// ----- foot geometry (panel-local, normalized 0..1) ----------------------
// Right-foot layout matching Fig.4.  (x: 0=lateral edge, 1=medial edge)
float[] SX = { 0.64, 0.34, 0.66, 0.50 };   // MF, LF, MM, Heel
float[] SY = { 0.26, 0.34, 0.55, 0.84 };
PVector[] footOutline;                      // built in setup()

// ----- fonts -------------------------------------------------------------
PFont fBig, fMed, fSmall, fMono;

// =========================================================================
void settings() { size(1280, 800); smooth(4); }

void setup() {
  surface.setTitle("ai-ui · Smart-Shoe Gait Analysis");
  fBig   = createFont("Segoe UI Semibold", 30);
  fMed   = createFont("Segoe UI", 16);
  fSmall = createFont("Segoe UI", 12);
  fMono  = createFont("Consolas", 13);
  buildFootOutline();
  setupSerial();
  startCalibration();
  lastUpdateMs = millis();
}

// -------------------------------------------------------------------------
void setupSerial() {
  println("Serial ports:");
  printArray(Serial.list());
  String[] ports = Serial.list();

  // Use PORT_NAME if it's actually plugged in; otherwise fall back to the
  // last port (an Arduino is usually the most-recently-added one). This is
  // why calibration/logging "did nothing" before: COM7 wasn't the real port,
  // the open failed silently, and no serial data ever arrived.
  String chosen = null;
  for (String p : ports) if (p.equalsIgnoreCase(PORT_NAME)) chosen = p;
  if (chosen == null && ports.length > 0) chosen = ports[ports.length - 1];

  if (chosen == null) {
    println("!! No serial ports found — plug in the Arduino and restart.");
    return;
  }
  try {
    port = new Serial(this, chosen, BAUD);
    port.bufferUntil('\n');
    PORT_NAME = chosen;                       // so the footer shows the real port
    println("Opened serial port: " + chosen);
  } catch (Exception e) {
    println("!! Could not open " + chosen + " — close the Serial Monitor / set PORT_NAME. (" + e.getMessage() + ")");
  }
}

void startCalibration() {
  calibrating = true;
  calibStartMs = millis();
  calibCount = 0;
  for (int i = 0; i < NS; i++) calibSum[i] = 0;
}

// =========================================================================
//  DRAW
// =========================================================================
void draw() {
  drawBackdrop();
  drawHeader();
  drawFootPanel(40, 120, 420, 620);
  drawGraphsPanel(500, 120, 470, 620);
  drawStatsPanel(1000, 120, 240, 620);
  drawFooter();
}

void drawBackdrop() {
  background(0x12, 0x16, 0x21);
  // subtle top glow
  noStroke();
  for (int i = 0; i < 90; i++) {
    fill(0x1B, 0x22, 0x33, 3);
    rect(0, 0, width, i * 2);
  }
}

void drawHeader() {
  textFont(fBig); fill(255);
  textAlign(LEFT, CENTER);
  text("Gait Analysis", 40, 46);
  textFont(fSmall); fill(0x7E, 0x8A, 0xA0);
  text("Smart-Shoe Insole  ·  BME/CS 479 Lab 4", 42, 78);

  // status pill (In Motion / Standing Still)
  float px = width - 300, py = 30, pw = 260, ph = 54;
  color pc = moving ? color(0x2E, 0xCC, 0x71) : color(0x5B, 0x6B, 0x82);
  noStroke(); fill(red(pc), green(pc), blue(pc), 40);
  rrect(px, py, pw, ph, 14);
  fill(pc); ellipse(px + 28, py + ph/2, 16, 16);
  if (moving) { noFill(); stroke(pc); strokeWeight(2);
                ellipse(px + 28, py + ph/2, 28 + 6*sin(millis()/180.0), 28 + 6*sin(millis()/180.0)); }
  noStroke();
  textAlign(LEFT, CENTER); textFont(fMed); fill(255);
  text(moving ? "In Motion" : "Standing Still", px + 48, py + ph/2 - 8);
  textFont(fSmall); fill(0x9E, 0xAC, 0xC4);
  float period = moving ? (millis() - periodStartMs) / 1000.0 : 0;
  text("active now " + nf(period,0,1) + "s  ·  total " + nf(activeMs/1000.0,0,1) + "s",
       px + 48, py + ph/2 + 11);

  if (calibrating) {
    textAlign(RIGHT, CENTER); textFont(fSmall); fill(0xFF, 0xB3, 0x4D);
    text("● calibrating baseline — keep foot still", width - 320, 56);
  }
}

// -------------------------------------------------------------------------
//  FOOT PANEL  — live heat-map + bubble heat-map markers
// -------------------------------------------------------------------------
void drawFootPanel(float x, float y, float w, float h) {
  panel(x, y, w, h, "Foot Pressure Heat-map");

  float fx = x + 40, fy = y + 60, fw = w - 80, fh = h - 110;

  // 1) painted heat-map: inverse-distance interpolation of the 4 forces,
  //    rendered as a coarse grid clipped to the foot outline.
  int step = 9;
  noStroke();
  for (float gx = fx; gx < fx + fw; gx += step) {
    for (float gy = fy; gy < fy + fh; gy += step) {
      float nx = (gx - fx) / fw, ny = (gy - fy) / fh;
      if (!inFoot(nx, ny)) continue;
      float v = heatAt(nx, ny);
      color c = heatColor(v);
      fill(red(c), green(c), blue(c), 190);
      rect(gx, gy, step, step);
    }
  }

  // 2) foot outline on top
  noFill(); stroke(0x3A, 0x45, 0x59); strokeWeight(2);
  beginShape();
  for (PVector p : footOutline) vertex(fx + p.x * fw, fy + p.y * fh);
  endShape(CLOSE);

  // 3) bubble heat-map markers (grow + recolor with force)
  textAlign(CENTER, CENTER);
  for (int i = 0; i < NS; i++) {
    float cx = fx + SX[i] * fw, cy = fy + SY[i] * fh;
    float f  = constrain(fsr[i] / YMAX, 0, 1);
    float d  = 26 + f * 60;
    color c  = heatColor(f);
    noStroke(); fill(red(c), green(c), blue(c), 70); ellipse(cx, cy, d + 18, d + 18);
    fill(c); ellipse(cx, cy, d, d);
    fill(0x10, 0x14, 0x1E); textFont(fSmall);
    text(NAME[i], cx, cy - 3);
    text(int(fsr[i]), cx, cy + 11);
  }
  textAlign(LEFT, BASELINE);
}

// inverse-distance-weighted force at a normalized foot point
float heatAt(float nx, float ny) {
  float num = 0, den = 0;
  for (int i = 0; i < NS; i++) {
    float dx = nx - SX[i], dy = ny - SY[i];
    float d2 = dx*dx + dy*dy + 0.002;
    float wgt = 1.0 / (d2 * d2);
    num += (fsr[i] / YMAX) * wgt;
    den += wgt;
  }
  return constrain(num / den, 0, 1);
}

// blue -> cyan -> green -> yellow -> red
color heatColor(float t) {
  t = constrain(t, 0, 1);
  float[] stops = { 0.0, 0.25, 0.5, 0.75, 1.0 };
  color[] cols  = { color(0x10,0x2A,0x5E), color(0x1E,0x88,0xE5),
                    color(0x2E,0xCC,0x71), color(0xF1,0xC4,0x0F),
                    color(0xE7,0x4C,0x3C) };
  for (int i = 0; i < stops.length - 1; i++) {
    if (t <= stops[i+1]) {
      float u = map(t, stops[i], stops[i+1], 0, 1);
      return lerpColor(cols[i], cols[i+1], u);
    }
  }
  return cols[cols.length - 1];
}

// -------------------------------------------------------------------------
//  GRAPHS PANEL — numerical FSR line graphs, labeled axes
// -------------------------------------------------------------------------
void drawGraphsPanel(float x, float y, float w, float h) {
  panel(x, y, w, h, "Sensor Readings (last " + N + " samples)");
  float gh = (h - 70) / NS;
  for (int s = 0; s < NS; s++) {
    drawGraph(s, x + 56, y + 50 + s * gh, w - 86, gh - 18);
  }
}

void drawGraph(int s, float x, float y, float w, float h) {
  // frame
  noFill(); stroke(0x27, 0x30, 0x42); strokeWeight(1);
  rect(x, y, w, h);
  // y axis labels
  fill(0x7E, 0x8A, 0xA0); textFont(fSmall); textAlign(RIGHT, CENTER);
  text(YMAX, x - 6, y + 2);
  text(0,    x - 6, y + h - 2);
  // title + current value
  textAlign(LEFT, BASELINE); fill(TINT[s]);
  text(NAME[s] + " — " + LONG[s], x + 4, y - 4);
  textAlign(RIGHT, BASELINE); fill(0xC8, 0xD2, 0xE0);
  text(int(fsr[s]), x + w - 4, y - 4);
  // x axis label (only on the last graph)
  if (s == NS - 1) {
    textAlign(CENTER, TOP); fill(0x7E, 0x8A, 0xA0);
    text("time  →", x + w/2, y + h + 4);
  }
  // trace
  textAlign(LEFT, BASELINE);
  noFill(); stroke(TINT[s]); strokeWeight(1.6);
  beginShape();
  for (int i = 0; i < N; i++) {
    float vx = x + map(i, 0, N - 1, 0, w);
    float vy = y + h - map(constrain(history[s][i], 0, YMAX), 0, YMAX, 0, h);
    vertex(vx, vy);
  }
  endShape();
}

// -------------------------------------------------------------------------
//  STATS PANEL
// -------------------------------------------------------------------------
void drawStatsPanel(float x, float y, float w, float h) {
  panel(x, y, w, h, "Analysis");

  int steps = strideCount * 2;
  float cadence = currentCadence();
  int stepLen = stepLenStr.length() > 0 ? int(stepLenStr) : 0;
  float speed = stepLen * cadence / 6000.0;   // cm/step * steps/min -> m/s

  float cy = y + 54;

  // Gait badge
  noStroke(); fill(red(gaitColor), green(gaitColor), blue(gaitColor), 45);
  rrect(x + 16, cy, w - 32, 70, 12);
  fill(0x7E, 0x8A, 0xA0); textFont(fSmall); textAlign(LEFT, TOP);
  text("DETECTED GAIT", x + 30, cy + 12);
  fill(gaitColor); textFont(fMed); textAlign(LEFT, TOP);
  text(gait, x + 30, cy + 30);
  fill(0x9E, 0xAC, 0xC4); textFont(fSmall); textAlign(RIGHT, TOP);
  text("MFP " + nf(mfp,0,1) + "%", x + w - 30, cy + 34);
  cy += 84;

  fill(0x20, 0x28, 0x38); noStroke();
  rrect(x + 16, cy, w - 32, 92, 10);
  fill(0x7E, 0x8A, 0xA0); textFont(fSmall); textAlign(LEFT, TOP);
  text("ACTIVITY", x + 28, cy + 9);
  fill(0xFF, 0xD5, 0x4F); textFont(fMed);
  text(activityType(cadence), x + 28, cy + 27);
  fill(0x7E, 0x8A, 0xA0); textFont(fSmall); textAlign(RIGHT, TOP);
  text("IMPACT METER", x + w - 28, cy + 9);
  color ic = impactColor(impactG);
  fill(ic); textFont(fMed);
  text(nf(impactG, 0, 2) + " g", x + w - 28, cy + 27);
  // horizontal foot-strike meter: 0 .. IMPACT_FULL g
  float mx = x + 28, mw = w - 56, my = cy + 60, mh = 12;
  noStroke(); fill(0x12, 0x16, 0x21); rrect(mx, my, mw, mh, 6);
  float frac = constrain(impactG / IMPACT_FULL, 0, 1);
  fill(ic); rrect(mx, my, max(mh, mw * frac), mh, 6);
  fill(0x5B, 0x6B, 0x82); textFont(fSmall); textAlign(LEFT, TOP);
  text("soft", mx, my + 15);
  textAlign(RIGHT, TOP); text("hard", mx + mw, my + 15);
  cy += 106;

  cy = stat(x, cy, w, "Step count", str(steps), "");
  cy = cadenceStat(x, cy, w, cadence);
  cy = stat(x, cy, w, "Step length",    stepLenStr,                 "cm  (type to edit)");
  cy = stat(x, cy, w, "Stride length",  str(stepLen * 2),           "cm");
  cy = stat(x, cy, w, "Walking speed",  nf(speed,0,2),              "m/s");
  cy = stat(x, cy, w, "Step gap",       str(stepGap),               "ms");

  // keys hint
  fill(0x5B, 0x6B, 0x82); textFont(fSmall); textAlign(LEFT, TOP);
  text("C recalibrate   K rhythm   R reset", x + 24, y + h - 56);
  text("L " + (logging ? "stop log" : "start log"), x + 24, y + h - 38);
  if (logging) { fill(0xE7,0x4C,0x3C); text("● REC", x + w - 70, y + h - 38); }
}

float stat(float x, float cy, float w, String label, String val, String unit) {
  fill(0x7E, 0x8A, 0xA0); textFont(fSmall); textAlign(LEFT, TOP);
  text(label, x + 30, cy);
  fill(255); textFont(fBig); textAlign(LEFT, TOP);
  text(val, x + 30, cy + 13);
  float vw = textWidth(val);
  fill(0x7E, 0x8A, 0xA0); textFont(fSmall);
  text(unit, x + 36 + vw, cy + 32);
  stroke(0x23, 0x2B, 0x3B); strokeWeight(1);
  line(x + 24, cy + 51, x + w - 24, cy + 51); noStroke();
  return cy + 54;
}

float cadenceStat(float x, float cy, float w, float cadence) {
  fill(0x7E, 0x8A, 0xA0); textFont(fSmall); textAlign(LEFT, TOP);
  text("Cadence (target " + TARGET_CADENCE_MIN + "-" + TARGET_CADENCE_MAX + ")", x + 30, cy);
  color targetColor = cadenceColor(cadence);
  fill(targetColor); textFont(fBig); textAlign(LEFT, TOP);
  String value = nf(cadence, 0, 1);
  text(value, x + 30, cy + 15);
  fill(0x7E, 0x8A, 0xA0); textFont(fSmall);
  text("steps/min", x + 36 + textWidth(value), cy + 34);
  textAlign(LEFT, TOP); fill(targetColor);
  text(cadenceStatus(cadence), x + 30, cy + 47);
  stroke(0x23, 0x2B, 0x3B); strokeWeight(1);
  line(x + 24, cy + 60, x + w - 24, cy + 60); noStroke();
  return cy + 62;
}

String activityType(float cadence) {
  if (!moving) return "Waiting";
  if (cadence <= 0) return "Detecting";
  if (cadence < 120) return "Walking";
  if (cadence < TARGET_CADENCE_MIN) return "Jogging";
  return "Running";
}

String cadenceStatus(float cadence) {
  if (cadence <= 0) return "Waiting for steps";
  if (cadence < TARGET_CADENCE_MIN) return "Too slow";
  if (cadence <= TARGET_CADENCE_MAX) return "On target";
  return "Too fast";
}

color cadenceColor(float cadence) {
  if (cadence <= 0) return color(0x9E, 0xAC, 0xC4);
  if (cadence < TARGET_CADENCE_MIN || cadence > TARGET_CADENCE_MAX) return color(0xE7, 0x4C, 0x3C);
  return color(0x2E, 0xCC, 0x71);
}

// green (soft) -> amber -> red (hard) by foot-strike intensity
color impactColor(float g) {
  float t = constrain(g / IMPACT_FULL, 0, 1);
  if (t < 0.5) return lerpColor(color(0x2E,0xCC,0x71), color(0xF1,0xC4,0x0F), t / 0.5);
  return lerpColor(color(0xF1,0xC4,0x0F), color(0xE7,0x4C,0x3C), (t - 0.5) / 0.5);
}

float currentCadence() {
  if (strideCount < 2 || previousStrideMs <= 0 || lastStrideMs <= previousStrideMs) return 0;
  if (millis() - lastStrideMs > 2500) return 0;
  return 120000.0 / (lastStrideMs - previousStrideMs);
}

void drawFooter() {
  fill(0x5B, 0x6B, 0x82); textFont(fSmall); textAlign(LEFT, TOP);
  text("Port " + PORT_NAME + " @ " + BAUD + " baud", 40, height - 28);
  textAlign(RIGHT, TOP);
  text("Hot colors = more force", width - 40, height - 28);
  textAlign(LEFT, BASELINE);
}

// ----- UI helpers --------------------------------------------------------
void panel(float x, float y, float w, float h, String title) {
  noStroke(); fill(0x1A, 0x20, 0x2E); rrect(x, y, w, h, 16);
  stroke(0x25, 0x2E, 0x40); strokeWeight(1); noFill(); rrect(x, y, w, h, 16);
  noStroke(); fill(0xC8, 0xD2, 0xE0); textFont(fMed); textAlign(LEFT, TOP);
  text(title, x + 20, y + 16);
  stroke(0x25, 0x2E, 0x40); line(x + 20, y + 42, x + w - 20, y + 42); noStroke();
}
void rrect(float x, float y, float w, float h, float r) { rect(x, y, w, h, r); }

// =========================================================================
//  SERIAL EVENT  — parse the Arduino stream
// =========================================================================
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;
  line = trim(line);
  String[] t = splitTokens(line, ", ");
  if (t.length < 7) return;

  for (int i = 0; i < NS; i++) raw[i] = float(t[i]);

  // baseline calibration: average the resting values per sensor
  if (calibrating) {
    for (int i = 0; i < NS; i++) calibSum[i] += raw[i];
    calibCount++;
    if (millis() - calibStartMs >= CALIB_MS && calibCount > 0) {
      for (int i = 0; i < NS; i++) baseline[i] = calibSum[i] / calibCount;
      calibrating = false;
    }
    return;
  }

  // subtract baseline, apply deadband -> clean force
  for (int i = 0; i < NS; i++) {
    float v = raw[i] - baseline[i];
    fsr[i] = v > DEADBAND ? v : 0;
  }

  onSample(float(t[4]), float(t[5]), float(t[6]));
  sendLeds();
  logRow();
}

// Map each FSR force to an LED brightness (0-255) and send it to the Arduino.
// Uses LED_FULL as the full-brightness point (lower = brighter), matching the
// behaviour of the working non-AI sketch.
void sendLeds() {
  if (port == null) return;
  String msg = "L";
  for (int i = 0; i < NS; i++)
    msg += "," + int(map(constrain(fsr[i], 0, LED_FULL), 0, LED_FULL, 0, 255));
  port.write(msg + "\n");
}

// =========================================================================
//  SAMPLE PROCESSING  — stride detection, gait, motion
// =========================================================================
void onSample(float ax, float ay, float az) {
  int now = millis();
  updateRhythmCalib();
  float accelMagnitudeG = sqrt(ax*ax + ay*ay + az*az) / 16384.0;
  float dynamicG = abs(accelMagnitudeG - 1.0);

  if (strikeWindowEndMs > 0) {
    if (now <= strikeWindowEndMs) strikePeakG = max(strikePeakG, dynamicG);
    else {
      impactG = strikePeakG;
      strikeWindowEndMs = 0;
    }
  }

  float total = 0;
  for (int i = 0; i < NS; i++) {
    force[i] = fsr[i] > THRESHOLD ? fsr[i] : 0;
    total += force[i];
    arrayCopy(history[i], 1, history[i], 0, N - 1);
    history[i][N - 1] = fsr[i];
  }

  // footfall detection
  if (!onGround && total > 0 && now - lastStrideMs > stepGap) {
    onGround = true;
    if (strideCount == 0) firstStrideMs = now;
    previousStrideMs = lastStrideMs;
    strideCount++;
    onFootfall(now);
    lastStrideMs = now;
    strikePeakG = dynamicG;
    strikeWindowEndMs = now + 150;
    stanceSum = new float[NS];
  } else if (onGround && total == 0) {
    onGround = false;
    findGait();
  }
  if (onGround) for (int i = 0; i < NS; i++) stanceSum[i] += force[i];

  // motion / activity from accelerometer magnitude
  activity = 0.9 * activity + 0.1 * dynamicG;
  if (activity > 0.08) lastMotionMs = now;
  boolean nowMoving = now - lastMotionMs < 1000;
  if (nowMoving && !moving) periodStartMs = now;
  if (moving) activeMs += now - lastUpdateMs;
  lastUpdateMs = now;
  moving = nowMoving;
}

// Medial Force Percentage per the lab equation + heel share -> 5 gaits
void findGait() {
  float all = stanceSum[0] + stanceSum[1] + stanceSum[2] + stanceSum[3] + 0.001;
  mfp = (stanceSum[2] + stanceSum[0]) * 100 / all;   // (MM + MF)
  float heelPct = stanceSum[3] * 100 / all;

  if      (heelPct < 10) { gait = "Tip-toeing";   gaitColor = color(0xFF,0x6B,0x6B); }
  else if (heelPct > 60) { gait = "Heel walking"; gaitColor = color(0xFF,0xD5,0x4F); }
  else if (mfp > 60)     { gait = "Out-toeing";   gaitColor = color(0x4F,0xC3,0xF7); }
  else if (mfp < 40)     { gait = "In-toeing";    gaitColor = color(0xBA,0x68,0xC8); }
  else                   { gait = "Normal gait";  gaitColor = color(0x2E,0xCC,0x71); }
}

// ----- rhythm calibration ------------------------------------------------
void startRhythmCalib() { rhythmCalib = true; rhythmStartMs = millis(); footCount = 0; }
void onFootfall(int now) {
  if (!rhythmCalib) return;
  if (footCount == 0) firstFootMs = now;
  lastFootMs = now; footCount++;
}
void updateRhythmCalib() {
  if (!rhythmCalib || millis() - rhythmStartMs < 10000) return;
  if (footCount >= 3) stepGap = int((lastFootMs - firstFootMs) / (footCount - 1) * 0.5);
  rhythmCalib = false;
}

// ----- logging -----------------------------------------------------------
void toggleLog() {
  if (!logging) {
    logFile = createWriter("profile_" + (millis() / 1000) + ".csv");
    logFile.println("time_ms,MF,LF,MM,Heel,gait,mfp,activity,cadence,impact_g");
    logging = true;
  } else {
    logFile.flush(); logFile.close(); logging = false;
  }
}
void logRow() {
  if (!logging) return;
  float cadence = currentCadence();
  logFile.println(millis() + "," + int(fsr[0]) + "," + int(fsr[1]) + "," +
                  int(fsr[2]) + "," + int(fsr[3]) + "," + gait + "," + nf(mfp,0,1) + "," +
                  activityType(cadence) + "," + nf(cadence,0,1) + "," + nf(impactG,0,2));
}

// ----- keys --------------------------------------------------------------
void keyPressed() {
  if (key == 'c' || key == 'C') startCalibration();
  if (key == 'k' || key == 'K') startRhythmCalib();
  if (key == 'l' || key == 'L') toggleLog();
  if (key == 'r' || key == 'R') {
    strideCount = 0;
    firstStrideMs = lastStrideMs = previousStrideMs = 0;
  }
  if (key >= '0' && key <= '9' && stepLenStr.length() < 3) stepLenStr += key;
  if (key == BACKSPACE && stepLenStr.length() > 0)
    stepLenStr = stepLenStr.substring(0, stepLenStr.length() - 1);
}

// ----- foot outline (normalized, right foot) -----------------------------
void buildFootOutline() {
  float[][] pts = {
    {0.50,0.03},{0.66,0.05},{0.78,0.14},{0.82,0.28},{0.80,0.40},
    {0.74,0.52},{0.72,0.64},{0.74,0.76},{0.70,0.88},{0.58,0.96},
    {0.44,0.97},{0.31,0.92},{0.24,0.80},{0.25,0.66},{0.28,0.54},
    {0.24,0.42},{0.20,0.30},{0.24,0.16},{0.36,0.06}
  };
  footOutline = new PVector[pts.length];
  for (int i = 0; i < pts.length; i++) footOutline[i] = new PVector(pts[i][0], pts[i][1]);
}

boolean inFoot(float nx, float ny) {
  boolean in = false;
  int n = footOutline.length;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    PVector a = footOutline[i], b = footOutline[j];
    if (((a.y > ny) != (b.y > ny)) &&
        (nx < (b.x - a.x) * (ny - a.y) / (b.y - a.y) + a.x)) in = !in;
  }
  return in;
}
