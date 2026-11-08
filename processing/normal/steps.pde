
int stepGap = 300;
boolean rhythmCalib = false;
int rhythmStartMs = 0;
int firstFootMs = 0;
int lastFootMs = 0;
int footCount = 0;

void startRhythmCalib() {
    rhythmCalib = true;
    rhythmStartMs = millis();
    footCount = 0;
}

// Call once per detected footfall.
void onFootfall(int now) {
    if (!rhythmCalib) return;
    if (footCount == 0) firstFootMs = now;
    lastFootMs = now;
    footCount++;
}

// Call once per sample; ends calibration after 10 s and sets stepGap.
void updateRhythmCalib() {
    if (!rhythmCalib || millis() - rhythmStartMs < 10000) return;
    if (footCount >= 3) stepGap = int((lastFootMs - firstFootMs) / (footCount - 1) * 0.5);
    rhythmCalib = false;
}
