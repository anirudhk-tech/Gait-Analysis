
PrintWriter logFile;
boolean logging = false;

void toggleLog() {
    if (!logging) {
        logFile = createWriter("profile_" + (millis() / 1000) + ".csv");
        logFile.println("time_ms,MF,LF,MM,Heel,gait,mfp,activity,cadence,impact_g");
        logging = true;
    } else {
        logFile.flush();
        logFile.close();
        logging = false;
    }
}

void logRow() {
    if (!logging) return;
    float cadence = currentCadence();
    logFile.println(millis() + "," + int(fsr[0]) + "," + int(fsr[1]) + "," + int(fsr[2]) + "," + int(fsr[3]) + "," + gait + "," + nf(mfp, 0, 1) + "," + activityType(cadence) + "," + nf(cadence, 0, 1) + "," + nf(impactG, 0, 2));
}
