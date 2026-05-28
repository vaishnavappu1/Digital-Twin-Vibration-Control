#include <Wire.h>
#include <MPU6050.h>
#include <arduinoFFT.h>

MPU6050 mpu;
ArduinoFFT<double> FFT;

const int MOTOR_A_PWM = 25;
const int MOTOR_B_PIN = 32;
const int CURRENT_PIN = 34;
const int IR_PIN = 27;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;

float sensitivity = 0.100;
float zeroOffset = 0.0;

float VIB_SLOPE = 0.00075;
float VIB_OFFSET = -0.014;
float RPM_SLOPE = 0.807;
float RPM_OFFSET = -12.5;
float I_SLOPE = 0.0;
float I_OFFSET = 0.0;

float VIB_ERROR_THRESHOLD = 0.15;
float VIB_STD_THRESHOLD = 0.042;
float RESONANCE_THRESHOLD = 0.285;
float RPM_DROP_THRESHOLD = 0.45;
float CURRENT_DEV_THRESHOLD = 1.5;

int RESONANCE_ZONE_LOW = 181;
int RESONANCE_ZONE_HIGH = 210;

// ==============================
// FFT SETTINGS
// 128 samples at 10ms = 100Hz
// Resolution = 0.78Hz per bin
// Only runs during commissioning
// ==============================
const int FFT_SAMPLES = 128;
const float SAMPLING_FREQ = 100.0;
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
float resonanceFreqHz = 0.0;

const int SAMPLE_COUNT = 50;
float currentSamples[SAMPLE_COUNT];
float vibSamples[SAMPLE_COUNT];

// ==============================
// IR / RPM
// ==============================
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

float motorRPM = 0;
float prevRPM = 0;
float filteredRPM = 0;
bool rpmValid = false;

unsigned long irNoSignalSince = 0;
bool irFaultDetected = false;
bool irFaultPrinted = false;
const unsigned long IR_FAULT_TIMEOUT = 5000;

void IRAM_ATTR irPulseISR() {
unsigned long now = micros();
unsigned long interval = now - lastPulseTime;

if (interval < 20000) return;
if (interval > 5000000) {
lastPulseTime = now;
return;
}

unsigned long minInterval = 200000;
if (pulseInterval > 0) {
unsigned long adaptive = pulseInterval / 2;
if (adaptive > minInterval)
minInterval = adaptive;
}

if (interval > minInterval) {
pulseInterval = interval;
lastPulseTime = now;
}
}

int pwmValue = 0;
float vibrationRMS = 0;
float vibStd = 0;
float gravityBaseline = 1.0;
float currentMean = 0;
float currentMeanSmooth = 0;
bool firstCurrentRead = true;

// ==============================
// MOTOR B ADAPTIVE SWEEP
// ==============================
bool motorBActive = false;
float vibBeforeControl = 0.0;
float vibDuringControl = 0.0;
float reductionPercent = 0.0;

int sweepPWMLow = 80;
int sweepPWMHigh = 160;
int sweepPWMLowDefault = 80;
int sweepPWMHighDefault = 160;
int sweepPWMCurr = 80;
int sweepDir = 1;
bool sweepMode = true;
int lockedPWM = 100;

bool bandNarrowed = false;
bool bandResetDone = false;
unsigned long bandNarrowedTime = 0;

int badPhaseCount = 0;
const int BAD_PHASE_PERSIST = 3;

float minVibDuringSweep = 999.0;
int bestSweepPWM = 100;
float vibAvgWindow[3] = {0, 0, 0};
int vibAvgIdx = 0;
float vibAvgSum = 0.0;
float vibAvgSmooth = 0.0;

unsigned long lastSweepTime = 0;
const unsigned long SWEEP_STEP_MS = 100;
unsigned long motorBStartTime = 0;
const unsigned long MOTORB_TIMEOUT = 120000;

float totalReductionSum = 0.0;
int reductionSamples = 0;
float maxReductionSeen = 0.0;
float minVibSeen = 999.0;

bool zoneMode = false;
bool resonanceFaultActive = false;

int mechFaultCount = 0;
const int MECH_FAULT_PERSIST = 3;

unsigned long faultFirstSeen = 0;
const unsigned long FAULT_PERSIST = 1500;
bool persistMessageShown = false;

const int MA_SIZE = 5;
float maBuffer[MA_SIZE];
int maIndex = 0;
float maSum = 0.0;

const int SOFT_START_STEP = 5;
const int SOFT_START_DELAY = 50;

bool reportHoldMode = false;

// ==============================
// MOVING AVERAGE
// ==============================
void initMovingAverage(float initValue) {
for (int i = 0; i < MA_SIZE; i++)
maBuffer[i] = initValue;
maSum = initValue * MA_SIZE;
maIndex = 0;
}

float movingAverage(float newValue) {
maSum -= maBuffer[maIndex];
maBuffer[maIndex] = newValue;
maSum += newValue;
maIndex = (maIndex + 1) % MA_SIZE;
return maSum / MA_SIZE;
}

// ==============================
// RPM UPDATE
// ==============================
void updateRPM() {
if (pulseInterval == 0) {
filteredRPM = 0;
prevRPM = 0;
rpmValid = false;
return;
}

if ((micros() - lastPulseTime) < 3000000) {
motorRPM = 60000000.0 / pulseInterval;
motorRPM = constrain(motorRPM, 0, 500);

if (motorRPM > 1000) {
filteredRPM = prevRPM;
} else if (motorRPM > prevRPM * 1.7 &&
motorRPM < prevRPM * 2.3 &&
prevRPM > 0) {
filteredRPM = prevRPM;
} else {
filteredRPM = motorRPM;
prevRPM = filteredRPM;
}
} else {
filteredRPM = 0;
prevRPM = 0;
}

rpmValid = (filteredRPM > 20);
}

// ==============================
// RUN FFT
// Called only during commissioning
// 128 samples at 10ms = 100Hz
// Resolution = 0.78Hz per bin
// ==============================
float runFFT(int atPWM) {
Serial.println("Collecting FFT samples...");
Serial.println("(1.3 seconds)");

for (int i = 0; i < FFT_SAMPLES; i++) {
int16_t ax, ay, az;
mpu.getAcceleration(&ax, &ay, &az);
float mag = sqrt((float)ax*ax +
(float)ay*ay +
(float)az*az) / 16384.0;
vReal[i] = (double)abs(mag - gravityBaseline);
vImag[i] = 0.0;
delay(10);
}

FFT.windowing(vReal, FFT_SAMPLES,
FFT_WIN_TYP_HAMMING,
FFT_FORWARD);

FFT.compute(vReal, vImag,
FFT_SAMPLES,
FFT_FORWARD);

FFT.complexToMagnitude(vReal, vImag,
FFT_SAMPLES);

double maxMag = 0;
int maxIdx = 1;
for (int i = 1; i < FFT_SAMPLES / 2; i++) {
if (vReal[i] > maxMag) {
maxMag = vReal[i];
maxIdx = i;
}
}

float dominantFreq = maxIdx * (SAMPLING_FREQ / FFT_SAMPLES);
return dominantFreq;
}

// ==============================
// SOFT START MOTOR B
// ==============================
void softStartMotorB(int targetPWM) {
Serial.println("Motor B starting...");
for (int p = 0; p <= targetPWM; p += SOFT_START_STEP) {
ledcWrite(MOTOR_B_PIN, p);
delay(SOFT_START_DELAY);
}
ledcWrite(MOTOR_B_PIN, targetPWM);
}

// ==============================
// PRINT SESSION REPORT
// ==============================
void printSessionReport() {
Serial.println("=============================");
Serial.println(" SESSION REPORT ");
Serial.println("=============================");
Serial.print("Baseline Vib: ");
Serial.println(vibBeforeControl, 3);
Serial.print("Min Vib achieved: ");
Serial.println(minVibSeen, 3);

if (vibBeforeControl > 0 && minVibSeen < 999.0) {
float bestRed = ((vibBeforeControl - minVibSeen)
/ vibBeforeControl) * 100.0;
Serial.print("Best reduction: ");
Serial.print(bestRed, 1);
Serial.println("%");
}

if (reductionSamples > 0) {
float avgRed = totalReductionSum / reductionSamples;
Serial.print("Avg reduction: ");
Serial.print(avgRed, 1);
Serial.println("%");
}

Serial.print("Max reduction: ");
Serial.print(maxReductionSeen, 1);
Serial.println("%");
Serial.print("Optimal PWM: ");
Serial.println(lockedPWM);

if (resonanceFreqHz > 0) {
Serial.print("Resonance Freq: ");
Serial.print(resonanceFreqHz, 2);
Serial.print("Hz = ");
Serial.print(resonanceFreqHz * 60.0, 1);
Serial.println("RPM");
}

if (bandNarrowed) {
Serial.println("Band narrowing: YES");
Serial.print("Narrowed around: PWM ");
Serial.println(bestSweepPWM);
} else {
Serial.println("Band narrowing: NOT reached");
}

Serial.println("=============================");
Serial.println(">> Press Enter to continue");
Serial.println("=============================");
}

// ==============================
// STOP MOTOR B
// ==============================
void stopMotorB() {
int curr = sweepMode ? sweepPWMCurr : lockedPWM;
curr = constrain(curr, 0, 180);
for (int p = curr; p >= 0; p -= 5) {
ledcWrite(MOTOR_B_PIN, p);
delay(30);
}
ledcWrite(MOTOR_B_PIN, 0);

motorBActive = false;
sweepMode = true;

printSessionReport();
reportHoldMode = true;

sweepPWMLow = sweepPWMLowDefault;
sweepPWMHigh = sweepPWMHighDefault;
sweepPWMCurr = sweepPWMLow;
sweepDir = 1;
minVibDuringSweep = 999.0;
bestSweepPWM = sweepPWMLow;
reductionPercent = 0.0;
vibAvgSum = 0.0;
vibAvgIdx = 0;
vibAvgSmooth = 0.0;
totalReductionSum = 0.0;
reductionSamples = 0;
maxReductionSeen = 0.0;
minVibSeen = 999.0;
bandNarrowed = false;
bandResetDone = false;
bandNarrowedTime = 0;
badPhaseCount = 0;
for (int i = 0; i < 3; i++)
vibAvgWindow[i] = 0;
}

// ==============================
// START MOTOR B
// ==============================
void startMotorB() {
if (motorBActive) {
Serial.println("Motor B already running");
return;
}

vibBeforeControl = vibrationRMS;
motorBStartTime = millis();
sweepMode = true;
sweepPWMLow = sweepPWMLowDefault;
sweepPWMHigh = sweepPWMHighDefault;
sweepPWMCurr = sweepPWMLow;
sweepDir = 1;
minVibDuringSweep = vibrationRMS;
bestSweepPWM = sweepPWMLow;
lastSweepTime = millis();
vibAvgSum = vibrationRMS * 3;
vibAvgIdx = 0;
vibAvgSmooth = vibrationRMS;
totalReductionSum = 0.0;
reductionSamples = 0;
maxReductionSeen = 0.0;
minVibSeen = vibrationRMS;
bandNarrowed = false;
bandResetDone = false;
bandNarrowedTime = 0;
badPhaseCount = 0;
for (int i = 0; i < 3; i++)
vibAvgWindow[i] = vibrationRMS;

initMovingAverage(vibrationRMS);
softStartMotorB(sweepPWMLow);
motorBActive = true;

Serial.println("=============================");
Serial.println("MOTOR B ADAPTIVE SWEEP");
Serial.print("Baseline: ");
Serial.println(vibBeforeControl, 3);
Serial.print("Sweep: ");
Serial.print(sweepPWMLow);
Serial.print(" to ");
Serial.println(sweepPWMHigh);
Serial.println("S=Stop L=Lock best now");
Serial.println("=============================");
}

// ==============================
// COMPUTE ADAPTIVE SWEEP
// ==============================
void computeSweep() {
if (!motorBActive) return;

if (millis() - motorBStartTime > MOTORB_TIMEOUT) {
Serial.println("Timeout - stopping");
stopMotorB();
return;
}

float smoothedVib = movingAverage(vibrationRMS);

vibAvgSum -= vibAvgWindow[vibAvgIdx];
vibAvgWindow[vibAvgIdx] = smoothedVib;
vibAvgSum += smoothedVib;
vibAvgIdx = (vibAvgIdx + 1) % 3;
vibAvgSmooth = vibAvgSum / 3.0;

if (vibrationRMS < minVibSeen)
minVibSeen = vibrationRMS;

unsigned long elapsed = millis() - motorBStartTime;

// Phase 2 narrow after 15s
if (elapsed > 15000 && !bandNarrowed) {
int newLow = max(sweepPWMLowDefault, bestSweepPWM - 20);
int newHigh = min(sweepPWMHighDefault, bestSweepPWM + 20);
if (newHigh > newLow + 8) {
sweepPWMLow = newLow;
sweepPWMHigh = newHigh;
minVibDuringSweep = vibrationRMS;
bandNarrowed = true;
bandNarrowedTime = millis();
Serial.println("NARROWING sweep");
Serial.print("Around PWM=");
Serial.println(bestSweepPWM);
Serial.print("Range: ");
Serial.print(sweepPWMLow);
Serial.print("-");
Serial.println(sweepPWMHigh);
}
}

// Fallback 1 bad phase after 20s
if (elapsed > 20000 && bandNarrowed && !bandResetDone) {
if (reductionPercent < -10.0) {
sweepPWMLow = sweepPWMLowDefault;
sweepPWMHigh = sweepPWMHighDefault;
minVibDuringSweep = vibrationRMS;
bestSweepPWM = sweepPWMLow;
bandNarrowed = false;
bandNarrowedTime = 0;
bandResetDone = true;
Serial.println("BAD PHASE - band reset");
}
}

// Fallback 2 weak phase after 10s in narrow band
if (!bandResetDone && bandNarrowed && bandNarrowedTime > 0) {
if (millis() - bandNarrowedTime > 10000) {
float avgRed = (reductionSamples > 0)
? totalReductionSum / reductionSamples
: 0.0;
if (avgRed < 8.0) {
sweepPWMLow = sweepPWMLowDefault;
sweepPWMHigh = sweepPWMHighDefault;
minVibDuringSweep = vibrationRMS;
bestSweepPWM = sweepPWMLow;
bandNarrowed = false;
bandNarrowedTime = 0;
bandResetDone = true;
Serial.print("WEAK PHASE AvgRed=");
Serial.print(avgRed, 1);
Serial.println("% resetting");
}
}
}

// Fallback 3 immediate escape
if (reductionPercent < -5.0) {
badPhaseCount++;
} else {
badPhaseCount = 0;
}

if (badPhaseCount >= BAD_PHASE_PERSIST) {
sweepPWMLow = sweepPWMLowDefault;
sweepPWMHigh = sweepPWMHighDefault;
minVibDuringSweep = vibrationRMS;
bestSweepPWM = sweepPWMLow;
bandNarrowed = false;
bandNarrowedTime = 0;
bandResetDone = true;
badPhaseCount = 0;
Serial.println("BAD PHASE ESCAPE");
}

// PWM damping in bad phase
if (reductionPercent < 0.0 && sweepMode) {
int dampedPWM = max(sweepPWMLow, sweepPWMCurr - 15);
ledcWrite(MOTOR_B_PIN, dampedPWM);
}

if (sweepMode) {
if (millis() - lastSweepTime >= SWEEP_STEP_MS) {
lastSweepTime = millis();

if (vibAvgSmooth < minVibDuringSweep) {
minVibDuringSweep = vibAvgSmooth;
bestSweepPWM = sweepPWMCurr;
}

sweepPWMCurr += sweepDir;

if (sweepPWMCurr >= sweepPWMHigh) {
sweepPWMCurr = sweepPWMHigh;
sweepDir = -1;
}
if (sweepPWMCurr <= sweepPWMLow) {
sweepPWMCurr = sweepPWMLow;
sweepDir = 1;
}

if (reductionPercent >= 0.0) {
ledcWrite(MOTOR_B_PIN, sweepPWMCurr);
}
}
} else {
ledcWrite(MOTOR_B_PIN, lockedPWM);
}

vibDuringControl = vibrationRMS;

if (vibBeforeControl > 0) {
reductionPercent = ((vibBeforeControl - vibDuringControl)
/ vibBeforeControl) * 100.0;
totalReductionSum += reductionPercent;
reductionSamples++;
if (reductionPercent > maxReductionSeen)
maxReductionSeen = reductionPercent;
}
}

// ==============================
// COLLECT SAMPLES
// ==============================
void collectSamples(float &outVibRMS,
float &outVibStd,
float &outCurrentMean) {

float vSamples[SAMPLE_COUNT];
float cSamples[SAMPLE_COUNT];

for (int i = 0; i < SAMPLE_COUNT; i++) {
float voltage = analogRead(CURRENT_PIN) * (3.3 / 4095.0);
cSamples[i] = abs((voltage - zeroOffset) / sensitivity);

int16_t ax, ay, az;
mpu.getAcceleration(&ax, &ay, &az);
float mag = sqrt((float)ax*ax +
(float)ay*ay +
(float)az*az) / 16384.0;
vSamples[i] = abs(mag - gravityBaseline);
delay(1);
}

float cSum = 0;
for (int i = 0; i < SAMPLE_COUNT; i++)
cSum += cSamples[i];
outCurrentMean = cSum / SAMPLE_COUNT;

float vSum = 0;
for (int i = 0; i < SAMPLE_COUNT; i++)
vSum += pow(vSamples[i], 2);
outVibRMS = sqrt(vSum / SAMPLE_COUNT);

float vMean = 0;
for (int i = 0; i < SAMPLE_COUNT; i++)
vMean += vSamples[i];
vMean /= SAMPLE_COUNT;

float vVar = 0;
for (int i = 0; i < SAMPLE_COUNT; i++)
vVar += pow(vSamples[i] - vMean, 2);
outVibStd = sqrt(vVar / SAMPLE_COUNT);
}

// ==============================
// AUTO COMMISSIONING
// ==============================
void autoCommission() {

Serial.println("=============================");
Serial.println("COMMISSIONING");
Serial.println("Screws tight, no extra mass");
Serial.println("Starting in 3 seconds...");
Serial.println("=============================");
delay(3000);

const int NUM_POINTS = 10;
int sweepPWM[NUM_POINTS] = {
80, 100, 115, 130, 140,
150, 155, 160, 165, 170
};

float sVib[NUM_POINTS];
float sVstd[NUM_POINTS];
float sRPM[NUM_POINTS];
float sCurr[NUM_POINTS];
float sCurrDev[NUM_POINTS];
float maxNormalVstd = 0;
bool failed = false;

Serial.println("Ramping up to PWM 80...");
for (int p = 0; p <= 80; p += 5) {
ledcWrite(MOTOR_A_PWM, p);
delay(100);
}
delay(2000);
updateRPM();

Serial.println("Sweeping PWM points...");

for (int i = 0; i < NUM_POINTS; i++) {
ledcWrite(MOTOR_A_PWM, sweepPWM[i]);
Serial.print("PWM="); Serial.println(sweepPWM[i]);
delay(3000);

for (int u = 0; u < 5; u++) {
updateRPM();
delay(100);
}

float vA=0, vsA=0, rA=0, cA=0;
float cR[3];

for (int r = 0; r < 3; r++) {
float sV=0, sVs=0, sC=0;
collectSamples(sV, sVs, sC);
updateRPM();
vA += sV;
vsA += sVs;
rA += filteredRPM;
cA += sC;
cR[r] = sC;
delay(200);
}

sVib[i] = vA / 3.0;
sVstd[i] = vsA / 3.0;
sRPM[i] = rA / 3.0;
sCurr[i] = cA / 3.0;

float dS = 0;
for (int r = 0; r < 3; r++)
dS += abs(cR[r] - sCurr[i]);
sCurrDev[i] = dS / 3.0;

if (sVstd[i] > maxNormalVstd)
maxNormalVstd = sVstd[i];

Serial.print(" V="); Serial.print(sVib[i], 4);
Serial.print(" Vs="); Serial.print(sVstd[i], 4);
Serial.print(" R="); Serial.print(sRPM[i], 1);
Serial.print(" I="); Serial.println(sCurr[i], 3);

if (sVib[i] > 0.8) {
Serial.println("WARN: High vib - fallback");
ledcWrite(MOTOR_A_PWM, 0);
failed = true;
break;
}
}

if (failed) {
ledcWrite(MOTOR_A_PWM, 0);
return;
}

Serial.println("Extended sweep to 220...");
ledcWrite(MOTOR_A_PWM, 220);
delay(4000);
for (int u = 0; u < 5; u++) {
updateRPM(); delay(100);
}
float extMax = 0;
for (int r = 0; r < 5; r++) {
float sV=0, sVs=0, sC=0;
collectSamples(sV, sVs, sC);
updateRPM();
if (sVs > extMax) extMax = sVs;
delay(200);
}
if (extMax > maxNormalVstd) maxNormalVstd = extMax;

// Resonance probe
Serial.println("Resonance probing...");
int rP[] = {182,186,190,194,198,202,206,210};
float rV[8];
int pkIdx = 0;
float pkVib = 0;

for (int i = 0; i < 8; i++) {
ledcWrite(MOTOR_A_PWM, rP[i]);
delay(2000);
updateRPM();
float sV=0, sVs=0, sC=0;
collectSamples(sV, sVs, sC);
rV[i] = sV;
Serial.print(" P="); Serial.print(rP[i]);
Serial.print(" V="); Serial.println(rV[i], 4);
if (rV[i] > pkVib) { pkVib=rV[i]; pkIdx=i; }
if (rV[i] > 1.5) { Serial.println("High vib stop"); break; }
}

RESONANCE_ZONE_LOW = rP[pkIdx] - 12;
RESONANCE_ZONE_HIGH = rP[pkIdx] + 12;

// ==============================
// FFT AT RESONANCE PEAK
// ==============================
Serial.println("=============================");
Serial.println("FFT ANALYSIS at resonance PWM");
Serial.print("Running at PWM=");
Serial.println(rP[pkIdx]);
ledcWrite(MOTOR_A_PWM, rP[pkIdx]);
delay(2000);
updateRPM();

resonanceFreqHz = runFFT(rP[pkIdx]);
float resonanceRPM_FFT = resonanceFreqHz * 60.0;

Serial.println("=============================");
Serial.println(" FFT RESULT ");
Serial.println("=============================");
delay(300);
Serial.print("Dominant freq: ");
Serial.print(resonanceFreqHz, 2);
Serial.println(" Hz");
delay(300);
Serial.print("Equivalent RPM: ");
Serial.print(resonanceRPM_FFT, 1);
Serial.println(" RPM");
delay(300);
Serial.print("Resonance zone: ");
Serial.print(RESONANCE_ZONE_LOW);
Serial.print(" - ");
Serial.println(RESONANCE_ZONE_HIGH);
delay(300);

// ==============================
// SUBHARMONIC CHECK
// If FFT freq does not match IR
// check if it is half frequency
// 1/2 subharmonic is real and
// known in rotating machinery
// due to structural flexibility
// ==============================
if (rpmValid && filteredRPM > 0) {
float diff = abs(resonanceRPM_FFT - filteredRPM);
float pct = (diff / filteredRPM) * 100.0;

Serial.print("IR sensor RPM: ");
Serial.println(filteredRPM, 1);
delay(300);

if (pct < 25.0) {
Serial.println("Verification: CONFIRMED");
Serial.println("FFT freq matches rotation");
Serial.println("Mechanical resonance valid");
} else {
// Check subharmonic
// FFT may show half frequency
// due to structural flexibility
float subRPM = resonanceFreqHz * 2.0 * 60.0;
float diffSub = abs(subRPM - filteredRPM);
float pctSub = (diffSub / filteredRPM) * 100.0;

if (pctSub < 25.0) {
Serial.println("Verification: SUBHARMONIC");
Serial.println("1/2 order subharmonic found");
Serial.println("Structural flexibility response");
Serial.println("Resonance zone still valid");
} else {
Serial.print("Freq deviation: ");
Serial.print(pct, 0);
Serial.println("%");
Serial.println("Approximate validation only");
}
}
delay(300);
}

Serial.println("=============================");
Serial.println(">> Press Enter to continue ");
Serial.println(" commissioning... ");
Serial.println("=============================");

while (true) {
if (Serial.available() > 0) {
Serial.readStringUntil('\n');
break;
}
delay(100);
}

Serial.println("Continuing commissioning...");
delay(500);

int n = NUM_POINTS;

// Vibration twin all points
float sx=0,sy=0,sxy=0,sx2=0;
for (int i=0;i<n;i++) {
float x=sweepPWM[i],y=sVib[i];
sx+=x;sy+=y;sxy+=x*y;sx2+=x*x;
}
float dn=n*sx2-sx*sx;
if (abs(dn)>0.0001) {
VIB_SLOPE=(n*sxy-sx*sy)/dn;
VIB_OFFSET=(sy-VIB_SLOPE*sx)/n;
}

// RPM twin PWM 155+ only
float rx=0,ry=0,rxy=0,rx2=0; int rn=0;
for (int i=0;i<n;i++) {
if (sweepPWM[i]>=155 && sRPM[i]>5) {
float x=sweepPWM[i],y=sRPM[i];
rx+=x;ry+=y;rxy+=x*y;rx2+=x*x;rn++;
}
}
if (rn>=2) {
float rd=rn*rx2-rx*rx;
if (abs(rd)>0.0001) {
RPM_SLOPE=(rn*rxy-rx*ry)/rd;
RPM_OFFSET=(ry-RPM_SLOPE*rx)/rn;
Serial.print("RPM twin: ");
Serial.print(rn);
Serial.println(" reliable points");
}
} else {
Serial.println("WARN: RPM data insufficient");
}

// Current twin
float cx=0,cy=0,cxy=0,cx2=0;
for (int i=0;i<n;i++) {
float x=sweepPWM[i],y=sCurr[i];
cx+=x;cy+=y;cxy+=x*y;cx2+=x*x;
}
float cd=n*cx2-cx*cx;
if (abs(cd)>0.0001) {
I_SLOPE=(n*cxy-cx*cy)/cd;
I_OFFSET=(cy-I_SLOPE*cx)/n;
}

float maxDev=0;
for (int i=0;i<n;i++)
if (sCurrDev[i]>maxDev) maxDev=sCurrDev[i];
CURRENT_DEV_THRESHOLD = max(maxDev*3.0f, 1.0f);

float maxE=0;
for (int i=0;i<n;i++) {
float e=abs(sVib[i]-(VIB_SLOPE*sweepPWM[i]+VIB_OFFSET));
if (e>maxE) maxE=e;
}
VIB_ERROR_THRESHOLD = max(maxE*3.5f, 0.05f);
VIB_STD_THRESHOLD = max(maxNormalVstd*1.2f, 0.03f);

float maxV=0;
for (int i=0;i<n;i++)
if (sVib[i]>maxV) maxV=sVib[i];
RESONANCE_THRESHOLD = max(maxV*1.5f, 0.15f);
RPM_DROP_THRESHOLD = 0.55f;

Serial.println("=============================");
Serial.println("COMMISSIONING COMPLETE");
delay(200);
Serial.print("VIB slope: "); Serial.print(VIB_SLOPE,6);
Serial.print(" x + "); Serial.println(VIB_OFFSET,6);
delay(200);
Serial.print("RPM slope: "); Serial.print(RPM_SLOPE,4);
Serial.print(" x + "); Serial.println(RPM_OFFSET,4);
delay(200);
Serial.print("VIB_ERR th: "); Serial.println(VIB_ERROR_THRESHOLD,4);
delay(200);
Serial.print("RES thresh: "); Serial.println(RESONANCE_THRESHOLD,4);
delay(200);
Serial.print("RES zone: "); Serial.print(RESONANCE_ZONE_LOW);
Serial.print(" - "); Serial.println(RESONANCE_ZONE_HIGH);
delay(200);
Serial.print("RES freq: "); Serial.print(resonanceFreqHz,2);
Serial.print(" Hz = "); Serial.print(resonanceFreqHz*60.0,1);
Serial.println(" RPM");
delay(200);
Serial.print("CURR_DEV th: "); Serial.println(CURRENT_DEV_THRESHOLD,4);
delay(200);
Serial.print("RPM_DROP th: "); Serial.println(RPM_DROP_THRESHOLD,2);
Serial.println("=============================");
Serial.println(">> Press Enter to go READY");
Serial.println("=============================");

while (true) {
if (Serial.available() > 0) {
Serial.readStringUntil('\n');
break;
}
delay(100);
}

Serial.println("Ramping down...");
for (int p=220;p>=0;p-=5) {
ledcWrite(MOTOR_A_PWM,p);
delay(100);
}
ledcWrite(MOTOR_A_PWM,0);
pwmValue=0;
firstCurrentRead=true;
resonanceFaultActive=false;
mechFaultCount=0;
irNoSignalSince=0;
irFaultDetected=false;
irFaultPrinted=false;
initMovingAverage(0.0);
delay(1000);

Serial.println("=============================");
Serial.println("READY");
Serial.println("0-255 = Set Motor A PWM");
Serial.println("R = Recalibrate");
Serial.println("M = Start Motor B manual");
Serial.println("P = Start Motor B (resonance)");
Serial.println("L = Lock best phase");
Serial.println("S = Stop Motor B");
Serial.println("Z = Zone mode toggle");
Serial.println("=============================");
Serial.println("FAULTS ACTIVE:");
Serial.println(" 1. Resonance fault");
Serial.println(" 2. Structural change");
Serial.println(" 3. IR sensor fault");
Serial.println(" 4. Electrical");
Serial.println("=============================");
}

// ==============================
// SETUP
// ==============================
void setup() {
Serial.begin(115200);
Serial.setTimeout(10);

Wire.begin();
mpu.initialize();

if (!mpu.testConnection()) {
Serial.println("MPU FAILED");
while(1);
}

pinMode(26, OUTPUT);
digitalWrite(26, LOW);

ledcAttach(MOTOR_A_PWM, PWM_FREQ, PWM_RESOLUTION);
ledcWrite(MOTOR_A_PWM, 0);

ledcAttach(MOTOR_B_PIN, PWM_FREQ, PWM_RESOLUTION);
ledcWrite(MOTOR_B_PIN, 0);

pinMode(IR_PIN, INPUT);
attachInterrupt(digitalPinToInterrupt(IR_PIN),
irPulseISR, FALLING);

initMovingAverage(0.0);

Serial.println("Calibrating current sensor...");
delay(1000);

long rawSum=0;
for (int i=0;i<200;i++) {
rawSum+=analogRead(CURRENT_PIN);
delay(2);
}
zeroOffset=(rawSum/200.0)*(3.3/4095.0);
Serial.print("Current offset: ");
Serial.println(zeroOffset,4);

Serial.println("Calibrating vibration baseline...");
delay(1000);

float gravSum=0;
for (int i=0;i<100;i++) {
int16_t ax,ay,az;
mpu.getAcceleration(&ax,&ay,&az);
float mag=sqrt((float)ax*ax+
(float)ay*ay+
(float)az*az)/16384.0;
gravSum+=mag;
delay(5);
}
gravityBaseline=gravSum/100.0;
Serial.print("Gravity baseline: ");
Serial.println(gravityBaseline,4);

autoCommission();
}

// ==============================
// LOOP
// ==============================
void loop() {

if (reportHoldMode) {
if (Serial.available() > 0) {
Serial.readStringUntil('\n');
reportHoldMode = false;
Serial.println("Resuming system...");
Serial.println("=============================");
}
delay(100);
return;
}

if (Serial.available() > 0) {
String input = Serial.readStringUntil('\n');
input.trim();

if (input=="R"||input=="r") {
pwmValue=0;
ledcWrite(MOTOR_A_PWM,0);
if (motorBActive) stopMotorB();
reportHoldMode=false;
faultFirstSeen=0;
persistMessageShown=false;
firstCurrentRead=true;
resonanceFaultActive=false;
mechFaultCount=0;
irNoSignalSince=0;
irFaultDetected=false;
irFaultPrinted=false;
initMovingAverage(0.0);
delay(500);
autoCommission();

} else if (input=="P"||input=="p") {
if (resonanceFaultActive) {
startMotorB();
} else {
Serial.println("No resonance fault active");
Serial.println("Use M for manual start");
}

} else if (input=="M"||input=="m") {
startMotorB();

} else if (input=="L"||input=="l") {
if (motorBActive && sweepMode) {
float currentRed = ((vibBeforeControl - vibAvgSmooth)
/ vibBeforeControl) * 100.0;
if (currentRed > 8.0 &&
minVibDuringSweep < vibBeforeControl * 0.92) {
sweepMode = false;
lockedPWM = bestSweepPWM;
ledcWrite(MOTOR_B_PIN, lockedPWM);
Serial.println("=============================");
Serial.print("LOCKED PWM=");
Serial.println(lockedPWM);
float bestRed = ((vibBeforeControl - minVibDuringSweep)
/ vibBeforeControl) * 100.0;
Serial.print("Best reduction=");
Serial.print(bestRed, 1);
Serial.println("%");
Serial.println("=============================");
} else {
Serial.println("Reduction not strong enough to lock");
Serial.println("Keep sweeping - wait for better phase");
}
} else if (!motorBActive) {
Serial.println("Motor B not running");
} else {
Serial.print("Already locked at PWM=");
Serial.println(lockedPWM);
}

} else if (input=="S"||input=="s") {
if (motorBActive) stopMotorB();
else Serial.println("Motor B not running");

} else if (input=="Z"||input=="z") {
zoneMode = !zoneMode;
Serial.println(zoneMode ?
"ZONE MODE ON - auto Motor B in resonance zone" :
"ZONE MODE OFF");

} else {
int value = input.toInt();
if (value >= 0 && value <= 255) {
pwmValue = value;
ledcWrite(MOTOR_A_PWM, pwmValue);
Serial.print("PWM set: ");
Serial.println(pwmValue);
}
}
}

// Sampling
for (int i=0;i<SAMPLE_COUNT;i++) {
float voltage=analogRead(CURRENT_PIN)*(3.3/4095.0);
currentSamples[i]=abs((voltage-zeroOffset)/sensitivity);

int16_t ax,ay,az;
mpu.getAcceleration(&ax,&ay,&az);
float mag=sqrt((float)ax*ax+
(float)ay*ay+
(float)az*az)/16384.0;
vibSamples[i]=abs(mag-gravityBaseline);
delay(1);
}

float cSum=0;
for (int i=0;i<SAMPLE_COUNT;i++)
cSum+=currentSamples[i];
currentMean=cSum/SAMPLE_COUNT;

if (firstCurrentRead) {
currentMeanSmooth=currentMean;
firstCurrentRead=false;
} else {
currentMeanSmooth=0.7*currentMeanSmooth
+0.3*currentMean;
}

float vibSum=0;
for (int i=0;i<SAMPLE_COUNT;i++)
vibSum+=pow(vibSamples[i],2);
vibrationRMS=sqrt(vibSum/SAMPLE_COUNT);

float vibMean=0;
for (int i=0;i<SAMPLE_COUNT;i++)
vibMean+=vibSamples[i];
vibMean/=SAMPLE_COUNT;

float vibVar=0;
for (int i=0;i<SAMPLE_COUNT;i++)
vibVar+=pow(vibSamples[i]-vibMean,2);
vibStd=sqrt(vibVar/SAMPLE_COUNT);

updateRPM();

float vibExpected = VIB_SLOPE * pwmValue + VIB_OFFSET;
float vibError = abs(vibrationRMS - vibExpected);
float rpmExpected = RPM_SLOPE * pwmValue + RPM_OFFSET;
float currentExpected = I_SLOPE * pwmValue + I_OFFSET;
float currentDev = abs(currentMeanSmooth - currentExpected);

float rpmDropPct=0;
if (rpmValid && rpmExpected > 0) {
rpmDropPct = (rpmExpected - filteredRPM) / rpmExpected;
rpmDropPct = constrain(rpmDropPct, 0.0, 1.0);
}

int zone;
const char* zoneName;
if (pwmValue < RESONANCE_ZONE_LOW) {
zone=1; zoneName="NRM";
} else if (pwmValue <= RESONANCE_ZONE_HIGH) {
zone=2; zoneName="RES";
} else {
zone=3; zoneName="PST";
}

if (zoneMode) {
if (zone==2 && !motorBActive) {
Serial.println("ZONE: Auto starting Motor B");
startMotorB();
} else if (zone!=2 && motorBActive) {
Serial.println("ZONE: Stopping Motor B");
stopMotorB();
}
}

// IR sensor fault detection
if (pwmValue > 100 && !rpmValid) {
if (irNoSignalSince == 0) {
irNoSignalSince = millis();
irFaultPrinted = false;
} else if ((millis() - irNoSignalSince) >= IR_FAULT_TIMEOUT) {
irFaultDetected = true;
if (!irFaultPrinted) {
Serial.println("=============================");
Serial.println("FAULT: IR SENSOR - NO SIGNAL");
Serial.println("Check sensor alignment/wiring");
Serial.println("Electrical fault disabled");
Serial.println("=============================");
irFaultPrinted = true;
if (motorBActive) stopMotorB();
}
} else {
Serial.print("WARN: IR no signal for ");
Serial.print((millis() - irNoSignalSince) / 1000);
Serial.println("s");
}
} else {
if (irFaultDetected) {
Serial.println("IR SENSOR: Signal restored");
}
irNoSignalSince = 0;
irFaultDetected = false;
irFaultPrinted = false;
}

Serial.print("PWM:"); Serial.print(pwmValue);
Serial.print(" Z:"); Serial.print(zoneName);
Serial.print(" RPM:"); Serial.print(filteredRPM,1);
Serial.print(" Vib:"); Serial.print(vibrationRMS,3);
Serial.print(" Exp:"); Serial.print(vibExpected,3);
Serial.print(" Err:"); Serial.print(vibError,3);
Serial.print(" I:"); Serial.print(currentMeanSmooth,2);
Serial.print(" Idev:");Serial.println(currentDev,2);

if (pwmValue <= 50) {
if (motorBActive) stopMotorB();
faultFirstSeen = 0;
persistMessageShown = false;
resonanceFaultActive = false;
mechFaultCount = 0;
irNoSignalSince = 0;
irFaultDetected = false;
irFaultPrinted = false;
Serial.println("-- Motor stopped --");
Serial.println("---");
delay(200);
return;
}

bool fault = false;

// Fault 1 Resonance
if (zone==2 &&
pwmValue >= RESONANCE_ZONE_LOW+2 &&
vibrationRMS > RESONANCE_THRESHOLD) {
float conf = (vibrationRMS / RESONANCE_THRESHOLD) * 100.0;
Serial.print("FAULT: RESONANCE Conf=");
Serial.print(conf, 0);
Serial.println("%");
fault = true;
resonanceFaultActive = true;
mechFaultCount = 0;
if (!motorBActive)
Serial.println("-> Press P or M to start Motor B");

} else if (zone==2) {
Serial.println("WARN: Inside resonance zone");
resonanceFaultActive = false;
mechFaultCount = 0;
faultFirstSeen = 0;
}

// Fault 2 Structural Change
if (!fault &&
vibrationRMS < vibExpected * 0.35 &&
pwmValue > 100) {
Serial.println("FAULT: STRUCTURAL CHANGE");
Serial.println("Vibration abnormally low");
fault = true;
resonanceFaultActive = false;
mechFaultCount = 0;
}

// Fault 3 IR Sensor
if (!fault && irFaultDetected) {
Serial.println("FAULT: IR SENSOR FAILURE");
fault = true;
resonanceFaultActive = false;
}

// Fault 4 Electrical
if (!fault &&
pwmValue > 155 &&
rpmValid &&
!irFaultDetected) {

float effThresh = RPM_DROP_THRESHOLD;
if (pwmValue >= 155 && pwmValue < 185)
effThresh = RPM_DROP_THRESHOLD * 1.8f;

bool rpmFault = (rpmDropPct > effThresh);
bool currFault = (currentDev > CURRENT_DEV_THRESHOLD);

if (rpmFault && currFault) {
float conf = (rpmDropPct / effThresh) * 100.0;
Serial.print("FAULT: ELECTRICAL Conf=");
Serial.print(conf, 0);
Serial.println("% [RPM+Current]");
fault = true;
resonanceFaultActive = false;
mechFaultCount = 0;
} else if (rpmFault) {
float conf = (rpmDropPct / effThresh) * 100.0;
Serial.print("FAULT: ELECTRICAL Conf=");
Serial.print(conf, 0);
Serial.println("% [RPM only]");
fault = true;
resonanceFaultActive = false;
mechFaultCount = 0;
}
}

if (!fault) {
if (zone != 2) resonanceFaultActive = false;
faultFirstSeen = 0;
persistMessageShown = false;
if (zone != 2) mechFaultCount = 0;
}

// Motor B output
if (motorBActive) {
computeSweep();

if (sweepMode) {
Serial.print("SWEEP: MB=");
Serial.print(sweepPWMCurr);
Serial.print(" Best=");
Serial.print(bestSweepPWM);
Serial.print(" MinVib=");
Serial.print(minVibDuringSweep, 3);
Serial.print(" Red=");
Serial.print(reductionPercent, 1);
Serial.print("%");

if (badPhaseCount > 0) {
Serial.print(" BadPh=");
Serial.print(badPhaseCount);
}

if (reductionPercent > 10)
Serial.println(" >> PHASE:GOOD");
else if (reductionPercent > 0)
Serial.println(" >> PHASE:WEAK");
else
Serial.println(" >> PHASE:BAD");

if (bandNarrowed) {
Serial.print("NARROWED: ");
Serial.print(sweepPWMLow);
Serial.print(" - ");
Serial.println(sweepPWMHigh);
}

} else {
Serial.print("LOCKED: MB=");
Serial.print(lockedPWM);
Serial.print(" Red=");
Serial.print(reductionPercent, 1);
Serial.println("%");
}
}

if (!fault) Serial.println("OK");
Serial.println("---");
delay(200);
}

