import processing.serial.*;

// ==========================================
// CONFIGURATION & SWITCHES
// ==========================================
Serial port;
boolean useSerial = true;
String serialPortName = "COM3";
int baudRate = 115200;

// ==========================================
// TELEMETRY & RUNTIME STATE VARIABLES
// ==========================================
String flightState = "GROUND IDLE";
float pitch = 0, roll = 0, yaw = 0;
float altitude = 0;
float aviBatt = 8.47, propBatt = 30.12;
float vnImuT = 17.3, baroT = 15.4;
int signalDbm = -36;

// Extended Telemetry to fill out old layout specs
float accX = 0.00, accY = 0.00, accZ = 1.00;
float vX = 0.16, vY = -0.19, vZ = -93.70;
float pX = -1.41, pY = 2.82;

int packetCount = 0;
float tlmRate = 7.0;
int lastHzCheckTime = 0;
int lastPacketCount = 0;
int lastPacketTime = 0;

// Mission Stopwatch Variables
boolean isLaunched = false;
int launchTimeMarker = 0;
String stopwatchStr = "T+ 00:00:00";

// Offsets for Zeroing Logic
float pitchOffset = 0, rollOffset = 0, yawOffset = 0;

// Multiple Plot History Streams
int maxPoints = 120;
float[][] gyroHist = new float[3][maxPoints]; // 0=pitch, 1=roll, 2=yaw
float[] altHist = new float[maxPoints];
int histIdx = 0;
float previousAlt = 0;

// ==========================================
// COLOR PALETTE
// ==========================================
color BG_COLOR      = color(10, 10, 10);
color PANEL_BORDER  = color(40, 40, 40);
color STATE_BLUE    = color(74, 144, 226); 
color TEXT_MAIN     = color(230, 230, 230);
color TEXT_MUTED    = color(100, 100, 100);
color BTN_BG        = color(25, 25, 25);
color BTN_HOVER     = color(45, 45, 45);
color LAUNCH_RED    = color(219, 68, 85);
color LAUNCH_HOVER  = color(245, 93, 109);
color LINE_RED      = color(219, 68, 85);
color LINE_YELLOW   = color(244, 180, 26);
color LINE_GREEN    = color(15, 157, 88);

PFont UI_Font_Bold;

// ==========================================
// SETUP
// ==========================================
void setup() {
  size(1280, 720, P3D);
  frameRate(60);
  UI_Font_Bold = createFont("SF Pro Text Bold", 13, true);
  textFont(UI_Font_Bold);
  
  if (useSerial) {
    tryInitSerial();
  }
}

void tryInitSerial() {
  try {
    if (port != null) {
      port.stop();
    }
    port = new Serial(this, serialPortName, baudRate);
    port.bufferUntil('\n');
    lastPacketTime = millis();
    println("Serial connected to " + serialPortName);
  } catch (Exception e) {
    println("Serial failed: " + e.getMessage());
  }
}

// ==========================================
// MAIN REFRESH LOOP
// ==========================================
void draw() {
  background(BG_COLOR);
  
  hint(DISABLE_DEPTH_TEST);
  
  if (useSerial) {
    checkSerialWatchdog(); 
  } else {
    simulateTelemetry();
  }
  calculateTlmRate();
  updateMissionStopwatch();
  
  // Render Old-Style Layout Structure
  drawTopHorizontalGraphs();
  drawLeftVerticalGraphs();
  
  hint(ENABLE_DEPTH_TEST);
  drawCenter3DModelSpace();
  hint(DISABLE_DEPTH_TEST);
  
  drawRightMetadataPanel();
  drawTopBranding();
}

void checkSerialWatchdog() {
  int timeSinceLastPacket = millis() - lastPacketTime;
  if (timeSinceLastPacket > 2000) {
    tlmRate = 0.0;
  }
}

// ==========================================
// SERIAL EVENT PARSING
// ==========================================
void serialEvent(Serial p) {
  String data = p.readStringUntil('\n');
  if (data != null) {
    data = trim(data);
    if (data.startsWith("#")) return;
    
    if (data.contains("/")) {
      String[] parts = split(data, '/');
      if (parts.length >= 3) {
        roll = float(parts[0]);
        pitch = float(parts[1]);
        yaw = float(parts[2]);
        
        gyroHist[0][histIdx] = pitch;
        gyroHist[1][histIdx] = roll;
        gyroHist[2][histIdx] = yaw;
        histIdx = (histIdx + 1) % maxPoints;
        
        packetCount++;
        lastPacketTime = millis();
      }
    }
    else if (data.contains("Alt(ft):")) {
      String altStr = data.replace("Alt(ft):", "").trim();
      altitude = float(altStr);
      altHist[histIdx] = altitude;
      
      if (altitude > 2.0 && !isLaunched) {
        isLaunched = true;
        launchTimeMarker = millis();
      }
      
      if (!isLaunched && altitude < 1.0) {
        flightState = "GROUND IDLE";
      } else if (isLaunched) {
        if (altitude >= previousAlt) {
          flightState = "POWERED FLIGHT / BOOST";
        } else if (altitude < previousAlt && altitude > 2.0) {
          flightState = "DESCENT / RECOVERY";
        } else if (altitude <= 2.0) {
          flightState = "TOUCHDOWN / GROUND";
          isLaunched = false;
        }
      }
      previousAlt = altitude;
    }
  }
}

// ==========================================
// RUNTIME UTILS
// ==========================================
void updateMissionStopwatch() {
  if (isLaunched) {
    int elapsedMillis = millis() - launchTimeMarker;
    int totalSecs = elapsedMillis / 1000;
    int mins = totalSecs / 60;
    int secs = totalSecs % 60;
    int hundredths = (elapsedMillis % 1000) / 10;
    stopwatchStr = "T+ " + nf(mins, 2) + ":" + nf(secs, 2) + ":" + nf(hundredths, 2);
  } else {
    stopwatchStr = "T+ 00:00:00";
  }
}

void calculateTlmRate() {
  int currentTime = millis();
  if (currentTime - lastHzCheckTime >= 1000) {
    tlmRate = (packetCount - lastPacketCount) / ((currentTime - lastHzCheckTime) / 1000.0);
    lastPacketCount = packetCount;
    lastHzCheckTime = currentTime;
  }
}

// ==========================================
// RENDER OLD INTERFACE PANELS
// ==========================================
void drawTopHorizontalGraphs() {
  int graphW = 133;
  int graphH = 100;
  int startY = 10;
  
  // Panel 1: VN IMU Gyros
  drawPanelOutline(10, startY, graphW, graphH, "VN IMU Gyros");
  drawMultiStreamGraphWithAxis(10, startY, graphW, graphH, gyroHist, 3, new color[]{LINE_GREEN, LINE_RED, LINE_YELLOW});

  // Panel 2: VN IMU Accels
  drawPanelOutline(148, startY, graphW, graphH, "VN IMU Accels");
  drawMultiStreamGraphWithAxis(148, startY, graphW, graphH, gyroHist, 3, new color[]{LINE_GREEN, LINE_RED, LINE_YELLOW});

  // Panel 3: Body Orientation X
  drawPanelOutline(286, startY, graphW, graphH, "Body Orientation X");
  drawMultiStreamGraphWithAxis(286, startY, graphW, graphH, gyroHist, 1, new color[]{LINE_YELLOW}, 0);

  // Panel 4: Body Orientation Y
  drawPanelOutline(424, startY, graphW, graphH, "Body Orientation Y");
  drawMultiStreamGraphWithAxis(424, startY, graphW, graphH, gyroHist, 1, new color[]{LINE_GREEN}, 1);

  // Panel 5: Body Orientation Z
  drawPanelOutline(562, startY, graphW, graphH, "Body Orientation Z");
  drawMultiStreamGraphWithAxis(562, startY, graphW, graphH, gyroHist, 1, new color[]{LINE_RED}, 2);
}

void drawTopBranding() {
  int brandX = 705;          
  int brandY = 10;           
  int panelW = 390;          
  int panelH = 100;          
  
  drawPanelOutline(brandX, brandY, panelW, panelH, "");
  
  textAlign(CENTER, CENTER);
  textSize(24);
  fill(TEXT_MAIN);
  text("AVIA", brandX + (panelW / 2), brandY + 38);
  
  textSize(11);
  fill(TEXT_MUTED);
  text("Ground Control", brandX + (panelW / 2), brandY + 68);
}

void drawLeftVerticalGraphs() {
  int panelW = 210;
  
  // State Window Box
  stroke(PANEL_BORDER);
  fill(0);
  rect(10, 120, panelW, 200);
  
  if (isLaunched) {
    fill(LAUNCH_RED);
  } else {
    fill(STATE_BLUE);
  }
  noStroke();
  rect(15, 125, panelW - 10, 35);
  
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(11);
  text(flightState + " [" + stopwatchStr + "]", 15 + (panelW-10)/2, 142);
  
  // Environmental Status Readouts
  fill(TEXT_MAIN);
  textSize(11);
  int leftX = 20;
  int rightX = 120;
  
  textAlign(LEFT, TOP);
  text("AVI Batt: " + nf(aviBatt, 1, 2) + " V", leftX, 175);
  text("Prop Batt: " + nf(propBatt, 1, 2) + " V", leftX, 192);
  text("VN IMU T: " + nf(vnImuT, 1, 1) + " °C", leftX, 209);
  text("Baro T:   " + nf(baroT, 1, 1) + " °C", leftX, 226);
  
  text("Signal: " + signalDbm + " dBm", rightX, 175);
  text("TLM Rate: " + nf(tlmRate, 1, 1) + " Hz", rightX, 192);

  // Stacked Secondary History Plots
  int itemH = 110;
  drawPanelOutline(10, 330, panelW, itemH, "Altitude");
  drawSingleStreamGraphWithAxis(10, 330, panelW, itemH, altHist, LINE_YELLOW, "ft"); 

  drawPanelOutline(10, 450, panelW, itemH, "XY Position");
  drawStaticBaselineGraph(10, 450, panelW, itemH);

  drawPanelOutline(10, 570, panelW, itemH, "XY Velocity");
  drawStaticBaselineGraph(10, 570, panelW, itemH);
}

// ==========================================
// DATA HISTORY GRAPH PLOTTERS
// ==========================================
void drawSingleStreamGraphWithAxis(int x, int y, int w, int h, float[] data, color clr, String unit) {
  float minV = Float.MAX_VALUE;
  float maxV = -Float.MAX_VALUE;
  
  for (int i = 0; i < maxPoints; i++) {
    if (Float.isNaN(data[i]) || Float.isInfinite(data[i])) continue;
    if (data[i] < minV) minV = data[i];
    if (data[i] > maxV) maxV = data[i];
  }
  
  if (minV == Float.MAX_VALUE) { minV = -1.0; maxV = 1.0; }
  float range = maxV - minV;
  if (abs(range) < 0.001) { minV -= 1.0; maxV += 1.0; } 
  else { minV -= range * 0.1; maxV += range * 0.1; }
  
  drawGraphScaleLabels(x, y, w, h, nf(maxV, 1, 3) + " " + unit, nf(minV, 1, 3) + " " + unit);
  
  float axisY = map(0, minV, maxV, y + h - 12, y + 22);
  axisY = constrain(axisY, y + 22, y + h - 12);
  stroke(PANEL_BORDER, 120);
  line(x + 6, axisY, x + w - 45, axisY);
  
  noFill();
  stroke(clr);
  strokeWeight(1.2);
  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    int idx = (histIdx + i) % maxPoints;
    float val = data[idx];
    float graphX = map(i, 0, maxPoints, x + 6, x + w - 45);
    float graphY = map(val, minV, maxV, y + h - 12, y + 22);
    vertex(graphX, graphY);
  }
  endShape();
}

void drawMultiStreamGraphWithAxis(int x, int y, int w, int h, float[][] data, int numStreams, color[] colors) {
  drawMultiStreamGraphWithAxis(x, y, w, h, data, numStreams, colors, -1);
}

void drawMultiStreamGraphWithAxis(int x, int y, int w, int h, float[][] data, int numStreams, color[] colors, int forcedStreamIdx) {
  float minV = Float.MAX_VALUE;
  float maxV = -Float.MAX_VALUE;
  
  int startStream = (forcedStreamIdx == -1) ? 0 : forcedStreamIdx;
  int endStream   = (forcedStreamIdx == -1) ? numStreams : forcedStreamIdx + 1;
  
  for (int s = startStream; s < endStream; s++) {
    for (int i = 0; i < maxPoints; i++) {
      if (data[s][i] < minV) minV = data[s][i];
      if (data[s][i] > maxV) maxV = data[s][i];
    }
  }
  
  if (minV == Float.MAX_VALUE) { minV = -1.0; maxV = 1.0; }
  float range = maxV - minV;
  if (abs(range) < 0.001) { minV -= 1.0; maxV += 1.0; } 
  else { minV -= range * 0.1; maxV += range * 0.1; }
  
  drawGraphScaleLabels(x, y, w, h, nf(maxV, 1, 3), nf(minV, 1, 3));
  
  float axisY = map(0, minV, maxV, y + h - 12, y + 22);
  axisY = constrain(axisY, y + 22, y + h - 12);
  stroke(PANEL_BORDER, 120);
  line(x + 6, axisY, x + w - 32, axisY);
  
  int colorMapIdx = 0;
  for (int s = startStream; s < endStream; s++) {
    noFill();
    stroke(colors[colorMapIdx % colors.length]);
    strokeWeight(1);
    beginShape();
    for (int i = 0; i < maxPoints; i++) {
      int idx = (histIdx + i) % maxPoints;
      float val = data[s][idx];
      float graphX = map(i, 0, maxPoints, x + 6, x + w - 32);
      float graphY = map(val, minV, maxV, y + h - 12, y + 22);
      vertex(graphX, graphY);
    }
    endShape();
    colorMapIdx++;
  }
}

void drawStaticBaselineGraph(int x, int y, int w, int h) {
  drawGraphScaleLabels(x, y, w, h, "0.0", "0.0");
  stroke(PANEL_BORDER, 150);
  line(x + 6, y + h - 25, x + w - 32, y + h - 25);
}

// ==========================================
// 3D MODEL VIEWPORT (YOUR WORKING ROCKET)
// ==========================================
void drawCenter3DModelSpace() {
  stroke(PANEL_BORDER);
  fill(5);
  rect(230, 120, 865, 575);
  
  pushMatrix();
  translate(230 + 865/2, 120 + 575/2, 150);
  
  ambientLight(120, 120, 120);
  directionalLight(255, 255, 255, 0.5, 1, -1);
  
  rotateY(radians(yaw - yawOffset));
  rotateX(radians(pitch - pitchOffset));
  rotateZ(radians(roll - rollOffset));
  
  stroke(40);
  fill(240);
  strokeWeight(1);
  
  int segments = 32;
  float radius = 35;
  float length = 260;
  
  // Rocket Cylinder Cylinder
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= segments; i++) {
    float angle = TWO_PI * i / segments;
    float x = cos(angle) * radius;
    float z = sin(angle) * radius;
    vertex(x, -length/2, z);
    vertex(x, length/2, z);
  }
  endShape();
  
  // Cap Nosecone
  pushMatrix();
  translate(0, -length/2, 0);
  fill(210, 50, 50);
  cone(radius, 65);
  popMatrix();
  
  // Stabilizer Fins
  fill(70);
  for(int i = 0; i < 4; i++) {
    pushMatrix();
    rotateY(HALF_PI * i);
    translate(radius + 10, length/2 - 10, 0);
    box(25, 10, 8);
    popMatrix();
  }
  
  popMatrix();
}

// ==========================================
// RIGHT PANEL (COMMANDS & RAW TELEMETRY OVERHAUL)
// ==========================================
void drawRightMetadataPanel() {
  int panelX = 1105;
  int panelW = 165;

  drawPanelOutline(panelX, 120, panelW, 145, "System Commands");
  drawInteractiveButton(panelX + 10, 145, panelW - 20, 26, "ZERO GYROS");
  drawInteractiveButton(panelX + 10, 182, panelW - 20, 26, "RESET FLIGHT");
  drawLaunchButton(panelX + 10, 219, panelW - 20, 26, "LAUNCH SEQUENCE");

  drawPanelOutline(panelX, 275, panelW, 420, "Raw Telemetry");
  
  boolean isConnected = useSerial && (millis() - lastPacketTime < 2000);
  if (isConnected) { fill(LINE_GREEN); } else { fill(LAUNCH_RED); }
  noStroke();
  rect(panelX + 10, 298, panelW - 20, 24);
  
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(9);
  text(isConnected ? "STATUS: CONNECTED" : "STATUS: DISCONNECTED", panelX + (panelW / 2), 310);
  
  // Fully Detailed Text Readouts
  int startY = 344;
  int spacing = 16;
  textSize(10);
  fill(TEXT_MAIN);
  textAlign(LEFT, TOP);
  
  text("Pitch: " + nf(pitch, 1, 2) + "°", panelX + 10, startY);
  text("Roll:  " + nf(roll, 1, 2) + "°", panelX + 10, startY + spacing);
  text("Yaw:   " + nf(yaw, 1, 2) + "°", panelX + 10, startY + (spacing*2));
  
  text("accX:  " + nf(accX, 1, 2) + " g", panelX + 10, startY + (spacing*3));
  text("accY:  " + nf(accY, 1, 2) + " g", panelX + 10, startY + (spacing*4));
  text("accZ:  " + nf(accZ, 1, 2) + " g", panelX + 10, startY + (spacing*5));
  
  text("vX:    " + nf(vX, 1, 2) + " g", panelX + 10, startY + (spacing*6));
  text("vY:   " + nf(vY, 1, 2) + " g", panelX + 10, startY + (spacing*7));
  text("vZ:   " + nf(vZ, 1, 2) + " g", panelX + 10, startY + (spacing*8));
  
  text("pX:   " + nf(pX, 1, 2) + " m", panelX + 10, startY + (spacing*9));
  text("pY:    " + nf(pY, 1, 2) + " m", panelX + 10, startY + (spacing*10));
  text("Alt:   " + nf(altitude, 1, 1) + " ft", panelX + 10, startY + (spacing*11));
}

// ==========================================
// LAYOUT INTERACTION & INTERFACE DECORATORS
// ==========================================
void mousePressed() {
  int panelX = 1105;
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 145 && mouseY >= 145 && mouseY <= 145 + 26) {
    pitchOffset = pitch; rollOffset = roll; yawOffset = yaw;
    println("ZERO GYROS - Offsets set");
  }
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 145 && mouseY >= 182 && mouseY <= 182 + 26) {
    packetCount = 0; altitude = 0; previousAlt = 0;
    isLaunched = false; flightState = "GROUND IDLE";
    println("RESET FLIGHT");
  }
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 145 && mouseY >= 219 && mouseY <= 219 + 26) {
    if (!isLaunched) {
      isLaunched = true; launchTimeMarker = millis();
      flightState = "POWERED FLIGHT / BOOST";
      println("LAUNCH SEQUENCE INITIATED!");
    }
  }
}

void drawPanelOutline(int x, int y, int w, int h, String title) {
  stroke(PANEL_BORDER);
  strokeWeight(1);
  fill(0);
  rect(x, y, w, h);
  
  fill(TEXT_MUTED);
  textSize(9);
  textAlign(LEFT, TOP);
  text(title, x + 8, y + 6);
}

void drawGraphScaleLabels(int x, int y, int w, int h, String maxStr, String minStr) {
  fill(TEXT_MUTED);
  textSize(8);
  textAlign(RIGHT, TOP);
  text(maxStr, x + w - 6, y + 6);
  textAlign(RIGHT, BOTTOM);
  text(minStr, x + w - 6, y + h - 6);
}

void drawInteractiveButton(int x, int y, int w, int h, String label) {
  if (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h) { fill(BTN_HOVER); } 
  else { fill(BTN_BG); }
  stroke(PANEL_BORDER);
  rect(x, y, w, h, 2);
  
  fill(TEXT_MAIN);
  textAlign(CENTER, CENTER);
  textSize(9);
  text(label, x + w/2, y + h/2 - 1);
}

void drawLaunchButton(int x, int y, int w, int h, String label) {
  if (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h) { fill(LAUNCH_HOVER); } 
  else { fill(LAUNCH_RED); }
  stroke(PANEL_BORDER);
  rect(x, y, w, h, 2);
  
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(9);
  text(label, x + w/2, y + h/2 - 1);
}

void simulateTelemetry() {
  pitch = 8 * sin(frameCount * 0.03);
  roll  = 5 * cos(frameCount * 0.02);
  yaw   = (frameCount * 0.2) % 360;
  altitude = isLaunched ? altitude + random(0.8, 2.2) : random(-0.02, 0.02);
  if (altitude < 0) altitude = 0;
  
  gyroHist[0][histIdx] = pitch;
  gyroHist[1][histIdx] = roll;
  gyroHist[2][histIdx] = yaw;
  altHist[histIdx] = altitude;
  histIdx = (histIdx + 1) % maxPoints;
  
  if (frameCount % 4 == 0) packetCount++;
}

void cone(float r, float h) {
  int sides = 32;
  beginShape(TRIANGLE_FAN);
  vertex(0, 0, 0);
  for (int i = 0; i <= sides; i++) {
    float angle = TWO_PI * i / sides;
    vertex(cos(angle) * r, h, sin(angle) * r);
  }
  endShape();
}
