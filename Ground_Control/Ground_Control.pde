import processing.serial.*;
import java.nio.ByteBuffer; // <-- Needed for byte conversion
import java.nio.ByteOrder;  // <-- Needed for Little-Endian format

// ==========================================
// CONFIGURATION & SWITCHES
// ==========================================
Serial port;
boolean useSerial = true; // Set true when hardware is active
String serialPortName = "COM4"; // Make sure this matches your Ground COM port
int baudRate = 115200;
float altMin = 0;
float altMax = 10;
float altGroundOffset = 0;
boolean altInitialized = false;
int lastPacketTime = 0;
int lastReconnectAttempt = 0; // Tracks reconnection interval rates

// ==========================================
// TELEMETRY & RUNTIME STATE VARIABLES
// ==========================================
String flightState = "GROUND IDLE";
float pitch = 0, roll = 0, yaw = 0;
float altitude = 0, vSpeed = 0, tempC = 17.3;
float accelX = 0, accelY = 0, accelZ = 1.0; // Fixed: MPU6050 rest state is 1.0G, not 9.81
float aviBatt = 8.47, propBatt = 30.12;
int signalDbm = -36;

int packetCount = 0;
float tlmRate = 0;
int lastHzCheckTime = 0;
int lastPacketCount = 0;

// Mission Stopwatch Variables
boolean isLaunched = false;
int launchTimeMarker = 0;
String stopwatchStr = "T+ 00:00:00";

// Offsets for Zeroing Logic
float pitchOffset = 0, rollOffset = 0, yawOffset = 0;

// Multiple Plot History Streams (For the multi-graph array)
int maxPoints = 120;
float[][] gyroHist = new float[3][maxPoints]; // [X, Y, Z]
float[][] accelHist = new float[3][maxPoints]; // [X, Y, Z]
float[] altHist = new float[maxPoints];
int histIdx = 0;
float previousAlt = 0;

// ==========================================
// COLOR PALETTE (SPRITE Mission Control Theme)
// ==========================================
color BG_COLOR      = color(10, 10, 10);
color PANEL_BORDER  = color(60, 60, 60);
color STATE_BLUE    = color(74, 144, 226); 
color TEXT_MAIN     = color(230, 230, 230);
color TEXT_MUTED    = color(130, 130, 130);
color BTN_BG        = color(25, 25, 25);
color BTN_HOVER     = color(45, 45, 45);

// Launch Mechanics Coloring Assets
color LAUNCH_RED    = color(219, 68, 85);
color LAUNCH_HOVER  = color(245, 93, 109);

// Graph Line Colors
color LINE_RED      = color(219, 68, 85);
color LINE_YELLOW   = color(244, 180, 26);
color LINE_GREEN    = color(15, 157, 88);

// Typography Font Fields
PFont UI_Font_Bold;

// ==========================================
// SETUP
// ==========================================
void setup() {
  size(1280, 720, P3D);
  frameRate(165);
  UI_Font_Bold = createFont("SF Pro Text Bold", 13, true);
  textFont(UI_Font_Bold);
  
  if (useSerial) {
    tryInitSerial();
  }
}

// Helper method to isolate serial connection initialization
void tryInitSerial() {
  try {
    if (port != null) {
      port.stop(); // Clean up old reference if it exists
    }
    port = new Serial(this, serialPortName, baudRate);
    lastPacketTime = millis(); // Reset timer on successful bind
    println("Serial link successfully established on " + serialPortName);
  } catch (Exception e) {
    println("Serial setup failed on " + serialPortName + ": " + e.getMessage());
  }
}

// ==========================================
// MAIN REFRESH LOOP
// ==========================================
void draw() {
  background(BG_COLOR);
  
  hint(DISABLE_DEPTH_TEST);
  
  if (useSerial) {
    readSerialTelemetry();
    checkSerialWatchdog(); 
  } else {
    simulateTelemetry();
  }
  calculateTlmRate();
  updateMissionStopwatch();
  
  // Render Panels
  drawTopHorizontalGraphs();
  drawLeftVerticalGraphs();
  
  hint(ENABLE_DEPTH_TEST);
  drawCenter3DModelSpace();
  hint(DISABLE_DEPTH_TEST);
  
  drawRightMetadataPanel();
  drawTopBranding();
}

// ==========================================
// WATCHDOG RECONNECT STRATEGY
// ==========================================
void checkSerialWatchdog() {
  int timeSinceLastPacket = millis() - lastPacketTime;
  
  if (timeSinceLastPacket > 2000) {
    tlmRate = 0.0; 
    
    if (millis() - lastReconnectAttempt > 2000) {
      println("Link timeout detected (" + nf(timeSinceLastPacket/1000.0, 1, 1) + "s). Attempting hardware reconnection...");
      lastReconnectAttempt = millis();
      tryInitSerial();
    }
  }
}

// Keep track of timing between radio packets for the math integration
int lastPacketTimestampMicros = 0; 

// ==========================================
// BINARY TELEMETRY PARSER ENGINE
// ==========================================
void readSerialTelemetry() {
  if (port == null) return;

  // Read lines from the serial buffer
  while (port.available() > 0) {
    String inString = port.readStringUntil('\n');
    
    if (inString != null) {
      inString = trim(inString);
      
      try {
        // 1. Parse Gyroscope Line
        if (inString.contains("Gyro (Pitch/Roll/Yaw):")) {
  String data = inString.replace("Gyro (Pitch/Roll/Yaw):", "").trim();
  float[] values = float(split(data, ','));
  if (values.length >= 3) {
    // Gyro rates: values[0] = pitch rate, values[1] = roll rate, values[2] = yaw rate
    float gyroPitchRate = values[1];
    float gyroRollRate = values[0];
    float gyroYawRate = values[2];
    
    // TIMING DELTA FOR INTEGRATION
    int nowMillis = millis();
    float dt = (nowMillis - lastPacketTime) / 1000.0f;
    if (dt <= 0 || dt > 0.2) dt = 0.02f;
    lastPacketTime = nowMillis;
    packetCount++;

    // SIMPLE GYRO INTEGRATION FIRST (for testing)
    // Remove the complementary filter temporarily to debug
    pitch += gyroPitchRate * dt;
    roll  += gyroRollRate * dt;
    yaw   += gyroYawRate * dt;
    
    // Normalize angles to -180 to 180 range
    if (pitch > 180) pitch -= 360;
    if (pitch < -180) pitch += 360;
    if (roll > 180) roll -= 360;
    if (roll < -180) roll += 360;
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
    
    // Update Graph History
    gyroHist[0][histIdx] = pitch;  
    gyroHist[1][histIdx] = roll;
    gyroHist[2][histIdx] = yaw;
  }
}
        
        // 2. Parse Accelerometer Line
        else if (inString.contains("Accel(Pitch/Roll/Yaw):")) {
          String data = inString.replace("Accel(Pitch/Roll/Yaw):", "").trim();
          float[] values = float(split(data, ','));
          if (values.length >= 3) {
            // Your Python code sends: tx_ax, tx_ay, tx_az
            // Where: tx_ax = accel['z'] (physical Z = pitch axis)
            //        tx_ay = accel['y'] (physical Y = roll axis)
            //        tx_az = accel['x'] (physical X = vertical)
            accelX = values[0];  // Pitch acceleration (G's)
            accelY = values[1];  // Roll acceleration (G's)
            accelZ = values[2];  // Vertical acceleration (G's)
            
            // Maintain scrolling history
            histIdx = (histIdx + 1) % maxPoints;
            accelHist[0][histIdx] = accelX;
            accelHist[1][histIdx] = accelY;
            accelHist[2][histIdx] = accelZ;
          }
        } 
        
        // 3. Parse Altitude Line
        else if (inString.contains("Alt(ft):")) {
          String data = inString.replace("Alt(ft):", "").trim();
          altitude = float(data);
          altHist[histIdx] = altitude;
          
          // AUTO FLIGHT STATE MACHINE ENTRY
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
              isLaunched = false; // Reset for next flight
            }
          }
          previousAlt = altitude;
        }
        
      } catch (Exception e) {
        println("String Format Parse Error: " + e.getMessage());
      }
    }
  }
}
// ==========================================
// RUNTIME UTILS: STOPWATCH TRACKING
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

// ==========================================
// TOP ROW: MULTIPLE SMALL GRAPHS & BRANDING
// ==========================================
void drawTopHorizontalGraphs() {
  int graphWidth = 165;
  int graphHeight = 100;
  int startY = 10;
  
  drawPanelOutline(10, startY, graphWidth, graphHeight, "VN IMU Gyros");
  drawMultiStreamGraphWithAxis(10, startY, graphWidth, graphHeight, gyroHist, 3, new color[]{LINE_RED, LINE_YELLOW, LINE_GREEN});

  drawPanelOutline(185, startY, graphWidth, graphHeight, "VN IMU Accels");
  drawMultiStreamGraphWithAxis(185, startY, graphWidth, graphHeight, accelHist, 3, new color[]{LINE_RED, LINE_YELLOW, LINE_GREEN});

  drawPanelOutline(360, startY, graphWidth, graphHeight, "Body Orientation X");
  drawMultiStreamGraphWithAxis(360, startY, graphWidth, graphHeight, gyroHist, 1, new color[]{LINE_YELLOW}, 0);

  drawPanelOutline(535, startY, graphWidth, graphHeight, "Body Orientation Y");
  drawMultiStreamGraphWithAxis(535, startY, graphWidth, graphHeight, gyroHist, 1, new color[]{LINE_GREEN}, 1);

  drawPanelOutline(710, startY, graphWidth, graphHeight, "Body Orientation Z");
  drawMultiStreamGraphWithAxis(710, startY, graphWidth, graphHeight, gyroHist, 1, new color[]{LINE_RED}, 2);
}

void drawTopBranding() {
  int brandX = 885;          
  int brandY = 10;           
  int panelW = 385;          
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

// ==========================================
// LEFT COLUMN: GRAPHS & CONDENSED PANEL
// ==========================================
void drawLeftVerticalGraphs() {
  int panelW = 210;
  
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
  
  fill(TEXT_MAIN);
  textSize(11);
  int textX = 20;
  
  textAlign(LEFT, TOP);
  text("AVI Batt:  " + nf(aviBatt, 1, 2) + " V", textX, 170);
  text("Prop Batt: " + nf(propBatt, 1, 2) + " V", textX, 185);
  text("VN IMU T:  " + nf(tempC, 1, 1) + " °C", textX, 200);
  text("Baro T:    " + "15.4 °C", textX, 215);
  
  textAlign(RIGHT, TOP);
  text("Signal: " + signalDbm + " dBm", textX + 190, 170);
  text("TLM Rate: " + nf(tlmRate, 1, 1) + " Hz", textX + 190, 185);

  int graphH = 115;
  drawPanelOutline(10, 330, panelW, graphH, "Altitude");
  drawSingleStreamGraphWithAxis(10, 330, panelW, graphH, altHist, LINE_YELLOW, "ft"); 
  
  drawPanelOutline(10, 455, panelW, graphH, "XY Position");
  drawGraphScaleLabels(10, 455, panelW, graphH, "0.0", "0.0");
  stroke(PANEL_BORDER, 90);
  line(10 + 6, 455 + graphH / 2 + 5, 10 + panelW - 42, 455 + graphH / 2 + 5);
  
  drawPanelOutline(10, 580, panelW, graphH, "XY Velocity");
  drawGraphScaleLabels(10, 580, panelW, graphH, "0.0", "0.0");
  stroke(PANEL_BORDER, 90);
  line(10 + 6, 580 + graphH / 2 + 5, 10 + panelW - 42, 580 + graphH / 2 + 5);
}

// ==========================================
// RENDER HELPERS WITH DYNAMIC SCALING & AXES
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
  if (abs(range) < 0.001) { 
    minV -= 1.0; 
    maxV += 1.0; 
  } else { 
    minV -= range * 0.1; 
    maxV += range * 0.1; 
  }
  
  drawGraphScaleLabels(x, y, w, h, nf(maxV, 1, 3) + " " + unit, nf(minV, 1, 3) + " " + unit);
  
  float axisY = map(0, minV, maxV, y + h - 12, y + 22);
  axisY = constrain(axisY, y + 22, y + h - 12);
  stroke(PANEL_BORDER, 100);
  strokeWeight(1);
  line(x + 6, axisY, x + w - 42, axisY);
  
  noFill();
  stroke(clr);
  strokeWeight(1.2);
  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    int idx = (histIdx + i) % maxPoints;
    float val = data[idx];
    if (Float.isNaN(val) || Float.isInfinite(val)) val = 0;
    
    float graphX = map(i, 0, maxPoints, x + 6, x + w - 42);
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
      if (Float.isNaN(data[s][i]) || Float.isInfinite(data[s][i])) continue;
      if (data[s][i] < minV) minV = data[s][i];
      if (data[s][i] > maxV) maxV = data[s][i];
    }
  }
  
  if (minV == Float.MAX_VALUE) { minV = -1.0; maxV = 1.0; }
  
  float range = maxV - minV;
  if (abs(range) < 0.001) { 
    minV -= 1.0; 
    maxV += 1.0; 
  } else { 
    minV -= range * 0.1; 
    maxV += range * 0.1; 
  }
  
  drawGraphScaleLabels(x, y, w, h, nf(maxV, 1, 3), nf(minV, 1, 3));
  
  float axisY = map(0, minV, maxV, y + h - 12, y + 22);
  axisY = constrain(axisY, y + 22, y + h - 12);
  stroke(PANEL_BORDER, 100);
  strokeWeight(1);
  line(x + 6, axisY, x + w - 28, axisY);
  
  int colorMapIdx = 0;
  for (int s = startStream; s < endStream; s++) {
    noFill();
    stroke(colors[colorMapIdx % colors.length]);
    strokeWeight(1);
    beginShape();
    for (int i = 0; i < maxPoints; i++) {
      int idx = (histIdx + i) % maxPoints;
      float val = data[s][idx];
      if (Float.isNaN(val) || Float.isInfinite(val)) val = 0;
      
      float graphX = map(i, 0, maxPoints, x + 6, x + w - 28);
      float graphY = map(val, minV, maxV, y + h - 12, y + 22);
      vertex(graphX, graphY);
    }
    endShape();
    colorMapIdx++;
  }
}

// ==========================================
// CENTER VIEWPORT: 3D MODEL RENDER SPACE
// ==========================================
void drawCenter3DModelSpace() {
  stroke(PANEL_BORDER);
  fill(5);
  rect(230, 120, 875, 575);
  
  pushMatrix();
  translate(230 + 875/2, 120 + 575/2, 150); 
  
  ambientLight(120, 120, 120);
  directionalLight(255, 255, 255, 0.5, 1, -1);
  
  // Apply rotations in CORRECT order: Yaw -> Roll -> Pitch
  // This matches how your sensor data is oriented
  
  // Yaw (rotate around vertical axis - FIXED inversion)
  rotateY(radians(yaw - yawOffset));  // Removed the negative sign
  
  // Roll (rotate around X axis)
  rotateZ(radians(roll - rollOffset));
  
  // Pitch (rotate around Y axis)
  rotateX(radians(pitch - pitchOffset));
  
  stroke(40);
  fill(240); 
  strokeWeight(1);
  
  int segments = 32;
  float radius = 35;
  float length = 260;
  
  // Render Rocket Body Cylinder
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= segments; i++) {
    float angle = TWO_PI * i / segments;
    float x = cos(angle) * radius;
    float z = sin(angle) * radius;
    vertex(x, -length/2, z);
    vertex(x, length/2, z);
  }
  endShape();
  
  // Render Rocket Nosecone
  pushMatrix();
  translate(0, -length/2, 0);
  fill(210, 50, 50); 
  cone(radius, 65);
  popMatrix();
  
  // Render Rocket Fins
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
// RIGHT PANEL: UTILITY SYSTEM INTERFACES
// ==========================================
void drawRightMetadataPanel() {
  int panelX = 1115;
  int panelW = 155;

  drawPanelOutline(panelX, 120, panelW, 145, "System Commands");
  drawInteractiveButton(panelX + 10, 145, panelW - 20, 26, "ZERO GYROS");
  drawInteractiveButton(panelX + 10, 182, panelW - 20, 26, "RESET FLIGHT");
  drawLaunchButton(panelX + 10, 219, panelW - 20, 26, "LAUNCH SEQUENCE");

  drawPanelOutline(panelX, 275, panelW, 420, "Raw Telemetry");
  
  boolean isConnected = useSerial && (millis() - lastPacketTime < 2000);
  
  if (isConnected) {
    fill(LINE_GREEN); 
  } else {
    fill(LAUNCH_RED);  
  }
  noStroke();
  rect(panelX + 10, 298, panelW - 20, 24); 
  
  fill(0); 
  textAlign(CENTER, CENTER);
  textSize(9);
  if (isConnected) {
    text("STATUS: CONNECTED", panelX + (panelW / 2), 310);
  } else {
    text("STATUS: DISCONNECTED", panelX + (panelW / 2), 310);
  }
  
  float secSinceLast = (millis() - lastPacketTime) / 1000.0;
  if (!useSerial) secSinceLast = 0.0; 
  
  textAlign(CENTER, TOP);
  textSize(9);
  if (isConnected) {
    fill(LINE_GREEN);
    text("TIMEOUT: " + nf(secSinceLast, 1, 1) + "s", panelX + (panelW / 2), 327);
  } else {
    fill(LAUNCH_RED);
    text("TIMEOUT: " + nf(secSinceLast, 1, 1) + "s", panelX + (panelW / 2), 327);
  }
  
  int startY = 344; 
  int spacing = 16;
  textSize(10);
  fill(TEXT_MAIN);
  textAlign(LEFT, TOP);
  
  text("Pitch: " + nf(pitch, 1, 2) + "°", panelX + 10, startY);
  text("Roll:  " + nf(roll, 1, 2) + "°", panelX + 10, startY + spacing);
  text("Yaw:   " + nf(yaw, 1, 2) + "°", panelX + 10, startY + (spacing*2));
  
  text("accX:  " + nf(accelX, 1, 2) + " g", panelX + 10, startY + (spacing*3));
  text("accY:  " + nf(accelY, 1, 2) + " g", panelX + 10, startY + (spacing*4));
  text("accZ:  " + nf(accelZ, 1, 2) + " g", panelX + 10, startY + (spacing*5));
  
  text("vX:  0.16 g", panelX + 10, startY + (spacing*6));
  text("vY: -0.19 g", panelX + 10, startY + (spacing*7));
  text("vZ: -93.70 g", panelX + 10, startY + (spacing*8));
  
  text("pX: -1.41 m", panelX + 10, startY + (spacing*9));
  text("pY:  2.82 m", panelX + 10, startY + (spacing*10));
  text("Alt: " + nf(altitude, 1, 1) + " ft", panelX + 10, startY + (spacing*11));
}

// ==========================================
// MOUSE INTERACTION HANDLING
// ==========================================
void mousePressed() {
  int panelX = 1115;
  
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 135 && mouseY >= 145 && mouseY <= 145 + 26) {
    pitchOffset = pitch; rollOffset = roll; yawOffset = yaw;
    println("Command Broadcasted: IMU Orientation Axes Zeroed out.");
    if(useSerial && port != null) port.write("CMD,ZERO_IMU\n"); 
  }
  
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 135 && mouseY >= 182 && mouseY <= 182 + 26) {
    packetCount = 0; altitude = 0; previousAlt = 0;
    isLaunched = false;
    flightState = "GROUND IDLE";
    println("Command Broadcasted: Purging local telemetry cache frames.");
    if(useSerial && port != null) port.write("CMD,RESET_FLIGHT\n");
  }
  
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 135 && mouseY >= 219 && mouseY <= 219 + 26) {
    if (!isLaunched) {
      isLaunched = true;
      launchTimeMarker = millis();
      flightState = "POWERED FLIGHT / BOOST";
      println("CRITICAL COMMAND: Launch Sequence Initiated!");
      if(useSerial && port != null) port.write("CMD,LAUNCH\n");
    }
  }
}

// ==========================================
// STANDALONE DRAW DRAWING BLOCKS UTILS
// ==========================================
void drawPanelOutline(int x, int y, int w, int h, String title) {
  stroke(PANEL_BORDER);
  strokeWeight(1);
  fill(0);
  rect(x, y, w, h);
  
  fill(TEXT_MUTED);
  textSize(10); 
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
  if (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h) {
    fill(BTN_HOVER);
  } else {
    fill(BTN_BG);
  }
  stroke(PANEL_BORDER);
  rect(x, y, w, h, 3);
  
  fill(TEXT_MAIN);
  textAlign(CENTER, CENTER);
  textSize(9);
  text(label, x + w/2, y + h/2 - 1);
}

void drawLaunchButton(int x, int y, int w, int h, String label) {
  if (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h) {
    fill(LAUNCH_HOVER);
  } else {
    fill(LAUNCH_RED);
  }
  stroke(PANEL_BORDER);
  rect(x, y, w, h, 3);
  
  fill(0); 
  textAlign(CENTER, CENTER);
  textSize(9);
  text(label, x + w/2, y + h/2 - 1);
}

// ==========================================
// BACKGROUND TELEMETRY EMULATOR ENGINE
// ==========================================
void simulateTelemetry() {
  pitch = 8 * sin(frameCount * 0.03);
  roll  = 5 * cos(frameCount * 0.02);
  yaw   = (frameCount * 0.2) % 360;
  
  accelX = random(-0.02, 0.02);
  accelY = random(-0.02, 0.02);
  accelZ = 1.0 + random(-0.01, 0.01);
  
  if (isLaunched) {
    altitude += random(0.8, 2.2); 
  } else {
    altitude = 0.0 + random(-0.02, 0.02); 
  }
  
  histIdx = (histIdx + 1) % maxPoints;
  gyroHist[0][histIdx] = pitch;
  gyroHist[1][histIdx] = roll;
  gyroHist[2][histIdx] = yaw;
  
  accelHist[0][histIdx] = accelX;
  accelHist[1][histIdx] = accelY;
  accelHist[2][histIdx] = accelZ;
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
    }
  }
  
  previousAlt = altitude;
  
  if (frameCount % 4 == 0) packetCount++;
}

void calculateTlmRate() {
  int currentTime = millis();
  if (currentTime - lastHzCheckTime >= 1000) {
    tlmRate = (packetCount - lastPacketCount) / ((currentTime - lastHzCheckTime) / 1000.0);
    lastPacketCount = packetCount;
    lastHzCheckTime = currentTime;
  }
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
