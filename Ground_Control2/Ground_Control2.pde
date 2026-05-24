import processing.serial.*;
import java.nio.ByteBuffer; 
import java.nio.ByteOrder;  

// ==========================================
// CONFIGURATION & SWITCHES
// ==========================================
Serial port;
boolean useSerial = true; 
String serialPortName = "COM4"; 
int baudRate = 115200;
float altMin = 0;
float altMax = 10;
float altGroundOffset = 0;
boolean altInitialized = false;
int lastPacketTime = 0;
int lastReconnectAttempt = 0; 

// SEPARATE BUFFERS TO KILL THE VOID FOREVER
PGraphics hud;
PGraphics view3D;

// ==========================================
// TELEMETRY & RUNTIME STATE VARIABLES
// ==========================================
String flightState = "GROUND IDLE";
float pitch = 0, roll = 0, yaw = 0;
float altitude = 0, vSpeed = 0, tempC = 17.3;
float accelX = 0, accelY = 0, accelZ = 1.0; 
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

// Multiple Plot History Streams 
int maxPoints = 120;
float[][] gyroHist = new float[3][maxPoints]; 
float[][] accelHist = new float[3][maxPoints]; 
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
  
  // Initialize dedicated layer canvases
  hud = createGraphics(1280, 720, JAVA2D);
  hud.beginDraw();
  hud.textFont(UI_Font_Bold);
  hud.endDraw();
  
  // Create an isolated 3D viewport box matching your exact center window layout dimensions
  view3D = createGraphics(875, 575, P3D);
  
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
    lastPacketTime = millis(); 
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
  
  if (useSerial) {
    readSerialTelemetry();
    checkSerialWatchdog(); 
  } else {
    simulateTelemetry();
  }
  calculateTlmRate();
  updateMissionStopwatch();
  
  // LAYER 1: Draw the 3D Rocket inside its safe, isolated window space
  drawCenter3DModelSpace();
  image(view3D, 230, 120); // Composite the 3D window onto the dashboard
  
  // LAYER 2: Compute 2D Data over the top
  hud.beginDraw();
  hud.background(0, 0); // Keep alpha clear
  drawTopHorizontalGraphs();
  drawLeftVerticalGraphs();
  drawRightMetadataPanel();
  drawTopBranding();
  hud.endDraw();
  
  image(hud, 0, 0); // Composite HUD
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

// ==========================================
// BINARY TELEMETRY PARSER ENGINE
// ==========================================
void readSerialTelemetry() {
  if (port == null) return;

  while (port.available() > 0) {
    String inString = port.readStringUntil('\n');
    
    if (inString != null) {
      inString = trim(inString);
      
      try {
        if (inString.contains("Gyro (Pitch/Roll/Yaw):")) {
          String data = inString.replace("Gyro (Pitch/Roll/Yaw):", "").trim();
          float[] values = float(split(data, ','));
          if (values.length >= 3) {
            float rawGyroX = values[0]; 
            float rawGyroY = values[1]; 
            float rawGyroZ = values[2]; 
            
            int nowMillis = millis();
            float dt = (nowMillis - lastPacketTime) / 1000.0f;
            if (dt <= 0 || dt > 0.2) dt = 0.1f; 
            lastPacketTime = nowMillis;
            packetCount++;

            float pitch_acc = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.29578f;
            float roll_acc  = atan2(accelY, accelZ) * 57.29578f;
            
            pitch = 0.95f * (pitch + rawGyroX * dt) + 0.05f * pitch_acc;
            roll  = 0.95f * (roll  + rawGyroY * dt) + 0.05f * roll_acc;
            yaw  += rawGyroZ * dt; 

            if (roll > 180)  roll -= 360;  if (roll < -180)  roll += 360;
            if (pitch > 180) pitch -= 360; if (pitch < -180) pitch += 360;
            if (yaw > 180)   yaw -= 360;   if (yaw < -180)   yaw += 360;
            
            gyroHist[0][histIdx] = pitch;  
            gyroHist[1][histIdx] = roll;
            gyroHist[2][histIdx] = yaw;
          }
        } 
        else if (inString.contains("Accel(Pitch/Roll/Yaw):")) {
          String data = inString.replace("Accel(Pitch/Roll/Yaw):", "").trim();
          float[] values = float(split(data, ','));
          if (values.length >= 3) {
            accelX = values[0];
            accelY = values[1];
            accelZ = values[2];
            
            histIdx = (histIdx + 1) % maxPoints;
            accelHist[0][histIdx] = accelX;
            accelHist[1][histIdx] = accelY;
            accelHist[2][histIdx] = accelZ;
          }
        } 
        else if (inString.contains("Alt(ft):")) {
          String data = inString.replace("Alt(ft):", "").trim();
          altitude = float(data);
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
        }
        
      } catch (Exception e) {
        println("String Format Parse Error: " + e.getMessage());
      }
    }
  }
}

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
// HUD DRAW DATA OVERLAYS
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
  
  hud.textAlign(CENTER, CENTER);
  hud.textSize(24);
  hud.fill(TEXT_MAIN);
  hud.text("AVIA", brandX + (panelW / 2), brandY + 38);
  
  hud.textSize(11);
  hud.fill(TEXT_MUTED);
  hud.text("Ground Control", brandX + (panelW / 2), brandY + 68);
}

void drawLeftVerticalGraphs() {
  int panelW = 210;
  
  hud.stroke(PANEL_BORDER);
  hud.fill(0);
  hud.rect(10, 120, panelW, 200);
  
  if (isLaunched) {
    hud.fill(LAUNCH_RED);
  } else {
    hud.fill(STATE_BLUE);
  }
  hud.noStroke();
  hud.rect(15, 125, panelW - 10, 35);
  
  hud.fill(0);
  hud.textAlign(CENTER, CENTER);
  hud.textSize(11);
  hud.text(flightState + " [" + stopwatchStr + "]", 15 + (panelW-10)/2, 142);
  
  hud.fill(TEXT_MAIN);
  hud.textSize(11);
  int textX = 20;
  
  hud.textAlign(LEFT, TOP);
  hud.text("AVI Batt:  " + nf(aviBatt, 1, 2) + " V", textX, 170);
  hud.text("Prop Batt: " + nf(propBatt, 1, 2) + " V", textX, 185);
  hud.text("VN IMU T:  " + nf(tempC, 1, 1) + " °C", textX, 200);
  hud.text("Baro T:    " + "15.4 °C", textX, 215);
  
  hud.textAlign(RIGHT, TOP);
  hud.text("Signal: " + signalDbm + " dBm", textX + 190, 170);
  hud.text("TLM Rate: " + nf(tlmRate, 1, 1) + " Hz", textX + 190, 185);

  int graphH = 115;
  drawPanelOutline(10, 330, panelW, graphH, "Altitude");
  drawSingleStreamGraphWithAxis(10, 330, panelW, graphH, altHist, LINE_YELLOW, "ft"); 
  
  drawPanelOutline(10, 455, panelW, graphH, "XY Position");
  drawGraphScaleLabels(10, 455, panelW, graphH, "0.0", "0.0");
  hud.stroke(PANEL_BORDER, 90);
  hud.line(10 + 6, 455 + graphH / 2 + 5, 10 + panelW - 42, 455 + graphH / 2 + 5);
  
  drawPanelOutline(10, 580, panelW, graphH, "XY Velocity");
  drawGraphScaleLabels(10, 580, panelW, graphH, "0.0", "0.0");
  hud.stroke(PANEL_BORDER, 90);
  hud.line(10 + 6, 580 + graphH / 2 + 5, 10 + panelW - 42, 580 + graphH / 2 + 5);
}

// ==========================================
// DYNAMIC PLOT DRAW ENGINES
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
    minV -= 1.0; maxV += 1.0; 
  } else { 
    minV -= range * 0.1; maxV += range * 0.1; 
  }
  
  drawGraphScaleLabels(x, y, w, h, nf(maxV, 1, 3) + " " + unit, nf(minV, 1, 3) + " " + unit);
  
  float axisY = map(0, minV, maxV, y + h - 12, y + 22);
  axisY = constrain(axisY, y + 22, y + h - 12);
  hud.stroke(PANEL_BORDER, 100);
  hud.strokeWeight(1);
  hud.line(x + 6, axisY, x + w - 42, axisY);
  
  hud.noFill();
  hud.stroke(clr);
  hud.strokeWeight(1.2);
  hud.beginShape();
  for (int i = 0; i < maxPoints; i++) {
    int idx = (histIdx + i) % maxPoints;
    float val = data[idx];
    if (Float.isNaN(val) || Float.isInfinite(val)) val = 0;
    
    float graphX = map(i, 0, maxPoints, x + 6, x + w - 42);
    float graphY = map(val, minV, maxV, y + h - 12, y + 22);
    hud.vertex(graphX, graphY);
  }
  hud.endShape();
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
    minV -= 1.0; maxV += 1.0; 
  } else { 
    minV -= range * 0.1; maxV += range * 0.1; 
  }
  
  drawGraphScaleLabels(x, y, w, h, nf(maxV, 1, 3), nf(minV, 1, 3));
  
  float axisY = map(0, minV, maxV, y + h - 12, y + 22);
  axisY = constrain(axisY, y + 22, y + h - 12);
  hud.stroke(PANEL_BORDER, 100);
  hud.strokeWeight(1);
  hud.line(x + 6, axisY, x + w - 28, axisY);
  
  int colorMapIdx = 0;
  for (int s = startStream; s < endStream; s++) {
    hud.noFill();
    hud.stroke(colors[colorMapIdx % colors.length]);
    hud.strokeWeight(1);
    hud.beginShape();
    for (int i = 0; i < maxPoints; i++) {
      int idx = (histIdx + i) % maxPoints;
      float val = data[s][idx];
      if (Float.isNaN(val) || Float.isInfinite(val)) val = 0;
      
      float graphX = map(i, 0, maxPoints, x + 6, x + w - 28);
      float graphY = map(val, minV, maxV, y + h - 12, y + 22);
      hud.vertex(graphX, graphY);
    }
    hud.endShape();
    colorMapIdx++;
  }
}

// ==========================================
// CENTER VIEWPORT: 3D MODEL RENDER SPACE
// ==========================================
void drawCenter3DModelSpace() {
  view3D.beginDraw();
  view3D.background(5); // Internal background fill cleanly clears depth buffer inside box
  
  // Set up view border bounds inside its own canvas matrix 
  view3D.stroke(PANEL_BORDER);
  view3D.noFill();
  view3D.rect(0, 0, view3D.width - 1, view3D.height - 1);
  
  view3D.pushMatrix();
  view3D.translate(view3D.width / 2, view3D.height / 2, 0); 
  
  view3D.ambientLight(120, 120, 120);
  view3D.directionalLight(255, 255, 255, 0.5, 1, -1);
  
  // --- FIXED WORLD EXTRINSIC ROTATION WITH SWAPPED AXES ---
  view3D.rotateY(radians(yaw - yawOffset));
  view3D.rotateX(radians(-(roll - rollOffset)));   // Swapped roll value into pitch axis
  view3D.rotateZ(radians(-(pitch - pitchOffset))); // Swapped pitch value into roll axis
  // ============================================================
  
  view3D.stroke(40);
  view3D.fill(240); 
  view3D.strokeWeight(1);
  
  int segments = 32;
  float radius = 35;
  float length = 260;
  float coneHeight = 65;
  
  // Render Rocket Body Cylinder
  view3D.beginShape(QUAD_STRIP);
  for (int i = 0; i <= segments; i++) {
    float angle = TWO_PI * i / segments;
    float x = cos(angle) * radius;
    float z = sin(angle) * radius;
    view3D.vertex(x, -length/2, z);
    view3D.vertex(x, length/2, z);
  }
  view3D.endShape();
  
  // Render Rocket Nosecone - Fixed placement transform & normal profile heights
  view3D.pushMatrix();
  view3D.translate(0, -length/2 - coneHeight, 0);
  view3D.fill(210, 50, 50); 
  coneHelper(radius, coneHeight); 
  view3D.popMatrix();
  
  // Render Rocket Fins
  view3D.fill(70);
  for(int i = 0; i < 4; i++) {
    view3D.pushMatrix();
    view3D.rotateY(HALF_PI * i);
    view3D.translate(radius + 10, length/2 - 10, 0);
    view3D.box(25, 10, 8);
    view3D.popMatrix();
  }
  
  view3D.popMatrix();
  view3D.endDraw();
}

// Fixed nested matrix cone renderer targeting the local 3D view buffer
void coneHelper(float r, float h) {
  int sides = 32;
  
  view3D.beginShape(TRIANGLES);
  for (int i = 0; i < sides; i++) {
    float angle1 = TWO_PI * i / sides;
    float angle2 = TWO_PI * (i + 1) / sides;
    
    view3D.vertex(0, -h, 0); // Corrected inverted apex vertex offset direction
    view3D.vertex(cos(angle1) * r, 0, sin(angle1) * r);
    view3D.vertex(cos(angle2) * r, 0, sin(angle2) * r);
  }
  view3D.endShape();
  
  view3D.beginShape(TRIANGLE_FAN);
  view3D.vertex(0, 0, 0);
  for (int i = 0; i <= sides; i++) {
    float angle = TWO_PI * i / sides;
    view3D.vertex(cos(angle) * r, 0, sin(angle) * r);
  }
  view3D.endShape();
}

// ==========================================
// RIGHT PANEL: METADATA & CONTROL DECK
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
    hud.fill(LINE_GREEN); 
  } else {
    hud.fill(LAUNCH_RED);  
  }
  hud.noStroke();
  hud.rect(panelX + 10, 298, panelW - 20, 24); 
  
  hud.fill(0); 
  hud.textAlign(CENTER, CENTER);
  hud.textSize(9);
  if (isConnected) {
    hud.text("STATUS: CONNECTED", panelX + (panelW / 2), 310);
  } else {
    hud.text("STATUS: DISCONNECTED", panelX + (panelW / 2), 310);
  }
  
  float secSinceLast = (millis() - lastPacketTime) / 1000.0;
  if (!useSerial) secSinceLast = 0.0; 
  
  hud.textAlign(CENTER, TOP);
  hud.textSize(9);
  if (isConnected) {
    hud.fill(LINE_GREEN);
    hud.text("TIMEOUT: " + nf(secSinceLast, 1, 1) + "s", panelX + (panelW / 2), 327);
  } else {
    hud.fill(LAUNCH_RED);
    hud.text("TIMEOUT: " + nf(secSinceLast, 1, 1) + "s", panelX + (panelW / 2), 327);
  }
  
  int startY = 344; 
  int spacing = 16;
  hud.textSize(10);
  hud.fill(TEXT_MAIN);
  hud.textAlign(LEFT, TOP);
  
  hud.text("Pitch: " + nf(pitch, 1, 2) + "°", panelX + 10, startY);
  hud.text("Roll:  " + nf(roll, 1, 2) + "°", panelX + 10, startY + spacing);
  hud.text("Yaw:   " + nf(yaw, 1, 2) + "°", panelX + 10, startY + (spacing*2));
  
  hud.text("accX:  " + nf(accelX, 1, 2) + " g", panelX + 10, startY + (spacing*3));
  hud.text("accY:  " + nf(accelY, 1, 2) + " g", panelX + 10, startY + (spacing*4));
  hud.text("accZ:  " + nf(accelZ, 1, 2) + " g", panelX + 10, startY + (spacing*5));
  
  hud.text("vX:  0.16 m/s", panelX + 10, startY + (spacing*6));
  hud.text("vY: -0.19 m/s", panelX + 10, startY + (spacing*7));
  hud.text("vZ: -93.70 m/s", panelX + 10, startY + (spacing*8));
  
  hud.text("pX: -1.41 m", panelX + 10, startY + (spacing*9));
  hud.text("pY:  2.82 m", panelX + 10, startY + (spacing*10));
  hud.text("Alt: " + nf(altitude, 1, 1) + " ft", panelX + 10, startY + (spacing*11));
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
  hud.stroke(PANEL_BORDER);
  hud.strokeWeight(1);
  hud.fill(0);
  hud.rect(x, y, w, h);
  
  hud.fill(TEXT_MUTED);
  hud.textSize(10); 
  hud.textAlign(LEFT, TOP);
  hud.text(title, x + 8, y + 6);
}

void drawGraphScaleLabels(int x, int y, int w, int h, String maxStr, String minStr) {
  hud.fill(TEXT_MUTED);
  hud.textSize(8);
  hud.textAlign(RIGHT, TOP);
  hud.text(maxStr, x + w - 6, y + 6);
  hud.textAlign(RIGHT, BOTTOM);
  hud.text(minStr, x + w - 6, y + h - 6);
}

void drawInteractiveButton(int x, int y, int w, int h, String label) {
  if (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h) {
    hud.fill(BTN_HOVER);
  } else {
    hud.fill(BTN_BG);
  }
  hud.stroke(PANEL_BORDER);
  hud.rect(x, y, w, h, 3);
  
  hud.fill(TEXT_MAIN);
  hud.textAlign(CENTER, CENTER);
  hud.textSize(9);
  hud.text(label, x + w/2, y + h/2 - 1);
}

void drawLaunchButton(int x, int y, int w, int h, String label) {
  if (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h) {
    hud.fill(LAUNCH_HOVER);
  } else {
    hud.fill(LAUNCH_RED);
  }
  hud.stroke(PANEL_BORDER);
  hud.rect(x, y, w, h, 3);
  
  hud.fill(0); 
  hud.textAlign(CENTER, CENTER);
  hud.textSize(9);
  hud.text(label, x + w/2, y + h/2 - 1);
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
