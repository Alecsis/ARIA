import processing.serial.*;
import java.nio.ByteBuffer; 
import java.nio.ByteOrder;  

// ==========================================
// CONFIGURATION & SWITCHES
// ==========================================
Serial port;
boolean useSerial = true; 
String serialPortName = "COM3"; 
int baudRate = 115200;
float altMin = 0;
float altMax = 10;
float altGroundOffset = 0;
boolean altInitialized = false;
int lastPacketTime = 0;
int lastReconnectAttempt = 0; 
int debugCounter = 0; // Added missing global variable for loop tracking

// Offscreen buffer for 2D UI elements to prevent 3D clipping
PGraphics hud;

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
  frameRate(165);
  UI_Font_Bold = createFont("SF Pro Text Bold", 13, true);
  hud = createGraphics(1280, 720, JAVA2D);
  hud.beginDraw();
  hud.textFont(UI_Font_Bold);
  hud.endDraw();
  
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
    checkSerialWatchdog(); 
  } else {
    simulateTelemetry();
  }
  calculateTlmRate();
  updateMissionStopwatch();
  
  // LAYER 1: Draw the 3D rocket model space natively in the center viewport
  drawCenter3DModelSpace();
  
  // LAYER 2: Draw the HUD over the graphics matrix channel
  hud.beginDraw();
  hud.background(0, 0); 
  drawTopHorizontalGraphs();
  drawLeftVerticalGraphs();
  drawRightMetadataPanel();
  drawTopBranding();
  hud.endDraw();
  
  image(hud, 0, 0);
}

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
// SERIAL PACKET STREAM LOOP PARSER
// ==========================================
void serialEvent(Serial p) {
  while (p.available() > 0) {
    String data = p.readStringUntil('\n');
    if (data != null) {
      data = trim(data);
      
      debugCounter++;
      if (debugCounter % 50 == 0) {
        println("RX: " + data);
      }
      
      if (data.startsWith("#")) return;
      
      // Parse Slash Delimited Orientation Line
      if (data.contains("/")) {
        String[] parts = split(data, '/');
        if (parts.length >= 3) {
          try {
            roll = float(parts[0]);   
            pitch = float(parts[1]);  
            yaw = float(parts[2]);    
            
            gyroHist[0][histIdx] = pitch;
            gyroHist[1][histIdx] = roll;
            gyroHist[2][histIdx] = yaw;
            histIdx = (histIdx + 1) % maxPoints;
            
            packetCount++;
            lastPacketTime = millis();
          } catch (Exception e) {
            println("Parse error: " + data);
          }
        }
      }
      
      // Parse Explicit Altitude Header Line
      else if (data.contains("Alt(ft):")) {
        try {
          String altStr = data.replace("Alt(ft):", "").trim();
          altitude = float(altStr);
          altHist[histIdx] = altitude;
          
          if (altitude > 2.0 && !isLaunched) {
            isLaunched = true;
            launchTimeMarker = millis();
            println("🚀 LAUNCH DETECTED! Altitude: " + altitude + " ft");
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
        } catch (Exception e) {
          println("Altitude parse error: " + data);
        }
      }
    }
  }
}

// ==========================================
// TIMING LOGIC UTILS
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
// THE TRADITIONAL HORIZONTAL HEADER UI STACK
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
  drawMultiStreamGraphWithAxis(148, startY, graphW, graphH, accelHist, 3, new color[]{LINE_GREEN, LINE_RED, LINE_YELLOW});

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
  
  hud.textAlign(CENTER, CENTER);
  hud.textSize(24);
  hud.fill(TEXT_MAIN);
  hud.text("AVIA", brandX + (panelW / 2), brandY + 38);
  
  hud.textSize(11);
  hud.fill(TEXT_MUTED);
  hud.text("Ground Control", brandX + (panelW / 2), brandY + 68);
}

// ==========================================
// VERTICAL LEFT MULTI-PANEL DATA TRACKS
// ==========================================
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
  int leftX = 20;
  int rightX = 120;
  
  hud.textAlign(LEFT, TOP);
  hud.text("AVI Batt: " + nf(aviBatt, 1, 2) + " V", leftX, 175);
  hud.text("Prop Batt: " + nf(propBatt, 1, 2) + " V", leftX, 192);
  hud.text("VN IMU T: " + nf(tempC, 1, 1) + " °C", leftX, 209);
  hud.text("Baro T:   15.4 °C", leftX, 226);
  
  hud.text("Signal: " + signalDbm + " dBm", rightX, 175);
  hud.text("TLM Rate: " + nf(tlmRate, 1, 1) + " Hz", rightX, 192);

  int itemH = 110;
  drawPanelOutline(10, 330, panelW, itemH, "Altitude");
  drawSingleStreamGraphWithAxis(10, 330, panelW, itemH, altHist, LINE_YELLOW, "ft"); 

  drawPanelOutline(10, 450, panelW, itemH, "XY Position");
  drawStaticBaselineGraph(10, 450, panelW, itemH);

  drawPanelOutline(10, 570, panelW, itemH, "XY Velocity");
  drawStaticBaselineGraph(10, 570, panelW, itemH);
}

// ==========================================
// RENDER MATH FOR STREAM GRAPH HISTORIES
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
  hud.stroke(PANEL_BORDER, 120);
  hud.line(x + 6, axisY, x + w - 45, axisY);
  
  hud.noFill();
  hud.stroke(clr);
  hud.strokeWeight(1.2);
  hud.beginShape();
  for (int i = 0; i < maxPoints; i++) {
    int idx = (histIdx + i) % maxPoints;
    float val = data[idx];
    float graphX = map(i, 0, maxPoints, x + 6, x + w - 45);
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
  if (abs(range) < 0.001) { minV -= 1.0; maxV += 1.0; } 
  else { minV -= range * 0.1; maxV += range * 0.1; }
  
  drawGraphScaleLabels(x, y, w, h, nf(maxV, 1, 3), nf(minV, 1, 3));
  
  float axisY = map(0, minV, maxV, y + h - 12, y + 22);
  axisY = constrain(axisY, y + 22, y + h - 12);
  hud.stroke(PANEL_BORDER, 120);
  hud.line(x + 6, axisY, x + w - 32, axisY);
  
  int colorMapIdx = 0;
  for (int s = startStream; s < endStream; s++) {
    hud.noFill();
    hud.stroke(colors[colorMapIdx % colors.length]);
    hud.strokeWeight(1);
    hud.beginShape();
    for (int i = 0; i < maxPoints; i++) {
      int idx = (histIdx + i) % maxPoints;
      float val = data[s][idx];
      float graphX = map(i, 0, maxPoints, x + 6, x + w - 32);
      float graphY = map(val, minV, maxV, y + h - 12, y + 22);
      hud.vertex(graphX, graphY);
    }
    hud.endShape();
    colorMapIdx++;
  }
}

void drawStaticBaselineGraph(int x, int y, int w, int h) {
  drawGraphScaleLabels(x, y, w, h, "0.0", "0.0");
  hud.stroke(PANEL_BORDER, 150);
  hud.line(x + 6, y + h - 25, x + w - 32, y + h - 25);
}

// ==========================================
// 3D MODEL VIEWPORT (LOCK ORIGIN CENTERED)
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
  rotateX(radians(-(roll - rollOffset)));   
  rotateZ(radians(-(pitch - pitchOffset))); 
  
  stroke(40);
  fill(240);
  strokeWeight(1);
  
  int segments = 32;
  float radius = 35;
  float length = 260;
  
  // Cylinder Body Tube
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= segments; i++) {
    float angle = TWO_PI * i / segments;
    float x = cos(angle) * radius;
    float z = sin(angle) * radius;
    vertex(x, -length/2, z);
    vertex(x, length/2, z);
  }
  endShape();
  
  // Apex Cone
  pushMatrix();
  translate(0, -length/2, 0);
  fill(210, 50, 50);
  cone(radius, -65);
  popMatrix();
  
  // Symmetrical Fins
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
// RIGHT PANEL COMMAND LAYOUTS & DETAILED RX TEXT
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
  if (isConnected) { hud.fill(LINE_GREEN); } else { hud.fill(LAUNCH_RED); }
  hud.noStroke();
  hud.rect(panelX + 10, 298, panelW - 20, 24);
  
  hud.fill(0);
  hud.textAlign(CENTER, CENTER);
  hud.textSize(9);
  hud.text(isConnected ? "STATUS: CONNECTED" : "STATUS: DISCONNECTED", panelX + (panelW / 2), 310);
  
  float secSinceLast = (millis() - lastPacketTime) / 1000.0;
  if (!useSerial) secSinceLast = 0.0;
  
  hud.textAlign(CENTER, TOP);
  hud.textSize(9);
  hud.fill(isConnected ? LINE_GREEN : LAUNCH_RED);
  hud.text("TIMEOUT: " + nf(secSinceLast, 1, 1) + "s", panelX + (panelW / 2), 327);
  
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
  
  hud.text("vX:    0.16 m/s", panelX + 10, startY + (spacing*6));
  hud.text("vY:   -0.19 m/s", panelX + 10, startY + (spacing*7));
  hud.text("vZ:   -93.70 m/s", panelX + 10, startY + (spacing*8));
  
  hud.text("pX:   -1.41 m", panelX + 10, startY + (spacing*9));
  hud.text("pY:    2.82 m", panelX + 10, startY + (spacing*10));
  hud.text("Alt:   " + nf(altitude, 1, 1) + " ft", panelX + 10, startY + (spacing*11));
}

// ==========================================
// MOUSE INTERACTION & BUTTON INTERFACE HANDLERS
// ==========================================
void mousePressed() {
  int panelX = 1105;
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 145 && mouseY >= 145 && mouseY <= 145 + 26) {
    pitchOffset = pitch; rollOffset = roll; yawOffset = yaw;
    println("Command Broadcasted: IMU Orientation Axes Zeroed out.");
    if(useSerial && port != null) port.write("CMD,ZERO_IMU\n"); 
  }
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 145 && mouseY >= 182 && mouseY <= 182 + 26) {
    packetCount = 0; altitude = 0; previousAlt = 0;
    isLaunched = false; flightState = "GROUND IDLE";
    println("Command Broadcasted: Purging local telemetry cache frames.");
    if(useSerial && port != null) port.write("CMD,RESET_FLIGHT\n");
  }
  if (mouseX >= panelX + 10 && mouseX <= panelX + 10 + 145 && mouseY >= 219 && mouseY <= 219 + 26) {
    if (!isLaunched) {
      isLaunched = true; launchTimeMarker = millis();
      flightState = "POWERED FLIGHT / BOOST";
      println("CRITICAL COMMAND: Launch Sequence Initiated!");
      if(useSerial && port != null) port.write("CMD,LAUNCH\n");
    }
  }
}

void drawPanelOutline(int x, int y, int w, int h, String title) {
  hud.stroke(PANEL_BORDER);
  hud.strokeWeight(1);
  hud.fill(0);
  hud.rect(x, y, w, h);
  
  hud.fill(TEXT_MUTED);
  hud.textSize(9);
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
  if (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h) { hud.fill(BTN_HOVER); } 
  else { fill(BTN_BG); }
  hud.stroke(PANEL_BORDER);
  hud.rect(x, y, w, h, 2);
  
  hud.fill(TEXT_MAIN);
  hud.textAlign(CENTER, CENTER);
  hud.textSize(9);
  hud.text(label, x + w/2, y + h/2 - 1);
}

void drawLaunchButton(int x, int y, int w, int h, String label) {
  if (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h) { hud.fill(LAUNCH_HOVER); } 
  else { fill(LAUNCH_RED); }
  hud.stroke(PANEL_BORDER);
  hud.rect(x, y, w, h, 2);
  
  hud.fill(0);
  hud.textAlign(CENTER, CENTER);
  hud.textSize(9);
  hud.text(label, x + w/2, y + h/2 - 1);
}

void simulateTelemetry() {
  pitch = 8 * sin(frameCount * 0.03);
  roll  = 5 * cos(frameCount * 0.02);
  yaw   = (frameCount * 0.2) % 360;
  
  accelX = random(-0.02, 0.02);
  accelY = random(-0.02, 0.02);
  accelZ = 1.0 + random(-0.01, 0.01);
  
  altitude = isLaunched ? altitude + random(0.8, 2.2) : random(-0.02, 0.02);
  if (altitude < 0) altitude = 0;
  
  histIdx = (histIdx + 1) % maxPoints;
  gyroHist[0][histIdx] = pitch;
  gyroHist[1][histIdx] = roll;
  gyroHist[2][histIdx] = yaw;
  
  accelHist[0][histIdx] = accelX;
  accelHist[1][histIdx] = accelY;
  accelHist[2][histIdx] = accelZ;
  altHist[histIdx] = altitude;
  
  if (frameCount % 4 == 0) packetCount++;
}

void cone(float r, float h) {
  int sides = 32;
  hud.beginShape(TRIANGLES);
  for (int i = 0; i < sides; i++) {
    float angle1 = TWO_PI * i / sides;
    float angle2 = TWO_PI * (i + 1) / sides;
    vertex(0, h, 0); 
    vertex(cos(angle1) * r, 0, sin(angle1) * r);
    vertex(cos(angle2) * r, 0, sin(angle2) * r);
  }
  hud.endShape();
  
  hud.beginShape(TRIANGLE_FAN);
  vertex(0, 0, 0);
  for (int i = 0; i <= sides; i++) {
    float angle = TWO_PI * i / sides;
    vertex(cos(angle) * r, 0, sin(angle) * r);
  }
  hud.endShape();
}
