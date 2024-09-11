/**
 * Sketch for capturing Kinect depth image data, performing blob detection, and sending out blob coordinates/size over OSC.  
 *
 * See:
 * https://soft-lab.com/project/position-tracking/
 * 
 */

import oscP5.*;
import netP5.*;
import controlP5.*;
import KinectPV2.*;
import blobDetection.*;

// Options
boolean bDrawDepth = true;
boolean bDrawBlobs = true;
boolean bDrawEdges = true;
boolean bBroadcastOSC = true;
boolean bPerformanceMode = false;

// OSC
OscP5 oscClient;
NetAddress oscTargetNetAddr;
float debugPosX;

// Kinect
KinectPV2 kinect;
int kinectDepthMin = 100;  // 0.1m
int kinectDepthMax = 4000; // 4.0m
int kinectResX = 512;
int kinectResY = 424;
float kinectAspectRatio = 1.21;

// Blob
BlobDetection blobDetector;
int blobDetectArea = 100;

// GUI
int displayWidth = 1024;  //1920; //1024;
int displayHeight = 648;  //1080; //648;
int appWidth = displayWidth;
int appHeight = displayHeight;
int imgResX = displayWidth/2;
int imgResY = round(imgResX/kinectAspectRatio);

ControlP5 cp5;
PFont uiFont;
PImage kinImg;
PGraphics kinGfx;
PGraphics blobGfx;

void setup() {
  frameRate(25);
  size(displayWidth, displayHeight);
  surface.setResizable(true);
  registerMethod("pre", this);
  
  kinImg = new PImage(kinectResX, kinectResY);
  kinGfx = createGraphics(kinectResX, kinectResY);
  blobGfx = createGraphics(kinectResX, kinectResY);

  // OSC
  oscClient = new OscP5(this,7001);
  oscTargetNetAddr = new NetAddress("127.0.0.1",7000);
  debugPosX = 1;
  
  // Kinect
  kinect = new KinectPV2(this);
  kinect.enableDepthImg(true);
  kinect.enablePointCloud(true);
  kinect.enableColorImg(true);
  kinect.init();
  
  // UI
  cp5 = new ControlP5(this);
  uiFont = createFont("Arial",20,true);
  setupUI();

  // Blob
  blobDetector = new BlobDetection(kinImg.width, kinImg.height);
  blobDetector.setPosDiscrimination(true);
  blobDetector.setThreshold(0.1f); 
}

void pre() {
  // If app was resized, update rendering
  if (appWidth != width || appHeight != height) {
    appWidth = width; //<>//
    appHeight = height;
    imgResX = width/2;
    imgResY = round(imgResX/kinectAspectRatio);
  }
}

void draw() {
  background(0);
  textFont(uiFont);
  text(frameRate, 50, height - 50);
  
  // Set thresholds and pull image from Kinect
  kinect.setLowThresholdPC(kinectDepthMin);
  kinect.setHighThresholdPC(kinectDepthMax);
  kinImg = kinect.getPointCloudDepthImage();
  
  // Create PGraphics and send pixels to blob detection
  kinGfx.beginDraw();
  kinGfx.background(0);
  kinGfx.image(kinImg,0,0);
  kinGfx.endDraw();
  blobDetector.computeBlobs(kinGfx.pixels);
  
  ProcessResults(bDrawDepth, bDrawBlobs, bDrawEdges, bBroadcastOSC, bPerformanceMode);
}

void ProcessResults(boolean drawDepth, boolean drawBlobs, boolean drawEdges, boolean broadcastOSC, boolean performanceMode)
{
  // Draw kinect depth field
  if(drawDepth)
  {
    if(performanceMode) {
      image(kinGfx, imgResX,0); 
    } else {
      //Resize to screen
      PImage kinResizeImg = kinGfx.get();
      kinResizeImg.resize(imgResX, imgResY);
      image(kinResizeImg,imgResX,0); 
    }
  }
  
  Blob b;
  EdgeVertex eA,eB;
  FloatList blobCoords = new FloatList();
  blobGfx.beginDraw();
  blobGfx.clear();
  blobGfx.noFill();
  blobGfx.strokeWeight(1);
  blobGfx.stroke(0,255,255);
  for (int n=0 ; n<blobDetector.getBlobNb() ; n++)
  {
    b=blobDetector.getBlob(n);
    if (b!=null)
    {
      // Edges
      if(drawEdges)
      {
        blobGfx.strokeWeight(1);
        blobGfx.stroke(0,255,255);
        for (int m=0; m < b.getEdgeNb(); m++)
        {
          eA = b.getEdgeVertexA(m);
          eB = b.getEdgeVertexB(m);
          if (eA != null && eB != null)
          {
            blobGfx.line(
              eA.x*kinImg.width, eA.y*kinImg.height, 
              eB.x*kinImg.width, eB.y*kinImg.height
            );
          }
        }
      }
      
      //Blobs
      if(b.w*kinImg.width > blobDetectArea && b.h*kinImg.height > blobDetectArea)
      {
        blobCoords.append(b.x);
        blobCoords.append(b.y);
        blobCoords.append(b.w);
        blobCoords.append(b.h);
        
        if(drawBlobs)
        {
          float cx = ((b.xMin*kinImg.width) + b.w*kinImg.width/2);
          float cy = ((b.yMin*kinImg.height) + b.h*kinImg.height/2);
        
          blobGfx.strokeWeight(1);
          blobGfx.stroke(255,0,0);
          blobGfx.rect(b.xMin*kinImg.width,b.yMin*kinImg.height,b.w*kinImg.width,b.h*kinImg.height);
          
          blobGfx.fill(255);
          blobGfx.noStroke();
          blobGfx.ellipse(cx,cy,10,10);
          
          blobGfx.noFill();
        }
      }
    }
  }
  blobGfx.endDraw();

  if(drawBlobs || drawEdges)
  {
    if(performanceMode) {
      image(blobGfx, imgResX, 0); 
    } else {
      //Resize to screen
      PImage blobResizeImg = blobGfx.get();
      blobResizeImg.resize(imgResX, imgResY);
      image(blobResizeImg, imgResX, 0); 
    }
  }

  // Broadcast blob data via OSC
  if(broadcastOSC)
  {
    //PrintCoords(float blobCoords) 
    SendBundle(blobCoords);
  }
}

void SendBundle(FloatList blobData) {
  int blobCount = (blobData.size() / 4);
  if(blobCount == 0)
    return;
  
  OscBundle bundle = new OscBundle();
  
  OscMessage headerMessage = new OscMessage("/header");
  headerMessage.add(""); // Empty
  headerMessage.add(blobCount); // Blob message count
  bundle.add(headerMessage);
  
  for (int n=0; n<blobCount; n++)
  {
    int bindex = n*4;
    OscMessage message = new OscMessage("/blob" + (n+1));
    message.add(new float[] {
      blobData.get(bindex), blobData.get(bindex+1),  //x,y
      blobData.get(bindex+2), blobData.get(bindex+3) //w,h
    });
    bundle.add(message);
  }
  
  oscClient.send(bundle, oscTargetNetAddr); 
}

void PrintCoords(FloatList blobCoords) 
{
    int blobCount = (blobCoords.size() / 4);
    println("BlobCount: " + blobCount);
    
    for (int n=0; n<blobCount; n++)
    {
      int bindex = n*4;
      println("N: " + n + ", " + bindex);
      println("x: " + blobCoords.get(bindex));
      println("y: " + blobCoords.get(bindex+1));
      println("w: " + blobCoords.get(bindex+2));
      println("h: " + blobCoords.get(bindex+3));
    }
}

void TestDataSend() {
  OscBundle bundle = new OscBundle();
  
  OscMessage headerMessage = new OscMessage("/header");
  headerMessage.add(""); // Empty
  headerMessage.add(3); // Blob message count
  bundle.add(headerMessage);
  
  OscMessage m1 = new OscMessage("/blob1");
  m1.add(debugPosX); // x pos
  m1.add(3.0);  // y pos
  m1.add(2.0);  // x size
  m1.add(1.0);  // y size
  bundle.add(m1);
  debugPosX = debugPosX + 1.0;
  
  OscMessage m2 = new OscMessage("/blob2");
  m2.add(new float[] {1.0, 100.0, 1.0, 1.0});
  bundle.add(m2);

  oscClient.send(bundle, oscTargetNetAddr); 
}

void oscEvent(OscMessage message) {
  println("Recieved message but I am only a client!");
}

void setupUI()
{
  cp5.addSlider("kinectDepthMin")
    .setFont(uiFont)
    .setPosition(50,60)
    .setRange(0,4000)
    .setSize(200,20)
    .setValue(100)
    .setColorForeground(color(20,200,200))
     .setColorLabel(color(255))
     .setColorBackground(color(70,70,70))
     .setColorValue(color(0,0,0))
     .setColorActive(color(0,255,255));
     
  cp5.addSlider("kinectDepthMax")
    .setFont(uiFont)
    .setPosition(50,100)
    .setRange(0,4000)
    .setSize(200,20)
    .setValue(3000)
    .setColorForeground(color(20,200,200))
     .setColorLabel(color(255))
     .setColorBackground(color(70,70,70))
     .setColorValue(color(0,0,0))
     .setColorActive(color(0,255,255));
  
  cp5.addSlider("blobDetectArea")
    .setFont(uiFont)
    .setLabel("Detect Area")
    .setPosition(50,140)
    .setRange(0,500)
    .setSize(200,20)
    .setValue(100)
    .setColorForeground(color(20,200,200))
     .setColorLabel(color(255))
     .setColorBackground(color(70,70,70))
     .setColorValue(color(0,0,0))
     .setColorActive(color(0,255,255));

  cp5.addTextlabel("bDrawDepthLabel", "Depth", 70, 177).setFont(uiFont);
  cp5.addToggle("bDrawDepth")
    .setLabel("")
    .setPosition(50,180)
    .setSize(20,20)
    .setColorForeground(color(20,20,20))
     .setColorBackground(color(70,70,70))
     .setColorValue(0xffff88ff)
     .setColorActive(color(0,200,200));
     

  cp5.addTextlabel("bDrawBlobsLabel", "Blobs", 70, 217).setFont(uiFont);
  cp5.addToggle("bDrawBlobs")
    .setLabel("")
    .setPosition(50,220)
    .setSize(20,20)
    .setColorForeground(color(20,20,20))
     .setColorBackground(color(70,70,70))
     .setColorValue(0xffff88ff)
     .setColorActive(color(0,200,200));    
     
  cp5.addTextlabel("bDrawEdgesLabel", "Edges", 70, 257).setFont(uiFont);
  cp5.addToggle("bDrawEdges")
    .setLabel("")
    .setPosition(50,260)
    .setSize(20,20)
    .setColorForeground(color(20,20,20))
     .setColorBackground(color(70,70,70))
     .setColorValue(0xffff88ff)
     .setColorActive(color(0,200,200));
  
  cp5.addTextlabel("bBroadcastOSCLabel", "Broadcast", 70, 297).setFont(uiFont);
  cp5.addToggle("bBroadcastOSC")
    .setLabel("")
    .setPosition(50,300)
    .setSize(20,20)
    .setColorForeground(color(20,20,20))
     .setColorBackground(color(70,70,70))
     .setColorValue(0xffff88ff)
     .setColorActive(color(0,200,200));
     
  cp5.addTextlabel("bPerformanceModeLabel", "Performance Mode", 70, 337).setFont(uiFont);
  cp5.addToggle("bPerformanceMode")
    .setLabel("")
    .setPosition(50,340)
    .setSize(20,20)
    .setColorForeground(color(20,20,20))
     .setColorBackground(color(70,70,70))
     .setColorValue(0xffff88ff)
     .setColorActive(color(0,200,200));     
}
