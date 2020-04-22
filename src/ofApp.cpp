#include "ofApp.h"

/*
 If you are struggling to get the device to connect ( especially Windows Users )
 please look at the ReadMe: in addons/ofxKinect/README.md
 */

// Dan Buzzo 2018 - github.com/uwe-creative-technology/
// for UWE Bristol, Creative Technology MSc, Creative Technology Toolkit module 2018-19
// with modified portions from oF kinect example;

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetLogLevel(OF_LOG_VERBOSE);

    gui.setup("Parameters","settings.xml");
    gui.add(sizeBox.setup("Box Size", 20, 0, 100));

    

/*    drum.load("A.mp3");
    gabba.load("E.mp3");
    top.load("G.mp3");
    */

    aString.load("A.wav");
    bString.load("B.wav");
    cString.load("c.mp3");
    dString.load("d.mp3");
    eString.load("E.wav");
    fString.load("f.mp3");
    gString.load("g.mp3");
    

    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    // other options we could use
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();        // opens first available kinect
    // other options we could use
    //kinect.open(1);    // open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");    // open a kinect using it's unique serial #
    
    // send some setup info out to the console for debugging
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    // set up the size of our image buffers we are going to use -
    // we make them all the same width and height as the data we will get from the kinect
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    // set some values for filtering the data from the kinect
    nearThreshold = 255;
    farThreshold = 250;
    b_ThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    b_DrawPointCloud = false;
}

//--------------------------------------------------------------
void ofApp::update() {
    boxSize = sizeBox;
    halfBoxSize = boxSize / 2;

    sphere.set(20, 20);
    musicBox.set(boxSize);
    musicBoxB.set(boxSize);
    ofBackground(100, 100, 100);
    
    // get fresh data from the kinect
    kinect.update();
    
    // if there is a new frame from the kinect and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(b_ThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
    }
    
}

//--------------------------------------------------------------
void ofApp::draw() {

    
    
    ofSetColor(255, 255, 255);

/*

    glm::vec3 drumCircle;

    drumCircle.x = (kinect.width / 2) - 100;
    drumCircle.y = (kinect.height / 2);
    
    glm::vec3 gabbaCircle;

    gabbaCircle.x = (kinect.width / 2) + 100;
    gabbaCircle.y = (kinect.height / 2);
    
    glm::vec3 topCircle;

    topCircle.x = (kinect.width / 2) ;
    topCircle.y = (kinect.height / 2)- 100;*/

    aXY1.x = 75;
    aXY1.y = 50;
    aXY2.x = 75;
    aXY2.y = 450;
    
    bXY1.x = 150;
    bXY1.y = 50;
    bXY2.x = 150;
    bXY2.y = 450;
    
    cXY1.x = 225;
    cXY1.y = 50;
    cXY2.x = 225;
    cXY2.y = 450;
    
    dXY1.x = 300;
    dXY1.y = 50;
    dXY2.x = 300;
    dXY2.y = 450;
    
    eXY1.x = 375;
    eXY1.y = 50;
    eXY2.x = 375;
    eXY2.y = 450;
    
    fXY1.x = 450;
    fXY1.y = 50;
    fXY2.x = 450;
    fXY2.y = 450;
    
    gXY1.x = 525;
    gXY1.y = 50;
    gXY2.x = 525;
    gXY2.y = 450;


    if(b_DrawPointCloud) {
        
        // draw the data from the kinect as a 3d point cloud
        easyCam.begin();
        drawPointCloud();
        
        easyCam.end();
        
    } else {
        // draw from raw data from the live kinect
       // kinect.drawDepth(10, 10, 400, 300);
        
        kinect.draw(0,0,kinect.width,kinect.height);
        
        
       // grayImage.draw(0, 0, kinect.width, kinect.height);
        contourFinder.draw(0, 0, kinect.width, kinect.height);
        

        
        
        
        /*ofDrawCircle(drumCircle, 20);
        ofDrawCircle(gabbaCircle, 20);
        ofDrawCircle(topCircle, 20);*/

        // draw strings
        ofSetLineWidth(3);
        ofSetColor(0, 0, 255);
        ofDrawLine(aXY1, aXY2);
        ofSetColor(255, 0, 0);
        ofDrawLine(bXY1, bXY2);
        ofSetColor(0,255,0);
        ofDrawLine(cXY1, cXY2);
        ofSetColor(255, 255, 0);
        ofDrawLine(dXY1, dXY2);
        ofSetColor(0, 255, 255);
        ofDrawLine(eXY1, eXY2);
        ofSetColor(255, 0, 255);
        ofDrawLine(fXY1, fXY2);
        ofSetColor(255,127,127);
        ofDrawLine(gXY1, gXY2);
        ofSetColor(255, 255, 255);
        ofSetLineWidth(1);

        for (int i = 0; i < contourFinder.nBlobs; i++) {
            ofRectangle r = contourFinder.blobs.at(i).boundingRect;
            int rectStartX = r.x;
            int rectEndX = r.x + (r.width);
            int rectStartY = r.y;
            int rectEndY = r.y + (r.height);

            
            glm::vec3 rectCenter;

            rectCenter.x = r.x + (r.width / 2);
            rectCenter.y = r.y + (r.height / 2);

            ofDrawCircle(rectCenter, 10);

            for (int p = rectStartX; p < rectEndX; p += 1) {
                for (int q = rectStartY; q < rectEndY; q += 1) {
                    glm::vec3 point;
                    point.x = p;
                    point.y = q;

                    if ((point.x == aXY1.x) && (point.y > aXY1.y) && (point.y < aXY2.y)) {
                        if (!aString.isPlaying()) {
                            //int speed = ofMap(point.y, aXY1.y, aXY1.y, 1, 1.5);
                            aString.setSpeed(ofMap(point.y, 50, 450, 1.5, 0.5));
                            aString.play();
                        }
                    }
                    if ((point.x == bXY1.x) && (point.y > bXY1.y) && (point.y < bXY2.y)) {
                        if (!bString.isPlaying()) {
                            bString.setSpeed(ofMap(point.y, 50, 450, 1.5, 0.5));
                            bString.play();
                        }
                    }
                    if ((point.x == cXY1.x) && (point.y > cXY1.y) && (point.y < cXY2.y)) {
                        if (!cString.isPlaying()) {
                            cString.setSpeed(ofMap(point.y, 50, 450, 1.5, 0.5));
                            cString.play();
                        }
                    }
                    if ((point.x == dXY1.x) && (point.y > dXY1.y) && (point.y < dXY2.y)) {
                        if (!dString.isPlaying()) {
                            dString.setSpeed(ofMap(point.y, 50, 450, 1.5, 0.5));
                            dString.play();
                        }
                    }
                    if ((point.x == eXY1.x) && (point.y > eXY1.y) && (point.y < eXY2.y)) {
                        if (!eString.isPlaying()) {
                            eString.setSpeed(ofMap(point.y, 50, 450, 1.5, 0.5));
                            eString.play();
                        }
                    }
                    if ((point.x == fXY1.x) && (point.y > fXY1.y) && (point.y < fXY2.y)) {
                        if (!fString.isPlaying()) {
                            fString.setSpeed(ofMap(point.y, 50, 450, 1.5, 0.5));
                            fString.play();
                        }
                    }
                    if ((point.x == gXY1.x) && (point.y > gXY1.y) && (point.y < gXY2.y)) {
                        if (!gString.isPlaying()) {
                            gString.setSpeed(ofMap(point.y, 50, 450, 1.5, 0.5));
                            gString.play();
                        }
                    }
                }
            }

        /*
            if ((rectCenter.x == aXY1.x) && (aXY1.y < rectCenter.y) && (rectCenter.y < aXY2.y)) {
                if (!aString.isPlaying()) {
                    aString.play();
                }
            }
            if ((rectCenter.x == bXY1.x) && (bXY1.y < rectCenter.y) && (rectCenter.y < bXY2.y)) {
                if (!bString.isPlaying()) {
                    bString.play();
                }
            }
            if ((rectCenter.x == cXY1.x) && (cXY1.y < rectCenter.y) && (rectCenter.y < cXY2.y)) {
                if (!cString.isPlaying()) {
                    cString.play();
                }
            }
            if ((rectCenter.x == dXY1.x) && (dXY1.y < rectCenter.y) && (rectCenter.y < dXY2.y)) {
                if (!dString.isPlaying()) {
                    dString.play();
                }
            }
            if ((rectCenter.x == eXY1.x) && (eXY1.y < rectCenter.y) && (rectCenter.y < eXY2.y)) {
                if (!eString.isPlaying()) {
                    eString.play();
                }
            }
            if ((rectCenter.x == fXY1.x) && (fXY1.y < rectCenter.y) && (rectCenter.y < fXY2.y)) {
                if (!fString.isPlaying()) {
                    fString.play();
                }
            }
            if ((rectCenter.x == gXY1.x) && (gXY1.y < rectCenter.y) && (rectCenter.y < gXY2.y)) {
                if (!gString.isPlaying()) {
                    gString.play();
                }
            }
            */
        }
        //glScalef(1, -1, 1);
    }


    
    
    
    // draw instructions
    ofSetColor(255, 255, 255);
    stringstream reportStream; // make a stringstream object that we can put text
    
    // here we assemble a string of text giving us a readout of what the kinect data is doing
    // and put it into the stringstream object we just made
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
        << "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
    << "using opencv threshold = " << b_ThreshWithOpenCV <<" (press spacebar)" << endl
    << "set near threshold " << nearThreshold << " (press: + -)" << endl
    << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
    << ", fps: " << ofGetFrameRate() << endl
    << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
    
    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    // here we draw our report stringstream object to the screen
    ofDrawBitmapString(reportStream.str(), 20, 652);
    
    gui.draw();
}

//----------------------------------------------------------------
void ofApp::draw3d(ofVec3f pos,ofColor colorBoxA) {
    
    ofSetColor(colorBoxA);
    musicBox.drawWireframe();
    musicBox.draw();
    musicBox.setGlobalPosition(pos);
}
void ofApp::draw3dB(ofVec3f pos, ofColor colorBoxB) {
    ofSetColor(colorBoxB);
    
    musicBoxB.drawWireframe();
    musicBoxB.draw();
    musicBoxB.setGlobalPosition(pos);
}

// this is our custom function that we call to make a 3D point cloud from the raw kinect data
void ofApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    
    /*glm::vec3 boxPos;
    boxPos.x = 0;
    boxPos.y = 0;
    boxPos.z = 750;*/

    ofVec3f boxPos;
    boxPos.set(500, 500, 1750);

    ofVec3f boxPosB;
    boxPosB.set(0, 325, 1250);

    //glm::vec3 p;
    // make a new 3d mesh, called 'mesh'
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    // step through each row of the data from the kinect using a a loop inside a loop
    // this loops through each line (the inner x loop) and after each line steps down to the next line (the outer y loop)
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                // we get the kinect data for each pixel and change each point in our mesh to correspond to the x,y,z and colour data
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));

                //p = kinect.getWorldCoordinateAt(x,y);

                /*ofNoFill();
                ofDrawRectangle(p, 20, 20);
                ofFill();*/

            }
        }
    }
    
    
    vector<glm::vec3>& verts = mesh.getVertices();


    boxA.r = 100;
    boxA.g = 255;
    boxA.b = 100;
    boxA.a = 50;

    boxBCol.r = 100;
    boxBCol.g = 200;
    boxBCol.b = 100;
    boxBCol.a = 50;

    glPointSize(3); // this sets the size of the dots we use when we draw the mesh as a 3Dpoint cloud
    
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();
    
    // here we draw our mesh object to the screen
    mesh.drawVertices();
    
    
    
    for (int G = 0; G < verts.size(); G += 1) {

        int zTrue;
        int yTrue;
        int xTrue;

        xRange(boxPos.x + halfBoxSize, boxPos.x - halfBoxSize, verts[G].x) ? xTrue = 1 : xTrue = 0;
        yRange(boxPos.y + halfBoxSize, boxPos.y - halfBoxSize, verts[G].y) ? yTrue = 1 : yTrue = 0;
        zRange(boxPos.z + halfBoxSize, boxPos.z - halfBoxSize, verts[G].z) ? zTrue = 1 : zTrue = 0;
        
        if ((zTrue == 1) && (yTrue == 1) && (xTrue == 1)) {
            boxA.r = 99;
            boxA.g = 0;
            boxA.b = 255;

            if (!cString.isPlaying()) {
cString.play();
            }
            
        }

        int zTrueB;
        int yTrueB;
        int xTrueB;

        xRange(boxPosB.x + halfBoxSize, boxPosB.x - halfBoxSize, verts[G].x) ? xTrueB = 1 : xTrueB = 0;
        yRange(boxPosB.y + halfBoxSize, boxPosB.y - halfBoxSize, verts[G].y) ? yTrueB = 1 : yTrueB = 0;
        zRange(boxPosB.z + halfBoxSize, boxPosB.z - halfBoxSize, verts[G].z) ? zTrueB = 1 : zTrueB = 0;

        if ((zTrueB == 1) && (yTrueB == 1) && (xTrueB == 1)) {
            boxBCol.r = 200;
            boxBCol.g = 0;
            boxBCol.b = 100;
            if (!dString.isPlaying()) {
dString.play();
            }
            
        }
    }
    draw3d(boxPos,boxA);
draw3dB(boxPosB,boxBCol);
    ofDisableDepthTest();
    ofPopMatrix();

    
    
}

//--------------------------------------------------------------
void ofApp::exit() {
    // when we quit our app we remember to close the connection to the kinect sensor
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    // this is a case statement -
    // rather than dozens of if - then statements we can say - get the keypress from the keyboard and do one from this list depending on the key value.
    switch (key) {
        case ' ':
            b_ThreshWithOpenCV = !b_ThreshWithOpenCV;
            break;
            
        case'p':
            b_DrawPointCloud = !b_DrawPointCloud;
            break;
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;
            
        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
            
             case 'f':
            ofToggleFullscreen();
            break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
    
}
