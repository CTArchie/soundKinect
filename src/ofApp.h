#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"


class ofApp: public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    void draw3d(ofVec3f pos, ofColor colorBoxA);
    void draw3dB(ofVec3f pos, ofColor colorBoxB);
    
    void keyPressed(int key);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    
    // this is the definition of a custom function that we will use to turn kinect data into a 3d point cloud
    void drawPointCloud();
    
    // here we define an instance of the kinect object to talk to our kinect sensor
    ofxKinect kinect;
    
    // define some image objects to store our image data as we work
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    ofxCvContourFinder contourFinder;
    
    // define boolean options for how the data will be rendered on screen- it is good practice to begin boolean variabes with a b_ to make your code more readable
    bool b_ThreshWithOpenCV;
    bool b_DrawPointCloud;
    // define variables for filtering the data from the kinect
    int nearThreshold;
    int farThreshold;
    int angle;
    
    // define a new virtual camera to be used for viewing our 3d data onscreen
    ofEasyCam easyCam;
    
    ofRectangle rectangle;
    
    ofSoundPlayer drum;
    ofSoundPlayer gabba;
    ofSoundPlayer top;
    
    ofBoxPrimitive musicBox;
    ofBoxPrimitive musicBoxB;
    
    int boxSize;
    int halfBoxSize;
    
    ofColor boxA;
    ofColor boxBCol;
    
    
    ofSpherePrimitive sphere;
    
    
    // strings
    ofSoundPlayer aString;
    ofSoundPlayer bString;
    ofSoundPlayer cString;
    ofSoundPlayer dString;
    ofSoundPlayer eString;
    ofSoundPlayer fString;
    ofSoundPlayer gString;
    
    
    glm::vec3 aXY1, aXY2, bXY1, bXY2, cXY1, cXY2, dXY1, dXY2, eXY1, eXY2, fXY1, fXY2, gXY1, gXY2;
    
    bool xRange(unsigned high, unsigned low, unsigned x) {
        return  ((x - low) <= (high - low));
    }
    
    bool yRange(unsigned high, unsigned low, unsigned x) {
        return  ((x - low) <= (high - low));
    }
    
    bool zRange(unsigned high, unsigned low, unsigned x) {
        return  ((x - low) <= (high - low));
    }
    
    ofxPanel gui;
    ofxIntSlider sizeBox;
};
