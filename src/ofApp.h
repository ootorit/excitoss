#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxBox2d.h"
#include "ofxGui.h"
#include "ofxXmlSettings.h"
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include "sys/time.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    bool bThreshWithOpenCV;


    
    int centerX[10];
    int centerY[10];
    string center;
    int rightCenterX[10];
    int rightCenterY[10];
    string rightCenter;
    
    bool titleFlag;
    bool grayFlag;
    bool rsmFlag;
    int counter;
    int beat_circle_r;
    
    ofImage titleImg;
    ofImage aniImg;
    
    int tempo_speed[100];
    int tempo_radius[100];
    int tempo_y[100];
    int tempo_num;
 //   int tempo_list[1000]={379,420,460,500,542,584,626,680,720,730,740,765,805,815,825,850,890,900,910,935,975,985,995,1000,1180,1222,1264,1306,1370,1392,1413,1434,1456,1478,1497,1519,1541,1560,1581,1602,1622,1643,1664,1685,1706,1727,1748,1769,1790,1811,1832,1853,1874,1895,1916,1937,1958,1979,2000,2021}
    std::vector<int> tempo_list;
    
//    int tempo_list[1000]={7423,8123,8830,9549,13048,13847,14467,15227,15893,16648,17333,18016,18712,19482,20132,20758,21534,22282,23047,23778,25833,26076,26434,28649,28970,29300,31415,31749,32051,34119,34451,34792,35749,36382,37084,37737,38398,39132,39867,40587};
    
    clock_t startClock,nowClock;
    
    //Kinect用配列
    ofxKinect kinect;
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    ofxCvContourFinder contourFinder;
    
    ofxKinect kinect2;
    ofxCvColorImage colorImg2;
    ofxCvGrayscaleImage grayImage2; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear2; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar2; // the far thresholded image
    ofxCvContourFinder contourFinder2;
    
    //box2d
    ofxBox2d box2d;
    ofPolyline line,groundLine,centerLine,badCupLine,rightLine,rightBadCupLine;
    ofPtr<ofxBox2dPolygon> cup,centerWall,badCup,rightCup,rightBadCup;
//    std::vector<ofPtr< ofxBox2dPolygon > > groundCups;
    std::vector<ofPtr<ofxBox2dCircle> > circles;
    
    struct Ball{
        ofPtr<ofxBox2dCircle> circle;
        int color;
        //独自で奥行きをもたせて、描く大きさに反映
        float z;
        float zVelocity;
        float zAcceleration;
        Ball(ofPtr<ofxBox2dCircle> thisCircle , int thisColor =0xffffff){
            circle = thisCircle;
            color = thisColor;
            z = 0;
        };
        float baseRadius;
        int score;
    };
    std::vector<ofPtr<Ball> > balls;
    

    ofImage backgroundImage;
    ofImage backgroundImageBlue;
    ofImage backgroundImageRed;
    ofImage frontImage;
    ofImage guideball;
  
    //検知された手が新しいものか検知用
    std::vector<ofPtr<ofRectangle> > preRectangles;
    std::vector<ofPtr<ofRectangle> > preRightRectangles;
    bool detectRectangleCollision( const ofRectangle& firstRectangle , const ofRectangle& secondRectangle);
    
    bool rhysm_flag;
    bool game_flag;
    ofSoundPlayer mySound;
    
    //衝突判定変更用
    b2Filter goodFilter;
    b2Filter badFilter;
    b2Filter initFilter;
    
    typedef enum{
        kCategoryBitTypeA = 0x0001,
        kCategoryBitTypeB = 0x0002,
        kCategoryBitTypeC = 0x0004,
        kCategoryBitTypeD = 0x0008,
     }CategoryBits;
    
    int redNumber;
    int blueNumber;
        ofTrueTypeFont numberFont;
    ofTrueTypeFont numberFont2;
    
    bool redWinFlag;
    bool blueWinFlag;
    bool drawFlag;
    
    
    //GUIを用いたkinectのパラメータ調整用
    bool parameterMode = false;
    ofxPanel panel;
    ofxSlider <int> minDistance;
    ofxSlider <int> maxDistance;
    ofxSlider <int> ofFrameRate;
    
    ofxSlider <int> minDetectSize;
    ofxSlider <int> maxDetectSize;
    ofxSlider <int> maxDetectNumber;
    ofxSlider <int> tiltAngle;
    ofxSlider <int> farThreshold;
    ofxSlider <int> nearThreshold;
    ofxSlider <int> farThresholdDistance;
    ofxSlider <int> nearThresholdDistance;
    ofxSlider <float> lowPassValue;
    ofxSlider <int> blurValue;
    //しきい値を距離に変換するため
    float rateOfthresholdToDistance;
    
    

    int nearThreshold2;
    int farThreshold2;

    
    
    void onMinDistanceChanged(int& num);
    void onMaxDistanceChanged(int& num);
    

    void onOfFrameRateChanged(int& num);
    void onTiltAngleChanged(int& num);
    //値が変わった時に保存だけすればよい変数のリスナ
    void onValueChanged(int& num);
    void onNearThresholdDistanceChanged(int& num);
    void onFarThresholdDistanceChanged(int& num);
    void onLowPassValueChanged(float& num);
    
    //周知fiterSize分の輝度情報を元に平滑化
    void medianFilter( ofxCvGrayscaleImage& image, unsigned int filterSize);
    

    struct timeval start,now;
};




