#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
   redNumber = 0;
  blueNumber = 0;
    
    titleImg.loadImage("logo.png");
    aniImg.loadImage("nijicon_guideball.gif");
    
    beat_circle_r = 30;
    counter = 0;
    rsmFlag = false;
    titleFlag = false;
    grayFlag = false;
    redWinFlag = false;
    blueWinFlag = false;
    drawFlag = false;
    
    mySound.loadSound("nijion.wav");
    mySound.setLoop(false);

    tempo_num = 100;
    numberFont.loadFont("OstrichSans-Heavy.otf",108);
    numberFont2.loadFont("OstrichSans-Heavy.otf",108);
    
    for(int i=0;i<tempo_num;i++){
        tempo_radius[i]=10;
        tempo_y[i]=-10*tempo_list[i];
        tempo_speed[i]=10;
    }
    

    ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    //***kinect***//
    
    // enable depth->video image calibration
    //open kinect1
    kinect.setRegistration(true);
    kinect.init();
    kinect.open(0);		// opens first available kinect
    
    //open kinect2
    kinect2.setRegistration(true);
    kinect2.init();
    kinect2.open(1);		// opens first available kinect
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }

    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);

    colorImg2.allocate(kinect2.width, kinect2.height);
    grayImage2.allocate(kinect2.width, kinect2.height);
    grayThreshNear2.allocate(kinect2.width, kinect2.height);
    grayThreshFar2.allocate(kinect2.width, kinect2.height);
    
    //Threshold
    nearThreshold = 255;
    farThreshold = 185;
    nearThreshold2 = 255;
    farThreshold2 = 190;
    blobThreshold = 1000;
    bThreshWithOpenCV = true;
    
    ofSetWindowShape(1920,1080);
    ofSetFrameRate(30);
    
    // zero the tilt on startup
    angle = 30;
    kinect.setCameraTiltAngle(angle);
    kinect2.setCameraTiltAngle(angle);
    
    //***Box2D***//
    
    box2d.init();
    box2d.setGravity( 0 , 9.8 );
    box2d.setFPS(10.0);
    
    box2d.registerGrabbing();
    box2d.createGround(0,ofGetHeight(),ofGetWidth(),ofGetHeight());
    line.addVertex(0.0362 * ofGetWidth(), 0.2583 * ofGetHeight());
    line.addVertex(0.0362 * ofGetWidth(), 0.6725 * ofGetHeight());
    line.addVertex(0.242 * ofGetWidth() , 0.8735 * ofGetHeight());
    line.addVertex(0.4427 * ofGetWidth() , 0.6735 * ofGetHeight() );
    line.addVertex(0.4427 * ofGetWidth(), 0.2583 * ofGetHeight());
    line.addVertex(0.4427 * ofGetWidth() - 2 , 0.2583 * ofGetHeight() - 2);
    line.addVertex(0.4427 * ofGetWidth() - 2 , 0.6725 * ofGetHeight() - 2 );
    line.addVertex(0.242 * ofGetWidth() , 0.8735 * ofGetHeight() - 2);
    line.addVertex(0.0362 * ofGetWidth() + 3   , 0.6725 * ofGetHeight() - 2);
    line.addVertex(0.0362 * ofGetWidth() + 1 , 0.2583*ofGetHeight() - 1 );
    
    
//    line.addVertex(ofGetWidth()/2-160, ofGetHeight()-50);
//    line.addVertex(ofGetWidth()/2-160, 200);
//    line.addVertex(ofGetWidth()/2-150, 200);
//    line.addVertex(ofGetWidth()/2-150, ofGetHeight()-40);
//    line.addVertex(150, ofGetHeight()-40);
    line.close();
    cup = ofPtr<ofxBox2dPolygon>(new ofxBox2dPolygon);
    cup.get()->addVertexes(line);
    cup.get()->triangulatePoly(10);
    cup.get()->setPhysics(0.0, 0.5, 0.1);
    cup->fixture.filter.categoryBits = kCategoryBitTypeA;
    cup.get()->create(box2d.getWorld());
    
    
    rightLine.addVertex(0.0362 * ofGetWidth() + ofGetWidth() / 1.93, 0.2583 * ofGetHeight());
    rightLine.addVertex(0.0362 * ofGetWidth() + ofGetWidth() / 1.93,  0.6725 * ofGetHeight());
    rightLine.addVertex(0.242 * ofGetWidth()  + ofGetWidth() / 1.93, 0.8735 * ofGetHeight());
    rightLine.addVertex(0.4427 * ofGetWidth()  + ofGetWidth() / 1.93, 0.6735 * ofGetHeight() );
    rightLine.addVertex(0.4427 * ofGetWidth() + ofGetWidth() / 1.93, 0.2583 * ofGetHeight());
    rightLine.addVertex(0.4427 * ofGetWidth() - 2  + ofGetWidth() / 1.93, 0.2583 * ofGetHeight() - 2);
    rightLine.addVertex(0.4427 * ofGetWidth() - 2 + ofGetWidth() / 1.93 , 0.6725 * ofGetHeight() - 2 );
    rightLine.addVertex(0.242 * ofGetWidth()  + ofGetWidth() / 1.93, 0.8735 * ofGetHeight() - 2);
    rightLine.addVertex(0.0362 * ofGetWidth() + 3   + ofGetWidth() / 1.93 , 0.6725 * ofGetHeight() - 2);
    rightLine.addVertex(0.0362 * ofGetWidth() + 1  + ofGetWidth() / 1.93, 0.2583*ofGetHeight() - 1 );
    
    
    //    line.addVertex(ofGetWidth()/2-160, ofGetHeight()-50);
    //    line.addVertex(ofGetWidth()/2-160, 200);
    //    line.addVertex(ofGetWidth()/2-150, 200);
    //    line.addVertex(ofGetWidth()/2-150, ofGetHeight()-40);
    //    line.addVertex(150, ofGetHeight()-40);
    rightLine.close();
    rightCup = ofPtr<ofxBox2dPolygon>(new ofxBox2dPolygon);
    rightCup.get()->addVertexes(rightLine);
    rightCup.get()->triangulatePoly(10);
    rightCup.get()->setPhysics(0.0, 0.5, 0.1);
    rightCup->fixture.filter.categoryBits = kCategoryBitTypeA;
    rightCup.get()->create(box2d.getWorld());

    
    centerLine.addVertex(ofGetWidth()/2 -1 ,0);
    centerLine.addVertex(ofGetWidth()/2 -1 ,ofGetHeight());
    centerLine.addVertex(ofGetWidth()/2 + 1 ,ofGetHeight());
    centerLine.addVertex(ofGetWidth()/2 + 1 ,0);
    centerLine.close();
    centerWall = ofPtr<ofxBox2dPolygon>(new ofxBox2dPolygon);
    centerWall.get()->addVertexes(centerLine);
    centerWall.get()->triangulatePoly(10);
    centerWall.get()->setPhysics(0.0, 0.5, 0.1);
    centerWall->fixture.filter.categoryBits = kCategoryBitTypeC;
    centerWall.get()->create(box2d.getWorld());

    
    
    badCupLine.addVertex(0.0375*ofGetWidth(),0.3611*ofGetHeight());
    badCupLine.addVertex(0.2398*ofGetWidth(),0.5514*ofGetHeight());
    badCupLine.addVertex(0.4429*ofGetWidth(),0.3611*ofGetHeight());
    badCupLine.addVertex(0.4429*ofGetWidth() -2 ,0.3611*ofGetHeight() - 2);
    badCupLine.addVertex(0.2398*ofGetWidth(),0.5514*ofGetHeight() - 2);
    badCupLine.addVertex(0.0375*ofGetWidth() + 2 ,0.3611*ofGetHeight() - 2);
    badCupLine.close();
    
    
    
    badCup = ofPtr<ofxBox2dPolygon>(new ofxBox2dPolygon);
    badCup.get()->addVertexes(badCupLine);
    badCup.get()->setPhysics(0.0, 0.5, 0.1);
    badCup.get()->fixture.filter.categoryBits = kCategoryBitTypeB;
    badCup.get()->create(box2d.getWorld());
    
    rightBadCupLine.addVertex(0.0375*ofGetWidth() + ofGetWidth() / 1.93 ,0.3611*ofGetHeight());
    rightBadCupLine.addVertex(0.2398*ofGetWidth() + ofGetWidth() / 1.93,0.5514*ofGetHeight());
    rightBadCupLine.addVertex(0.4429*ofGetWidth()+ ofGetWidth() / 1.93,0.3611*ofGetHeight());
    rightBadCupLine.addVertex(0.4429*ofGetWidth() + ofGetWidth() / 1.93-2 ,0.3611*ofGetHeight() - 2);
    rightBadCupLine.addVertex(0.2398*ofGetWidth()+ ofGetWidth() / 1.93,0.5514*ofGetHeight() - 2);
    rightBadCupLine.addVertex(0.0375*ofGetWidth()+ ofGetWidth() / 1.93 + 2 ,0.3611*ofGetHeight() - 2);
    rightBadCupLine.close();
    
    rightBadCup = ofPtr<ofxBox2dPolygon>(new ofxBox2dPolygon);
    rightBadCup.get()->addVertexes(rightBadCupLine);
    rightBadCup.get()->setPhysics(0.0, 0.5, 0.1);
    rightBadCup.get()->fixture.filter.categoryBits = kCategoryBitTypeB;
    rightBadCup.get()->create(box2d.getWorld());

    
    groundLine.addVertex(0,kinect.height);
    groundLine.addVertex(ofGetWidth(),kinect.height);
    
    backgroundImage.loadImage("background.png");
    backgroundImageRed.loadImage("background_red.png");
    backgroundImageBlue.loadImage("background_blue.png");
    frontImage.loadImage("front.png");
    //衝突判定用

    goodFilter.categoryBits = kCategoryBitTypeA;
    goodFilter.maskBits = kCategoryBitTypeA + kCategoryBitTypeC;

    badFilter.categoryBits = kCategoryBitTypeB;
    badFilter.maskBits = kCategoryBitTypeB + kCategoryBitTypeC;
    
    initFilter.maskBits = 0x0000;
}

//--------------------------------------------------------------
void ofApp::update(){
    
    for(int i=0;i<tempo_num;i++){
        tempo_y[i] = tempo_y[i]+tempo_speed[i];
        if(tempo_y[i]<400 && tempo_y[i]>330){
            tempo_radius[i] = 60;
            rhysm_flag = true;
        }else{
            tempo_radius[i] = 30;
            rhysm_flag = false;
        }
    }
    
    //***kinect***//
    
    ofBackground(100, 100, 100);
    kinect.update();
    kinect2.update();
    
    // there is a new frame and we are connected
    if(kinect2.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage2.setFromPixels(kinect2.getDepthPixels(), kinect2.width, kinect2.height);
       // grayImage2.mirror(false,true);
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear2 = grayImage2;
            grayThreshFar2 = grayImage2;
            grayThreshNear2.threshold(nearThreshold2, true);
            grayThreshFar2.threshold(farThreshold2);
            cvAnd(grayThreshNear2.getCvImage(), grayThreshFar2.getCvImage(), grayImage2.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix2 = grayImage2.getPixels();
            
            int numPixels2 = grayImage2.getWidth() * grayImage2.getHeight();
            for(int i = 0; i < numPixels2; i++) {
                if(pix2[i] < nearThreshold2 && pix2[i] > farThreshold2) {
                    pix2[i] = 255;
                } else {
                    pix2[i] = 0;
                }
            }
        }
    }
    
    
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
      //  grayImage.mirror(false,true);
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage.getPixels();
            
            int numPixels = grayImage.getWidth() * grayImage.getHeight();
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
        // update the cv images
        grayImage2.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayImage, 2000, (kinect.width*kinect.height)/2, blobThreshold, false);
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder2.findContours(grayImage2, 2000, (kinect2.width*kinect2.height)/2, blobThreshold, false);
      
        for( int i=0; i<contourFinder.nBlobs; i++ ) {

            bool isDraw = false;
            //一つ前の時間に検知された長方形との衝突判定

            for ( std::vector<ofPtr <ofRectangle> >::iterator it = preRectangles.begin() ; it != preRectangles.end() ; it ++ ){
                if ( detectRectangleCollision(contourFinder.blobs[i].boundingRect, **it )){
                    isDraw = true;

                }
            }
            if( !isDraw ){
                //add center cross and center
                centerX[i] = contourFinder.blobs[i].centroid.x;
                centerY[i] = contourFinder.blobs[i].centroid.y;
                center = "blob"+ofToString(i+1)+" "+ofToString(centerX[i])+","+ofToString(centerY[i]);
                cout << center << endl;
                
                //ball
                float r = ofRandom(40,80);
                //ひとまずtimingはランダム
                //score == 0 bad timing
                //score == 1 good timing
                /*------------------------------------
                 手動でタイミングをいじる場合、それに応じてスコアが変わるようにここを変えて下さい。
                 --------------------------------------*/
                int score = (int)ofRandom(0,10000) % 3;
                if(rsmFlag){
                    score = 1;
                    cout << "good" << endl;
                }
                if(score > 0){
                    score = 1;
                }
                if(score == 1){
                    redNumber++;
                }
                
                nowClock = clock();
                //tempo_list/30 のうちもっとも近い値を持ってくる
                float nowTime = (float)(nowClock - startClock)/CLOCKS_PER_SEC;
                for ( int i = 0 ; i < 60 ; i ++ ){
                    if( nowTime < (tempo_list[i]/30.0)){
                        
                    }
                    else if(i>1){
                        //どちらかが0.2秒以内ならgood scoreを1にする
                        float diff_1 = abs(nowTime-tempo_list[i]/30.0);
                        float diff_2 = abs(nowTime-tempo_list[i-1]/30.0);
                        if( diff_1 < 0.2 || diff_2 < 0.2 ){
                            score = 1;
                        }
                    }
                }
                

                balls.push_back(ofPtr<Ball>(new Ball( ofPtr<ofxBox2dCircle>(new ofxBox2dCircle()) )));
//                glBlendFunc(GL_SRC_ALPHA, GL_ONE);


                balls.back().get()->circle->setPhysics(3.0, 0.5, 0.1);
                balls.back().get()->score = score;

                int colorRandom = ofRandom(0,7);
                switch( colorRandom ){
                    case 0:
                        balls.back().get()->color = 0xe60012;
                        break;
                    case 1:
                        balls.back().get()->color = 0xf39800;
                        break;
                    case 2:
                        balls.back().get()->color = 0xfff100;
                        break;
                    case 3:
                        balls.back().get()->color = 0x009944;
                        break;
                    case 4:
                        balls.back().get()->color = 0x0068b7;
                        break;
                    case 5:
                        balls.back().get()->color = 0x1d2088;
                        break;
                    case 6:
                        balls.back().get()->color = 0x920783;
                        break;
                }
                //ひとまず左右からボールがでてくるように設定
                int launchHeight;
                int horizontalVelocity,verticalVelocity=-15;
                
                if( balls.back().get()->score == 0){
                    launchHeight = ofGetHeight()-10;
                }else{
                    launchHeight = ofGetHeight()/2;
                }
                

                

                balls.back().get()->circle->setup(box2d.getWorld(), centerX[i] * ofGetWidth() / ( 2 * kinect.width) ,launchHeight,r);


                horizontalVelocity = ( ofGetWidth() / 4.0 - centerX[i] * ofGetWidth() / ( 2 * kinect.width)  ) * 0.02  ;
                balls.back().get()->circle->setVelocity(horizontalVelocity, verticalVelocity);
                balls.back().get()->baseRadius = balls.back().get()->circle->getRadius();
                balls.back().get()->zVelocity = 1;
                if( balls.back().get()->score == 1){
                    balls.back().get()->circle->body->GetFixtureList()->SetFilterData(initFilter);
                }else if( balls.back().get()->score == 0){
                    balls.back().get()->circle->body->GetFixtureList()->SetFilterData(badFilter);
                }
 
            }
            
        }

        
    }
    preRectangles.clear();
    for( int k = 0 ; k < contourFinder.nBlobs ;  k ++ ){
        preRectangles.push_back(ofPtr<ofRectangle>( new ofRectangle(contourFinder.blobs[k].boundingRect)));
    }
    //ひとまずベタ書き
    for( int i=0; i<contourFinder2.nBlobs; i++ ) {
        bool isDraw = false;
        //一つ前の時間に検知された長方形との衝突判定
        
        for ( std::vector<ofPtr <ofRectangle> >::iterator it = preRightRectangles.begin() ; it != preRightRectangles.end() ; it ++ ){
            if ( detectRectangleCollision(contourFinder2.blobs[i].boundingRect, **it )){
                isDraw = true;
                
            }
        }
        if( !isDraw ){
            //add center cross and center
            rightCenterX[i] = contourFinder2.blobs[i].centroid.x;
            rightCenterY[i] = contourFinder2.blobs[i].centroid.y;
            rightCenter = "blob"+ofToString(i+1)+" "+ofToString(centerX[i])+","+ofToString(centerY[i]);
            cout << center << endl;
            
            //ball
            float r = ofRandom(40,80);
            /*------------------------------------
             手動でタイミングをいじる場合、それに応じてスコアが変わるようにここを変えて下さい。
             --------------------------------------*/
            int score = (int)ofRandom(0,10000) % 3;
            if(rsmFlag){
                score = 1;
                cout << "good" << endl;
            }
            if(score > 0){
                score = 1;
            }
            if(score == 1){
                blueNumber++;
            }
            
            nowClock = clock();
            //tempo_list/30 のうちもっとも近い値を持ってくる
            float nowTime = (float)(nowClock - startClock)/CLOCKS_PER_SEC;
            for ( int i = 0 ; i < 60 ; i ++ ){
                if( nowTime < (tempo_list[i]/30.0)){
                    
                }
                else if(i>1){
                    //どちらかが0.2秒以内ならgood scoreを1にする
                    float diff_1 = abs(nowTime-tempo_list[i]/30.0);
                    float diff_2 = abs(nowTime-tempo_list[i-1]/30.0);
                    if( diff_1 < 0.2 || diff_2 < 0.2 ){
                        score = 1;
                    }
                }
            }
            
            balls.push_back(ofPtr<Ball>(new Ball( ofPtr<ofxBox2dCircle>(new ofxBox2dCircle()) )));
            
            balls.back().get()->circle->setPhysics(3.0, 0.5, 0.1);
            balls.back().get()->score = score;
            
            int colorRandom = ofRandom(0,7);
            switch( colorRandom ){
                case 0:
                    balls.back().get()->color = 0xe60012;
                    break;
                case 1:
                    balls.back().get()->color = 0xf39800;
                    break;
                case 2:
                    balls.back().get()->color = 0xfff100;
                    break;
                case 3:
                    balls.back().get()->color = 0x009944;
                    break;
                case 4:
                    balls.back().get()->color = 0x0068b7;
                    break;
                case 5:
                    balls.back().get()->color = 0x1d2088;
                    break;
                case 6:
                    balls.back().get()->color = 0x920783;
                    break;
            }

            int launchHeight;
            int horizontalVelocity,verticalVelocity=-15;
            
            if( balls.back().get()->score == 0){
                launchHeight = ofGetHeight()-10;
            }else{
                launchHeight = ofGetHeight()/2;
            }
            
            
            
            
            balls.back().get()->circle->setup(box2d.getWorld(), rightCenterX[i] * ofGetWidth() / ( 2 * kinect2.width) + ofGetWidth()/1.93,launchHeight,r);
            
            horizontalVelocity = ( ofGetWidth() / 4.0 - rightCenterX[i] * ofGetWidth() / ( 2 * kinect2.width)  ) * 0.02  ;

            balls.back().get()->circle->setVelocity(horizontalVelocity, verticalVelocity);
            balls.back().get()->baseRadius = balls.back().get()->circle->getRadius();
            balls.back().get()->zVelocity = 1;
            if( balls.back().get()->score == 1){
                balls.back().get()->circle->body->GetFixtureList()->SetFilterData(initFilter);
            }else if( balls.back().get()->score == 0){
                balls.back().get()->circle->body->GetFixtureList()->SetFilterData(badFilter);
            }
            
        }
        
    }
    preRightRectangles.clear();
    for( int k = 0 ; k < contourFinder2.nBlobs ;  k ++ ){
        preRightRectangles.push_back(ofPtr<ofRectangle>( new ofRectangle(contourFinder2.blobs[k].boundingRect)));
    }
    


    /*-------------------------------------------------------------------
     毎updateごとにボールへ加える処理をここに書く
     -------------------------------------------------------------------*/
    //奥行きによってボールをだんだん小さくする
    for ( int i = 0 ; i < balls.size() ;i ++ ){
           balls[i].get()->zVelocity *= 0.98;

        if( balls[i].get()->zVelocity > 0.4 ){
            balls[i].get()->circle->setRadius( balls[i].get()->baseRadius *( balls[i].get()->zVelocity ));
        }

        if(balls[i].get()->circle->getVelocity().y > 0 ){
            if ( balls[i].get()->score  == 0){
                balls[i].get()->circle->body->GetFixtureList()->SetFilterData(badFilter);
            }
            else if  ( balls.back().get()->score == 1 ){
                balls[i].get()->circle->body->GetFixtureList()->SetFilterData(goodFilter);
            }

        }

    }

    //***Box2D***//
    //rsm_counter
    if(rsmFlag){
        if(counter>3){
            counter = 0;
            rsmFlag = false;
            cout << "false" << endl;
        }else{
            beat_circle_r = 60-10*counter;
            counter++;
        }
    }
    
    box2d.update();
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0,191,255);
    backgroundImage.draw(0,0, ofGetWidth(), ofGetHeight());
    
    if(redWinFlag){
        backgroundImageRed.draw(0,0, ofGetWidth(), ofGetHeight());
        numberFont.drawString("RED WIN!!", ofGetWidth()/4 - numberFont.stringWidth(ofToString("RED WIN!!")) /2, ofGetHeight()/2-100);
    }
    if(blueWinFlag){
        backgroundImageBlue.draw(0,0, ofGetWidth(), ofGetHeight());
        numberFont.drawString("BLUE WIN!!", ofGetWidth()*3/4 - numberFont.stringWidth(ofToString("BLUE WIN!!"))/2, ofGetHeight()/2-100);
    }
    if(drawFlag){
        numberFont.drawString("DRAW", ofGetWidth()/2 - numberFont.stringWidth(ofToString("DRAW"))/2, ofGetHeight()/2-100);
        
    }
    
    ofSetColor(255, 255, 255);
    if (grayFlag) {
        grayImage.draw(0, 0, 640, 480);
        grayImage2.draw(640, 0, 640, 480);
    }
    contourFinder.draw(0, 0, 640, 480);
    contourFinder2.draw(640, 0, 640, 480);
    //stringstream reportStream;
    string blob_num;
    
    //reportStream << nearThreshold << endl
    //<< farThreshold << endl;
    //ofDrawBitmapString(reportStream.str(), 20, 450);
    
    blob_num = "blobs ";
    for(int i=0;i<contourFinder.nBlobs;i++){
        blob_num += "[*]";
    }
//   ofDrawBitmapString(blob_num, 20, 20);
    
    ofSetColor(255, 0, 0);
    for( int i=0; i<contourFinder.nBlobs; i++ ) {
        ofLine(centerX[i]-5, centerY[i], centerX[i]+5, centerY[i]);
        ofLine(centerX[i], centerY[i]-5, centerX[i], centerY[i]+5);
    }
    
    
    //***Box2D***//

    for ( std::vector< ofPtr< Ball > >::iterator it = balls.begin() ; it != balls.end(); it ++ ){
        if( it->get()->circle->getVelocity().y >= -3 && it->get()->score == 1 ){
            ofFill();
            ofSetHexColor(it->get()->color);
            it->get()->circle->draw();
        }
    }

    
    ofSetHexColor(0xffffff);
    line.draw();
    centerLine.draw();
    groundLine.draw();
    box2d.draw();
     ofSetColor(255, 255, 255);
    frontImage.draw(0,0,ofGetWidth(),ofGetHeight());

    for ( std::vector< ofPtr< Ball > >::iterator it = balls.begin() ; it != balls.end(); it ++ ){
        if( it->get()->circle->getVelocity().y < -3 || it->get()->score == 0 ){
            ofFill();
            ofSetHexColor(it->get()->color);
            it->get()->circle->draw();
        }
    }
    ofSetColor(255, 255, 255);
    //sound//
//    ofFill();
//    for(int i=0;i<tempo_num;i++){
//        ofCircle(ofGetWidth()/2, tempo_y[i], tempo_radius[i]);
//    }

    numberFont2.drawString(ofToString(redNumber), 40 , 120);
    numberFont2.drawString(ofToString(blueNumber), ofGetWidth() - 40 - numberFont.stringWidth(ofToString(blueNumber)), 120);
    
    ofFill();
    ofCircle(ofGetWidth()/2, ofGetHeight()/2, beat_circle_r);
    if(titleFlag){
        titleImg.draw(290,120,700,480);
    }
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch(key){
        case ' ': //rhism counter
        {
            rsmFlag = true;
            cout << "true" << endl;
            break;
        }
        case 'g': //title
        {
            if (grayFlag) {
                grayFlag = false;
            }else{
                grayFlag = true;
            }
            break;
        }
        case 't': //title
        {
            if (titleFlag) {
                titleFlag = false;
            }else{
                titleFlag = true;
            }
            break;
        }
        case 'c':
        {
            //circles.push_back(ofPtr<ofxBox2dCircle>(new ofxBox2dCircle()));
            //circles.back().get()->setPhysics(3.0, 0.5, 0.1);
            //circles.back().get()->setup(box2d.getWorld(),mouseX,mouseY,r);
            
            
            //ball
            float r = ofRandom(40,80);
            //ひとまずtimingはランダム
            //score == 0 bad timing
            //score == 1 good timing
            int score = (int)ofRandom(0,10000) % 2;
            
            
            balls.push_back(ofPtr<Ball>(new Ball( ofPtr<ofxBox2dCircle>(new ofxBox2dCircle()) )));
            
            balls.back().get()->circle->setPhysics(3.0, 0.5, 0.1);
            balls.back().get()->score = score;
            
            int colorRandom = ofRandom(0,7);
            switch( colorRandom ){
                case 0:
                    balls.back().get()->color = 0xe60012;
                    break;
                case 1:
                    balls.back().get()->color = 0xf39800;
                    break;
                case 2:
                    balls.back().get()->color = 0xfff100;
                    break;
                case 3:
                    balls.back().get()->color = 0x009944;
                    break;
                case 4:
                    balls.back().get()->color = 0x0068b7;
                    break;
                case 5:
                    balls.back().get()->color = 0x1d2088;
                    break;
                case 6:
                    balls.back().get()->color = 0x920783;
                    break;
            }
            
            int launchHeight = 0;
            int horizontalVelocity,verticalVelocity=-15;
            
            if( balls.back().get()->score == 0){
                launchHeight = ofGetHeight()-10;
            }else{
                launchHeight = ofGetHeight()/2;
            }
            
            
            balls.back().get()->circle->setup(box2d.getWorld(), mouseX * ofGetWidth() / ( 2 * kinect.width) ,launchHeight,r);
            
            if( balls.back().get()->circle->getPosition().x >= ofGetWidth()/2 ){
                horizontalVelocity = ( ofGetWidth()  * 3/ 4.0 - mouseX * ofGetWidth() / ( 2 * kinect.width)  ) * 0.02  ;
            }
            else{
                horizontalVelocity = ( ofGetWidth() / 4.0 - mouseX * ofGetWidth() / ( 2 * kinect.width)  ) * 0.02  ;
            }
            balls.back().get()->circle->setVelocity(horizontalVelocity, verticalVelocity);
            balls.back().get()->baseRadius = balls.back().get()->circle->getRadius();
            balls.back().get()->zVelocity = 1;
            if( balls.back().get()->score == 1){
                balls.back().get()->circle->body->GetFixtureList()->SetFilterData(initFilter);
            }else if( balls.back().get()->score == 0){
                balls.back().get()->circle->body->GetFixtureList()->SetFilterData(badFilter);
            }
            break;
        }
        case 's':
        {
            mySound.stop();
            mySound.play();
            startClock = clock();
            redNumber = 0;
            blueNumber = 0;
            balls.clear();
            redWinFlag = false;
            blueWinFlag = false;
            drawFlag = false;
            break;
        }
        case 'q':
        {
            mySound.stop();
            break;
        }
        case 'e':
        {
            mySound.stop();
            if( redNumber > blueNumber){
                redWinFlag = true;
            }
            else if ( redNumber < blueNumber ){
                blueWinFlag = true;
            }
            else{
                //引き分け
                drawFlag = true;
            }
                break;
        }
        case '>':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
            
        case '.':
            farThreshold2 ++;
            if (farThreshold2 > 255) farThreshold2 = 255;
            break;
        case ',':
            farThreshold2 --;
            if (farThreshold2 < 0) farThreshold2 = 0;
            break;
        case '=':
            nearThreshold2 ++;
            if (nearThreshold2 > 255) nearThreshold2 = 255;
            break;
        case '~':
            nearThreshold2 --;
            if (nearThreshold2 < 0) nearThreshold2 = 0;
            break;
            
    }

}
//--------------------------------------------------------------


//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(30); // zero the tilt on exit
    kinect.close();
}
//--------------------------------------------------------------

void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    cout << "x:" << x << ",y:" << y <<endl;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

bool ofApp::detectRectangleCollision( const ofRectangle& firstRectangle,const ofRectangle& secondRectangle){
    //なぜかrectangle.x が不定のデータが入ってくることがあるため緊急的な措置
    if ( firstRectangle.getWidth() > 0 && secondRectangle.getWidth() > 0 && firstRectangle.getHeight() > 0 && secondRectangle.getWidth() > 0) {
        if ( firstRectangle.getWidth() < 1000000 && secondRectangle.getWidth() < 1000000 && firstRectangle.getHeight()  < 1000000 && secondRectangle.getWidth() < 1000000) {
            if (  ( secondRectangle.getPosition().x <  firstRectangle.getPosition().x+ firstRectangle.getWidth() ) &&  ( firstRectangle.getPosition().x < secondRectangle.getPosition().x + secondRectangle.getWidth() )){
                if ( ( secondRectangle.getPosition().y < firstRectangle.getPosition().y + firstRectangle.getHeight() ) && (firstRectangle.getPosition().y < secondRectangle.getPosition().y + secondRectangle.getHeight() ) ){

                    return true;
                }
            }
        }
    }

    return false;
}


