//overall logic
//landmark detect =>into=> purple extract >>> detected =>into=> stop the car >> servo motor >> scan >> detailed task>>>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sys/timeb.h>
#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringSerial.h>
#include <iostream>
#include <math.h>
#include <softPwm.h>
#include <opencv2/videoio.hpp>
#include <lcd.h>

void forward(int,int);
void turn_left(int,int);
void turn_right(int,int);
void control(float,int);
void setCamera(int,int);
void Linefollowing();
void turnL();
void turnR();
void traffic();
void ultrasonic();
void football();

using namespace cv;
using namespace std;

Mat C3T3R2, C2T2R2, C2T2R1,Football,MeasureDistance;
Mat ShortCutBlue,ShortCutGreen,ShortCutRed,ShortCutYellow;
Mat TrafficLight;
Mat TurnLeft,TurnRight;
Point findContourCentre(std::vector<cv::Point> contour);
Mat transformPerspective(std::vector<Point> boundingContour, Mat frame, int x_size, int y_size);
float compareImages(Mat cameraImage, Mat librarySymbol);
//extract purple
Mat purpleExtract(Mat frame);
//purple square(landmark) detect
Mat landmarkDetect(Mat frame);
//scan and compare
void scan(Mat frame);
void servoUp();
void servoDown();
void setCamera(int wid, int hei);
void countShape(Mat transformed);

VideoCapture cap(0);
int servo = 27;
int ultrasonic_out=29;
int ultrasonic_in=28;
int flag=0;
int maxSp = 90;
int minSp = -60;
int flag_purple=0;
int flag_purple_pre=0;
int robot;
int E=0;
int D0=13;
int D1=14;
int D2=30;
int D3=21;
int D4=6;
int D5=1;
int D6=5;
int D7=4;
int lcd;

int main()
{
    wiringPiSetup();
    lcd=lcdInit(2,16,4,3,E,D4,D5,D6,D7,0,0,0,0);
    //template loading
    C3T3R2 = imread("C3T3R2.PNG", IMREAD_GRAYSCALE);
    threshold(C3T3R2,C3T3R2,200,255,THRESH_BINARY_INV);

    C2T2R2 = imread("C2T2R2.PNG", IMREAD_GRAYSCALE);
    threshold(C2T2R2,C2T2R2,200,255,THRESH_BINARY_INV);

    C2T2R1 = imread("C2T2R1.PNG", IMREAD_GRAYSCALE);
    threshold(C2T2R1,C2T2R1,200,255,THRESH_BINARY_INV);

    Football = imread("Football.PNG", IMREAD_GRAYSCALE);
    threshold(Football,Football,200,255,THRESH_BINARY_INV);

    MeasureDistance = imread("MeasureDistance.PNG",IMREAD_GRAYSCALE);
    threshold(MeasureDistance,MeasureDistance,200,255,THRESH_BINARY_INV);

    ShortCutBlue = imread("ShortCutBlue.PNG", IMREAD_GRAYSCALE);
    threshold(ShortCutBlue,ShortCutBlue,250,255,THRESH_BINARY_INV);

    ShortCutGreen = imread("ShortcutGreen.PNG", IMREAD_GRAYSCALE);
    threshold(ShortCutGreen,ShortCutGreen,250,255,THRESH_BINARY_INV);

    ShortCutRed = imread("ShortcutRed.PNG", IMREAD_GRAYSCALE);
    threshold(ShortCutRed,ShortCutRed,250,255,THRESH_BINARY_INV);

    ShortCutYellow = imread("ShortcutYellow.PNG", IMREAD_GRAYSCALE);
    threshold(ShortCutYellow,ShortCutYellow,250,255,THRESH_BINARY_INV);

    TrafficLight = imread("TrafficLight.PNG", IMREAD_GRAYSCALE);
    threshold(TrafficLight,TrafficLight,250,255,THRESH_BINARY_INV);

    TurnLeft = imread("TurnLeft.PNG", IMREAD_GRAYSCALE);
    threshold(TurnLeft,TurnLeft,250,255,THRESH_BINARY_INV);

    TurnRight = imread("TurnRight.PNG", IMREAD_GRAYSCALE);
    threshold(TurnRight,TurnRight,250,255,THRESH_BINARY_INV);

    setCamera(350, 240);

    Mat frame;

    while(1)
    {
        cap >> frame;

        if(frame.empty())//if empty->break
             break;
        //namedWindow("frame",WINDOW_NORMAL);
        //imshow("frame",frame);
        waitKey(1);

        Linefollowing();
    }

    cap.release();

    return 0;
}

Point findContourCentre(std::vector<cv::Point> contour)
{
    Moments foundRegion;    // Variables to store the region moment and the centre point
    Point centre;
    foundRegion = moments(contour, false);      // Calculate the moment for the contour
    centre.x = (foundRegion.m10/foundRegion.m00);  //Calculate the X and Y positions
    centre.y = (foundRegion.m01/foundRegion.m00);

    return centre;
}

Mat transformPerspective(std::vector<Point> boundingContour, Mat frame, int x_size, int y_size)
{
    if(boundingContour.size() != 4)
    {
        // Error the contour has too many points. Only 4 are allowed

        Mat fake = frame(Rect(0,0,320,240));
        return fake;
    }

    Mat symbol(y_size,x_size,CV_8UC1, Scalar(0));

    Point2f symbolCorners[4], boundingCorners[4];      // Create (and populate) variables containing the corner locations for the transform
    symbolCorners[0] = Point2f(0,0);
    symbolCorners[1] = Point2f(symbol.cols - 1,0);
    symbolCorners[2] = Point2f(symbol.cols - 1,symbol.rows - 1);
    symbolCorners[3] = Point2f(0,symbol.rows - 1);

    Point contourCentre = findContourCentre(boundingContour);   // To populate the contour corners we need to check the order of the points

    int point1, point2, point3, point4;

    if(boundingContour[0].x > contourCentre.x)
    {
        if(boundingContour[0].y > contourCentre.y)
            point3 = 0;
        else
            point2 = 0;
    }
    else
    {
        if(boundingContour[0].y > contourCentre.y)
            point4 = 0;
        else
            point1 = 0;
    }

    if(boundingContour[1].x > contourCentre.x)
    {
        if(boundingContour[1].y > contourCentre.y)
            point3 = 1;
        else
            point2 = 1;
    }
    else
    {
        if(boundingContour[1].y > contourCentre.y)
            point4 = 1;
        else
            point1 = 1;
    }

    if(boundingContour[2].x > contourCentre.x)
    {
        if(boundingContour[2].y > contourCentre.y)
            point3 = 2;
        else
            point2 = 2;
    }
    else
    {
        if(boundingContour[2].y > contourCentre.y)
            point4 = 2;
        else
            point1 = 2;
    }

    if(boundingContour[3].x > contourCentre.x)
    {
        if(boundingContour[3].y > contourCentre.y)
            point3 = 3;
        else
            point2 = 3;
    }
    else
    {
        if(boundingContour[3].y > contourCentre.y)
            point4 = 3;
        else
            point1 = 3;
    }

    if(point1 + point2 + point3 + point4 != 6)
    {
        return frame(Rect(0,0,320,240));
    }

    boundingCorners[0] = boundingContour[point1];
    boundingCorners[1] = boundingContour[point2];
    boundingCorners[2] = boundingContour[point3];
    boundingCorners[3] = boundingContour[point4];

    Mat transformMatrix = cv::getPerspectiveTransform(boundingCorners, symbolCorners); // Calculate the required transform operation
    Mat transformedSymbol(240,320,CV_8UC1,Scalar(0));
    warpPerspective(frame, transformedSymbol, transformMatrix, cv::Size(symbol.cols,symbol.rows));  // Perform the transformation

    return transformedSymbol;
}

float compareImages(Mat cameraImage, Mat librarySymbol)
{
    float matchPercent = 100 - (100/((float)librarySymbol.cols*(float)librarySymbol.rows) * (2*(float)countNonZero(librarySymbol^cameraImage))); // An incorrect pixel has double weighting
    return matchPercent;
}

void servoUp()
{

    softPwmCreate(servo, 6, 200);
    delay(1000);
    //softPwmWrite(servo, 5);
    //delay(3000);
    softPwmStop(servo);
}

void servoDown()
{

    softPwmCreate(servo, 11, 200);
    delay(1000);
    //softPwmWrite(servo, 5);
    //delay(3000);
    softPwmStop(servo);
}

Mat purpleExtract(Mat frame)   //function to extract purple
{

    //extract purple pixel
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(130, 50, 80), Scalar(170, 255, 240), hsv);
    //inRange(frame, Scalar(79, 0, 99), Scalar(179, 80, 255), hsv);
    //namedWindow("hsv",WINDOW_NORMAL);
    //imshow("hsv",hsv);
    //waitKey(1);
    return hsv;
}

Mat landmarkDetect(Mat frame) //function that check the purple square
{
    Mat hsv = purpleExtract(frame);
    //count purple pixel
    int numPurple = 0;
    numPurple = countNonZero(hsv);
    cout << "PPPPPPPPPPurple==" << numPurple << endl;

    //if met purple square, scan
    flag_purple_pre=flag_purple;
    if (numPurple > 2000)
    {
        flag_purple=1;
    }

    else{
        flag_purple=0;
    }


    if(flag_purple==1&&flag_purple_pre!=flag_purple){
        serialPrintf(robot,"#ha");
        scan(frame);
    }

    return hsv;
}

void scan(Mat frame)
{
    //set servo
    setCamera(640, 480);
    servoUp();
    int flag_scan=1;
    while(flag_scan==1){
    Mat pic;
    cap>>pic;
    imshow("pic",pic);
    //extract purple
    Mat hsv = purpleExtract(pic);

    //find contour
    vector<vector<Point>>contours;
    vector <Vec4i> hierarchy;
    findContours(hsv, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
    cout << "CCCCCCCContours==="<<contours.size();
    //approximate contour
    vector<vector<Point>> approxedcontours(contours.size());


    //ROI transform

    for(int i=0; i < contours.size(); i++)
    {

        approxPolyDP(contours[i],approxedcontours[i],10,true);

        vector<Point>boundingcounter(approxedcontours[i].size());

        for(int j=0; j<approxedcontours[i].size();j++)
        {
            boundingcounter[j] = approxedcontours[i][j];
        }

        //Mat transformed = transformPerspective(boundingcounter,hsv, 320, 240);
        Mat eroded = transformPerspective(boundingcounter,hsv, 320, 240);
        //erode
        //Mat eroded;
        Mat kernel=getStructuringElement(MORPH_ELLIPSE, Size(5,5));
        Mat transformed;
        erode(eroded, transformed, kernel);


        //imshow("trans",transformed);
        //imshow("er",eroded);
        //waitKey(0);

        //similarity matching
        float similarity = compareImages(transformed, C3T3R2);
        float similarity2=0;
        cout << "similarity_of_count===" <<similarity<<endl;
        if(similarity > 82)
        {
            servoDown();
            countShape(transformed);
            cout << "*********C3T3R2"<<endl;
            flag_scan=0;
            break;

        }

        similarity = compareImages(transformed, C2T2R2);
        cout << "similarity_of_count===" <<similarity<<endl;
        if(similarity > 83)
        {
            servoDown();
            countShape(transformed);
            cout << "*********C2T2R2"<<endl;
            flag_scan=0;
            break;

        }

        similarity = compareImages(transformed, C2T2R1);
        cout << "similarity_of_count===" <<similarity<<endl;
        if(similarity > 82)
        {
            servoDown();
            countShape(transformed);
            cout << "*********C2T2R1"<<endl;
            flag_scan=0;
            break;

        }

        similarity = compareImages(transformed, Football);
        cout << "similarity_of_football==="<<similarity<<endl;
        if(similarity > 60)
        {
            //football();
            servoDown();
            cout<< "***********football"<<endl;
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"                       ");
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"football");
            flag_scan=0;
            //football();
            break;
        }

        similarity = compareImages(transformed, MeasureDistance);
        cout << "similarity_of_measureDistance==="<<similarity<<endl;
        if(similarity > 70)
        {
            //distMeasure();
            servoDown();
            cout<< "**********dist"<<endl;
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"                       ");
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"distance");
            flag_scan=0;
            //ultrasonic();
            break;
        }

        similarity = compareImages(transformed, ShortCutYellow);
        cout << "similarity_of_Yellow==="<<similarity<<endl;
        if(similarity > 57)
        {
            double sim[4];
            int maxi = 0;
            sim[0] = compareImages(transformed, ShortCutYellow);
            sim[1] = compareImages(transformed, ShortCutRed);
            sim[2] = compareImages(transformed, ShortCutBlue);
            sim[3] = compareImages(transformed, ShortCutGreen);

            for (int u = 0; u< 3; u++){
                if (sim[maxi] < sim[u+1]){
                    maxi = u+1;
                }
            }

            if (maxi == 0)
            {

                servoDown();
                turn_right(60, 20);
                delay(1500);
                cout<< "************short_Y"<< endl;
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"                       ");
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"short_Y");
                flag_scan=0;
                break;
            }

            else if(maxi == 1)
            {
                servoDown();
                forward(50, 10);
                delay(1000);
                cout<< "**********short_R"<<endl;
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"                       ");
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"short_R");
                flag_scan=0;
                break;
            }

            else if(maxi == 2)
            {
                servoDown();
                cout<< "**********short_B"<<endl;
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"                       ");
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"short_B");
                flag_scan=0;
                break;
            }

            else
            {
                servoDown();
                turn_left(20,60);
                delay(1500);
                cout<< "*********short_G"<<endl;
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"                       ");
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"short_G");
                flag_scan=0;
                break;
            }

        }

        similarity = compareImages(transformed, TrafficLight);
        cout << "similarity_of_trafficLight==="<<similarity<<endl;
        if(similarity > 50)
        {

            cout<< "**********traffic"<<endl;
            //servoDown();
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"                       ");
            lcdPosition(lcd,0,0);
            lcdPuts(lcd,"traffic");
            flag_scan=0;
            traffic();
            break;
        }

        similarity = compareImages(transformed, TurnLeft);
        cout << "similarity_of_Left==="<<similarity<<endl;


        similarity2 = compareImages(transformed, TurnRight);
        cout << "similarity_of_Right"<<similarity2<<endl;
        if(similarity2 > 72||similarity > 78)
        {
            if(similarity>=similarity2){
                servoDown();
                cout<< "**********turn_L"<<endl;
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"                       ");
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"left");
                flag_scan=0;
                //turnL();
                break;
            }
            else{
                servoDown();
                cout<< "***********turn_R"<<endl;
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"                       ");
                lcdPosition(lcd,0,0);
                lcdPuts(lcd,"right");
                flag_scan=0;
                //turnR();
                break;
            }

        }
        //if(i==contours.size()-1&&j==approxedcontours[contours.size()-1].size()-1)

    } // end for

    }
    Mat a;
    for(int i=0;i<=4;i++){
        cap>>a;
        //imshow("line",test);
    }
    setCamera(350, 240);
}
void Linefollowing(){


	//setCamera(350, 250);
    robot=serialOpen("/dev/ttyAMA0",57600);//openit
    //int robot=0;

    Mat test;

    float dis=0;
    float dis_pre=0;
    float dis_total=0;

    float P=0.25;
    float I=0;
    float D=0;

    float result;

    while(1)
    {
        int flag_shortcut;

        cap >> test;

        landmarkDetect(test);
        cvtColor(test, test, COLOR_BGR2HSV);


        Mat save=test;

        flag_shortcut=0;
        inRange(test, Scalar(60,77,60), Scalar(90,255,255), test); // GREEN
        int num =countNonZero(test);
        cout<<num<<endl;
        if(num<=2000){
            test=save;
            inRange(test, Scalar(20,110,100), Scalar(30,255,255), test); // YELLOW
            num =countNonZero(test);
            cout<<num<<endl;
            flag_shortcut=1;
            if(num<=2000){
                test=save;
                 test=save;
                //inRange(test, Scalar(0,110,100), Scalar(10,255,255), test);
                inRange(test, Scalar(170,110,100), Scalar(180,255,255), test); // RED
                //imshow("1", test);
                //imshow("2", test2);
                //add(test2,test,test);
                num =countNonZero(test);

                flag_shortcut=2;
                cout<<num<<endl;
                if(num<=2000){
                    test=save;

                    inRange(test, Scalar(80,40,50), Scalar(130,255,255), test); // BLUE
                    num =countNonZero(test);
                    flag_shortcut=3;
//imshow("4", test);
                    cout<<num<<endl;
                    if(num<=2000){
                        test=save;
                        inRange(test, Scalar(0,0,0), Scalar(180,255,46), test); // BLACK
                        flag_shortcut=4;
                    }
                }
            }
        }
//end

        cout<<"flag_shortcut"<<flag_shortcut;
        imshow("two", test);

        int value;
        double count=0;;
        double total=0;

        for(int a=0;a<test.cols;a++){
            value = (int)test.at<uchar>((Point(a,220)));
            // cout<<a<<" : "<<value<<endl;
            if(value>200){
                    count++;
                    total+=a;
            }
        }

        flag=0;
        if(count>=80){
            flag=1;
        }
        cout<<"COUNT"<<count<<endl;
        cout<<"flag"<<flag<<endl;

        if(count!=0){
            dis=(total/count)-(test.cols)/2;
        }

        else{
            if(dis<=-30){
                dis=-175;
                cout<<"in"<<endl;
            }

            if(dis>=30){
                dis=175;
                cout<<"in"<<endl;
            }
        }
        cout<<"dis: "<<dis<<endl;
        //cout<<"dis_total: "<<dis_total<<endl;
        //cout<<"dis_pre"<<dis_pre<<endl;
        if(dis==dis){
            dis_total=dis_total+dis;
        }

        result=P*dis+D*(dis-dis_pre);
        cout<<"result: "<<result<<endl;
        control(result,flag);

        dis_pre=dis;
        if(waitKey(1)!=255){

            while(waitKey(1)==255){
                serialPrintf(robot,"#ha");

            }

        }

    }
	serialPrintf(robot, "#Ha");
    serialClose(robot);
}


void control(float val,int flag){


    int left=20+(int)val;

    if(left>=maxSp){
        left=maxSp;
    }

    if(left<=minSp){
        left=minSp;
    }

    int right=20-(int)val;

    if(right>=maxSp){
        right=maxSp;
    }

    if(right<=minSp){
        right=minSp;
    }

    if(val<=0&&flag==1){
        left=minSp;
        right=maxSp;
    }

    if(val>=0&&flag==1){
        left=maxSp;
        right=minSp;
    }


    cout<< "speed";
    cout<<left<<endl;
    cout<<right<<endl;
    if(left>=0&&right>=0){
        forward(left,right);

    }

    if(left<=0&&right>=0){
        left=-1*left;
        turn_left(left,right);

    }

    if(left>=0&&right<=0){
        right=-1*right;
        turn_right(left,right);

    }

}

void forward(int l,int r)
{
    serialPrintf(robot,"#Barfrf0%d,0%d,0%d,0%d",r,r,l,l);
}


void turn_left(int l,int r)
{
    serialPrintf(robot,"#Barffr0%d,0%d,0%d,0%d",r,r,l,l);
}

void turn_right(int l,int r)
{
    serialPrintf(robot,"#Bafrrf0%d,0%d,0%d,0%d",r,r,l,l);
}

void setCamera(int wid, int hei)
{
	cap.set(CAP_PROP_FRAME_WIDTH, wid);
	cap.set(CAP_PROP_FRAME_HEIGHT, hei);
}

void ultrasonic()
{
    pinMode(ultrasonic_out,OUTPUT);
    pinMode(ultrasonic_in,INPUT);

    digitalWrite(ultrasonic_out,HIGH);

    clock_t start, end;


    digitalWrite(ultrasonic_out,LOW);
    delay(2000);
    digitalWrite(ultrasonic_out,HIGH);
    delay(0.01);
    digitalWrite(ultrasonic_out,LOW);
    while(digitalRead(ultrasonic_in)==0){
        start=clock();
    }
    while(digitalRead(ultrasonic_in)==1){
        end=clock();
    }
    double time=(double)(end-start)/CLOCKS_PER_SEC;
    time=time*17150;
    time=(time*0.97)-71;
    cout<<"distance"<<time<<endl;
    lcdPosition(lcd,0,0);
    lcdPuts(lcd,"                       ");
    lcdPosition(lcd,0,0);
    lcdPrintf(lcd,"%f",time);

}
void turnL(){
    //serialPrintf(robot,"#Barfrf030,030,030,030);
    //delay(1000);
    //forward(20,20);
    //delay(500);
    turn_left(20,70);
    delay(1500);//need debug
}

void turnR(){
    //serialPrintf(robot,"#Barfrf030,030,030,030);
    //
    //forward(20,20);
    //delay(500);
    turn_right(70,20);
    delay(1500);//need debug
}

void football()
{
    turn_left(20,70);
    delay(1000);
    Mat black;
    serialPrintf(robot,"#Bafrrf070,070,020,020");
    delay(3000);
    while(1){
        cap>>black;
        serialPrintf(robot,"#Bafrrf070,070,020,020");
        inRange(black, Scalar(0,0,0), Scalar(180,255,46), black);
        int num_black=countNonZero(black);
        if(num_black>=3000){
            break;
        }
    }
}

void traffic(){
//    turn_left(50,50);
//    delay(500);
//    serialPrintf(robot,"#ha");
//    int val=0;
//    int i_total=0;
//    int j_total=0;
//    int b_total=0;
//    int r_total=0;
//    int g_total=0;
//    int r=0;
//    int g=0;
//    int b=0;
//    int c=0;
    Mat img;
    int counter=0;
    for(int i=0;i<=5;i++){
        cap>>img;
    }
    while(1){
        cap>>img;
        cvtColor(img,img,COLOR_BGR2HSV);
        inRange(img,Scalar(36,45,152),Scalar(77,255,255),img);
        //imshow("traffic light",img);
        if(countNonZero(img)>30){
            counter++;
        }
        else{
            counter=0;
        }
        cout<<"counter"<<counter<<endl;
        if(counter>30){
            servoDown();
            break;
        }


   /* Mat img2;
    Mat img3;
    imshow("traffic",img);
    cvtColor(img,img2, CV_BGR2GRAY);
   // imshow("two",img);
    threshold(img2, img2, 250, 255, 0);
    imshow("traffic2",img2);
    Mat erodeStruct2= getStructuringElement(MORPH_RECT,Size(10,10));
    dilate(img2, img3, erodeStruct2);
    subtract(img3,img2,img3);
    //img3=img2;

    imshow("traffic3",img3);

    //imshow("one",img3);

    for(int i=1;i<=img3.rows;i++){
        for(int j=1;j<=img3.cols;j++){
            val = (int)img3.at<uchar>((Point(j,i)));
            // cout<<a< <" :"<<value<<endl;
            if(val>200){
                c++;
                b=img.at<Vec3b>(Point(j,i))[0];
                g=img.at<Vec3b>(Point(j,i))[1];
                r=img.at<Vec3b>(Point(j,i))[2];
                r_total+=r;
                g_total+=g;
                b_total+=b;
            }
        }
    }
    if(c!=0){
        r=r_total/c;
        g=g_total/c;
        b=b_total/c;
    }
    cout<<r<<endl;
    cout<<g<<endl;
    cout<<b<<endl;
    if((g-r)>=10){

        break;

    }*/

    }

}

void countShape(Mat transformed)
{
     vector< vector<Point> > contours;
     vector<Point> point;
     vector<Vec4i> hireachy;
     findContours(transformed, contours, hireachy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
     Mat display=transformed.clone();
     int triNum = 0;
     int rectNum = 0;
     int circleNum = 0;
     for (size_t t = 0; t < contours.size(); t++)
     {
         double area = contourArea(contours[t]);
         if (area < 10000)
         {
            int epsilon = 0.05*arcLength(contours[t], true);
            approxPolyDP(contours[t], point, epsilon, true);
            if(point.size()==3)
            {
                drawContours(display, contours, t, Scalar(0, 0, 255), 2, 8, Mat(), 0, Point());
                triNum++;
            }
            else if (point.size() == 4)
            {
                drawContours(display, contours, t, Scalar(0, 255, 0), 2, 8, Mat(), 0, Point());
                rectNum++;
            }

            else
            {
                drawContours(display, contours, t, Scalar(255, 0, 0), 2, 8, Mat(), 0, Point());
                circleNum++;
            }
         }

     }

     imshow("Display",display);
     lcdPosition(lcd,0,0);
     lcdPuts(lcd,"                       ");
     lcdPosition(lcd,0,0);
     lcdPrintf(lcd,"C%d R%d T%d",circleNum, rectNum, triNum);
}

