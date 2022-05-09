#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include<windows.h>
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include "CKobuki.h"
#include "rplidar.h"

#include "map_loader.h"
#include <QKeyEvent>


#include <queue>
#include<QMutex>


//moje--------------------------------
typedef struct{
    double xx;
    double yy;
    double fi;
    double distance;
}worldPoint;

typedef struct{
    int map1[360][400];
}dataMap;

typedef struct{
    int path[10][10];
}dataPath;


//end--------------------------------

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    double arrx[10];
    bool useCamera;
    bool pathDrive=false;
    bool Next=false;

    int index=0;
    bool Next2=false;
    bool dealocation=false;
    bool search=true;

    int inx=0;
    double pathx[10];
    double pathy[10];
    bool isPath=false;
    double fi,prewFi,xx,yy,fiAbs = 0.0;

    queue<pair<double,double>> transitions;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void robotprocess();
    void laserprocess();
    void processThisLidar(LaserMeasurement &laserData);

    void processThisRobot();
    HANDLE robotthreadHandle; // handle na vlakno
    DWORD robotthreadID;  // id vlakna
    static DWORD WINAPI robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
    HANDLE laserthreadHandle; // handle na vlakno
    DWORD laserthreadID;  // id vlakna
    static DWORD WINAPI laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;
    unsigned int las_slen;
    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
    unsigned int rob_slen;

    //----- moje funkcie--------------------------------------------------------
    worldPoint getTarget();
    void odometria();
    double getAngle(double xx1, double yy1, double xx2, double yy2);
    double getDistance(double xx1, double yy1, double xx2, double yy2);
    void turnRobotR();
    void turnRobotL();
    void upError();
    void lidarNavigation();
    void regulator();
    void rotateToError();
    void roundError();

    void createBlankMap(dataMap *map);
    void insertMap(double distance, double angle);
    void MapToTxt(string name, dataMap* map);

    void keyPressEvent(QKeyEvent *event);
    //end--------------------------------

    //uloha4
    void obtainPoints(int** map, int xSize, int ySize, double rX, double rY, double endX, double endY);
    void getArrayMap(string filename);
    void floodAlgo(int** mapa, int xSize, int ySize, queue<Point>* floodPoints, int x, int y);
    int decideDirection(int** mapa, int xSize, int ySize, int x, int y, int min, int direction);
    void expObs(int** mapa,int xSize, int ySize);
    bool finishEnd(int** mapa, Point p, int xSize, int ySize);

    void tMAP(int** tmpMap, int xSize, int ySize, bool dealocation);

    void putInTxt(string file,int** mapa, int xSize, int ySize, bool points);

    //uloha2------------------------
    QMutex mutex;

    double minLidarDist;
    double minAngle;
    double maxLidarDistL;
    double maxLidarAngleL;
    double maxLidarDistR;
    double maxLidarAngleR;

    bool minPoint = FALSE;
    bool maxPointL = FALSE;
    bool maxPointR = FALSE;

    double currentTargetFi = 0.0;
    double currentTargetDistance = 0.0;

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void getNewFrame();
    //moje--------------------------------
    void on_pushButton_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_checkBox_clicked(bool checked);
    //end--------------------------------


private:
     JOYINFO joystickInfo;
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     CKobuki robot;
     TKobukiData robotdata;
     int datacounter;
     QTimer *timer;
     //moje--------------------------------
     boolean Start = false;
     boolean isTurned = false;
     boolean ToPoint = false;
     boolean useMapping=false;
     double prewEncoderL,prewEncoderR,startEncL,startEncR = 0.0;
     double distanceL,distanceR,pDistanceL,pDistanceR = 0.0;

     boolean firstRun = true;
     boolean isLidarNavigation = false;
     worldPoint currentTarget;
     worldPoint endTarget;
     double angleError = 0.0;

     double currentTargetX = 0.0;
     double currentTargetY = 0.0;
     double endTargetDistance = 0.0;
     double endTargetFi = 0.0;
     double endTargetX = 0.0;
     double endTargetY = 0.0;
     int navigationCounter = 0;
     double regSpeed = 0.0;
     double regCirc = 0.0;
     double Kc = 60;
     double Ks = 700;
     const double regMaxspeed = 400;

     long double tickToMeter = 0.000085292090497737556558; // [m/tick]
     long double b = 0.23;
     double bb = 0.3;

     dataMap DataM;
     dataPath DataPath;

     int TL=0;
     int TR=0;

     //uloha 4
     map_loader mapLoader;
     TMapArea idealMap;
     queue<Point> wayPoints;
     int** arrayMap;
     int widthOfCell = 5; //cm

     int** path2;

     //uloha2
     void observeRoad();//LaserMeasurement &laserData
     void verifyPoint();
     void choseTransition();

     bool obstacle=false;

     void updateLidarNavigation();

     double end_region;
     double start_region;
     double angle,d_crit,d_scan;
     double dist;
     double alfa;
     double alfa_z;

     double dist_b;
     double d_scan_1;
     double d_scan_2;
     double x_p,y_p;

     bool condition1;
     bool condition2;
     bool obstacle2;

     queue<pair<double,double>> transition_tmp;
     //end--------------------------------
public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi);

};

#endif // MAINWINDOW_H
