#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#define _USE_MATH_DEFINES
#include <QtMath>
#include <QDebug>
#include<stdio.h>
#include<string.h>
#include <mutex>
//#include<QMutex>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";
    //cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
 //   connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera=false;

    datacounter=0;

    //createBlankMap(&DataM);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect(20,120,700,500);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);
  /*  if(useCamera==true)
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );
        painter.drawImage(rect,image.rgbSwapped());
    }
    else*/
    {
        if(updateLaserPicture==1)
        {
            updateLaserPicture=0;
            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;

            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {

                /*  int dist=rand()%500;
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-k)*3.14159/180.0))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-k)*3.14159/180.0))+rect.topLeft().y();*/

                int distance=copyOfLaserData.Data[k].scanDistance/20;
                int xp=rect.width()-(rect.width()/2+distance*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+distance*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();

                int xr=rect.width()-(rect.width()/2+xx)+rect.topLeft().x();
                int yr=rect.height()-(rect.height()/2+yy)+rect.topLeft().y();

                painter.drawEllipse(QPoint(xr, yr),4,4);
                //insertMap(copyOfLaserData.Data[k].scanDistance, 360-copyOfLaserData.Data[k].scanAngle);

                if(rect.contains(xp,yp))
                    painter.drawEllipse(QPoint(xp, yp),2,2);



            }
        }
    }
}

void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi*180/M_PI));

}

void MainWindow::processThisRobot()
{
    odometria();
    upError();
    regulator();
    if(obstacle==true){
        observeRoad();
    }


    if(Next2 && index<=inx){
        //printf("output %d", Next2);
        //printf("inx %f", pathx[index]);
        //printf("index %f \n", pathy[index]);
        isPath=false;
        endTargetX=pathx[index];
        endTargetY=pathy[index];

        pathDrive=true;
        on_pushButton_10_clicked();
        pathDrive=false;
        Next2=false;
    }

    if(datacounter>15){
       emit uiValuesChanged(xx,yy,fiAbs);
        datacounter=0;
    }
    datacounter++;
}

void MainWindow::processThisLidar(LaserMeasurement &laserData)
{

    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    //observeRoad(laserData);
    updateLaserPicture=1;

    update();

    if(TL==0 && TR==0 && useMapping){

        //tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

        for(int k=0; k<copyOfLaserData.numberOfScans; k++)
        {
            insertMap(copyOfLaserData.Data[k].scanDistance, 360-copyOfLaserData.Data[k].scanAngle);
        }
    }


}

void MainWindow::on_pushButton_9_clicked() //start button
{
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    laserthreadHandle=CreateThread(NULL,0,laserUDPVlakno, (void *)this,0,&laserthreadID);
    robotthreadHandle=CreateThread(NULL,0, robotUDPVlakno, (void *)this,0,&robotthreadID);
    /*  laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
      robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);*/
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));
    //DataM.map=[];

    pathDrive=false;
    Next2=false;

}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    TL=0;
    TR=0;
    std::vector<unsigned char> mess=robot.setTranslationSpeed(500);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_3_clicked() //back
{
    TL=0;
    TR=0;
    std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_6_clicked() //left
{
    TL=1;
    std::vector<unsigned char> mess=robot.setRotationSpeed(3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_5_clicked()//right
{
    TR=1;
    std::vector<unsigned char> mess=robot.setRotationSpeed(-3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    TL=0;
    TR=0;
    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    //pre pismena je key ekvivalent ich ascii hodnoty
    //pre ine klavesy pozrite tu: https://doc.qt.io/qt-5/qt.html#Key-enum
    if(event->key() == 56){
        on_pushButton_2_clicked();
    }else if(event->key() == 53){
        on_pushButton_4_clicked();
    }else if(event->key() == 52){
        on_pushButton_6_clicked();
    }else if(event->key() == 54){
        on_pushButton_5_clicked();
    }else if(event->key() == 50){
        on_pushButton_3_clicked();
}
}

void MainWindow::laserprocess()
{
    WSADATA wsaData = {0};
    int iResult = 0;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    int las_broadcastene=1;
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,(char*)&las_broadcastene,sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, las_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(1)
    {
        if ((las_recv_len = recvfrom(las_s, (char*)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, (int*)&las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));

        processThisLidar(measure);


    }
}


void MainWindow::robotprocess()
{
    WSADATA wsaData = {0};
    int iResult = 0;



    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    Sleep(100);
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(1)
    {
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other,(int*) &rob_slen)) == -1)
        {

            continue;
        }
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        //struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);
        //moje------------
        if(!firstRun){
            prewEncoderL = robotdata.EncoderLeft;
            prewEncoderR = robotdata.EncoderRight;
        }
        //end------------------
        int returnval=robot.fillData(robotdata,(unsigned char*)buff);
        //moje-----------------
        if(firstRun) {
            firstRun = false;
            prewEncoderL = robotdata.EncoderLeft;
            prewEncoderR = robotdata.EncoderRight;
            xx=0;
            yy=0;
        }
        //end----------------------
        if(returnval==0)
        {
            processThisRobot();
        }

    }
}

void MainWindow::on_pushButton_clicked()
{
    if(useMapping==true){
        createBlankMap(&DataM);
        useMapping=false;
        ui->pushButton->setText("On map");
    }else{
        useMapping=true;
        ui->pushButton->setText("Off map");

    }


}

void MainWindow::getNewFrame()
{

}

//Odometria------------
void MainWindow::odometria(){
    // prave
    int A = ((prewEncoderR-robotdata.EncoderRight)>(60000))*1;
    A = ((prewEncoderR-robotdata.EncoderRight)<(-60000))*2+A;
    switch (A) {
      case 1:
        distanceR = tickToMeter*(robotdata.EncoderRight-prewEncoderR + 65535);
        break;
      case 2:
        distanceR = tickToMeter*(robotdata.EncoderRight-prewEncoderR - 65535);
        break;
      default:
        distanceR = tickToMeter*(robotdata.EncoderRight- prewEncoderR);
    }

    // left
    int B = ((prewEncoderL-robotdata.EncoderLeft)>(60000))*1;
    B = ((prewEncoderL-robotdata.EncoderLeft)<(-60000))*2+B;

    switch (B) {
      case 1:
        distanceL = tickToMeter*(robotdata.EncoderLeft-prewEncoderL + 65535);
        break;
      case 2:
        distanceL = tickToMeter*(robotdata.EncoderLeft-prewEncoderL - 65535);
        break;
      default:
        distanceL = tickToMeter*(robotdata.EncoderLeft- prewEncoderL);
    }

    // uhol
    fi = prewFi + (distanceR - distanceL)/(1.0*b);
    fi = fmod(fi,(2*M_PI));

    // absolutny uhol
    if(fi < 0){
        fiAbs = (2*M_PI) + fi;
    }else{
        fiAbs = fi;
    }

    // x, y
    int compareRL = (distanceL == distanceR);
    switch (compareRL) {
      case 0:
        xx = xx + ((b*(distanceR + distanceL)/(2.0*(distanceR-distanceL)))*(sin(fi)-sin(prewFi)));
        yy = yy - ((b*(distanceR + distanceL)/(2.0*(distanceR-distanceL)))*(cos(fi)-cos(prewFi)));
        break;
      case 1:
        xx = xx + distanceR*cos(prewFi);
        yy = yy + distanceR*sin(prewFi);
        break;
      default:
        xx = xx + ((b*(distanceR + distanceL)/(2.0*(distanceR-distanceL)))*(sin(fi)-sin(prewFi)));
        yy = yy - ((b*(distanceR + distanceL)/(2.0*(distanceR-distanceL)))*(cos(fi)-cos(prewFi)));
    }

    prewFi = fi;


}
//end
// navigacia-----------------------------------------------------
/*
void MainWindow::on_pushButton_10_clicked()
{
    Start = true;
    endTargetY = (ui->lineEdit_6->text().toDouble());
    endTargetX = (ui->lineEdit_5->text().toDouble());
    currentTargetX = endTargetX;
    currentTargetY = endTargetY;
    navigationCounter = 1;
}
*/
void MainWindow::on_pushButton_10_clicked()
{
    Start = true;
    if(pathDrive==false){
    endTargetY = (ui->lineEdit_6->text().toDouble());
    endTargetX = (ui->lineEdit_5->text().toDouble());
    currentTargetX = endTargetX;
    currentTargetY = endTargetY;
    }else if(pathDrive==true){
        currentTargetX = endTargetX;
        currentTargetY = endTargetY;
    }
    navigationCounter = 1;
}
double MainWindow::getAngle(double xx1, double yy1, double xx2, double yy2){
    return atan2(yy2-yy1,xx2-xx1);
}

double MainWindow::getDistance(double xx1, double yy1, double xx2, double yy2){
   return (double)sqrt(pow(xx2-xx1,2)+pow(yy2-yy1,2));
}

void MainWindow::turnRobotR(){
   on_pushButton_5_clicked();
}

void MainWindow::turnRobotL(){
   on_pushButton_6_clicked();
}

void MainWindow::upError(){

    currentTargetFi = getAngle(xx,yy,currentTargetX,currentTargetY);
    currentTargetDistance = getDistance(xx,yy,currentTargetX,currentTargetY);
    endTargetDistance = getDistance(xx,yy,endTargetX,endTargetY);

    if(currentTargetFi<0.0){
        currentTargetFi = (2*M_PI) + currentTargetFi;
    }else{
        currentTargetFi =  currentTargetFi;
    }

    angleError =  currentTargetFi - fiAbs;

    if(angleError>M_PI){
       angleError = -((2*M_PI)-angleError);
    }
    else if(angleError<-M_PI){
       angleError = (2*M_PI)+angleError;
    }

}


void MainWindow::regulator(){
    if(Start){
        // otoc podla erroru
        rotateToError();

       // zaokruhlenie pozadovaneho miesta, nejdeme presne na danu polohu.
        roundError();

       // presun
       if(Start && !isTurned){

           // kruznica
           regSpeed = Ks*currentTargetDistance;
           regCirc = Kc/(angleError);
           if(regSpeed > regMaxspeed){
               regSpeed = regMaxspeed;
           }
           if(isinf(regCirc)){
               regCirc = 32000;
           }

           std::vector<unsigned char> mess=robot.setArcSpeed((int)regSpeed,(int)regCirc);
           if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
           {
           }
       }
    }
}

void MainWindow::rotateToError(){
    if(abs(angleError)>(M_PI/24)){

        if(angleError < 0){
            turnRobotR();
            isTurned = true;
        } else if (angleError > 0){
            turnRobotL();
            isTurned = true;
        }

    }else {
        MainWindow::on_pushButton_4_clicked();  // stop
        isTurned = false;
    }
}

void MainWindow::roundError(){
    if(currentTargetDistance< 0.005){
         //navigationCounter = 1;
         ToPoint = false;
         isTurned = false;
         Start = false;
         on_pushButton_4_clicked(); //stop
         //Next=false;
         Next2=true;
         index+=1;
     }else if(endTargetDistance > 0.005){
             currentTarget = endTarget;
             Start = true;
             ToPoint = true;
         }

}

//--------------MAP------------------
int mapsize1=360;
int mapsize2=400;
int resolution=40;
int xg,yg;

void MainWindow::createBlankMap(dataMap *map1){
    int i = 0;
    int j = 0;
    while(i < mapsize1){
        i+=1;
        while(j < mapsize2){
            j+=1;
            map1->map1[i][j] = 0;
        }
    }
}

void MainWindow::insertMap(double distance, double angle){
    if(distance >= 200 ){
        xg = ((xx*1000 + (distance*cos((angle*M_PI/180) + fiAbs)))/resolution);
        yg = ((yy*1000 + (distance*sin((angle*M_PI/180) + fiAbs)))/resolution);
        DataM.map1[xg+40][yg+40] = 1;
    }
}

void MainWindow::MapToTxt(string name, dataMap* map1){
    ofstream file;
    file.open(name+".txt", ios::trunc);
    for(int i=0; i<mapsize1; i++){
       if(!(i==0)){
           file<<endl;
       }
        for(int j=0; j<mapsize2; j++){
            file<<map1->map1[i][j];
        }
    }
    file.close();
}

void MainWindow::on_pushButton_7_clicked() //reset laser
{
    MapToTxt("mapO", &DataM);

}

//---------------------end Map------------------------

//---------------------Map Navigation-----------------

void MainWindow::on_pushButton_8_clicked()
{
    pathDrive=true;
    index=0;
    int xSize = 120;
    int ySize = 120;

    double curX=ui->lineEdit_2->text().toDouble()*100+50;
    double curY=ui->lineEdit_3->text().toDouble()*100+50;
    double reqX=ui->lineEdit_5->text().toDouble()*100;
    double reqY=ui->lineEdit_6->text().toDouble()*100;


    getArrayMap("priestor.txt");

    //vytvor pole
    int** tmpMap = new int*[xSize];
    dealocation=false;
    tMAP(tmpMap, xSize, ySize, dealocation);

    expObs(tmpMap, xSize, ySize);
    obtainPoints(tmpMap, xSize, ySize, curX, curY, reqX, reqY);
    putInTxt("mapaROAD.txt", tmpMap, xSize, ySize, true);

    //pole dealokuj
    dealocation=true;
    tMAP(tmpMap, xSize, ySize, dealocation);

}
//----------------------------------------------------

void MainWindow::expObs(int** map, int xSize, int ySize)
{
    double rRadius = 20.0; //cm
    int valueOfExpansion = (int) (rRadius/widthOfCell);

    for(int x = 0; x < xSize; x++){
        for(int y = 0; y < ySize; y++){
            if (map[x][y] == 1){
                for (int i = -valueOfExpansion; i <= valueOfExpansion; i++){
                    for (int j = -valueOfExpansion; j <= valueOfExpansion; j++){
                        if (x+i < xSize && x+i > 0 && y+j < ySize && y+j > 0 && map[x+i][y+j] == 0)
                            map[x+i][y+j] = 5;
                    }
                }
            }
        }
    }

    for(int x = 0; x < xSize; x++){
            for(int y = 0; y < ySize; y++){
                if (map[x][y] == 5)
                    map[x][y] = 1;
            }
        }

}


void MainWindow::obtainPoints(int** map, int xSize, int ySize, double rX, double rY, double endX, double endY)
{
    queue<Point> floodPoints;
    Point pointEnd, point;
    int currentDirection;
    int previewsDirection = 0;

    inx=0;

    pointEnd.x = (endX)/widthOfCell;
    pointEnd.y = (endY)/widthOfCell;

    int x = ((int)rX)/widthOfCell;
    int y = ((int)rY)/widthOfCell;

    //vyprazdni
    while(!wayPoints.empty()){
        wayPoints.pop();
    }

    //

    if (x < xSize*10 && y < ySize*10 ){

        if (map[x][y] == 0 && map[(int)pointEnd.x][(int)pointEnd.y] == 0){
            map[x][y] = -1;
            map[(int)pointEnd.x][(int)pointEnd.y] = 2;
        }
        else{        
            return;
        }
    }
    else{
        return;
    }

    floodPoints.push(pointEnd);

    //Zaplavovy algo
    while (!floodPoints.empty()){

        point = floodPoints.front();
        floodPoints.pop();
        floodAlgo(map, xSize, ySize, &floodPoints,(int)point.x,(int)point.y);


        if (finishEnd(map, point, xSize, ySize))
            break;

    }

    //naplanuje cestu
    search=true;
    while (search){
        int min = 100000;
        int direction = 20;
        currentDirection = decideDirection(map, xSize, ySize, x, y, min, direction);

        if(currentDirection != previewsDirection){
          map[x][y] = -2;
          double cx=((x)*5-50)/100.0;
          double cy=((y)*5-50)/100.0;
          inx+=1;
          pathx[inx] = cx;
          pathy[inx] = cy;

          point.x = ((x)*5-50)/100.0;
          point.y = ((y)*5-50)/100.0;
          wayPoints.push(point);
          //------drive---------
          pathDrive=true;
          Next2=true;
          endTargetX=cx;
          endTargetY=cy;
          //-------------
          //Next=true;
          //while(Next==true){
              //on_pushButton_10_clicked();
          //}

      }

        switch(currentDirection) {
          case 20://mimo
            search=false;
            break;

          case 10://ciel
            point.x = endX/100;
            point.y = endY/100;
            wayPoints.push(point);
            wayPoints.pop();

            pathDrive=true;
            endTargetX=point.x;
            endTargetY=point.y;
            //on_pushButton_10_clicked();
            search=false;
            break;

          default:
            search=true;
            break;
        }


        previewsDirection = currentDirection;

        switch(currentDirection) {
          case 1:
            x += 1;
            break;
          case 2:
            y += 1;
            break;
          case 3:
            x -= 1;
            break;
          default:
            y -= 1;
        }


    }
    isPath=true;
}


void MainWindow::floodAlgo(int** map,int xSize, int ySize, queue<Point>* floodPoints , int x, int y){

    Point point;
//x
    if (x+1 < xSize && map[x+1][y] == 0 ) {
        map[x+1][y] = map[x][y]+1;
        point.x = x+1;
        point.y = y;
        (*floodPoints).push(point);
    }
    if (x-1 >= 0 && map[x-1][y] == 0 ) {
        map[x-1][y] = map[x][y]+1;
        point.x = x-1;
        point.y = y;
        (*floodPoints).push(point);
    }
//y
    if (y+1 < ySize && map[x][y+1] == 0 ) {
        map[x][y+1] = map[x][y]+1;
        point.x = x;
        point.y = y+1;
        (*floodPoints).push(point);
    }
    if (y-1 >= 0 && map[x][y-1] == 0 ) {
        map[x][y-1] = map[x][y]+1;
        point.x = x;
        point.y = y-1;
        (*floodPoints).push(point);
    }
}

bool MainWindow::finishEnd(int** map, Point point, int xSize, int ySize){

    int end1 = -1;
    //vertikalne a horizontalne
    if ((int)point.x+1 < xSize){
        if (map[(int)point.x+1][(int)point.y] == end1)
            return true;
    }
    if ((int)point.x-1 > 0){
        if (map[(int)point.x-1][(int)point.y] == end1)
            return true;
    }
    if ((int)point.y+1 < ySize){
        if (map[(int)point.x][(int)point.y+1] == end1)
            return true;
    }
    if ((int)point.y-1 > 0){
        if (map[(int)point.x][(int)point.y-1] == end1)
            return true;
    }
    //diagonalne
    if ((int)point.x+1 < xSize && (int)point.y+1 < ySize){
        if (map[(int)point.x+1][(int)point.y+1] == end1)
            return true;
    }
    if ((int)point.x+1 < xSize && (int)point.y-1 > 0){
        if (map[(int)point.x+1][(int)point.y-1] == end1)
            return true;
    }
    if ((int)point.x-1 > 0 && (int)point.y+1 < ySize){
        if (map[(int)point.x-1][(int)point.y+1] == end1)
            return true;
    }
    if ((int)point.x-1 > 0 && (int)point.y-1 > 0){
        if (map[(int)point.x-1][(int)point.y-1] == end1)
            return true;
    }
    return false;

}

int MainWindow::decideDirection(int** map, int xSize, int ySize, int x, int y, int min, int direction){

//x
    if (x+1 < xSize && map[x+1][y] > 1 && map[x+1][y] <= min) {
        min = map[x+1][y];
        direction = 1;
    }
    if (x-1 > 0 && map[x-1][y] > 1 &&  map[x-1][y] <= min) {
         min = map[x-1][y];
         direction = 3;
    }
//y
    if (y+1 < ySize && map[x][y+1] > 1 && map[x][y+1] <= min) {
         min = map[x][y+1];
         direction = 2;

    }
    if (y-1 > 0 && map[x][y-1] > 1 && map[x][y-1] <= min) {
         min = map[x][y-1];
         direction = 4;
    }

    //ciel
    if (min == 2)
        return 10;

    return direction;
}


void MainWindow::putInTxt(string file,int** map, int xSize, int ySize, bool points){

    ofstream myfile (file);
    if (myfile.is_open())
    {
        for(int i = 0;i<xSize; i++)
        {
            for(int j = 0; j<ySize;j++)
            {
                myfile << map[i][j]<<' ';
            }
            myfile<<";"<<"\n";
        }

        if (points)
        {
            queue<Point> wayPointsCopy = wayPoints;
            if (wayPoints.empty())
                //int uu = 2;
                ui->lineEdit->setText("Nic");
            else{
               ui->lineEdit->setText("Idem");

            }
        }
        myfile.close();
    }

}

void MainWindow::tMAP(int** tmpMap, int xSize, int ySize, bool dealocation){
    if(dealocation==false){
        for(int i=0; i<xSize; i++)
        {
            tmpMap[i]=new int[ySize];

            for(int j=0; j<ySize; j++)
            {
                tmpMap[i][j] = arrayMap[i][j];
            }
        }
    }else{
        for( int i = 0 ; i < xSize; i++ )
        {
            delete[] tmpMap[i];
        }
        delete[] tmpMap;
    }
}

void MainWindow::getArrayMap(string filename){

    char* data = (char*) "priestor.txt";
    mapLoader.load_map(data,idealMap);

    int x1,x2,y1,y2;
    int pred, po;

    //alokacia
    arrayMap = new int*[120];

    for(int i=0; i<120; i++)
    {
        arrayMap[i]=new int[120];

        for(int j=0; j<120; j++)
        {
            arrayMap[i][j] = 0;
        }
    }

    //stena dookola
    for (int i = 0; i < idealMap.wall.numofpoints; ++i) {

        if (idealMap.wall.numofpoints - i > 1){
            pred = i;
            po = i+1;
        }
        else{
            pred = i;
            po = 0;
        }

        // x
        if (idealMap.wall.points[po].point.x != idealMap.wall.points[pred].point.x){

            x1 = ((int) idealMap.wall.points[pred].point.x) / widthOfCell;
            x2 = ((int) idealMap.wall.points[po].point.x) / widthOfCell;

            if (x2 < x1){
                int temp = x2;
                x2 = x1;
                x1 = temp;
            }

            for (int j = x1; j <= x2; j++){
                arrayMap[j][((int) idealMap.wall.points[i].point.y) / widthOfCell] = 1;
            }

            // y
        }else{

            y1 = ((int) idealMap.wall.points[pred].point.y) / widthOfCell;
            y2 = ((int) idealMap.wall.points[po].point.y) / widthOfCell;

            if (y2 < y1){
                int temp = y2;
                y2 = y1;
                y1 = temp;
            }

            for (int j = y1; j <= y2; j++){
                arrayMap[((int) idealMap.wall.points[i].point.x) / widthOfCell][j] = 1;
            }
        }
    }

    vector<TMapObject>::iterator it = idealMap.obstacle.begin();
    vector<TMapObject>::iterator end = idealMap.obstacle.end();

    // vnutro
    for (; it != end; it++)
    {
        for (int i = 0; i < it->numofpoints; ++i)
        {
            if (it->numofpoints - i > 1){
                pred = i;
                po = i+1;
            }
            else{
                pred = i;
                po = 0;
            }

            // x
            if (it->points[po].point.x != it->points[pred].point.x){

                x1 = ((int) it->points[pred].point.x) / widthOfCell;
                x2 = ((int) it->points[po].point.x) / widthOfCell;

                if (x2 < x1){
                    int temp = x2;
                    x2 = x1;
                    x1 = temp;
                }

                for (int j = x1; j <= x2; j++){
                    arrayMap[j][((int) it->points[i].point.y) / widthOfCell] = 1;
                }

                // y
            }else{

                y1 = ((int) it->points[pred].point.y) / widthOfCell;
                y2 = ((int) it->points[po].point.y) / widthOfCell;

                if (y2 < y1){
                    int temp = y2;
                    y2 = y1;
                    y1 = temp;
                }

                for (int r = y1; r <= y2; r++){
                    arrayMap[((int) it->points[i].point.x) / widthOfCell][r] = 1;
                }
            }
        }
    }
}

//-----------Uloha2---------------------
void MainWindow::observeRoad()
{

    dist=sqrt(pow(xx-endTargetX,2.0)+pow(yy-endTargetY,2.0));//vzdialenost start->ciel
    alfa=atan((endTargetY-yy)/(endTargetX-xx));//uhol start->ciel
    alfa_z=atan(bb/dist);//uhol pre polomer robota
    obstacle2=false;

    start_region=fi*(180/PI)-(alfa)*(180/PI);
    if (start_region>0){
        start_region=360-start_region;
        start_region=start_region-(alfa)*(180/PI);
        end_region=start_region+(2*alfa_z*(180/PI));
        if(end_region > 360){
            end_region=end_region-360;
        }
    }
    if (start_region<=0){
        start_region=abs(start_region);
        start_region=start_region-(alfa)*(180/PI);
        end_region=start_region+(2*alfa_z*(180/PI));
        if(end_region > 360){
            end_region=end_region-360;
        }
    }

    for (int i=0;i<copyOfLaserData.numberOfScans;i++)
    {
        angle=360-copyOfLaserData.Data[i].scanAngle;//uhol z LIDARu

        condition1=(start_region < end_region && angle >= start_region && angle <= end_region);
        condition2=(start_region > end_region && (angle >= start_region || angle <= end_region));

        if(condition1 || condition2)//ak sa jedna o uhol prekazka
        {
            d_crit= abs(b/sin(angle));
            d_scan= copyOfLaserData.Data[i].scanDistance/1000;

            if((abs(d_crit) >= d_scan && abs(d_crit) <= dist ) || d_scan < dist)//v ceste je prekazka
            {
                printf("kolizia %d \n", 1);
                obstacle2=true;
                break;
            }
        }
    }

//ak je prekazka-----------------------------------
    if(obstacle2)
    {
        //on_pushButton_4_clicked(); //stop

        d_scan_1=copyOfLaserData.Data[0].scanDistance; //prve meranie

        for (int i=0;i<copyOfLaserData.numberOfScans;i++)
        {
            d_scan_2=copyOfLaserData.Data[i].scanDistance; //druhe meranie na najdenie rohu pr
            if(abs(d_scan_1-d_scan_2) > 2*b*1000) //ak je viac ako toto potom tam je roh pr
            {
                angle=fi-(copyOfLaserData.Data[i].scanAngle*(PI/180)); //uhol medzi
                d_scan= copyOfLaserData.Data[i].scanDistance; //vzdialenost Lidar
                dist=(d_scan+(d_scan_1-d_scan)/2);
                alfa=atan(b*1000/dist);//uhol od rohu do polomeru robota
                dist_b=(b*1000)/sin(alfa);//kriticka vzdialenost

                if( d_scan_1 < d_scan)//ak je roh v lavo
                {
                    x_p=(dist*cos(angle-alfa)+xx)+dist_b;//suradnice kam chem ist
                    y_p=(dist*sin(angle-alfa)+yy)+dist_b;
                }else if (d_scan_1 >= d_scan)//ak je roh v pravo
                {
                    x_p=(dist*cos(angle+alfa)+xx)+dist_b;//suradnice kam chcem ist
                    y_p=(dist*sin(angle+alfa)+yy)+dist_b;
                }
                transitions.push(make_pair(x_p,y_p));//vlozim

            }
            d_scan_1=copyOfLaserData.Data[i].scanDistance;//posuniem sa s meranim
        }
    }
}


void MainWindow::on_checkBox_clicked(bool checked)
{
    obstacle=checked;
}

