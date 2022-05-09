#include "map_loader.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include "mainwindow.h"



map_loader::map_loader()
{
   // std::cout<<"Ide"<<endl;
}

void map_loader::load_map(char *filename,TMapArea &mapss)
{   //std::cout<<"Ide"<<endl;

    FILE *fp=fopen(filename,"r");
    if (fp==NULL)
    {
        printf("zly file\n");
        return ;
    }

    //tu nacitame obvodovu stenu
    char myLine[550];
    fgets(myLine,550,fp);
    printf("%s\n",myLine);
    char *myCopy=(char*)calloc(strlen(myLine)+2,sizeof(char));
    memcpy(myCopy,myLine,sizeof(char)*strlen(myLine));
    char *freeMyCopy;
    freeMyCopy=myCopy;
    myCopy=strtok(myCopy,"[]");
    mapss.wall.numofpoints=(atoi(myCopy));
    printf("num of points %i\n",mapss.wall.numofpoints);
    mapss.wall.points.reserve(mapss.wall.numofpoints);
    for(int i=0;i<mapss.wall.numofpoints;i++)
    {
        TMapPoint temp;
        myCopy=strtok(NULL,"[,");
        temp.point.x=atof(myCopy);
        myCopy=strtok(NULL,"[,");
        temp.point.y=atof(myCopy);
        mapss.wall.points.push_back(temp);
        //   mapss.wall.points[i/2].suradnice[i%2]=atof(myCopy);


    }
    free(freeMyCopy);

    //tu nacitame jednotlive prekazky
    mapss.numofObjects=0;
    mapss.obstacle.clear();
    while( fgets(myLine,550,fp))
    {
        printf("%s\n",myLine);
        myCopy=(char*)calloc(strlen(myLine)+2,sizeof(char));
        memcpy(myCopy,myLine,sizeof(char)*strlen(myLine));

        freeMyCopy=myCopy;
        myCopy=strtok(myCopy,"[]");
        if((atoi(myCopy))==0)
            break;
        TMapObject tempObstacle;
        mapss.numofObjects++;

        tempObstacle.numofpoints=(atoi(myCopy));
        for(int i=0;i< tempObstacle.numofpoints;i++)
        {
            TMapPoint temp;
            myCopy=strtok(NULL,"[,");
            temp.point.x=atof(myCopy);
            myCopy=strtok(NULL,"[,");
            temp.point.y=atof(myCopy);
            tempObstacle.points.push_back(temp);

        }
        free(freeMyCopy);
        mapss.obstacle.push_back(tempObstacle);
    }
}
