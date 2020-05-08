/* Final year project,BEng Mechanical Engineering (Hons) */
/* Creation and machining of doubly curved surfaces      */
/* (c) xxx, 1999; Supervisor: yyy                        */
#include <conio.h>    /* for screen input/output   */
#include <stdio.h>    /* for screen input/output   */
#include <stdlib.h>   /* basic library             */
#include <graphics.h> /* for graphic routines      */
#include <math.h>     /* for surfaces calculations */
#include <string.h>   /* for files reading/writing */
#include <dos.h>      /* for interrupts            */
#include <stdarg.h>   /* for gprintf function only */
#include <bios.h>     /* for system checks         */
#include <dir.h>      /* for files list            */

/* CONSTANTS */
#define TOL 0.001        /* Lower limit before zero */
#define PI 3.14159265359 /* Pi value                */
#define ARRAY_MAX 10     /* Maximum m and n values  */

/* VARIABLES */
/*Define types */
typedef struct
{
    float x,y,z;
} Point_3D; /* Coordinates in 3D world */
typedef struct
{
    int x,y;
} Point_Scr; /* Coordinates in 2D screen */
/* World limits in 2D */
float WMinX,WMaxX;
float WMinY,WMaxY;
/* Rotations for 3D view */
int DRx;
int DRy;
int DRz;
float Cos_Rx;
float Sin_Rx;
float Cos_Ry;
float Sin_Ry;
float Cos_Rz;
float Sin_Rz;
/* CNC parameters */
int NO,NUMB,RPM,FEED;
float DIAM,RAW,ROUGH,FINISH;
float Radius;
/* Surfaces data */
int mp,np,Pts; /* Size of data points array   */
int k,l,Ord;   /* B-splines orders            */
float Delta;   /* Accuracy; default: 0.01     */
char Invert;   /* Invert normal vector or not */
/* Misc */
Point_3D Points[ARRAY_MAX][ARRAY_MAX];
struct REGPACK Reg;       /* For interrupts    */
float MaxZ,MinZ;          /* Maximum Z value   */
char MaxZOK;              /* Max Z calculated? */
unsigned char i,j;        /* Loops             */
unsigned char Type_Curve; /* Choices of user   */
FILE *fp;                 /* Pointer for files */

/*** FUNCTIONS DECLARATION ***/
/* CHECK SYSTEM */
void Check_System(void);
/* 3D plots */
void Init_Graphics(void);
float Deg_To_Rad(float Deg);
Point_3D W3D_To_2D(Point_3D p);
Point_Scr W2D_To_Device(Point_3D p);
void Draw_Point(Point_3D p1);
void Draw_Line(Point_3D p1,Point_3D p2);
void Draw_Axis(void);
void Calculate_Angles(void);
void W2D_Min_Max(void);
/* MENU */
void Menu(void);
void Tool_Bar(void);
void Plain_Button(char Text[4],char Color);
void Plus_Minus_Button(char Text[4],char Color);
void Display_Data(void);
/* SURFACE DATA */
void Random_Points(void);
void Get_Points(void);
/* BEZIER SURFACE */
unsigned long Factorial(int e);
float Power(float c,int d);
float Bernstein(int r,int s,float t);
void Bezier_Surface(void);
/* B-SPLINE SURFACE */
char Knot(char f);
float Blend(char g,char h,float t);
void B_Spline_Surface(void);
/* MOUSE */
void Mouse_Cursor(char On_Off);
int Get_Mouse_X(void);
int Get_Mouse_Y(void);
char Button(void);
void Set_Mouse_Pos(int x,int y);
void Set_Cursor_Hand(void);
/* DISPLAY 0F NUMERIC VALUES IN GRAPHIC MODE */
int gprintf(int *xloc,int *yloc,char *fmt,...);
/* TOOLPATH SIMULATION 0N SCREEN */
void Bezier_Sim(void);
void B_Spline_Sim(void);
/* CNC FILES / SAVE PARAMETERS / SAVE SURFACE DATA */
void Files_Menu(void);
void Program_NC(void);
void Bezier_NC(void);
void B_Spline_NC(void);
Point_3D Bezier_Normal(float a,float b);
Point_3D B_Spline_Normal(float a,float b);
void Data_File(void);

/*********************** MAIN PROGRAM *************************/
void main()
{
    unsigned char Choice;
    char Data[6];
    _setcursortype(_NOCURSOR);
    Check_System();
    /* CNC parameters initialisation */
    if((fp=fopen("PARAMETR.DAT","rt"))==NULL)
    {
        /* File does not exist -> initialise */
	      NO=1000;
        NUMB=1;
        RPM=4000;
        FEED=3000;
        DIAM=10.0;
        RAW=200.0;
        ROUGH=4.0;
        FINISH=1.0;
    }
    else
    {
        /* File exists -> read data */
        fread(Data,5,1,fp);
        NO=abs(atoi(Data));
        fread(Data,5,1,fp);
        NUMB=abs(atoi(Data));
        fread(Data,5,1,fp);
        RPM=abs(atoi(Data));
        fread(Data,5,1,fp);
        FEED=abs(atoi(Data));
        fread(Data,5,1,fp);
        DIAM=fabs(atof(Data));
        fread(Data,5,1,fp);
        RAW=fabs(atof(Data));
        fread(Data,5,1,fp);
        ROUGH=fabs(atof(Data));
        fread(Data,5,1,fp);
        FINISH=fabs(atof(Data));
    }
    fclose(fp);
    /* Initialise tool radius */
    Radius=DIAM/2;
    /* Menu display label */
ZoliMenu:
    Menu();
    /* World limits in 2D */
    W2D_Min_Max();
    /* Initialisation of graphic screen */
    Init_Graphics();
    Tool_Bar();
    /* Surface drawing */
    if(Type_Curve%2==0) B_Spline_Surface();
    else Bezier_Surface();
    /* Activation of mouse for user interactivity */
    Set_Cursor_Hand();
    do
    {
        /* Empty keyboard buffer */
        while(kbhit()) getch();
        Choice=0;
        if(Button()!=0&&Get_Mouse_X()>600)
        {
            Choice=Get_Mouse_Y()/48+1;
            if(Button()==1) switch(Choice)
                {
                case 1:
                    if(Get_Mouse_X()>620) DRx-=10;
                    else DRx+=10;
                    break;
                case 2:
                    if(Get_Mouse_X()>620) DRy-=10;
                    else DRy+=10;
                    break;
                case 3:
                    if(Get_Mouse_X()>620) DRz-=10;
                    else DRz+=10;
                    break;
                case 4:
                    if(Get_Mouse_X()>620) Delta-=0.005;
                    else Delta+= 0.005;
                    break;
                case 5:
                    if(Get_Mouse_X()>620) k--;
                    else k++;
                    break;
                case 6:
                    if(Get_Mouse_X()>620) l--;
                    else l++;
                    break;
                case 7:
                {
                    Files_Menu();
                    /* Back to graphic mode */
                    W2D_Min_Max();
                    Init_Graphics();
                    /* Display screen */
                    Tool_Bar();
                    if(Type_Curve%2==0) B_Spline_Surface();
                    else Bezier_Surface();
                    /* Activation of mouse for user interactivity */
                    Set_Cursor_Hand();
                    Choice=0;
                    break;
                }
                case 8:
                {
                    Mouse_Cursor(0);
                    if(Type_Curve%2==0) B_Spline_Surface();
                    else Bezier_Surface();
                    /* Activation of mouse for user interactivity */
                    Set_Cursor_Hand();
                    Choice=0;
                }
                }
            if(Button()==2) switch(Choice)
                {
                case 1:
                    if(Get_Mouse_X()>620) DRx-=90;
                    else DRx+=90;
                    break;
                case 2:
                    if(Get_Mouse_X()>620) DRy-=90;
                    else DRy+=90;
                    break;
                case 3:
                    if(Get_Mouse_X()>620) DRz-=90;
                    else DRz+=90;
                    break;
                case 4:
                    if(Get_Mouse_X()>620) Delta-=0.01;
                    else Delta+=0.01;
                    break;
                case 5:
                    if(Get_Mouse_X()>620) k=2;
                    else k=mp+1;
                    break;
                case 6:
                    if(Get_Mouse_X()>620) l=2;
                    else l=np+1;
                    break;
                case 7:
                {
                    Files_Menu();
                    /* Back to graphic mode */
                    W2D_Min_Max();
                    Init_Graphics();
                    /* Display screen */
                    Tool_Bar();
                    if(Type_Curve%2==0) B_Spline_Surface();
                    else Bezier_Surface();
                    /* Activation of mouse for user interactivity */
                    Set_Cursor_Hand();
                    Choice=0;
                    break;
                }
                case 8:
                {
                    Mouse_Cursor(0);
                    if(Type_Curve%2==0) B_Spline_Sim();
                    else Bezier_Sim();
                    /* Activation of mouse for user interactivity */
                    Set_Cursor_Hand();
                    Choice=0;
                }
                }
            if(Delta<0.005) Delta=0.005;
            if(Delta>0.1) Delta=0.1;
            if(DRx>=360) DRx-=360;
            if(DRy>=360) DRy-=360;
            if(DRz>=360) DRz-=360;
            if(DRx<0) DRx+=360;
            if(DRy<0) DRy+=360;
            if(DRz<0) DRz+=360;
            if(k<2) k=2;
            if(k>mp+1) k=mp+1;
            if(l<2) l=2;
            if(l>np+1) l=np+1;
            /* Calculations of new parameters */
            Calculate_Angles();
            W2D_Min_Max();
            Display_Data();
        }
        /* Wait mouse button to be released */
	      do {}
        while(Button()!=0);
    }
    while(Choice<=8);
    if(Choice==9) goto ZoliMenu;
    /* End of program */
    Mouse_Cursor(0);
    closegraph();
    textmode(C80);
    textbackground(BLACK);
    clrscr();
    _setcursortype(_NORMALCURSOR);
    /* Save parameters to PARAMETR.DAT */
    fp=fopen("PARAMETR.DAT","wt");
    itoa(NO,Data,10);
    fwrite(&Data,5,1,fp);
    itoa(NUMB,Data,10);
    fwrite(&Data,5,1,fp);
    itoa(RPM,Data,10);
    fwrite(&Data,5,1,fp);
    itoa(FEED,Data,10);
    fwrite(&Data,5,1,fp);
    gcvt(DIAM,5,Data);
    fwrite(&Data,5,1,fp);
    gcvt(RAW,5,Data);
    fwrite(&Data,5,1,fp);
    gcvt(ROUGH,5,Data);
    fwrite(&Data,5,1,fp);
    gcvt(FINISH,5,Data);
    fwrite(&Data,5,1,fp);
    fclose(fp);
    /* Exiting */
    textcolor(LIGHTCYAN);
    cprintf("Bye! See you soon!");
    exit(1);
}

/*********************** CHECK SYSTEM ***********************/
void Check_System(void)
{
    union REGS regs;
    int ret;
    textmode(C80);
    textbackground(BLUE);
    textcolor(LIGHTRED);
    clrscr();
    textcolor(LIGHTRED);
    gotoxy(13,5);
    cprintf("System self check information");
    gotoxy(13,6);
    cprintf("_____________________________");
    textcolor(LIGHTCYAN);
    /* Checking mouse */
    gotoxy(13,8);
    cprintf("Checking for mouse............... ");
    Reg.r_ax=0x00;
    intr(0x33,&Reg);
    if(Reg.r_ax==0)
    {
        _setcursortype(_NOCURSOR);
        textcolor(LIGHTRED+BLINK);
        cprintf("NOT FOUND!!!");
        textcolor(WHITE);
        cprintf("Exiting.");
        sound(220);
        delay(200);
        nosound();
        exit(0);
    }
    else
        cprintf("%d buttons",Reg.r_bx);
    /* MS DOS version */
    gotoxy(13,10);
    cprintf("MS-DOS version................... ");
    regs.h.ah=0x30;
    ret=intdos(&regs,&regs);
    cprintf("%d.%d",regs.h.al,regs.h.ah);
    /* Checking memory */
    gotoxy(13,12);
    cprintf("Checking memory.................. ");
    cprintf("%d KB",biosmemory());
    /* Checking default drive */
    gotoxy(13,14);
    cprintf("Current drive letter............. ");
    regs.h.ah=0x19;
    ret=intdos(&regs,&regs);
    cprintf("%c",regs.h.al+65);
    /* Checking coprocessor */
    gotoxy(13,16);
    cprintf("Checking coprocessor............. ");
    ret=biosequip();
    if(ret&0x0002) cprintf("Installed");
    else cprintf("Not installed");
    /* Checking DMA */
    gotoxy(13,18);
    cprintf("Direct Memory Access............. ");
    ret=biosequip();
    if(ret&0x0100) cprintf("No");
    else cprintf("Yes");
    /* Checking boot */
    gotoxy(13,20);
    cprintf("Boot from disk................... ");
    ret=biosequip();
    if(ret&0x0001) cprintf("Yes");
    else cprintf("No");
    /* End screen */
    textcolor(LIGHTMAGENTA+BLINK);
    gotoxy(13,22);
    cprintf("Press a key or mouse button to continue...");
    /* Empty keyboard buffer */
    while(kbhit()) getch();
    /* Wait mouse button to be released */
    do {}
    while(Button()!=0);
    /* Wait button or key to be pressed */
    while(!kbhit()&&Button()==0);
}

/*********************** 3D FUNCTIONS ***********************/
/* Graphics initialisation */
void Init_Graphics(void)
{
    int GDriver=DETECT,GMode,ErrorCode;
    initgraph(&GDriver,&GMode,"");
    /* Hardware test */
    if(GMode!=VGAHI)
    {
        clrscr();
        gotoxy(10,10);
        _setcursortype(_NOCURSOR);
        textcolor(LIGHTRED+BLINK);
        cprintf("NO VGA DEVICE!!!");
        gotoxy(10,12);
        textcolor(WHITE);
        cprintf("Sorry! You need a VGA card using high resolution.\n\n");
        sound(220);
	      delay(200);
	      nosound();
        exit(1);
    }
    /* Software test */
    ErrorCode=graphresult();
    if(ErrorCode!=grOk)
    {
        clrscr();
        gotoxy(10,10);
        _setcursortype(_NOCURSOR);
        textcolor(LIGHTRED+BLINK);
        cprintf("GRAPHIC ERROR!!!");
        gotoxy(10,12);
        textcolor(WHITE);
        cprintf("%s\n\n",grapherrormsg(ErrorCode));
        sound(220);
        delay(200);
        nosound();
        exit(1);
    }
}

/* Convert degrees to radians */
float Deg_To_Rad(float Deg)
{
    return(Deg*PI/180.0);
}

/* Calculate axes angles and factors */
void Calculate_Angles(void)
{
    Cos_Rx=cos(Deg_To_Rad(DRx));
    Sin_Rx=sin(Deg_To_Rad(DRx));
    Cos_Ry=cos(Deg_To_Rad(DRy));
    Sin_Ry=sin(Deg_To_Rad(DRy));
    Cos_Rz=cos(Deg_To_Rad(DRz));
    Sin_Rz=sin(Deg_To_Rad(DRz));
}

/* Convert a point from 3D world to 2D world */
Point_3D W3D_To_2D(Point_3D p)
{
    Point_3D pTemp;
    /* Rotation around X */
    pTemp.x=p.x;
    pTemp.y=Cos_Rx*p.y-Sin_Rx*p.z;
    pTemp.z=Sin_Rx*p.y+Cos_Rx*p.z;
    p=pTemp;
    /* Rotation around Y */
    pTemp.x=Cos_Ry*p.x+Sin_Ry*p.z;
    pTemp.z=-Sin_Ry*p.x+Cos_Ry*p.z;
    p=pTemp;
    /* Rotation around Z */
    pTemp.x=Cos_Rz*p.x-Sin_Rz*p.y;
    pTemp.y=Sin_Rz*p.x+Cos_Rz*p.y;
    /* Filter tiny values */
    if(fabs(pTemp.x)<TOL) pTemp.x=0.0;
    if(fabs(pTemp.y)<TOL) pTemp.y=0.0;
    if(fabs(pTemp.z)<TOL) pTemp.z=0.0;
    return(pTemp);
}

/* Convert a point from 2D world to screen coordinates */
Point_Scr W2D_To_Device(Point_3D p)
{
    Point_Scr pTemp;
    pTemp.x=(WMinX-p.x)*598/(WMinX-WMaxX)+0.5;
    pTemp.y=(WMaxY-p.y)*458/(WMaxY-WMinY)+0.5;
    return(pTemp);
}

/* Display a point from 3D world on current view port */
void Draw_Point(Point_3D p1)
{
    Point_Scr p2;
    p1.z=-p1.z;
    p2=W2D_To_Device(W3D_To_2D(p1));
    rectangle(p2.x-2,p2.y-2,p2.x+2,p2.y+2);
}

/* Display a line between pl and p2 from 3D world on current view port */
void Draw_Line(Point_3D p1,Point_3D p2)
{
    Point_Scr p11,p22;
    p1.z=-p1.z;
    p2.z=-p2.z;
    p11=W2D_To_Device(W3D_To_2D(p1));
    p22=W2D_To_Device(W3D_To_2D(p2));
    line(p11.x,p11.y,p22.x,p22.y);
}

/* Draw X,Y and Z axis on current view port */
void Draw_Axis(void)
{
    Point_3D p1,p2;
    p1.x=p1.y=p1.z=0;
    p2.x=10;
    p2.y=p2.z=0;
    setcolor(LIGHTRED);
    Draw_Line(p1,p2);
    p2.x=p2.z=0;
    p2.y=10;
    setcolor(LIGHTGREEN);
    Draw_Line(p1,p2);
    p2.x=p2.y=0;
    p2.z=10;
    setcolor(LIGHTBLUE);
    Draw_Line(p1,p2);
    setcolor(WHITE);
}

/* Initialise minimum and maximum for world 2D */
void W2D_Min_Max(void)
{
    Point_3D P2D,P3D;
    P3D=Points[0][0];
    P3D.z=-P3D.z;
    WMinX=WMaxX=W3D_To_2D(P3D).x;
    WMinY=WMaxY=W3D_To_2D(P3D).y;
    for(i=0; i<=np; i++)
        for(j=0; j<=mp; j++)
        {
            P3D=Points[i][j];
            P3D.z=-P3D.z;
            P2D=W3D_To_2D(P3D);
            if(P2D.x<WMinX) WMinX=P2D.x;
            if(P2D.x>WMaxX) WMaxX=P2D.x;
            if(P2D.y<WMinY) WMinY=P2D.y;
            if(P2D.y>WMaxY) WMaxY=P2D.y;
        }
}

/************************ MENUS ******************************/
/* Beginning menu */
void Menu(void)
{
    char Key,File[12],numb[9],Dir[MAXDIR];
    struct ffblk fic;
    int done;
    /* Display the menu */
Again:
    textmode(C80);
    textbackground(BLUE);
    clrscr();
    _setcursortype(_NOCURSOR);
    textcolor(LIGHTRED);
    gotoxy(13,5);
    cprintf("Creation and machining of doubly curved surfaces");
    gotoxy(13,6);
    cprintf("________________________________________________");
    textcolor(LIGHTCYAN);
    gotoxy(13,22);
    cprintf("Programmed by xxx,199x.");
    textcolor(LIGHTGREEN);
    gotoxy(13,10);
    cprintf("What would you like to do ?");
    textcolor(YELLOW);
    gotoxy(13,12);
    cprintf("l -> Bezier surface");
    gotoxy(13,13);
    cprintf("2 -> B-spline surface");
    gotoxy(13,14);
    cprintf("3 -> Random Bezier surface");
    gotoxy(13,15);
    cprintf("4 -> Random B-spline surface");
    gotoxy(13,16);
    cprintf("5 -> Load data from file");
    gotoxy(13,19);
    cprintf("Press 'Esc' to exit the program.");
    /* Empty keyboard buffer */
    while(kbhit()) getch();
    /* Get user choice */
    Mouse_Cursor(1);
    Set_Mouse_Pos(320,100);
    Key=0;
    do
    {
        if(kbhit()) Key=getch();
        if(Button()!=0&&Get_Mouse_X()>=96&&Get_Mouse_X()<=312) Key=(Get_Mouse_Y()-80)/8+48;
    }
    while((Key<49||Key>53)&&Key!=27&&Key!=56);
    /* Wait mouse button to be released */
    do {}
    while(Button()!=0);
    Mouse_Cursor(0);
    /* Exit progam ? */
    if(Key==27||Key==56)
    {
        clrscr();
        textcolor(LIGHTCYAN);
        _setcursortype(_NORMALCURSOR);
        cprintf("Bye! See you later!");
        exit(1);
    }
    /* If no,choice of user ? */
    Type_Curve=Key-48;
    _setcursortype(_SOLIDCURSOR);
    /* Loading data from file */
    if(Type_Curve==5)
    {
        /* Display list of files in current directory */
        clrscr();
        textcolor(LIGHTCYAN);
        getcwd(Dir,MAXDIR);
        cprintf("Current directory is %s",Dir);
        gotoxy(6,3);
        textcolor(LIGHTRED);
        cprintf("File name Control points");
        gotoxy(1,4);
        done=findfirst("*.SUR",&fic,0);
        if(done!=0)
        {
            _setcursortype(_NOCURSOR);
            textcolor(WHITE);
            gotoxy(6,5);
            cprintf("No .SUR files available in this directory!");
            gotoxy(6,7);
            textcolor(LIGHTMAGENTA);
            cprintf("Press a key to return to main menu.");
            sound(220);
            delay(200);
            nosound();
            while(kbhit()) getch();
            getch();
            goto Again;
        }
        while(!done)
        {
            printf(" %15s %31d\n",fic. ff_name,(fic.ff_fsize-4)/27);
            done=findnext(&fic);
        }
        /* Which file to open ? */
        textcolor(YELLOW);
        cprintf("\n Enter the name of the file to open (without .SUR): ");
        scanf("%8s",File);
        strcat(File,".SUR");
        /* Getting m and n values */
        if((fp=fopen(File,"rt"))==NULL)
        {
            /* File does not exist -> initialise */
            clrscr();
            _setcursortype(_NOCURSOR);
            gotoxy(13,10);
            textcolor(LIGHTRED+BLINK);
            cprintf("ERROR!!!");
            textcolor(WHITE);
            cprintf("File not found in current directory.");
            textcolor(LIGHTMAGENTA);
            gotoxy(13,12);
            cprintf("Press a key to return to main menu.");
            sound(220);
            delay(200);
            nosound();
            while(kbhit()) getch();
            getch();
            goto Again;
        }
        fp=fopen(File,"rt");
        fread(numb,2,1,fp);
        np=atoi(numb);
        fread(numb,2,1,fp);
        mp=atoi(numb);
        np--;
        mp--;
        /* Getting points coordinates */
        for(i=0; i<=np; i++)
            for(j=0; j<=mp; j++)
            {
                fread(numb,9,1,fp);
                Points[i][j].x=atof(numb);
                fread(numb,9,1,fp);
                Points[i][j].y=atof(numb);
                fread(numb,9,1,fp);
                Points[i][j].z=atof(numb);
            }
        fclose(fp);
        /* Bezier or B-Spline surface ? */
        clrscr();
        _setcursortype(_NOCURSOR);
        gotoxy(13,10);
        textcolor(LIGHTGREEN);
        cprintf("Which kind of surface would you like ?");
        textcolor(YELLOW);
        gotoxy(13,12);
        cprintf("l or LEFT mouse button-Bezier surface");
        gotoxy(13,14);
        cprintf("2 or RIGHT mouse button-B-spline surface");
        /* Empty keyboard buffer */
        while(kbhit()) getch();
        /* Wait mouse button to be released */
        do {}
        while(Button()!=0);
        do
        {
	    if(kbhit()) Key=getch();
                }
            while(Key<49||Key>50&&Button()==0);
        if(Key==49||Button()==1) Type_Curve=5;
        if(Key==50||Button()==2) Type_Curve=6;
        np++;
        mp++;
        _setcursortype(_SOLIDCURSOR);
    }
    else
    {
        /* Get m and n,number of control points */
        clrscr();
        textcolor(YELLOW);
        gotoxy(13,10);
        cprintf("Enter the value for m (4 to %d): ",ARRAY_MAX);
        scanf("%d",&mp);
        if(mp<4) mp=4;
        if(mp>ARRAY_MAX) mp=ARRAY_MAX;
        gotoxy(13,12);
        cprintf("Enter the value for n (4 to %d): ",ARRAY_MAX);
        scanf("%d",&np);
        if(np<4) np=4;
        if(np>ARRAY_MAX) np=ARRAY_MAX;
    }
    /* Get k and 1: orders for B-splines */
    if(Type_Curve%2==0)
    {
        clrscr();
        textcolor(YELLOW);
        gotoxy(13,10);
        cprintf("Enter the value for k (2 to %d): ",mp);
        scanf("%d",&k);
        if(k<2) k=2;
        if(k>mp) k=mp;
        gotoxy(13,12);
        cprintf("Enter the value for l (2 to %d): ",np);
	      scanf("%d",&l);
        if(l<2) l=2;
        if(l>np) l=np;
    }
    mp--;
    np--;
    /* Get points or random initialisation */
    switch(Type_Curve)
    {
    case 1:
        Get_Points();
        break;
    case 2:
        Get_Points();
        break;
    case 3:
        Random_Points();
        break;
    case 4:
	Random_Points();
    }
    /* Set increment value for u and v */
    Delta=0.01;
    /* Get the axes angles */
    DRx=60;
    DRy=320;
    DRz=30;
    Calculate_Angles();
}

/* Draw the toolbar on the right */
void Tool_Bar(void)
{
    setviewport(0,0,639,479,1);
    setcolor(WHITE);
    rectangle(0,0,600,460);
    rectangle(0,460,600,479);
    i=0;
    Plus_Minus_Button("Rx",LIGHTRED);
    Plus_Minus_Button("Ry",LIGHTGREEN);
    Plus_Minus_Button("Rz",LIGHTBLUE);
    Plus_Minus_Button("dudv",LIGHTCYAN);
    if(Type_Curve%2==0)
    {
        Plus_Minus_Button("k",YELLOW);
        Plus_Minus_Button("l",YELLOW);
    }
    i=6;
    Plain_Button("File",CYAN);
    Plain_Button("Draw",LIGHTMAGENTA);
    Plain_Button("New",LIGHTGRAY);
    Plain_Button("Quit",RED);
}

/* Draw a normal button */
void Plain_Button(char Text[4],char Color)
{
    i++;
    setcolor(Color);
    rectangle(601,(i-1)*48,639,i*48-1);
    outtextxy(600+(41-textwidth(Text))/2,i*48-28,Text);
}

/* Draw a "+|-" button */
void Plus_Minus_Button(char Text[4],char Color)
{
    i++;
    setcolor(Color);
    rectangle(601,(i-1)*48,639,i*48-1);
    outtextxy(600+(41-textwidth(Text))/2,i*48-37,Text);
    outtextxy(607,i*48-14,"+");
    outtextxy(626,i*48-14,"-");
    line(601,i*48-20,639,i*48-20);
    line(620,i*48-20,620,i*48-1);
}

/* Display data in bottom box */
void Display_Data(void)
{
    int x,y; /* Output text co-ordinates */
    setviewport(1,461,599,478,1);
    clearviewport();
    setcolor(LIGHTRED);
    x=26;
    y=6;
    gprintf(&x,&y,"Rx: %d",DRx);
    setcolor(LIGHTGREEN);
    x=98;
    y=6;
    gprintf(&x,&y,"Ry: %d",DRy);
    setcolor(LIGHTBLUE);
    x=170;
    y=6;
    gprintf(&x,&y,"Rz: %d",DRz);
    setcolor(LIGHTCYAN);
    x=242;
    y=6;
    gprintf(&x,&y,"Delta: %g",Delta);
    setcolor(LIGHTMAGENTA);
    x=354;
    y=6;
    gprintf(&x,&y,"m: %d n: %d",mp+1,np+1);
    setcolor(YELLOW);
    x=474;
    y=6;
    if(Type_Curve%2==0) gprintf(&x,&y,"k: %d l: %d",k,l);
}

/*********** SURFACE DATA *****************/
/* Create a random mesh of mp x np points */
void Random_Points(void)
{
    randomize();
    clrscr();
    Points[0][0].x=random(5)-10;
    Points[0][0].y=random(5)-10;
    Points[0][0].z=random(5)-10;
    for(j=1; j<=mp; j++)
    {
        Points[0][j].x=random(15)+5+Points[0][j-1].x;
        Points[0][j].y=15+Points[1][j-1].y;
        Points[0][j].z=random(15)+5+Points[0][j-1].z;
    }
    for(i=1; i<=np; i++)
        for(j=0; j<=mp; j++)
        {
            Points[i][j].x=15+Points[i-1][j].x;
            Points[i][j].y=random(15)+5+Points[i-1][j].y;
            Points[i][j].z=random(15)+5+Points[i-1][j].z;
        }
}

/* Get the control points co-ordinates using keyboard */
void Get_Points(void)
{
    textbackground(BLUE);
    for(i=0; i<=np; i++)
        for(j=0; j<=mp; j++)
        {
            clrscr();
            textcolor(LIGHTGREEN);
            gotoxy(13,12);
            cprintf("Enter the coordinates for point [%d,%d]",i,j);
            textcolor(YELLOW);
            gotoxy(13,14);
            cprintf("Enter the value for x (in mm): ");
            scanf("%f",&Points[i][j].x);
            gotoxy(13,16);
            cprintf("Enter the value for y (in mm): ");
            scanf("%f",&Points[i][j].y);
            gotoxy(13,18);
            cprintf("Enter the value for z {in mm): ");
            scanf("%f",&Points[i][j].z);
        }
}

/********************** BEZIER SURFACE ***********************/
/* Return factorial n */
unsigned long Factorial(int e)
{
    unsigned long Temp=1;
    while(e>1)
    {
        Temp*=e;
        e--;
    }
    return(Temp);
}

/* Return c power d */
float Power( float c,int d)
{
    float Temp=1;
    while(d>0)
    {
        Temp*=c;
        d--;
    }
    return(Temp);
}

/* Return Bernstein basis function */
float Bernstein(int r,int s,float t)
{
    return(Power(1-t,s-r)*Power(t,r)*Factorial(s)/Factorial(r)/Factorial(s-r));
}

/* Draw a Bezier surface */
void Bezier_Surface(void)
{
    float u,v; /* parameter */
    float Bern_u,Bern_v; /* blending functions */
    Point_Scr P2D; /* point to display */
    Point_3D P3D; /* calculated point */
    /* Display stop message */
    setviewport(1,461,599,478,1);
    clearviewport();
    setcolor(LIGHTGREEN);
    outtextxy(30,5,"Press any key to stop...");
    /* Initialise screen */
    setviewport(1,1,599,459,1);
    clearviewport();
    Draw_Axis();
    /* MinZ and MaxZ calculation */
    MaxZOK=0;
    MinZ=MaxZ=Points[0][0].z;
    /* Draw wireframe */
    for(i=0; i<=np; i++)
        for(j=0; j<=mp; j++)
        {
            setcolor(YELLOW);
            Draw_Point(Points[i][j]);
            setcolor(LIGHTMAGENTA);
            if(i>0) Draw_Line(Points[i-1][j],Points[i][j]);
            if(j>0) Draw_Line(Points[i][j-1],Points[i][j]);
        }
    /* Draw surface */
    setcolor(LIGHTCYAN);
    /* Empty keyboard buffer */
    while(kbhit()) getch();
    for(u=0; u<=1; u+=Delta)
    {
        if(kbhit()) break;
        for(v=0; v<=1; v+=Delta)
        {
            P3D.x=P3D.y=P3D.z=0;
            for(i=0; i<=np; i++)
            {
                Bern_u=Bernstein(i,np,u);
                for(j=0; j<=mp; j++)
                {
                    Bern_v=Bern_u*Bernstein(j,mp,v);
                    P3D.x+=Points[i][j].x*Bern_v;
                    P3D.y+=Points[i][j].y*Bern_v;
                    P3D.z+=Points[i][j].z*Bern_v;
                }
            }
            if(MinZ>P3D.z) MinZ=P3D.z;
            if(MaxZ<P3D.z) MaxZ=P3D.z;
            P3D.z=-P3D.z;
            P2D=W2D_To_Device(W3D_To_2D(P3D));
            putpixel(P2D.x,P2D.y,LIGHTCYAN);
        }
    }
    if(u>=1&&v>=1) MaxZOK=1;
    Display_Data();
    sound(880);
    delay(60);
    nosound();
    /* Empty keyboard buffer */
    while(kbhit()) getch();
}

/************************ B-SPLINE SURFACE ***********************/
/* Calculate the knot vector for a B-spline */
char Knot(char f)
{
    if(f<Ord) return(0);
    else if(f>Pts) return(Pts-Ord+2);
    else return(f-Ord+1);
}

/* Calculate the basis function for a B-spline */
float Basis(char g,char h,float t)
{
    register int r;
    float s;
    if(h==1)
    {
        s=0;
        if(Knot(g)<=t&&t<Knot(g+1)) s=1;
    }
    else
    {
        s=0;
        r=Knot(g+h-1)-Knot(g);
        if(r!=0) s=(t-Knot(g))*Basis(g,h-1,t)/r;
        r=Knot(g+h)-Knot(g+1);
        if(r!=0) s+=(Knot(g+h)-t)*Basis(g+1,h-1,t)/r;
    }
    return(s);
}

/* Draw a B-spline surface */
void B_Spline_Surface(void)
{
    float u,v; /* parameter */
    float Basis_u,Basis_v; /* basis functions */
    Point_Scr P2D; /* point to display */
    Point_3D P3D; /* calculated point */
    /* Display stop message */
    setviewport(1,461,599,478,1);
    clearviewport();
    setcolor(LIGHTGREEN);
    outtextxy(30,5,"Press any key to stop...");
    /* Initialise screen */
    setviewport(1,1,599,459,1);
    clearviewport();
    Draw_Axis();
    /* MinZ and MaxZ calculation */
    MaxZOK=0;
    MinZ=MaxZ=Points[0][0].z;
    /* Draw wireframe */
    for(i=0; i<=np; i++)
        for(j=0; j<=mp; j++)
        {
            setcolor(YELLOW);
            Draw_Point(Points[i][j]);
            setcolor(LIGHTMAGENTA);
            if(i>0) Draw_Line(Points[i-1][j],Points[i][j]);
            if(j>0) Draw_Line(Points[i][j-1],Points[i][j]);
        }
    /* Draw surface */
    setcolor(LIGHTCYAN);
    /* Empty keyboard buffer */
    while(kbhit()) getch();
    for(u=0; u<=np-l+2; u+=Delta)
    {
        if(kbhit()) break;
        for(v=0; v<=mp-k+2; v+=Delta)
        {
            P3D.x=P3D.y=P3D.z=0;
            for(i=0; i<=np; i++)
            {
                Pts=np;
                Ord=l;
                Basis_u=Basis(i,l,u);
                for(j=0; j<=mp; j++)
                {
                    Pts=mp;
                    Ord=k;
                    Basis_v=Basis_u*Basis(j,k,v);
                    P3D.x+=Points[i][j].x*Basis_v;
                    P3D.y+=Points[i][j].y*Basis_v;
                    P3D.z+=Points[i][j].z*Basis_v;
                }
            }
            if(MinZ>P3D.z) MinZ=P3D.z;
            if(MaxZ<P3D.z) MaxZ=P3D.z;
            P3D.z=-P3D.z;
            P2D=W2D_To_Device(W3D_To_2D(P3D));
            putpixel(P2D.x,P2D.y,LIGHTCYAN);
        }
    }
    if((u>=np-l+2)&&(v>=mp-k+2)) MaxZOK=1;
    Display_Data();
    sound(880);
    delay(60);
    nosound();
    /* Empty keyboard buffer */
    while(kbhit()) getch();
}

/************** MOUSE *************/
/* Enable or disable mouse cursor */
void Mouse_Cursor(char On_Off)
{
    if(On_Off==0)
        Reg.r_ax=0x02;
    else
        Reg.r_ax=0x01;
    intr(0x33,&Reg);
}

/* Return the current mouse X coordinate */
int Get_Mouse_X(void)
{
    Reg.r_ax=0x03;
    intr(0x33,&Reg);
    return(Reg.r_cx);
}

/* Return the current mouse Y coordinate */
int Get_Mouse_Y(void)
{
    Reg.r_ax=0x03;
    intr(0x33,&Reg);
    return(Reg.r_dx);
}

/* Return the state of the mouse buttons */
char Button(void)
{
    Reg.r_ax=0x03;
    intr(0x33,&Reg);
    return(Reg.r_bx);
}

/* Set position of mouse cursor to x and y */
void Set_Mouse_Pos(int x,int y)
{
    Reg.r_ax=0x04;
    Reg.r_cx=x;
    Reg.r_dx=y;
    intr(0x33,&Reg);
}

/* Set a hand icon cursor for mouse */
void Set_Cursor_Hand(void)
{
//    struct SREGS sr;
    /* Pointing hand image */
//    static unsigned handIm[32]=
//    {
//	    0xF3FF,0xE1FF,0xE1FF,0xE1FF,/* screen mask */
//	    0xE1FF,0xE001,0xE000,0xE000,
//	    0x8000,0x0000,0x0000,0x0000,
//	    0x0000,0x0000,0x0000,0x8001,
//	    0x0C00,0x1200,0x1200,0x1200,/* cursor mask */
//	    0x1200,0x13B6,0x1249,0x1249,
//	    0x7249,0x9001,0x9001,0x9001,
//	    0x8001,0x8001,0x8001,0x7FFE
//    };
//    segread(&sr);
//    Reg.r_ax=0x09;
    /* Hotpoint: bx=5; cx=0 */
//    Reg.r_bx=0x05;
//    Reg.r_cx=0x00;
//    Reg.r_dx=255; //handIm;
//    Reg.r_es=sr.ds;
//    intr(0x33,&Reg);
    Mouse_Cursor(1);
    Set_Mouse_Pos(320,240);
}

/************** GPRINTF *************/
/* GPRINTF: Used like PRINTF except the output is sent to the */
/* screen in graphics mode at the specified co-ordinate. */
int gprintf(int *xloc,int *yloc,char *fmt,...)
{
    va_list argptr; /* Argument list pointer */
    char str[140]; /* Buffer to build sting into */
    int cnt; /* Result of SPRINTF for return */
    va_start(argptr,fmt); /* Initialize va_ functions */
    cnt=vsprintf(str,fmt,argptr); /* prints string to buffer */
    outtextxy(*xloc,*yloc,str); /* Send string in graphics mode */
    *yloc+=textheight("H")+2; /* Advance to next line */
    va_end(argptr); /* Close va_ functions */
    return(cnt); /* Return the conversion count */
}

/********************* TOOLPATH SIMULATION ***********************/
/* Simulate Bezier surface toolpath on screen */
void Bezier_Sim(void)
{
    float u,v; /* Parameters */
    float Bern_u,Bern_v,Norm; /* Blending functions */
    Point_Scr P2D; /* Point to display */
    Point_3D P3D,Normal; /* Calculated point */
    unsigned char Direction; /* Direction of toolpath */
    /* Display stop message */
    setviewport(1,461,599,478,1);
    clearviewport();
    setcolor(LIGHTCYAN);
    outtextxy(30,5,"Press any key to stop...");
    /* Initialise screen */
    setviewport(1,1,599,459,1);
    clearviewport();
    Draw_Axis();
    Invert=0;
    /* Empty keyboard buffer */
    while(kbhit()) getch();
    /* X and Y of first point */
    setcolor(LIGHTMAGENTA);
    P3D=Points[0][0];
    P3D.x-=Radius+10;
    P3D.y-=Radius+10;
    P3D.z=-RAW-Radius-10;
    P2D=W2D_To_Device(W3D_To_2D(P3D));
    moveto(P2D.x,P2D.y);
    P3D.x=P3D.y=P3D.z=0;
    for(i=0; i<=np; i++)
    {
        Bern_u=Bernstein(i,np,0);
        for(j=0; j<=mp; j++)
        {
            Bern_v=Bern_u*Bernstein(j,mp,0);
            P3D.x+=Points[i][j].x*Bern_v;
            P3D.y+=Points[i][j].y*Bern_v;
            P3D.z+=Points[i][j].z*Bern_v;
        }
    }
    /* Calculate normal vector to the surface at this point */
    Normal=Bezier_Normal(0,0);
    /* Implement tool radius to reduce redundant calculations */
    Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
    /* Update point coordinates with tool radius */
    P3D.x+=Normal.x*Norm;
    P3D.y+=Normal.y*Norm;
    P3D.z+=Normal.z*Norm;
    /* Show tool position on screen */
    P3D.z=-P3D.z;
    P2D=W2D_To_Device( W3D_To_2D(P3D));
    lineto(P2D.x,P2D.y);
    /* Starting toolpath */
    setcolor(LIGHTGREEN);
    Direction=1;
    for(u=0; u<=1; u+=Delta)
    {
        if(kbhit()) break;
        Direction++;
        if(Direction%2==0)
        {
            Direction=0;
            for(v=0; v<=1; v+=Delta)
            {
                /* Calculate real coordinates of point */
                P3D.x=P3D.y=P3D.z=0;
                for(i=0; i<=np; i++)
                {
                    Bern_u=Bernstein(i,np,u);
                    for(j=0; j<=mp; j++)
                    {
                        Bern_v=Bern_u*Bernstein(j,mp,v);
                        P3D.x+=Points[i][j].x*Bern_v;
                        P3D.y+=Points[i][j].y*Bern_v;
                        P3D.z+=Points[i][j].z*Bern_v;
                    }
                }
                /* Calculate normal vector to the surface at this point */
                Normal=Bezier_Normal(u,v);
                if(Normal.z<0) setcolor(LIGHTRED);
                else setcolor(LIGHTGREEN);
                /* Implement tool radius to reduce redundant calculations */
                Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
                /* Update point coordinates with tool radius */
                P3D.x+=Normal.x*Norm;
                P3D.y+=Normal.y*Norm;
                P3D.z+=Normal.z*Norm;
                /* Show tool position on screen */
                P3D.z=-P3D.z;
                P2D=W2D_To_Device(W3D_To_2D(P3D));
                lineto(P2D.x,P2D.y);
            }
        }
        else
        {
            for(v=1; v>=0; v-=Delta)
            {
                /* Calculate real coordinates of point */
                P3D.x=P3D.y=P3D.z=0;
                for(i=0; i<=np; i++)
                {
                    Bern_u=Bernstein(i,np,u);
                    for(j=0; j<=mp; j++)
                    {
                        Bern_v=Bern_u*Bernstein(j,mp,v);
                        P3D.x+=Points[i][j].x*Bern_v;
                        P3D.y+=Points[i][j].y*Bern_v;
                        P3D.z+=Points[i][j].z*Bern_v;
                    }
                }
                /* Calculate normal vector to the surface at this point */
                Normal=Bezier_Normal(u,v);
                if(Normal.z<0) setcolor(LIGHTRED);
                else setcolor(LIGHTGREEN);
                /* Implement tool radius to reduce redundant calculations */
                Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
                /* Update point coordinates with tool radius */
                P3D.x+=Normal.x*Norm;
                P3D.y+=Normal.y*Norm;
                P3D.z+=Normal.z*Norm;
                /* Show tool position on screen */
                P3D.z=-P3D.z;
                P2D=W2D_To_Device(W3D_To_2D(P3D));
                lineto(P2D.x,P2D.y);
            }
        }
    }
    /* Finished. Take tool away X=0,Y=0 */
    setcolor(LIGHTMAGENTA);
    P3D.z=-RAW-Radius-10;
    P2D=W2D_To_Device(W3D_To_2D(P3D));
    lineto(P2D.x,P2D.y);
    Display_Data();
    sound(880);
    delay(60);
    nosound();
    /* Empty keyboard buffer */
    while(kbhit()) getch();
}

/* Simulate B-spline surface toolpath on screen */
void B_Spline_Sim(void)
{
    float u,v; /* Parameter */
    float Basis_u,Basis_v,Norm; /* Basis functions */
    Point_Scr P2D; /* Point to display */
    Point_3D P3D,Normal; /* Calculated point */
    unsigned char Direction,mpk,npl; /* Direction of toolpath */
    /* Avoid redundant calculations */
    mpk=mp-k+2;
    npl=np-l+2;
    /* Display stop message */
    setviewport(1,461,599,478,1);
    clearviewport();
    setcolor(LIGHTCYAN);
    outtextxy(30,5,"Press any key to stop...");
    /* Initialise screen */
    setviewport(1,1,599,459,1);
    clearviewport();
    Draw_Axis();
    Invert=0;
    /* Empty keyboard buffer */
    while(kbhit()) getch();
    /* X and Y of first point */
    setcolor(LIGHTMAGENTA);
    P3D=Points[0][0];
    P3D.x-=Radius+10;
    P3D.y-=Radius+10;
    P3D.z=-RAW-Radius-10;
    P2D=W2D_To_Device(W3D_To_2D(P3D));
    moveto(P2D.x,P2D.y);
    P3D.x=P3D.y=P3D.z=0;
    for(i=0; i<=np; i++)
    {
        Pts=np;
        Ord=l;
        Basis_u=Basis(i,l,0);
        for(j=0; j<=mp; j++)
        {
            Pts=mp;
            Ord=k;
            Basis_v=Basis_u*Basis(j,k,0);
            P3D.x+=Points[i][j].x*Basis_v;
            P3D.y+=Points[i][j].y*Basis_v;
            P3D.z+=Points[i][j].z*Basis_v;
        }
    }
    /* Calculate normal vector to the surface at this point */
    Normal=B_Spline_Normal(0,0);
    /* Implement tool radius to reduce redundant calculations */
    Norm =Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
    /* Update point coordinates with tool radius */
    P3D.x+=Normal.x*Norm;
    P3D.y+=Normal.y*Norm;
    P3D.z+=Normal.z*Norm;
    /* Show tool position on screen */
    P3D.z=-P3D.z;
    P2D=W2D_To_Device(W3D_To_2D(P3D));
    lineto(P2D.x,P2D.y);
    /* Starting toolpath */
    setcolor(LIGHTGREEN);
    Direction=1;
    for(u=0; u<=npl-TOL; u+=Delta)
    {
	if(kbhit()) break;
		Direction++;
		if(Direction%2==0)
        {
            Direction=0;
            for(v=0; v<=mpk-TOL; v+=Delta)
                {
                    /* Calculate real coordinates of point */
                    P3D.x=P3D.y=P3D.z=0;
                    for(i=0; i<=np; i++)
                    {
                        Pts=np;
                        Ord=l;
                        Basis_u=Basis(i,l,u);
                        for(j=0; j<=mp; j++)
                        {
                            Pts=mp;
                            Ord=k;
                            Basis_v=Basis_u*Basis(j,k,v);
                            P3D.x+=Points[i][j].x*Basis_v;
                            P3D.y+=Points[i][j].y*Basis_v;
                            P3D.z+=Points[i][j].z*Basis_v;
                        }
                    }
                    /* Calculate normal vector to the surface at this point */
                    Normal=B_Spline_Normal(u,v);
                    if(Normal.z<0) setcolor(LIGHTRED);
                    else setcolor(LIGHTGREEN);
                    /* Implement tool radius to reduce redundant calculations */
                    Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
                    /* Update point coordinates with tool radius */
                    P3D.x+=Normal.x*Norm;
                    P3D.y+=Normal.y*Norm;
                    P3D.z+=Normal.z*Norm;
                    /* Show tool position on screen */
                    P3D.z=-P3D.z;
                    P2D=W2D_To_Device(W3D_To_2D(P3D));
                    lineto(P2D.x,P2D.y);
                }
            }
            else
            {
                for(v=mpk-Delta; v>=0; v-=Delta)
                {
                    /* Calculate real coordinates of point */
                    P3D.x=P3D.y=P3D.z=0;
                    for(i=0; i<=np; i++)
                    {
                        Pts=np;
                        Ord=l;
                        Basis_u=Basis(i,l,u);
                        for(j=0; j<=mp; j++)
                        {
                            Pts=mp;
                            Ord=k;
                            Basis_v=Basis_u*Basis(j,k,v);
                            P3D.x+=Points[i][j].x*Basis_v;
                            P3D.y+=Points[i][j].y*Basis_v;
                            P3D.z+=Points[i][j].z*Basis_v;
                        }
                    }
                    /* Calculate normal vector to the surface at this point */
                    Normal=B_Spline_Normal(u,v);
                    if(Normal.z<0) setcolor(LIGHTRED);
                    else setcolor(LIGHTGREEN);
                    /* Implement tool radius to reduce redundant calculations */
                    Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
                    /* Update point coordinates with tool radius */
                    P3D.x+=Normal.x*Norm;
                    P3D.y+=Normal.y*Norm;
                    P3D.z+=Normal.z*Norm;
                    /* Show tool position on screen */
                    P3D.z=-P3D.z;
                    P2D=W2D_To_Device(W3D_To_2D(P3D));
                    lineto(P2D.x,P2D.y);
                }
            }
    }
    /* Finished. Take tool away X=0,Y=0 */
    setcolor(LIGHTMAGENTA);
    P3D.z=-RAW-Radius-10;
    P2D=W2D_To_Device(W3D_To_2D(P3D));
    lineto(P2D.x,P2D.y);
    Display_Data();
    sound(880);
    delay(60);
    nosound();
    /* Empty keyboard buffer */
    while(kbhit()) getch();
}

/************* FILES MENU ************/
/* Menu for creation of files */
void Files_Menu(void)
{
    char Key; /* User choice */
    char File[9]; /* File name */
    /* Text mode initialisation */
    Mouse_Cursor(0);
    closegraph();
    textmode(C80);
FilesMenu:
    /* Display files menu */
    textbackground(BLUE);
    textcolor(LIGHTRED);
    clrscr();
    _setcursortype(_NOCURSOR);
    gotoxy(13,5);
    cprintf("Writing data to file");
    gotoxy(13,6);
    cprintf("____________________");
    textcolor(LIGHTCYAN);
    gotoxy(13,22);
    cprintf("Programmed by xxx,199x.");
    textcolor(LIGHTGREEN);
    gotoxy(13,10);
    cprintf("What would you like to do?");
    textcolor(YELLOW);
    gotoxy(13,12);
    cprintf("l -> Save surface data to file");
    gotoxy(13,13);
    cprintf("2 -> Create CNC program");
    gotoxy(13,14);
    cprintf("3 -> Create CNC points co-ordinates");
    gotoxy(13,15);
    cprintf("4 -> Show/modify CNC program parameters");
    gotoxy(13,17);
    cprintf("Type 'Esc' to go back to graphics");
    /* Empty keyboard buffer */
    while(kbhit()) getch();
    /* Wait mouse button to be released */
    do {}
    while(Button()!=0);
    /* Get user choice */
    Key=0;
    Mouse_Cursor(1);
    Set_Mouse_Pos(320,100);
    do
    {
        if(kbhit()) Key=getch();
        if(Button()!=0&&Get_Mouse_X()>=88&&Get_Mouse_X()<=392) Key=(Get_Mouse_Y()-80)/8+48;
    }
    while((Key<49||Key>52)&&Key!=27&&Key!=54);
    /* Wait mouse button to be released */
    do {}
    while(Button()!=0);
    Mouse_Cursor(0);
    if(Key==27||Key==54) goto Fin;
    _setcursortype(_SOLIDCURSOR);
    Key-=48;
    switch(Key)
    {
    case 1:
    {
        /* Get file name */
        clrscr();
        _setcursortype(_SOLIDCURSOR);
        textcolor(YELLOW);
        gotoxy(14,10);
        cprintf("Enter the name of the data file to create: ");
        scanf("%8s",File);
        strcat(File,".SUR");
        fp=fopen(File,"wt");
        Data_File();
        fclose(fp);
        sound(880);
        delay(60);
        nosound();
        break;
    }
    case 2:
    {
        clrscr();
        if(MaxZOK==0)
        {
            _setcursortype(_NOCURSOR);
            textcolor(LIGHTRED+BLINK);
            gotoxy(10,10);
            cprintf("WARNING!!!");
            textcolor(WHITE);
            cprintf("Minimum/maximum Z values NOT calculated.");
            gotoxy(10,12);
            cprintf("Please draw the WHOLE surface and try again.");
            textcolor(LIGHTMAGENTA);
            gotoxy(10,14);
            cprintf("Press any key to go back to files menu.");
            sound(220);
            delay(200);
            nosound();
            while(kbhit()) getch();
            getch();
            break;
        }
        if(MaxZ>RAW)
        {
            _setcursortype(_NOCURSOR);
            textcolor(LIGHTRED+BLINK);
            gotoxy(10,10);
            cprintf("WARNING!!!");
            textcolor(WHITE);
            cprintf("Maximum Z value is beyond raw material height.");
            gotoxy(10,12);
            cprintf("Please redefine the height of raw material block.");
            textcolor(LIGHTMAGENTA);
            gotoxy(10,14);
            cprintf("Press any key to go back to files menu.");
            sound(220);
            delay(200);
            nosound();
            while(kbhit()) getch();
            getch();
            break;
        }
        _setcursortype(_SOLIDCURSOR);
        textcolor(YELLOW);
        gotoxy(14,10);
        cprintf("Enter the name of the CNC file to create: ");
        scanf("%8s",File);
        strcat(File,".NC");
        _setcursortype(_NOCURSOR);
        fp=fopen(File,"wt");
        Program_NC();
        fclose(fp);
        sound(880);
        delay(60);
        nosound();
        break;
    }
    case 3:
    {
        /* Get file name */
        clrscr();
        _setcursortype(_SOLIDCURSOR);
        textcolor(YELLOW);
        gotoxy(14,10);
        cprintf("Enter the name of the CNC file to create: ");
        scanf("%8s",File);
        strcat(File,".PTS");
        _setcursortype(_NOCURSOR);
        textcolor(LIGHTCYAN);
        gotoxy(14,13);
        cprintf("Writing data to %s",File);
        gotoxy(14,15);
	cprintf("ÉÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ»");
	gotoxy(14,16);
	cprintf("º                                                  º");
	gotoxy(14,17);
	cprintf("ÈÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍÍ¼");
	fp=fopen(File,"wt");
	if(Type_Curve%2==0) B_Spline_NC();
	else Bezier_NC();
        fclose(fp);
        sound(880);
	delay(60);
        nosound();
        break;
    }
    case 4:
    {
Parameters:
        clrscr();
        _setcursortype(_NOCURSOR);
        textcolor(LIGHTRED);
        gotoxy(13,5);
        cprintf("Actual CNC parameters");
        gotoxy(13,6);
        cprintf("_____________________");
        textcolor(LIGHTCYAN);
        gotoxy(13,22);
	cprintf("Programmed by xxx,199x.");
        textcolor(YELLOW);
        gotoxy(13,9);
	cprintf("l-Macro program number: %d",NO);
	gotoxy(13,10);
        cprintf("2-Number of the tool: %d",NUMB);
        gotoxy(13,11);
        cprintf("3-Diameter of the tool (mm): %f",DIAM);
        gotoxy(13,12);
        cprintf("4-Rotation velocity of tool (rev/min): %d",RPM);
        gotoxy(13,13);
        cprintf("5-Feeding rate of the tool (mm/min): %d",FEED);
        gotoxy(13,14);
        cprintf("6-Height of raw material block (mm): %f",RAW);
        gotoxy(13,15);
        cprintf("7-Height of rough cutting cuts (mm): %f",ROUGH);
        gotoxy(13,16);
        cprintf("8-Height of finishing cut (mm): %f",FINISH);
        gotoxy(13,19);
        cprintf("Press a number to change value or 'Esc' to continue.");
        /* Empty keyboard buffer */
        while(kbhit()) getch();
        /* Wait mouse button to be released */
        do {}
	while(Button()!=0);
        /* Get user choice */
        Mouse_Cursor(1);
        Set_Mouse_Pos(320,100);
        Key=0;
        do
        {
            if(kbhit()) Key=getch();
            if(Button()!=0&&Get_Mouse_X()>=88&&Get_Mouse_X()<=600) Key=(Get_Mouse_Y()-56)/8+48;
        }
        while((Key<49||Key>56)&&Key!=27&&Key!=59);
        /* Wait mouse button to be released */
        do {}
        while(Button()!=0);
        Mouse_Cursor(0);
        if(Key!=27&&Key!=59)
        {
            _setcursortype(_SOLIDCURSOR);
            Key-=48;
            gotoxy(55,Key+8);
	    cprintf(" ");
            gotoxy(55,Key+8);
            switch(Key)
            {
            case 1:
                scanf("%d",&NO);
                break;
            case 2:
                scanf("%d",&NUMB);
                break;
            case 3:
                scanf("%f",&DIAM);
                break;
            case 4:
                scanf("%d",&RPM);
                break;
            case 5:
                scanf("%d",&FEED);
                break;
            case 6:
		scanf("%f",&RAW);
                break;
            case 7:
                scanf("%f",&ROUGH);
                break;
            case 8:
                scanf("%f",&FINISH);
            }
            NO=abs(NO);
            NUMB=abs(NUMB);
            DIAM=fabs(DIAM);
            RPM=abs(RPM);
            FEED=abs(FEED);
            RAW=fabs(RAW);
            ROUGH=fabs(ROUGH);
            FINISH=fabs(FINISH);
            /* Initialise tool radius value */
            Radius=DIAM/2;
            goto Parameters;
        }
    }
    }
    goto FilesMenu;
Fin:
}

/* Generate the common skeleton of CNC program */
void Program_NC(void)
{
    char far *str1,str2[10],essai[33]; /* String to write */
    float num;
    int p,Passes;
    /* Write beginning lines to file */
    fwrite("%\nO100\nG21\nG00 G17 G40 G49 G80 G94\nG54\nM09\nT",44,1,fp);
    itoa(NUMB,str2,10);
    strcpy(essai,str2);
    str1=" M06\nS";
    _fstrcat(essai,str1);
    itoa(RPM,str2,10);
    strcat(essai,str2);
    str1=" M03";
    _fstrcat(essai,str1);
    fwrite(&essai,strlen(essai),1,fp);
    /* Calculations of number of rough cutting passes to tool */
    Passes=ceil((RAW-MinZ-FINISH)/ROUGH);
    /* Redefinition of work coordinate system offset */
    str1="\n#5223=#5223+";
    _fstrcpy(essai,str1);
    num=Passes*ROUGH+FINISH;
    gcvt(num,7,str2);
    strcat(essai,str2);
    fwrite(&essai,strlen(essai),1,fp);
    /* Call of sub program for rough cutting */
    for(p=0; p<Passes; p++)
    {
        /* Offset Z for rough cutting */
        str1="\n#5223=#5223-";
        _fstrcpy(essai,str1);
        num=ROUGH;
        gcvt(num,7,str2);
	strcat(essai,str2);
        fwrite(&essai,strlen(essai),1,fp);
        /* X and Y of first point */
        str1="\nG00 X";
        _fstrcpy(essai,str1);
        num=Points[0][0].x-Radius-10;
        if(fabs(num)<TOL) num=0;
        gcvt(num,7,str2);
        strcat(essai,str2);
        str1=" Y";
        _fstrcat(essai,str1);
        num=Points[0][0].y-Radius-10;
        if(fabs(num)<TOL) num=0;
        gcvt(num,7,str2);
        strcat(essai,str2);
        fwrite(&essai,strlen(essai),1,fp);
        str1="\nG43 H";
        _fstrcpy(essai,str1);
        itoa(NUMB,str2,10);
        strcat(essai,str2);
	/* Max Z */
        str1=" Z";
        _fstrcat(essai,str1);
        num=RAW+Radius+10;
        if(fabs(num)<TOL) num=0;
        gcvt(num,7,str2);
        strcat(essai,str2);
        str1="\nG01 F";
        _fstrcat(essai,str1);
        itoa(FEED,str2,10);
        strcat(essai,str2);
        fwrite(&essai,strlen(essai),1,fp);
        /* Call macro */
        str1="\nG65 P";
        _fstrcpy(essai,str1);
        itoa(NO,str2,10);
        strcat(essai,str2);
        fwrite(&essai,strlen(essai),1,fp);
        /* Take tool away from surface */
        str1="\nG00 Z";
	_fstrcpy(essai,str1);
        num=RAW+Radius+10;
        if(fabs(num)<TOL) num=0;
        gcvt(num,7,str2);
        strcat(essai,str2);
        fwrite(&essai,strlen(essai),1,fp);
    }
    /* Offset Z for finishing cut */
    str1="\n#5223=#5223-";
    _fstrcpy(essai,str1);
    num=FINISH;
    gcvt(num,7,str2);
    strcat(essai,str2);
    fwrite(&essai,strlen(essai),1,fp);
    /* X and Y of first point */
    str1="\nG00 X";
    _fstrcpy(essai,str1);
    num=Points[0][0].x-Radius-10;
    if(fabs(num)<TOL) num=0;
    gcvt(num,7,str2);
    strcat(essai,str2);
    str1=" Y";
    _fstrcat(essai,str1);
    num=Points[0][0].y-Radius-10;
    if(fabs(num)<TOL) num=0;
    gcvt(num,7,str2);
    strcat(essai,str2);
    fwrite(&essai,strlen(essai),1,fp);
    str1="\nG43 H";
    _fstrcpy(essai,str1);
    itoa(NUMB,str2,10);
    strcat(essai,str2);
    /* Max Z */
    str1=" Z";
    _fstrcat(essai,str1);
    num=RAW+Radius+10;
    if(fabs(num)<TOL) num=0;
    gcvt(num,7,str2);
    strcat(essai,str2);
    str1="\nG01 F";
    _fstrcat(essai,str1);
    itoa(FEED,str2,10);
    strcat(essai,str2);
    fwrite(&essai,strlen(essai),1,fp);
    /* Call of sub program for finish cutting */
    str1="\nG65 P";
    _fstrcpy(essai,str1);
    itoa(NO,str2,10);
    strcat(essai,str2);
    fwrite(&essai,strlen(essai),1,fp);
    /* Take tool away from surface */
    str1="\nG00 Z";
    _fstrcpy(essai,str1);
    num=RAW+Radius+10;
    if(fabs(num)<TOL) num=0;
    gcvt(num,7,str2);
    strcat(essai,str2);
    fwrite(&essai,strlen(essai),1,fp);
    /* Write final lines for program finishing */
    str1="\nT0\nG53 X0 Y0\nM30\n%";
    _fstrcpy(essai,str1);
    fwrite(&essai,strlen(essai),1,fp);
}

/* Calculate Bezier surface toolpath and write points to file */
void Bezier_NC(void)
{
    char far *str1,str2[10],essai[33]; /* String to write */
    float u,v,num; /* Parameters */
    float Bern_u,Bern_v,Norm; /* Blending functions */
    Point_3D P3D,Normal; /* Calculated point */
    unsigned char Direction; /* Direction of toolpath */
    unsigned char Percent; /* Progress bar */
    /* Declaration of macro program number */
    str1="%\nO";
    _fstrcpy(essai,str1);
    itoa(NO,str2,10);
    strcat(essai,str2);
    fwrite(&essai,strlen(essai),1,fp);
    /* Write CNC data to file */
    Direction=1;
    textcolor(WHITE);
    for(u=0; u<=1; u+=Delta)
    {
        gotoxy(38,18);
        Percent=floor(100*u);
        cprintf("%d \%",Percent);
        Direction++;
        if(Direction%2==0)
        {
            Direction=0;
            for(v=0; v<=1; v+=Delta)
            {
                Percent=floor(u*v*50);
		gotoxy(15+Percent,16);
		cprintf("Û");
		/* Calculate real coordinates of point */
		P3D.x=P3D.y=P3D.z=0;
		for(i=0; i<=np; i++)
		{
		    Bern_u=Bernstein(i,np,u);
		    for(j=0; j<=mp; j++)
		    {
			Bern_v=Bern_u*Bernstein(j,mp,v);
			P3D.x+=Points[i][j].x*Bern_v;
			P3D.y+=Points[i][j].y*Bern_v;
			P3D.z+=Points[i][j].z*Bern_v;
		    }
		}
		/* Calculate normal vector to the surface at this point */
		Normal=Bezier_Normal(u,v);
		/* Implement tool radius to reduce redundant calculations */
		Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
		/* Update point coordinates with tool radius */
		P3D.x+=Normal.x*Norm;
		P3D.y+=Normal.y*Norm;
		P3D.z+=Normal.z*Norm;
		/* Write toolpath coordinates to file */
		str1="\nX";
		_fstrcpy(essai,str1);
		num=(ceil(P3D.x*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		str1="Y";
		_fstrcat(essai,str1);
		num=(ceil(P3D.y*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		str1="Z";
		_fstrcat(essai,str1);
		num=(ceil(P3D.z*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		fwrite(&essai,strlen(essai),1,fp);
	    }
	}
	else
	{
	    for(v=1; v>=0; v-=Delta)
	    {
		Percent=floor(u*(1-v)*50);
		gotoxy(15+Percent,16);
		cprintf("Û");
		/* Calculate real coordinates of point */
		P3D.x=P3D.y=P3D.z=0;
		for(i=0; i<=np; i++)
		{
		    Bern_u=Bernstein(i,np,u);
		    for(j=0; j<=mp; j++)
		    {
			Bern_v=Bern_u*Bernstein(j,mp,v);
			P3D.x+=Points[i][j].x*Bern_v;
			P3D.y+=Points[i][j].y*Bern_v;
			P3D.z+=Points[i][j].z*Bern_v;
		    }
		}
		/* Calculate normal vector to the surface at this point */
		Normal=Bezier_Normal(u,v);
		/* Implement tool radius to reduce redundant calculations */
		Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
		/* Update point coordinates with tool radius */
		P3D.x+=Normal.x*Norm;
		P3D.y+=Normal.y*Norm;
		P3D.z+=Normal.z*Norm;
		/* Write toolpath coordinates to file */
		str1="\nX";
		_fstrcpy(essai,str1);
		num=(ceil(P3D.x*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		str1="Y";
		_fstrcat(essai,str1);
		num=(ceil(P3D.y*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		str1="Z";
		_fstrcat(essai,str1);
		num=(ceil(P3D.z*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		fwrite(&essai,strlen(essai),1,fp);
	    }
	}
    }
    /* End of program */
    str1="\nM99\n%";
    _fstrcpy(essai,str1);
    fwrite(&essai,strlen(essai),1,fp);
}

/* Calculate B-spline surface toolpath and write points to file */
void B_Spline_NC(void)
{
    char far *str1,str2[10],essai[33]; /* String to write */
    float u,v,num; /* Parameter */
    float Basis_u,Basis_v,Norm; /* Basis functions */
    Point_3D P3D,Normal; /* Calculated point */
    unsigned char Direction,mpk,npl; /* Direction of toolpath */
    unsigned char Percent; /* Progress bar */
    /* Avoid redundant calculations */
    mpk=mp-k+2;
    npl=np-l+2;
    /* Declaration of macro program number */
    str1="%\nO";
    _fstrcpy(essai,str1);
    itoa(NO,str2,10);
    strcat(essai,str2);
    fwrite(&essai,strlen(essai),1,fp);
    /* Write CNC data to file */
    Direction=1;
    textcolor(WHITE);
    for(u=0; u<=npl-TOL; u+=Delta)
    {
	gotoxy(38,18);
	Percent=floor(100*u/npl);
	cprintf("%d \%",Percent);
	Direction++;
	if(Direction%2==0)
	{
	    Direction=0;
	    for(v=0; v<=mpk-TOL; v+=Delta)
	    {
		Percent=floor(u*v*50/mpk/npl);
		gotoxy(15+Percent,16);
		cprintf("Û");
		/* Calculate real coordinates of point */
		P3D.x=P3D.y=P3D.z=0;
		for(i=0; i<=np; i++)
		{
		    Pts=np;
		    Ord=l;
		    Basis_u=Basis(i,l,u);
		    for(j=0; j<=mp; j++)
		    {
			Pts=mp;
			Ord=k;
			Basis_v=Basis_u*Basis(j,k,v);
			P3D.x+=Points[i][j].x*Basis_v;
			P3D.y+=Points[i][j].y*Basis_v;
			P3D.z+=Points[i][j].z*Basis_v;
		    }
		}
		/* Calculate normal vector to the surface at this point */
		Normal=B_Spline_Normal(u,v);
		/* Implement tool radius to reduce redundant calculations */
		Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
		/* Update point coordinates with tool radius */
		P3D.x+=Normal.x*Norm;
		P3D.y+=Normal.y*Norm;
		P3D.z+=Normal.z*Norm;
		/* Write toolpath coordinates to file */
		str1="\nX";
		_fstrcpy(essai,str1);
		num=(ceil(P3D.x*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		str1="Y";
		_fstrcat(essai,str1);
		num=(ceil(P3D.y*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		str1="Z";
		_fstrcat(essai,str1);
		num=(ceil(P3D.z*1000))/1000;
		if(fabs(num)<TOL) num=0;
		gcvt(num,7,str2);
		strcat(essai,str2);
		fwrite(&essai,strlen(essai),1,fp);
	    }
	}
	else
	{
	    for(v=mpk-Delta; v>=0; v-=Delta)
	    {
		Percent=floor(u*(mpk-v)*50/mpk/npl);
		gotoxy(15+Percent,16);
		cprintf("Û");
		/* Calculate real coordinates of point */
		P3D.x=P3D.y=P3D.z=0;
		for(i=0; i<=np; i++)
		{
		    Pts=np;
		    Ord=l;
		    Basis_u=Basis(i,l,u);
		    for(j=0; j<=mp; j++)
		    {
                        Pts=mp;
                        Ord=k;
                        Basis_v=Basis_u*Basis(j,k,v);
                        P3D.x+=Points[i][j].x*Basis_v;
                        P3D.y+=Points[i][j].y*Basis_v;
                        P3D.z+=Points[i][j].z*Basis_v;
                    }
                }
                /* Calculate normal vector to the surface at this point */
                Normal=B_Spline_Normal(u,v);
                /* Implement tool radius to reduce redundant calculations */
                Norm=Radius/sqrt(Normal.x*Normal.x+Normal.y*Normal.y+Normal.z*Normal.z);
                /* Update point coordinates with tool radius */
                P3D.x+=Normal.x*Norm;
                P3D.y+=Normal.y*Norm;
                P3D.z+=Normal.z*Norm;
                /* Write toolpath coordinates to file */
                str1="\nX";
                _fstrcpy(essai,str1);
                num=(ceil(P3D.x*1000))/1000;
                if(fabs(num)<TOL) num=0;
                gcvt(num,7,str2);
                strcat(essai,str2);
                str1="Y";
                _fstrcat(essai,str1);
                num=(ceil(P3D.y*1000))/1000;
                if(fabs(num)<TOL) num=0;
                gcvt(num,7,str2);
                strcat(essai,str2);
                str1="Z";
                _fstrcat(essai,str1);
                num=(ceil(P3D.z*1000))/1000;
                if(fabs(num)<TOL) num=0;
                gcvt(num,7,str2);
                strcat(essai,str2);
                fwrite(&essai,strlen(essai),1,fp);
            }
        }
    }
    /* End of program */
    str1="\nM99\n%";
    _fstrcpy(essai,str1);
    fwrite(&essai,strlen(essai),1,fp);
}
/* Calculate normal vector at one point of Bezier surface */
Point_3D Bezier_Normal(float a,float b)
{
    Point_3D V1,V2,Vect; /* Vect=resultant normal vector */
    unsigned char l1,l2; /* Loops */
    float Ber_u,Ber_v; /* Inc=neighbour point parameter */
    if(a>1-TOL) a=1-TOL;
    if(a<TOL) a=TOL;
    if(b>1-TOL) b=1-TOL;
    if(b<TOL) b=TOL;
    /* Calculate the first tangent vector */
    V1.x=V1.y=V1.z=0;
    for(l1=0; l1<=np; l1++)
    {
        Ber_u=Bernstein(l1,np,a+TOL);
        for(l2=0; l2<=mp; l2++)
        {
            Ber_v=Ber_u*Bernstein(l2,mp,b);
	    V1.x+=Points[l1][l2].x*Ber_v;
	    V1.y+=Points[l1][l2].y*Ber_v;
	    V1.z+=Points[l1][l2].z*Ber_v;
	}
    }
    for(l1=0; l1<=np; l1++)
    {
	Ber_u=Bernstein(l1,np,a-TOL);
	for(l2=0; l2<=mp; l2++)
	{
	    Ber_v=Ber_u*Bernstein(l2,mp,b);
	    V1.x-=Points[l1][l2].x*Ber_v;
	    V1.y-=Points[l1][l2].y*Ber_v;
	    V1.z-=Points[l1][l2].z*Ber_v;
	}
    }
    /* Calculate the second tangent vector */
    V2.x=V2.y=V2.z=0;
    for(l1=0; l1<=np; l1++)
    {
	Ber_u=Bernstein(l1,np,a);
	for(l2=0; l2<=mp; l2++)
	{
	    Ber_v=Ber_u*Bernstein(l2,mp,b+TOL);
	    V2.x+=Points[l1][l2].x*Ber_v;
	    V2.y+=Points[l1][l2].y*Ber_v;
	    V2.z+=Points[l1][l2].z*Ber_v;
	}
    }
    for(l1=0; l1<=np; l1++)
    {
	Ber_u=Bernstein(l1,np,a);
	for(l2=0; l2<=mp; l2++)
	{
	    Ber_v=Ber_u*Bernstein(l2,mp,b-TOL);
	    V2.x-=Points[l1][l2].x*Ber_v;
	    V2.y-=Points[l1][l2].y*Ber_v;
	    V2.z-=Points[l1][l2].z*Ber_v;
	}
    }
    /* Vectorial product of tangent vectors give normal vector */
    Vect.x=V1.y*V2.z-V1.z*V2.y;
    Vect.y=V1.z*V2.x-V1.x*V2.z;
    Vect.z=V1.x*V2.y-V1.y*V2.x;
    if((a==0)&&(b==0)&&(Vect.z<0)) Invert=1;
    if(Invert==1)
    {
	Vect.x=-Vect.x;
	Vect.y=-Vect.y;
	Vect.z=-Vect.z;
    }
    return(Vect);
}
/* Calculate normal vector at one point of B-spline surface */
Point_3D B_Spline_Normal(float a,float b)
{
    Point_3D V1,V2,Vect; /* Vect=resultant normal vector */
    unsigned char l1,l2; /* Loops */
    float Bas_u,Bas_v; /* Inc=neighbour point parameter */
    if(a>np-l+2-TOL) a=np-l+2-TOL;
    if(a<TOL) a=TOL;
    if(b>mp-k+2-TOL) b=mp-k+2-TOL;
    if(b<TOL) b=TOL;
    /* Calculate the first tangent vector */
    V1.x=V1.y=V1.z=0;
    for(l1=0; l1<=np; l1++)
    {
        Pts=np;
        Ord=l;
        Bas_u=Basis(l1,l,a+TOL);
        for(l2=0; l2<=mp; l2++)
        {
            Pts=mp;
            Ord=k;
            Bas_v=Bas_u*Basis(l2,k,b);
            V1.x+=Points[l1][l2].x*Bas_v;
            V1.y+=Points[l1][l2].y*Bas_v;
            V1.z+=Points[l1][l2].z*Bas_v;
        }
    }
    for(l1=0; l1<=np; l1++)
    {
        Pts=np;
        Ord=l;
        Bas_u=Basis(l1,l,a-TOL);
        for(l2=0; l2<=mp; l2++)
        {
            Pts=mp;
            Ord=k;
            Bas_v=Bas_u*Basis(l2,k,b);
            V1.x-=Points[l1][l2].x*Bas_v;
            V1.y-=Points[l1][l2].y*Bas_v;
            V1.z-=Points[l1][l2].z*Bas_v;
        }
    }
    /* Calculate the second tangent vector */
    V2.x=V2.y=V2.z=0;
    for(l1=0; l1<=np; l1++)
    {
        Pts=np;
        Ord=l;
        Bas_u=Basis(l1,l,a);
        for(l2=0; l2<=mp; l2++)
        {
            Pts=mp;
            Ord=k;
            Bas_v=Bas_u*Basis(l2,k,b+TOL);
            V2.x+=Points[l1][l2].x*Bas_v;
            V2.y+=Points[l1][l2].y*Bas_v;
            V2.z+=Points[l1][l2].z*Bas_v;
        }
    }
    for(l1=0; l1<=np; l1++)
    {
        Pts=np;
        Ord=l;
        Bas_u=Basis(l1,l,a);
        for(l2=0; l2<=mp; l2++)
        {
            Pts=mp;
            Ord=k;
            Bas_v=Bas_u*Basis(l2,k,b-TOL);
            V2.x-=Points[l1][l2].x*Bas_v;
            V2.y-=Points[l1][l2].y*Bas_v;
            V2.z-=Points[l1][l2].z*Bas_v;
        }
    }
    /* Vectorial product of tangent vectors give normal vector */
    Vect.x=V1.y*V2.z-V1.z*V2.y;
    Vect.y=V1.z*V2.x-V1.x*V2.z;
    Vect.z=V1.x*V2.y-V1.y*V2.x;
    if(a==0&&b==0&&Vect.z<0) Invert=1;
    if(Invert==1)
    {
        Vect.x=-Vect.x;
        Vect.y=-Vect.y;
        Vect.z=-Vect.z;
    }
    return(Vect);
}

/* Write surface data to file for further use */
void Data_File(void)
{
    char Coord[10]; /* string to write */
    float num3; /* Co-ordinates */
    itoa(np+1,Coord,10);
    fwrite(&Coord,2,1,fp);
    itoa(mp+1,Coord,10);
    fwrite(&Coord,2,1,fp);
    for(i=0; i<=np; i++)
        for(j=0; j<=mp; j++)
        {
            num3=Points[i][j].x;
            gcvt(num3,7,Coord);
            fwrite(&Coord,9,1,fp);
            num3=Points[i][j].y;
            gcvt(num3,7,Coord);
            fwrite(&Coord,9,1,fp);
            num3=Points[i][j].z;
            gcvt(num3,7,Coord);
            fwrite(&Coord,9,1,fp);
        }
}
/************************ END 0F PROGRAM *************************/
