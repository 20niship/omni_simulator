#include "Simulator.h"

#define WHEEL_NUM 3
#define WALL_NUM  10
#define ZOOM_SIZE 3.0

namespace Simulator{
typedef struct{
    dBodyID  body;
    dGeomID  geom;
    dJointID joint;
} MyObject;


dWorldID world;
dSpaceID space;
dGeomID ground;
dJointGroupID contactgroup;
dsFunctions fn;
int argc;
char **argv;
int last_command;

dGeomID Walls[WALL_NUM];

// objects
MyObject wheel[WHEEL_NUM], base, ball;
const dReal WHEEL_MASS = 0.3; //kg
const dReal BASE_MASS  = 2.5; //kg
const dReal BASE_MASS_CENTER[3] = {0.0, 0.0, 0.0};

const dReal WHEEL_SIZE = 0.06015 * ZOOM_SIZE;
const dReal WHEEL_WIDTH = WHEEL_SIZE * 0.2 * ZOOM_SIZE; 
const dReal ROBOT_SIZE = 0.27 * ZOOM_SIZE;; // ロボットの中心からオムニまでの距離
const dReal DEFAULT_ATTITUDE[4] = {2.7 * ZOOM_SIZE, 0.25 * ZOOM_SIZE, 0.02 * ZOOM_SIZE, 0.0}; // (x,y,z,theta)
const dReal MOTOR_F_MAX = 10.0;

const dReal WALL_WIDTH = 0.001;
const dReal WALL_HEIGHT = 0.076;

// [[SIZE], [POS]]
dReal WALL_ATTITUDE[WALL_NUM*2][3] = {
        {4.238, WALL_WIDTH, WALL_HEIGHT}, {2.119, 3.638, WALL_HEIGHT/2.0},
        {4.238, WALL_WIDTH, WALL_HEIGHT}, {2.119, 0.0, WALL_HEIGHT/2.0},
        {WALL_WIDTH, 3.638, WALL_HEIGHT}, {4.238, 1.819, WALL_HEIGHT/2.0},
        {WALL_WIDTH, 3.638, WALL_HEIGHT}, {0.0, 1.819, WALL_HEIGHT/2.0},

        {WALL_WIDTH, 3.0, WALL_HEIGHT}, {1.200, 1.519, WALL_HEIGHT/2.0},
        {WALL_WIDTH, 3.0, WALL_HEIGHT}, {3.038, 2.157, WALL_HEIGHT/2.0},
        
        {0.562, WALL_WIDTH, WALL_HEIGHT}, {3.319, 1.800, WALL_HEIGHT/2.0},
        {0.562, WALL_WIDTH, WALL_HEIGHT}, {3.957, 2.400, WALL_HEIGHT/2.0},

        {1.762, 0.276, 0.04}, {2.119, 1.819, 0.02},
        {0.276, 0.276, 0.30}, {2.119, 1.819, 0.15}
    };



void makeRobot(){
    std::cout << "[Simulator] Making Robot...\n";
    //dMatrix3 R;
    dMass mass;

    // 土台を作成
    base.body  = dBodyCreate(world);

    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass, BASE_MASS, 3, ROBOT_SIZE - WHEEL_WIDTH/2.0, WHEEL_SIZE * 0.6);
    //dMassAdjust (&mass, BASE_MASS); // 追加
    for(int i=0; i<3; i++) mass.c[i] = BASE_MASS_CENTER[i];
    
    dBodySetMass(base.body,&mass);

    base.geom = dCreateCylinder(space, ROBOT_SIZE - WHEEL_WIDTH/2.0, WHEEL_SIZE * 0.6);
    dGeomSetBody(base.geom, base.body);
    dBodySetPosition(base.body, DEFAULT_ATTITUDE[0], DEFAULT_ATTITUDE[1], DEFAULT_ATTITUDE[2] + WHEEL_SIZE/2.0);


    dQuaternion Q1, Q2, Q3;
    dQFromAxisAndAngle (Q1, 1, 0, 0, M_PI/2.0);

    for(int i =0; i<WHEEL_NUM; i++){
        wheel[i].body = dBodyCreate(world);

        dQFromAxisAndAngle(Q1, 1, 0, 0, M_PI/2.0);
        dQFromAxisAndAngle(Q2, 0, 0, 1, M_PI * 2.0 * dReal(i)/dReal(WHEEL_NUM) + M_PI /2.0);
        
        dQMultiply0 (Q3, Q2, Q1);
        dBodySetQuaternion(wheel[i].body, Q3);

        wheel[i].geom = dCreateCylinder(space, WHEEL_SIZE, WHEEL_WIDTH);
        dGeomSetBody(wheel[i].geom, wheel[i].body);
        dBodySetPosition(wheel[i].body, DEFAULT_ATTITUDE[0] + ROBOT_SIZE * cos(M_PI * 2.0 * dReal(i) / WHEEL_NUM), 
                                        DEFAULT_ATTITUDE[1] + ROBOT_SIZE * sin(M_PI * 2.0 * dReal(i) / WHEEL_NUM),
                                         DEFAULT_ATTITUDE[2] + WHEEL_SIZE/2.0);

        wheel[i].joint = dJointCreateHinge(world,0);
        dJointAttach(wheel[i].joint, base.body, wheel[i].body);
        dJointSetHingeAxis(wheel[i].joint, cos(M_PI * 2.0 * dReal(i) / WHEEL_NUM), sin(M_PI * 2.0 * dReal(i) / WHEEL_NUM), 0);
        dJointSetHingeAnchor(wheel[i].joint, DEFAULT_ATTITUDE[0] + (ROBOT_SIZE - WHEEL_WIDTH/2.0) * cos(M_PI * 2.0 * dReal(i) / WHEEL_NUM),
                                             DEFAULT_ATTITUDE[1] + (ROBOT_SIZE - WHEEL_WIDTH/2.0) * sin(M_PI * 2.0 * dReal(i) / WHEEL_NUM), 
                                             DEFAULT_ATTITUDE[2] + WHEEL_SIZE/2.0);

    }
}

void makeWorld(){
    std::cout << "[Simulator] Making World...\n";

    // ボールを作成
    dMass mass;
    ball.body = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetSphereTotal(&mass,0.45, 0.11);
    dBodySetMass(ball.body, &mass);
    ball.geom = dCreateSphere(space, 0.11);
    dGeomSetBody(ball.geom,ball.body);
    dBodySetPosition(ball.body, 1.0, 0.0, 0.14);


    // 壁を作成
    for (int i = 0; i < WALL_NUM; i++)
    {
        Walls[i] = dCreateBox(space, WALL_ATTITUDE[i*2][0], WALL_ATTITUDE[i*2][1], WALL_ATTITUDE[i*2][2]);
        dGeomSetPosition(Walls[i], WALL_ATTITUDE[i*2+1][0], WALL_ATTITUDE[i*2+1][1], WALL_ATTITUDE[i*2+1][2]);
    }
}


void DrawRobot(){
    //std::cout << "[Simulator] Draw Robot...\n";

    dReal radius, length;
    // Draw Base
    // dsSetColor(0.0, 0.0, 1.3);  
    dsSetColor(0.1, 0.1, 0.1);
    dGeomCylinderGetParams(base.geom, &radius, &length);
    dsDrawCylinder(dGeomGetPosition(base.geom),
                    dGeomGetRotation(base.geom),length, radius);


    //Draw Wheel
    dsSetColor(1.1,1.1,1.1);

    for (int i=0; i< WHEEL_NUM; i++){
        dGeomCylinderGetParams(wheel[i].geom, &radius, &length);
        dsDrawCylinder(dGeomGetPosition(wheel[i].geom),
                       dGeomGetRotation(wheel[i].geom),length, radius);
    }

}


void DrawWorld(){
    //std::cout << "[Simulator] Draw World...\n";

    //Draw Ball
    dsSetColor(1.0,0.0,0.0);
        dsDrawSphere(dGeomGetPosition(ball.geom),
                    dGeomGetRotation(ball.geom),dGeomSphereGetRadius(ball.geom));

    // Draw Goal
    dsSetTexture(DS_NONE);
    for (int i = 0; i < WALL_NUM; i++){
        dsSetColor(1.3, 1.3, 0.5);
        dsDrawBox(dGeomGetPosition(Walls[i]), dGeomGetRotation(Walls[i]), WALL_ATTITUDE[i*2]);
    }
}


/*** �~�̕`�� ***/
void drawCircle(dReal r, dReal pos_X, dReal pos_Y, int angle1, int angle2)
{
    dReal pos1[3],pos2[3], z=0.005;
    pos1[0] = r * cos(angle1 * M_PI/180.0) + pos_X;
    pos1[1] = r * sin(angle1 * M_PI/180.0) + pos_Y;
    pos1[2] = z;
    for (int i = angle1; i < angle2; i++)
    {
        pos2[0] = r* cos(i * M_PI/180.0) + pos_X;
        pos2[1] = r* sin(i * M_PI/180.0) + pos_Y;
        pos2[2] = z;
        dsDrawLine(pos1,pos2);
        pos1[0] = pos2[0];
        pos1[1] = pos2[1];
        pos1[2] = pos2[2];
    }
}

/*** �����̕`�� ***/
void drawLine(){
    dReal z = 0.005;
    dReal center_r = 1.0, corner_r = 0.5;
    dReal pos[][3] = {{6.0, 4.0, z},{6.0, -4.0, z},{-6.0, -4.0, z},
        {-6.0, 4.0, z},{ 0.0, 4.0, z},{ 0.0, -4.0, z},
        { 6.0, 1.5, z},{ 5.5, 1.5, z},{ 5.5, -1.5, z}, { 6.0, -1.5, z},
        {-6.0, 1.5, z},{-5.5, 1.5, z},{-5.5, -1.5, z}, {-6.0, -1.5, z},
        { 6.0, 2.0, z},{ 4.5, 2.0, z},{ 4.5, -2.0, z}, { 6.0, -2.0, z},
        {-6.0, 2.0, z},{-4.5, 2.0, z},{-4.5, -2.0, z}, {-6.0, -2.0, z}
    };
    dsSetColor(1.3,1.3,1.3);
    dsDrawLine(pos[0],pos[1]);
    dsDrawLine(pos[1],pos[2]);
    dsDrawLine(pos[2],pos[3]);
    dsDrawLine(pos[3],pos[0]);
    dsDrawLine(pos[4],pos[5]);
    dsDrawLine(pos[6],pos[7]);
    dsDrawLine(pos[7],pos[8]);
    dsDrawLine(pos[8],pos[9]);
    dsDrawLine(pos[10],pos[11]);
    dsDrawLine(pos[11],pos[12]);
    dsDrawLine(pos[12],pos[13]);
    dsDrawLine(pos[14],pos[15]);
    dsDrawLine(pos[15],pos[16]);
    dsDrawLine(pos[16],pos[17]);
    dsDrawLine(pos[18],pos[19]);
    dsDrawLine(pos[19],pos[20]);
    dsDrawLine(pos[20],pos[21]);

    // �Z���^�[�T�[�N���̕`��
    drawCircle(center_r, 0, 0, 0, 360);
    drawCircle(corner_r,-6,-4,0,90);
    drawCircle(corner_r, -6,4,-90,0);
    drawCircle(corner_r,6,4,90,180);
    drawCircle(corner_r,6,-4,-180,-90);
}


//void setCamPos(float *pos, float *hpr){
void setCamPos(){
  float _xyz[3] = {2.0, -0.5, 10.0};    // 視点[m]
  float _hpr[3] = {-75, 0, 0}; // 視線[°]
  dsSetViewpoint (_xyz,_hpr);           // 視点と視線の設定
}



/*** �X�^�[�g�֐� ***/
static void start(){
    float xyz[3] = {2.0, -0.5, 5.0};
    float hpr[3] = { 30.0f, -30.0f, 0.0f};
    dsSetViewpoint(xyz,hpr);
    dsSetSphereQuality(3);
    //setCamPos();
}

/*** �R�[���o�b�N�֐� ***/
static void nearCallback(void *data, dGeomID o1, dGeomID o2){
    dVector3 tmp_fdir = {0, 0, 0, 0};
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

    int wheel_flag = 0;
    for (int j = 0; j < WHEEL_NUM; j++){
        if ((o1 == wheel[j].geom)||(o2 == wheel[j].geom)){
            wheel_flag = 1;
            dJointGetHingeAxis(wheel[j].joint,tmp_fdir);
            break;
        }
    }

    static const int N = 10;
    dContact contact[N];
    int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (n > 0)
    {
        if (wheel_flag == 1)
        {
            for (int i=0; i<n; i++){
                contact[i].surface.mode =  dContactFDir1| dContactMu2 | dContactSoftERP | dContactSoftCFM; //表面の性質 (接触フラグ)	 
                contact[i].fdir1[0] = tmp_fdir[0];   // 
                contact[i].fdir1[1] = tmp_fdir[1]; 	 // 
                contact[i].fdir1[2] = tmp_fdir[2];   //
                contact[i].surface.mu =  0.3;        // 摩擦係数 (摩擦方向1)	0	dInfinity)
                contact[i].surface.mu2 = dInfinity;  // 摩擦係数 (摩擦方向2)    0	dInfinity
                contact[i].surface.soft_erp = 0.9;   // ERPの値	
                contact[i].surface.soft_cfm = 0.001; // CFMの値

                dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
                dJointAttach(c,b1,b2);
            }
        }
        else
        {
            for (int i=0; i<n; i++)
            {
                contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
                contact[i].surface.mu  	= dInfinity;
                contact[i].surface.soft_erp = 0.8;
                contact[i].surface.soft_cfm = 1e-5;
                dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
                dJointAttach(c,b1,b2);
            }
        }
    }

    
}

static void command(int cmd)
{
    last_command = cmd;
    switch (cmd)
    {
    case '1':
        float xyz1[3],hpr1[3];
        dsGetViewpoint(xyz1,hpr1);
        printf("xyz=%4.2f %4.2f %4.2f ",xyz1[0],xyz1[1],xyz1[2]);
        printf("hpr=%6.2f %6.2f %5.2f \n",hpr1[0],hpr1[1],hpr1[2]);
        break;
   
    case 'b':
        dBodySetPosition(ball.body,0,0,0.14+0.0);
        dBodySetLinearVel(ball.body,0,0,0);
        dBodySetAngularVel(ball.body,0,0,0);
        break;
    }
}

/*** ��  �� ***/
// void control(){
//     // dReal tmp = dJointGetAMotorAngle(wheel[0].motor, 1);
//     // dReal u   = kp * (target - tmp);
//     if(last_command == 'a'){
//         std::cout << "[command] = a \n";
//         last_command = 0;
//         dJointSetHingeParam(wheel[0].joint, dParamVel , 1.0);
//         dJointSetHingeParam(wheel[0].joint, dParamFMax, MOTOR_F_MAX);
//     }

//     if(last_command == 's'){
//         std::cout << "[command] = s \n";

//         last_command = 0;
//         dJointSetHingeParam(wheel[0].joint, dParamVel , 0.0);
//         dJointSetHingeParam(wheel[0].joint, dParamFMax, MOTOR_F_MAX);
//     }

// }

static void simLoop(int pause){
    if (!pause){
        //TODO: loop();
        dSpaceCollide(space,0,&nearCallback); // add
        //control();

        control(last_command);
        last_command = 0;

        dWorldStep(world, 0.01);
        dJointGroupEmpty(contactgroup); // add

    }
    DrawRobot();
    DrawWorld();
}

void setDrawStuff(){
    fn.version = DS_VERSION;
    fn.start   = Simulator::start;
    fn.step    = Simulator::simLoop;
    fn.command = Simulator::command;
    fn.path_to_textures = "./textures";
}


void init(int argc_, char *argv_[]){
    last_command = 0;

    for(int i=0; i< WALL_NUM*2; i++){
        for(int j=0; j<3; j++) WALL_ATTITUDE[i][j] = WALL_ATTITUDE[i][j] * ZOOM_SIZE;
    }

    dInitODE();
    setDrawStuff();
    world        = dWorldCreate();   
    space        = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);   
    ground       = dCreatePlane(space,0,0,1,0); 

    argc = argc_;
    argv = argv_;
    dWorldSetGravity(world, 0.0, 0.0, - 9.8); 
    dWorldSetCFM(world,1e-3); // CFM
    dWorldSetERP(world,0.8);  // ERP

    makeRobot(); 
    makeWorld();
}


void loop(){
  dsSimulationLoop(argc,argv,640,480,&fn);
}

void quit(){
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space); 
    dWorldDestroy(world);
    dCloseODE();
}




/* #############################################################
                 以下、loopから呼び出す用の関数
############################################################# */
void setMotorCurrent(int id, float current){
    //MY_ASSERT(id >= WHEEL_NUM);
    if(id >= WHEEL_NUM) {std::cout << "[ERROR] in function setMotorCurrent()  --> 0 <= id < WHEEL_NUM に設定してください\n"; return;}
    dJointSetHingeParam(wheel[id].joint, dParamVel , current);
    dJointSetHingeParam(wheel[id].joint, dParamFMax, MOTOR_F_MAX);
}

void freeMotor(int id){
    //MY_ASSERT(id >= WHEEL_NUM);
    if(id >= WHEEL_NUM){ std::cout << "[ERROR] in function freeMotor()  --> 0 <= id < WHEEL_NUM に設定してください\n"; return;}
    dJointSetHingeParam(wheel[id].joint, dParamFMax, MOTOR_F_MAX);
}

void setMotorParams(int id, int Kp, int Ki, int Ti, int max_current){

}

 // エンコーダーの値を得る
std::uint16_t getEncTotal(int id){
    //MY_ASSERT(id >= 2);
    if(id >= 2){ std::cout << "[ERROR] in function getEncTotal()  --> 0 <= id < 2 に設定してください\n"; return -1;}

    if(id == 0){
        //TODO:
        return 0;
    }else{
        //TODO:
        return 0;
    }
}

 // ラインセンサーの各素子(16個)が読んだ色をまとめて得る
std::uint16_t getLineSensorColors(int id){
    //TODO:
    return 0;
}


//現実の位置や速度の情報を含んだログを出力する。
void printLog(){
    //TODO
}



}