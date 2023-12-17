#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include<math.h>
#include <stdio.h>
#include <cstdlib>	/*亂數相關函數*/
#include <ctime>	/*時間相關函數*/
#include <iostream>
#include <complex.h>
//#include <webots/Accelerometer.hpp>


#define TIME_STEP 64
#define MAX_SPEED 16
#define PS_THRESHOLD 75
#define PS_HIT 150 //PsValue 值大於 PS_HIT - PS_THRESHOLD 判定為碰撞
#define PI acos(-1)

#define S1 1
#define S2 2 
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7
#define S8 8
//9-12避障狀態
#define S9 9
#define S10 10
#define S11 11
#define S12 12


//與障礙物需判斷之距離
#define FORWARD_DIST 1
#define TURN_POS 2.125


using namespace webots;
using namespace std;



class myRobot : public Robot
{
public:

  myRobot()
  {
    left_motor = getMotor("left wheel motor");
    right_motor = getMotor("right wheel motor");
    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);
    
    char psStr[4];
    
    for (int i=0; i<8; i++)
    {
      sprintf(psStr,"ps%d",i);
      ps[i] = getDistanceSensor(psStr);
      ps[i]->enable(4*TIME_STEP);
    }
	
	readDistanceSensors();
    
	//取得左右馬達的位置資訊
    leftSensor = left_motor->getPositionSensor();
    rightSensor = right_motor->getPositionSensor();      
    leftSensor->enable(1);
    rightSensor->enable(1);	
	//accelerometer->enable(1);
    getPosition(refPositions);    
    State = S1;
	
	//accelerometer = getAccelerometer("accelerometer");
  }

  void goForward(double speed)
  {
      /* forward: speed > 0 , backward: speed < 0 */
      left_motor->setVelocity(speed);
      right_motor->setVelocity(speed);
  }

  void turn(double speed)
  {
      /* left turn: speed < 0, right turn: speed > 0 */
      left_motor->setVelocity(speed);
      right_motor->setVelocity(-speed);
  }


  void readDistanceSensors(){
	/*		
   double theta[8] = {0.3, 0.8, 1.57, 2.46,  -2.46,  -1.57,  -0.8,  -0.3};
   std::complex<double> z[8];
   std::complex<double> zc = 0;
   double m = 0;
   
   for (int i=0; i<8; i++)
    {
      if(((ps[i]->getValue())-PS_THRESHOLD) >0){
        psVal[i] = (ps[i]->getValue())-PS_THRESHOLD;
      }
      else{
        psVal[i]=0;
      }   
    }
   for (int i=0; i<8; i++)
   {
      z[i] = polar(psVal[i],theta[i]);
      zc += z[i];
      m  += psVal[i];
   }   
   zc = zc/m;  
   thetac = arg(zc);
   */
   double theta[8] = {0.30,  0.80,  1.57,  2.64, -2.64, -1.57, -0.80, -0.30};
   double ms[8];
   std::complex<double> z[8];

   std::complex<double> zc = 0;
   double m = 0;
   for (int i=0; i<8; i++){
	   psVal[i]=ps[i]->getValue();
   }
   
   for (int i=0; i<8; i++)
   {
      /* mass = sensor value minus sensor threshold */
      if (psVal[i] > PS_THRESHOLD )
      	ms[i] = psVal[i] - PS_THRESHOLD;
      else
        ms[i] = 0;
      
      z[i] = polar(ms[i],theta[i]);
      zc += z[i]; 
      m  += ms[i];
   }  
   if (m == 0)
      thetac = NAN;  // all sensors return below threshold
   else
   {
      zc = zc/m;
      thetac = arg(zc);
   }
	//const double *values = accelerometer->getValues();
  }
  
  
  //使用狀態機轉換
  void transition(){
    unsigned char nextState = State;
    //obsRight  = ( thetac<(2*PI/3) && thetac>0 ) || ( thetac<(-4*PI/3) && thetac>(-2*PI) );
    //obsLeft = ( thetac>(4*PI/3) && thetac<(2*PI) ) || ( thetac<0 && thetac>(-2*PI/3) );
	obsRight  = ( thetac<(2*PI/3) && thetac>0 ) ||(thetac<(2*PI) && thetac>(15*PI/8)) ||( thetac<(-4*PI/3) && thetac>(-2*PI) )
				||(thetac<0 && thetac>(-PI/8));
    obsLeft = ( thetac>(4*PI/3) && thetac<(2*PI) )||(thetac<(PI/8) && thetac>0) || ( thetac<0 && thetac>(-2*PI/3) )
				|| ( thetac<(-15*PI/8) && thetac>(-2*PI) );
    //obsNone  = ( thetac >= (2*PI/3) && thetac <= (4*PI/3) )|| ( thetac <= (-2*PI/3) && thetac >= (-4*PI/3) );
	 

    
	
    switch(State){
		 case S9: {
             //如果左右兩側皆無障礙物 就往前走
			 //cout<<"S9 "<<endl;
             goForward(0.2*MAX_SPEED);

			 if(obsLeft && obsRight){
				 nextState = S12;
				 //cout<<	"random"<<endl;			 
			 }
			 
             else if(obsLeft){
				 nextState = S10; 
             }
             else if(obsRight){
                 nextState = S11;
             }
             break;
         }
		 
		 case S10: {
             //右轉
             turn(0.1*MAX_SPEED);
			 //cout<<"S10 "<<endl;
             if(!obsLeft){
				nextState = S9; 
             }

             break;
		 }
		 case S11: {
             //左轉
             turn(-0.1*MAX_SPEED);
             //cout<<"S11 "<<endl;
             if(!obsRight){
				nextState = S9; 
             }

             break;
         }
		  
		 case S12: {
			 
			 //cout<<"S12 "<<endl;
			 srand(time(NULL));
			 // 產生 1-10 的整數亂數 
			 int x = rand() % 10 +1;
             //50%右轉 50%左轉 			 
             if(x>5)
                 nextState = S10; 
			 else if(x <= 5)
        	     nextState = S11;

             break;
         }
    }
    State = nextState;
  }


    void getPosition(double *pos){
      pos[0] = leftSensor->getValue();
      pos[1] = rightSensor->getValue();
    }
    
    void SquareTransition(double *pos){
      unsigned char nextState = State; /* default transition to same state */
      
       
      //如果沒有轉換狀態，就會一直作相同狀態的動作直到狀態改變
      switch (State) {
          case S1: {
              //s1時是往前直走
              goForward(0.3*MAX_SPEED); 
              if ( reach(pos) ) {
                 nextState = S2;
              }
           break;
          } 

          case S2: {
              //s2向左轉
              turn(-0.3*MAX_SPEED); 
              if ( reach(pos) ) {
                 nextState = S3;
              }
           break;
          }       

           case S3: {
			  //s3向右轉
              goForward(0.3*MAX_SPEED); 
              if ( reach(pos) ) {
                 nextState = S4;
              }
           break;
          }
          
           case S4: {
              turn(0.3*MAX_SPEED); 
              if ( reach(pos) ) {
                 nextState = S5;
              }
           break;
          }
           case S5: {
              goForward(-0.3*MAX_SPEED); 
              if ( reach(pos) ) {
                 nextState = S6;
              }
           break;
          } 
          case S6: {
              turn(-0.3*MAX_SPEED); 
              if ( reach(pos) ) {
                 nextState = S7;
              }
           break;
          }
          case S7: {
              goForward(-0.3*MAX_SPEED); 
              if ( reach(pos) ) {
                 nextState = S8;
              }
           break;
          }
          case S8: {
              turn(0.3*MAX_SPEED); 
              if ( reach(pos) ) {
                 nextState = S9;
              }
           break;
          }                              
    } /* end of switch */
    
    //如果狀態有改變，記錄下當前位置
    if(State != nextState){
      refPositions[0] = pos[0];
      refPositions[1] = pos[1];
    }
	
    State = nextState; 
}
   
   //判斷移動距離  
    bool reach(double *pos){
      bool reached = false;

      switch (State) {
          case S1: {
              reached = ( (pos[0]-refPositions[0] > 2*FORWARD_DIST ) ||
                         (pos[1]-refPositions[1] > 2*FORWARD_DIST ) );
             break;
          }
          case S2: {
             reached = ( (abs(pos[0]-refPositions[0]) > TURN_POS ) ||
                         (pos[1]-refPositions[1] > TURN_POS ) );
             //當左轉90度，將 reached 改變成 true
             break;
          }
          case S3: {
             reached = ( (pos[0]-refPositions[0] > 2*FORWARD_DIST ) ||
                         (pos[1]-refPositions[1] > 2*FORWARD_DIST ) );
             break;
          }
          case S4: {
             reached = ( (pos[0]-refPositions[0] > TURN_POS ) ||
                         (abs(pos[1]-refPositions[1]) > TURN_POS ) );
             break;
          }
          case S5: {
             reached = ( (abs(pos[0]-refPositions[0]) > 2*FORWARD_DIST ) ||
                         (abs(pos[1]-refPositions[1]) > 2*FORWARD_DIST ) );
             break;
          }
          case S6: {
             reached = ( (abs(pos[0]-refPositions[0]) > TURN_POS ) ||
                         (pos[1]-refPositions[1] > TURN_POS ) );
             break;
          }          
          case S7: {
             reached = ( (abs(pos[0]-refPositions[0]) > 2*FORWARD_DIST ) ||
                         (abs(pos[1]-refPositions[1]) > 2*FORWARD_DIST ) );
             break;
          }
          case S8: {
             reached = ( (pos[0]-refPositions[0] > TURN_POS ) ||
                         (abs(pos[1]-refPositions[1]) > TURN_POS ) );
             break;
          }     

      }
      return reached;
    }

public: 
    Motor *left_motor;
    Motor *right_motor;
	//Accelerometer *accelerometer;   //定義加速度感測器
    PositionSensor *left_pos;
    PositionSensor *right_pos;
    DistanceSensor *ps[8];
    double psVal[8];
    bool obsLeft;
    bool obsRight;
	bool obsNone;
	bool obsRand;
    bool obsHit;
	bool obsA;
	double thetac;
	
	
	
    //const double *Acc;
    
    PositionSensor *leftSensor, *rightSensor; //定義左右的位置感測器 
    double refPositions[2];
	

        
    unsigned char State;  
};
