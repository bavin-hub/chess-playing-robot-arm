#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <Servo.h> 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;
std_msgs::Int16 pnp_status;
ros::Publisher pub("/pnp_status", &pnp_status);
//ros::Publisher pub = nh.advertise<std_msgs::Int16>("/pnp_status", 5);
// Servo servo1,servo2,servo3,servo4,servo5,servo6;

// int curr_angle1, curr_angle2, curr_angle3, curr_angle4, curr_angle5;
int des_angle1, des_angle2, des_angle3, des_angle4, des_angle5;

int curr_angle1 = 0.0;
int curr_angle2 = 90.0;
int curr_angle3 = 90.0;
int curr_angle4 = 90.0;
int curr_angle5 = 0.0;

int curr_pulse1;
int curr_pulse2;
int curr_pulse3;
int curr_pulse4;
int curr_pulse5;

int des_pulse1;
int des_pulse2;
int des_pulse3;
int des_pulse4;
int des_pulse5;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates



void update_motors(int angle1, int angle2, int angle3, int angle4, int angle5)
{
    curr_pulse1 = map(curr_angle1, 0, 180, 125, 480);
    curr_pulse2 = map(curr_angle2, 0, 180, 115, 490);
    curr_pulse3 = map(curr_angle3, 0, 180, 100, 475);
    curr_pulse4 = map(curr_angle4, 0, 180, 110, 485);
    curr_pulse5 = map(curr_angle5, 0, 180, 110, 485);

    des_pulse1 = map(angle1, 0, 180, 125, 480);
    des_pulse2 = map(angle2, 0, 180, 115, 490);
    des_pulse3 = map(angle3, 0, 180, 100, 475);
    des_pulse4 = map(angle4, 0, 180, 110, 485);
    des_pulse5 = map(angle5, 0, 180, 110, 485);

    while(true)
    {
        if(abs(des_pulse1 - curr_pulse1) > 1){
            if(des_pulse1 > curr_pulse1){curr_pulse1+=1;}
            else{curr_pulse1-=1;}
        }

        if(abs(des_pulse2 - curr_pulse2) > 1){
            if(des_pulse2 > curr_pulse2){curr_pulse2+=1;}
            else{curr_pulse2-=1;}
        }


        if(abs(des_pulse3 - curr_pulse3) > 1){
            if(des_pulse3 > curr_pulse3){curr_pulse3+=1;}
            else{curr_pulse3-=1;}
        }


        if(abs(des_pulse4 - curr_pulse4) > 1){
            if(des_pulse4 > curr_pulse4){curr_pulse4+=1;}
            else{curr_pulse4-=1;}
        }
        
        pwm.setPWM(0, 0, curr_pulse1);
        pwm.setPWM(1, 0, curr_pulse2);
        pwm.setPWM(2, 0, curr_pulse3);
        pwm.setPWM(4, 0, curr_pulse4);
        pwm.setPWM(5, 0, des_pulse5);

        delay(20);

        if(abs(des_pulse1 - curr_pulse1) <= 1 and abs(des_pulse2 - curr_pulse2) <= 1 and abs(des_pulse3 - curr_pulse3) <= 1 and abs(des_pulse4 - curr_pulse4) <= 1)
        {
            pwm.setPWM(0, 0, des_pulse1);
            pwm.setPWM(1, 0, des_pulse2);
            pwm.setPWM(2, 0, des_pulse3);
            pwm.setPWM(4, 0, des_pulse4);
            pwm.setPWM(5, 0, des_pulse5);

            curr_angle1 = angle1;
            curr_angle2 = angle2;
            curr_angle3 = angle3;
            curr_angle4 = angle4;
            curr_angle5 = angle5;

            break;
        }
        
        
        
    }


}

void servo_cb( const rospy_tutorials::Floats& cmd_msg){
  nh.loginfo("Command Received ");

 
  
//   int new_pos[6]={cmd_msg.data[0],cmd_msg.data[1],cmd_msg.data[2],cmd_msg.data[3],cmd_msg.data[4],cmd_msg.data[5]};
//   int i=0;

   

     for(int i=0; i<5; i+=5)
     {
        
      
         des_angle1 = cmd_msg.data[i];
         des_angle2 = cmd_msg.data[i+1];
         des_angle3 = cmd_msg.data[i+2];
         des_angle4 = cmd_msg.data[i+3];
         des_angle5 = cmd_msg.data[i+4];

         Serial.println(des_angle1);
         Serial.println(des_angle2);
         Serial.println(des_angle3);
         Serial.println(des_angle4);
         Serial.println(des_angle5);
         Serial.println();

        // update_motors(des_angle1,
        //               des_angle2,
        //               des_angle3,
        //               des_angle4,
        //               des_angle5); 
     }

     nh.loginfo("action executed");
     delay(4000);
     
     
     
     pnp_status.data = 1;
     pub.publish(&pnp_status);

  
}

void test_cb( const std_msgs::Int32& cmd_msg){
  Serial.println(cmd_msg.data);
}




ros::Subscriber<rospy_tutorials::Floats> sub("/coords_to_arduino",servo_cb);
// ros::Subscriber<std_msgs::Int32> sub("/testing_arduino",test_cb);


void setup(){

  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  Serial.begin(57600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  curr_pulse1 = map(curr_angle1, 0, 180, 125, 480);
  curr_pulse2 = map(curr_angle2, 0, 180, 115, 490);
  curr_pulse3 = map(curr_angle3, 0, 180, 100, 475);
  curr_pulse4 = map(curr_angle4, 0, 180, 110, 485);
  curr_pulse5 = map(curr_angle5, 0, 180, 110, 485);

  pwm.setPWM(0, 0, curr_pulse1);
  pwm.setPWM(1, 0, curr_pulse2);
  pwm.setPWM(2, 0, curr_pulse3);
  pwm.setPWM(4, 0, curr_pulse4);
  pwm.setPWM(5, 0, curr_pulse5);
  
  delay(4000);
  
//   servo1.attach(3); //attach it to pin 9
//   servo2.attach(5);
//   servo3.attach(6);
//   servo4.attach(9); //attach it to pin 9
//   servo5.attach(10);
//   servo6.attach(11);
//   servo1.write(90);
//   servo2.write(90);
//   servo3.write(90);
//   servo4.write(10);
//   servo5.write(90);
//   servo6.write(90);
  
  
}

void loop(){

  nh.spinOnce();
  delay(1);
}
