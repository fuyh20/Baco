#include "I2Cdev.h"  //I2C协议库
#include "Wire.h"    //服务于I2C通信
#include "MPU6050.h" //MPU6050库文件

#include <MsTimer2.h> //定时中断

#define AIN1 11 //TB6612FNG驱动模块控制信号 共6个
#define AIN2 10
#define BIN1 9
#define BIN2 4
#define PWMA 5
#define PWMB 6 // 11号脚是废的

#define ENC_A_A 2  //编码器采集引脚 每路2个 共4个 ENCODER_L
#define ENC_A_B 12 //DIRECTION_L
#define ENC_B_A 3  //ENCODER_R
#define ENC_B_B 8  //DIRECTION_R

#define TrgPin 22 // 超声波触发引脚
#define EcoPin 23// 超声波接受引脚

double dist = 0;

MPU6050 mpu6050;        //实例化一个 MPU6050 对象，对象名称为 mpu6050

int16_t ax, ay, az, gx, gy, gz; //MPU6050的三轴加速度和三轴陀螺仪（角速度）数据

/******* 标志量 *********/
int flag_front, flag_back;
int isFollow = 1;

/***********************/


/******* PID 参数 *********/
double Balance_Kp = 25, Balance_Kd = 2; // 直立PD
double Balance_target = -2;
double Velocity_Kp = 0, Velocity_Ki = 0;
double rpmA, rpmB;
int Velocity_dt = 0;
double Gyro_x_bias = 4;
/*************************/

/***************下面是卡尔曼滤波相关变量***************/
float K1 = 0.05; // 对加速度计取值的权重
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5, C_0 = 1;
float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms


int Balance_Pwm, Velocity_Pwm; //直立 速度 转向环的PWM
int Motor1, Motor2; //电机叠加之后的PWM
float Anglex, Gryox; // MPU6050参量

int countA = 0, countB = 0;

class KalmanFilter
{
public:
    void FirstOrderFilter(float angle_m, float gyro_m, float dt, float K1);
    void Kalman_Filter(double angle_m, double gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0);
    void Angletest(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro,
                   float R_angle, float C_0, float K1);
    float Gyro_x, Gyro_y, Gyro_z;
    float accelz = 0;
    float angle;
    float angle6;

private:
    float angle_err, q_bias;
    float Pdot[4] = {0, 0, 0, 0};
    float P[2][2] = {{1, 0}, {0, 1}};
    float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    float angle_dot;
};

KalmanFilter KalFilter; //实例化一个卡尔曼滤波器对象，对象名称为 KalFilter

void KalmanFilter::FirstOrderFilter(float angle_m, float gyro_m, float dt, float K1)
{
    angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);
}

void KalmanFilter::Kalman_Filter(double angle_m, double gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0)
{
    angle += (gyro_m - q_bias) * dt;
    angle_err = angle_m - angle;
    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angle += K_0 * angle_err; //最优角度
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m - q_bias; //最优角速度
}

void KalmanFilter::Angletest(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro,
                             float R_angle, float C_0, float K1)
{
    // int flag;
    // 平衡参数
    float Angle = atan2(ay, az) * 57.3;                              //角度计算公式,Angle:一阶互补滤波计算出的小车最终倾斜角度
    Gyro_x = (gx - 128.1) / 131;                                     //角度转换
    Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0); //卡曼滤波
    //旋转角度Z轴参数
    if (gz > 32768)
        gz -= 65536;    //强制转换2g  1g
    Gyro_z = -gz / 131; //Z轴参数转换
    accelz = az / 16.4;
    float angleAx = atan2(ax, az) * 180 / PI;  //计算与x轴夹角
    Gyro_y = -gy / 131.00;                     //计算角速度
    FirstOrderFilter(angleAx, Gyro_y, dt, K1); //一阶滤波
}

void GetDistance()
{
    digitalWrite(TrgPin, LOW);
    delayMicroseconds(8);
    digitalWrite(TrgPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(TrgPin, LOW);
    dist = pulseIn(EcoPin, HIGH)/58.00;
}

void Follow()
{
    GetDistance();
    int target_lenth = 20;
    int bias = 5;
    if(isFollow)
    {
        if(dist >= 0 && dist > target_lenth - bias && dist < target_lenth + bias)
            flag_front = 0, flag_back = 0;
        else if(dist >= 0 && dist <= target_lenth - bias)
            flag_front = 0, flag_back = 1;
        else if(dist >= 0 && dist >= target_lenth + bias)
            flag_front = 1, flag_back = 0;
  }
}

int balance(float Angle, float Gyro)
{
    float Bias;
    int balance;
    Bias = Angle - Balance_target;                           //===求出平衡的角度中值 和机械相关
    balance = Balance_Kp * Bias + (Gyro-Gyro_x_bias) * Balance_Kd; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数; 15 ,0.4
    return balance;
}

int velocity(float rpmA, float rpmB)
{
    static float integral;
    float target_velocity = 0;
    if(flag_front) target_velocity = 100;
    else if(flag_back) target_velocity = -100;
    float err = target_velocity - (rpmA + rpmB) / 2;
    integral += err * 50;
    double Velocity = Velocity_Kp * err + Velocity_Ki * integral;
    double output = constrain(Velocity, 0, 255);
    return output;
}

void Set_Pwm(int moto1, int moto2)
{
    if (moto1 > 0)
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    } //TB6612的电平控制
    else
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }                              //TB6612的电平控制
    analogWrite(PWMA, abs(moto1)); //赋值给PWM寄存器

    if (moto2 < 0)
    {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } //TB6612的电平控制
    else
    {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }                              //TB6612的电平控制
    analogWrite(PWMB, abs(moto2)); //赋值给PWM寄存器
}

void constrain_Pwm(void)
{
    int Amplitude = 250; //===PWM满幅是255 限制在250
    if (Motor1 < -Amplitude)
        Motor1 = -Amplitude;
    if (Motor1 > Amplitude)
        Motor1 = Amplitude;
    if (Motor2 < -Amplitude)
        Motor2 = -Amplitude;
    if (Motor2 > Amplitude)
        Motor2 = Amplitude;
}

void control()
{
    sei(); //全局中断开启

    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                                   //获取MPU6050陀螺仪和加速度计的数据
    KalFilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); //通过卡尔曼滤波获取角度
    Anglex = KalFilter.angle;
    Gryox= KalFilter.Gyro_x;
    
    //直立PD控制 控制周期5ms
    Balance_Pwm = balance(Anglex, Gryox);

    //速度PI控制，控制周期50ms
//    if(++Velocity_dt == 50)
//    {
//        rpmA = (double)countA * 20 * 60 / 260;
//        rpmB = (double)countB * 20 * 60 / 260;
//        countA = 0, countB = 0;
//        Velocity_Pwm = velocity(rpmA, rpmB); 
//        Velocity_dt = 0;
//    }

    if(flag_front)
        Velocity_Pwm = 100;
    if(flag_back)
        Velocity_Pwm = -100;
    
    // 两环融合
    Motor1 = Balance_Pwm - Velocity_Pwm; //直立速度环的叠加
    Motor2 = Balance_Pwm - Velocity_Pwm; //直立速度环的叠加
    constrain_Pwm();      //限幅

    // 角度保护
    if (KalFilter.angle > 30 || KalFilter.angle < -30)
        Motor1 = 0, Motor2 = 0;

    Set_Pwm(Motor1, Motor2); //如果不存在异常，赋值给PWM寄存器控制电机
}

void CodeA()
{
    if(digitalRead(ENC_A_A) == LOW)
    {
        if(digitalRead(ENC_A_B) == LOW)
            countA++;
        if(digitalRead(ENC_A_B) == HIGH)
            countA--;
    }
}

void CodeB()
{
    if(digitalRead(ENC_B_A) == LOW)
    {
        if(digitalRead(ENC_B_B) == LOW)
            countB++;
        if(digitalRead(ENC_B_B) == HIGH)
            countB--;
    }
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);      //开启串口，设置波特率为 9600
    Wire.begin();            //加入 IIC 总线
    delay(1500);             //延时等待初始化完成
    mpu6050.initialize(); //初始化MPU6050
    delay(20);
    pinMode(AIN1, OUTPUT); //TB6612控制引脚，控制电机1的方向，01为正转，10为反转
    pinMode(AIN2, OUTPUT); //TB6612控制引脚，
    pinMode(BIN1, OUTPUT); //TB6612控制引脚，控制电机2的方向，01为正转，10为反转
    pinMode(BIN2, OUTPUT); //TB6612控制引脚，
    pinMode(PWMA, OUTPUT); //TB6612控制引脚，电机PWM
    pinMode(PWMB, OUTPUT); //TB6612控制引脚，电机PWM
    pinMode(TrgPin, OUTPUT); // 超声波触发引脚
    pinMode(EcoPin, INPUT); // 超声波接受引脚

    digitalWrite(AIN1, 0); //TB6612控制引脚拉低
    digitalWrite(AIN2, 0); //TB6612控制引脚拉低
    digitalWrite(BIN1, 0); //TB6612控制引脚拉低
    digitalWrite(BIN2, 0); //TB6612控制引脚拉低
    analogWrite(PWMA, 0);  //TB6612控制引脚拉低
    analogWrite(PWMB, 0);  //TB6612控制引脚拉低

    pinMode(ENC_A_A, INPUT); //编码器引脚
    pinMode(ENC_A_B, INPUT); //编码器引脚
    pinMode(ENC_B_A, INPUT); //编码器引脚
    pinMode(ENC_B_B, INPUT); //编码器引脚
    
//    attachInterrupt(0, CodeA, FALLING);
//    attachInterrupt(1, CodeB, FALLING);
    
    MsTimer2::set(1, control); //使用Timer2设置5ms定时器中断，
    MsTimer2::start();         //使用中断使能，Timer2开始计时，每5ms进入一次中断程序control
    delay(2000);
}

void loop() 
{
    Follow(); 
}
