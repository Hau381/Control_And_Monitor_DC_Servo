#ifndef _TN_Robot_Arduino_h
#define _TN_Robot_Arduino_h
#include <Arduino.h>
// các hằng số cho điều khiển động cơ
#define THUAN 1
#define NGUOC 0
class Motor
{ // lớp điều khiển động cơ cơ bản
private:
    byte _dirPin1; 
    byte _dirPin2;
    byte _PWMPin; // thông tin về chân điều khiển chiều, PWM, chiều hiện tại
public:
    Motor(const byte DirPin1,const byte DirPin2, const byte PWMPin);
    // khởi tạo các chân điều khiển chiều, PWM của động cơTham khảo Lap trinh Xe Robot Arduino.docx Page 17
    void Run(byte Dir, int Speed);
    // chạy theo chiều Dir (THUAN=1; NGUOC=0) với tốc độ Speed (0-255)
    void Stop();
    // Dừng động cơ, nên gọi trước khi đão chiều hay khởi tạo hệ thống
};

class STR
{
private:
    double _u;
    double _w;
    double _y;
    double _a1;
    double _a2;
    double _b1;
    double _b2;
    double _u1=0;
    double _u2=0;
    double _s1=0;
    double _s2=0;
public:
    double T0= 0.01;
    double xi=0.999;
    double wn=20;
    double d1;
    double d2;
    double gama;
    double q0;
    double q1;
    double q2;
    double r1;
    double s1 ;
    int str(int s,double a1,double a2,double b1,double b2);
    void Reset();
};
class Theta
{
private:
public:
double a1,a2,b1,b2,T=0.999;
double x1,x2,x3,x4,T1,E,_y;
double X[4][4],O[4][4],O1[4][4],P[4][4],P1[4][4],L[4][4],T2[4][4],T3[4][4],T4[4][4],Xt[4][4];
Theta();
void uocluong(double y,double y1,double y2,double u1,double u2);//
};
class PIDk
{
 // lớp PID cho tính hàm điều khiển PID
public:
    PIDk(float Kp, float Ki, float Kd,float Ts);
    // hàm khởi tạo bộ PID, khai báo ở đầu chương trình các hằng số điều
    // khiển Ki, Kd, Kd
    int pidk(float e); // hàm tính giá trị điều khiển PID theo sai lệch sl
    void reset();
private:
    // các thông số điểu khiển
    float _Ts;
    float _Ki;
    float _Kp;
    float _Kd;
    float _e;
    float _e1;
    float _e2;
    float _u1;   
};

#endif