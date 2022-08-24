#include "ahau.h"
// class Motor
Motor::Motor(const byte DirPin1,const byte DirPin2, const byte PWMPin)
{ // constructor
    // Motor: khởi tạo các chân điều khiển chiều, PWM của động cơ
    pinMode(DirPin1, OUTPUT);
    pinMode(DirPin2, OUTPUT);
    _dirPin1 = DirPin1;// khai báo chân điều khiển
    _dirPin2 = DirPin2; // khai báo chân điều khiển
    pinMode(PWMPin, OUTPUT);
    _PWMPin = PWMPin;
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
    digitalWrite(PWMPin, 0); // khởi tạo motor dừng
}
void Motor::Run(byte Dir, int Speed)
{ //điều khiển động cơ theo chiều dir,
    //toc do Speed 
    if (Dir)
    { // >0 THUAN
        digitalWrite(_dirPin1, LOW);
        digitalWrite(_dirPin2, HIGH);
        analogWrite(_PWMPin, Speed);
    }
    else
    { // quay ngươc
        digitalWrite(_dirPin1, HIGH);
        digitalWrite(_dirPin2, LOW);
        analogWrite(_PWMPin, Speed);
    }
}
void Motor::Stop()
{ // nên gọi trước khi đão chiều quay, Dir là    
        analogWrite(_PWMPin, 0);
    delay(100);
} // hết cấu trúc Motor

// class STR

int STR:: str(int s,double a1,double a2,double b1,double b2)
{   _a1=a1;
    _a2=a2;
    _b1=b1;
    _b2=b2;
    int u;
    if (xi<1)
    {
        d1 =-2*exp(-xi*wn*T0)*cos(wn*T0*sqrt(1-pow(xi,2))) ;
    }
    if(xi>=1)
    {
        d1 =-2*exp(-xi*wn*T0)*cosh(wn*T0*sqrt(pow(xi,2))-1) ;
    }
    d2 = exp(-2*xi*wn*T0);
    r1 = (_b1+_b2)*(_a1*_b1*_b2- _a2*pow(_b1,2)-pow(_b2,2));
    s1 = _a2*((_b1+_b2)*(_a1*_b2-_a2*_b1)+_b2*(_b1*d2-_b2*d1-_b2));
    q2 = s1/r1;
    q1 = _a2/_b2 - q2*(_b1/_b2-_a1/_a2 +1);
    gama =q2*_b2/_a2;
    q0=(d1+1-_a1-gama)/_b1;
    u = int (q0*s+q1*_s1+q2*_s2+(1-gama)*_u1+gama*_u2);
    _s2=_s1;
    _s1=s;
    _u2=_u1;
    _u1=u;
    return u;
}
void STR::Reset()
{
    _s1=0;
    _s2=0;
    _u1=0;
    _u2=0;
}
// class theta

Theta::Theta()
{
   for (int i =0;i<=3;i++)
    {
        P1[i][i]= 10000;
    }
    O1[0][0]=1;//b1=46 b2 =-60 a2=41000 a1 =-22000
    O1[1][0]=1;
    O1[0][0]=1;
    O1[0][0]=1;
}
void Theta::uocluong(double y,double y1,double y2,double u1,double u2)
{
    X[0][0]= -y1;
    X[1][0]= -y2;
    X[2][0]= u1;
    X[3][0]= u2;
    _y=y;
    for (int i =0;i<=3;i++)
    {
        Xt[0][i]= X[i][0];
    }
    for (int i =0;i<=0;i++)
    {
        for (int j=0;j<=3;j++)
        {
            T2[i][j]= 0;
            for(int k =0;k<=3;k++)
            {
                T2[i][j] += Xt[i][k]*P1[k][j];
            }
        }
    }
    T1=0;
     for(int k =0;k<=3;k++)
    {
     T1 += T2[0][k]*X[k][0];
    }
    T1 = T1+T;
    // tinh T4
    for (int i =0;i<=3;i++)
    {
        for (int j=0;j<=0;j++)
        {
            T4[i][j]= 0;
            for(int k =0;k<=3;k++)
            {
                T4[i][j] += P1[i][k]*X[k][j];
            }
        }
    }
    // T4Xt
    for (int i =0;i<=3;i++)
    {
        for (int j=0;j<=3;j++)
        {
            T3[i][j]= 0;
            for(int k =0;k<=0;k++)
            {
                T3[i][j] += T4[i][k]*Xt[k][j];
            }
        }
    }
     // thuat toan chia
    for (int i =0;i<=3;i++)
    {
        for (int j=0;j<=3;j++)
        {
            T3[i][j]= T3[i][j]/T1;
            
        }
    }
    //thuat toan nhan
    for (int i =0;i<=3;i++)
    {
        for (int j=0;j<=3;j++)
        {
            T2[i][j]= 0;
            for(int k =0;k<=3;k++)
            {
                T2[i][j] += T3[i][k]*P1[k][j];
            }
        }
    }
   
    // thuat toan tru
    for (int i =0;i<=3;i++)
    {
        for (int j=0;j<=3;j++)
        {
            T3[i][j]= P1[i][j]-T2[i][j];
  
        }
    }
    // tinh PK
    for (int i =0;i<=3;i++)
    {
        for (int j=0;j<=3;j++)
        {
            P[i][j]= T3[i][j]/T;
        }
    }
    //Tinh LK
        for (int j=0;j<=3;j++)
        {
            L[j][0]= T4[j][0]/T1;
            
        }
    //tinh e
    E = _y;
    
     for(int k =0;k<=3;k++)
     {
     E -= Xt[0][k]*O1[k][0];
    } 
    // tinh theta
        for (int i=0;i<=3;i++)
        {
            T2[i][0]= L[i][0]*E;
        }
    for (int j=0;j<=3;j++)
     {
      O[j][0]= O1[j][0]-T2[j][0];
     }

    a1= O[0][0];
    a2= O[1][0];
    b1= O[2][0];
    b2= O[3][0];
    // day gia tri
    for (int i =0;i<=3;i++)
    {
        for (int j=0;j<=3;j++)
        {
            P1[i][j]= P[i][j];
            O1[i][j]= O[i][j];
        }
    } 
}
// class PID rời rạc
PIDk::PIDk(float Kp, float Ki, float Kd,float Ts)
{ // hàm constructor
// khai báo các thông số của bộ điều khiển PID
    _Ki = Ki;
    _Kp = Kp;
    _Kd = Kd;
    _e1=0;
    _e2=0;
    _u1=0;
     _Ts =Ts;
}
// hàm tính toán giá trị điều khiển cho PID
int PIDk::pidk(float e) 
{
    float u;
    int pid;
    _e = e;
    u= _u1 + _Kp*(_e-_e1) + _Ki*_Ts*(_e+_e1)/2 + _Kd*(_e-2*_e1+_e2)/_Ts;
    pid = u;
    _e2 = _e1;
    _e1 = _e;
    _u1=pid;
    return pid;
}  
// reset pid
void PIDk::reset()
{
    _e1=0;
    _e2=0;
    _u1=0;
}