#include "mbed.h"
#include "math.h"
#include "EC.h"
#include "Location.h"

Location::Location(float r_sokuteiWheel, Ec &ec0, double theta0, Ec &ec1, double theta1)
{
    //移動距離の初期化
    x_ = 0.0;
    y_ = 0.0;
    //自己位置のズレ補正用の係数（初期設定は1.0）
    coef_x_ = 1.0; 
    coef_y_ = 1.0;
    //初期位置の座標（初期設定は（0,0））
    start_x = 0.0;
    start_y = 0.0;
    //前回の測定輪の回転角度の初期化
    for(int i =0; i<2; i++) {
        old_rad[i]=0;
    }
    //測定輪の半径
    r_sokuteiwheel_ = r_sokuteiWheel;
    //エンコーダ
    ec0_ = &ec0;
    ec1_ = &ec1;
    //機体座標系のx軸正の方向から測定輪のエンコーダ正回転の方向への角度
    //測定輪の取り付け角度を[deg]から[rad]に変換
    theta0_ = theta0 * M_PI / 180.0;
    theta1_ = theta1 * M_PI / 180.0;
    //機体座標系でのx軸、y軸方向の移動距離を出すときに用いる係数
    keisuu = 1 / (cos(theta0_)*sin(theta1_) - sin(theta0_)*cos(theta1_));
}
//フィールド座標系でのx軸、y軸方向の移動距離を求める関数
//theta_[rad] = 初期状態におけるフィールド座標系から機体座標系までの角度のずれ + 初期状態からの機体の回転角度（ジャイロなどで取る）
void Location::calXY(double theta_)
{
    double ec_rad[2]= {};
    double d0, d1, dx, dy;

    //測定輪の回転角度
    ec_rad[0] = ec0_ -> getRad();
    ec_rad[1] = ec1_ -> getRad();
    
    //測定輪の合計回転距離
    d0 = r_sokuteiwheel_*(ec_rad[0] - old_rad[0]);
    d1 = r_sokuteiwheel_*(ec_rad[1] - old_rad[1]);
    
    //機体座標系でのx軸、y軸方向の移動距離
    dx = keisuu * (d0*sin(theta1_) - d1*sin(theta0_));
    dy = keisuu * (-d0*cos(theta1_) + d1*cos(theta0_));
    
    //フィールド座標系でのx軸、y軸方向の移動距離
    x_ += (dx*cos(theta_) - dy*sin(theta_));
    y_ += (dx*sin(theta_) + dy*cos(theta_));

    //測定輪の回転角度の現在値を前回値に代入
    old_rad[0] = ec_rad[0];
    old_rad[1] = ec_rad[1];
}
//フィールド座標系でのx軸方向の移動距離を返す関数
double Location::getX() 
{
    return start_x + x_*coef_x_;
}
//フィールド座標系でのy軸方向の移動距離を返す関数
double Location::getY()
{
    return start_y + y_*coef_y_;
}
//初期位置の座標を設定する関数（初期設定は（0,0））
void Location::setStartLocation(double x, double y)
{
    start_x = x;
    start_y = y;
}
//自己位置のズレ補正用の係数を設定する関数（初期設定は1.0）
void Location::setCoef(double coef_x, double coef_y)
{
    coef_x_ = coef_x;
    coef_y_ = coef_y;
}