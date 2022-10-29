#pragma once
#include "EC.h"
/**
* @brief 自己位置取得用ライブラリ
* @date 2022/10/11
* @details 測定輪の取り付け角度によらず（測定輪同士が直交でなくても）自己位置を取得できるライブラリ
* @attention 測定輪の車軸は機体中心から測定輪に引いた直線と直交しないといけない
* @note 回転による自己位置のずれ(理論的には起こらないはず…)を補正する計算式を入れられていない
* @see https://robostepwiki.sakuratan.com/wp/f3rc%e5%90%91%e3%81%91-%e8%87%aa%e5%b7%b1%e4%bd%8d%e7%bd%ae%e6%8e%a8%e5%ae%9a%e7%9b%b4%e4%ba%a4%e3%81%a7%e3%81%aa%e3%81%84%e3%81%a8%e3%81%8d/
*/

/** @section SAMPLE
* @code
* //測定輪ユニットで自己位置取得するテストコード
* //mbed(LPC1768)を使用
* #include "mbed.h"
* #include "math.h"
* #include "EC.h"
* #include "BNO055.h"
* #include "Location.h"
* 
* //AMT102の分解能は2048
* #define RESOLUTION 2048
* //測定輪半径[mm]
* #define r_sokuteiWheel 19.0
* //機体座標系のx軸正の方向からエンコーダー正の方向までの測定輪の取り付け角度[deg]
* #define theta0 90.0
* #define theta1 0.0
* 
* //機体の位置[mm]
* double x = 0.0, y = 0.0;
* //機体の角度[deg]
* double theta = 0.0;
* 
* //測定輪(A相,B相,分解能)
* Ec1multi ecXY[]= {
*     Ec1multi(p15,p16,RESOLUTION),
*     Ec1multi(p18,p17,RESOLUTION)
* };
* //ジャイロ(SDA,SCL)
* BNO055 BNO055(p28,p27);
* 
* //割り込みタイマー
* Ticker ticker;
* //tickerを回す秒数の間隔
* #define DELTA_T 0.05
* 
* //クラスLocationを以下locationとする
* Location location(r_sokuteiWheel, ecXY[0], theta0, ecXY[1], theta1);
* 
* void timercallback();
* double getGyroYaw();
* 
* int main()
* {
*     //location.setStartLocation(250.0, 250.0);
*     //location.setCoef(1.0, 1.1);
* 
*     //ジャイロ初期化
*     BNO055.reset();
*     BNO055.setmode(OPERATION_MODE_NDOF);
* 
*     printf("START\r\n");
* 
*     //割り込みタイマーをオンにする
*     ticker.attach(&timercallback,DELTA_T);
* 
*     while(1) {
*         wait(5);
*         printf("x: %.2f, y: %.2f, theta: %.2f\r\n", x, y, theta);
*     }
* }
* //tickerで呼び出す関数
* void timercallback()
* {
*     //初期状態におけるフィールド座標系から機体座標系までの角度のずれ + 機体の回転角度[rad]
*     double rad_theta = getGyroYaw();
*     //自己位置を計算する
*     location.calXY(rad_theta);
*     //自己位置を取得する
*     x = location.getX();
*     y = location.getY();
* }
* //初期状態からの機体の回転角度[rad]を取得する関数
* double getGyroYaw()
* {
*     BNO055.get_calib();
*     BNO055.get_angles();
*     //反時計回りを正とした機体の回転角度[deg] (BNO055 yaw 時計回りに0~360[degree])
*     theta = -BNO055.euler.yaw + 360.0;
*     //機体の回転角度[rad]
*     double yaw_ = theta * M_PI / 180.0;
*     return yaw_;
* }
* @endcode
*/
class Location{
    public:
        /**
        * @brief コンストラクタの定義
        * @attention main関数の前に必ず一度宣言する
        * @param [in] r_sokuteiWheel 測定輪半径[mm]
        * @param [in] ec0 測定輪0についているエンコーダの名前
        * @param [in] theta0 機体座標系のx軸正の方向から測定輪0のエンコーダ正回転の方向への角度[deg]
        * @param [in] ec1 測定輪1についているエンコーダの名前
        * @param [in] theta1 機体座標系のx軸正の方向から測定輪1のエンコーダ正回転の方向への角度[deg]
        * @details 初期位置の座標を（0,0）に初期設定
        * @details 自己位置のズレ補正用の係数をx軸方向、y軸方向ともに1に初期設定
        */
        Location(float r_sokuteiWheel, Ec &ec0, double theta0, Ec &ec1, double theta1);
        /**
        * @brief フィールド座標系でのx軸、y軸方向の移動距離を求める関数
        * @param[in] theta_ フィールド座標系から機体座標系までの角度のずれ[rad]
        * @details theta_[rad] = 初期状態におけるフィールド座標系から機体座標系までの角度のずれ + 初期状態からの機体の回転角度（ジャイロなどで取る）
        */
        void calXY(double theta_);
        /**
        * @brief フィールド座標系でのx座標を返す関数
        * @return double フィールド座標系でのx座標[mm]
        * @details 初期位置のx座標 + calXYで求めたx軸方向の移動距離 × 自己位置のズレ補正用の係数
        */
        double getX();
        /**
        * @brief フィールド座標系でのy座標を返す関数
        * @return double フィールド座標系でのy座標[mm]
        * @details 初期位置のy座標 + calXYで求めたy軸方向の移動距離 × 自己位置のズレ補正用の係数
        */
        double getY();
        /**
        * @attention 以下はクラスのコンストラクタで初期設定されている
        * @attention 設定を変更したいときのみ呼び出す
        */
    
        /**
        * @brief フィールド座標系での初期位置の座標を設定する関数（初期設定は（0,0））
        * @param[in] x フィールド座標系での初期位置のx座標[mm]
        * @param[in] y フィールド座標系での初期位置のy座標[mm]
        */
        void setStartLocation(double x, double y);
        /**
        * @brief 自己位置のズレを補正するための係数を設定する関数（初期設定は1.0）
        * @param[in] coef_x フィールド座標系でのx軸方向の自己位置のズレを補正するための係数
        * @param[in] coef_y フィールド座標系でのx軸方向の自己位置のズレを補正するための係数
        */
        void setCoef(double coef_x, double coef_y);

    private:
        double x_;
        double y_;
        double coef_x_;
        double coef_y_;
        double start_x;
        double start_y;
        float r_sokuteiwheel_;
        Ec *ec0_;
        Ec *ec1_;
        double theta0_;
        double theta1_;
        double old_rad[2];
        float keisuu;
};
