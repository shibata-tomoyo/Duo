//離床判定プログラム
//加速度センサから取得した3軸合成加速度の絶対値の5回移動平均が5回連続で2.5[G]を超えたことを確認する
//加速度関係の変数や値はMPU6050のプログラムに合わせる

#define LAUNCH_ACC 2.50 //しきい値

int STANDBY_mode = 0; //勝手にモード決めた．モードは他の人のプログラムに合わせる．
int LAUNCH_mode = 0;

void setup(){
}

void loop(){
float aSqrt = sqrt(aX*aX+aY*aY+aZ*aZ); //3軸合成加速度の定義．引数において変数の四則演算ってできたかまた確認する．

//5回移動平均の取得
float aArray[5]; //配列定義
float aSum, aAve;
int i;
if(STANDBY_mode == 0){
for(i = 4; i > 0; i--){
aArray[i] = aArray[i-1];
}
aArray[0] = aSqrt;
for (i = 0; i < 5; i++){
aSum += aArray[i];
}
aAve = aSum / 5;

//離床判定（未完）
if(aAve > LAUNCH_ACC){

}


