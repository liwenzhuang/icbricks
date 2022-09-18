//ICBricks主控程序 
//最后更新时间 2022-07-17 19：00  
//具备蓝牙互动功能实现
//串口1write传感器值
//修复已知bug，增加开机提示音，解决开机按键长按问题
//最新修复：蓝牙名称取前两位16进制数，例如：Hello ICBricks 8a
//author : Mr.li
/* TCA9548
输入      输出
7          0
6          1
5          2
4          3
*/

#include <Arduino.h>
#include <ArduinoNvs.h>
#include <esp_task_wdt.h>
//15 秒 WDT 
#define WDT_TIMEOUT 15 
#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>
#include "Speaker.h"
#include "music_8bit.h"

//#include "SoundData.h"
//#include "XT_DAC_Audio.h"

//XT_DAC_Audio_Class DacAudio(25,0);   
//XT_Wav_Class Key_sound(Key_Audio); 
//XT_Wav_Class ON_sound(ON_Audio); 



#include <APDS9960.h>
//#include "Game_Audio.h"




//客户端导入
#include "BLEDevice.h"
//客户端导入到这里

//#include "EEPROM.h"//存储断电数据
//服务器端导入
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>  


// 各模块地址
#define diatance    0x29 //距离#define sound       0x35 //声音
#define gyroscope   0x68 //陀螺仪
//#define color       0x28 //颜色
#define encoder     0x36 //编码器

#define motor       0x67 //电机
#define led_matrix  0x38 //点阵
#define led_color   0x40 //彩灯
#define PCF8591 0x48                 //声音模块的PCF8591定义

#define ADDR        0x53  //颜色传感器
#define ControlReg_Addr 0x00 //颜色传感器用

//tca扫描用
#define NUM_TCA                  1
#define WIRE_ADDRESS_ERROR       2
#define WIRE_NACKORERROR_ERROR   3
#define TCAADDR                  0x70

#define NUMPIXELS 4   // 4个ws2812

//按键宏定义
#define  ring_button_1      18  //4联按键1
#define  ring_button_2      26  //4联按键2
#define  ring_button_3      27  //4联按键3
#define  ring_button_4      14  //4联按键4
#define  bluetooth_button   13  //4联按键4
#define  power_button       19  //开关机按键
#define  power_button_ctrl  12  //开关机检测
#define  power_state_detection    33  //充电器是否插入
#define  power_value_detection    32  //电量状态检测
#define  PIN           23       //ws2812指示灯，共有4个led.接gpio23
//#define  speaker_control    32  //控制声音功放开关
//下面是drv8830电机驱动用
#define DRV8830_ADDRESS 0x67
#define CTR_ADDRESS 0x00  // CONTROL_Reg

#define power_full_state_detection 5




byte adcvalue0, adcvalue1, adcvalue2, adcvalue3;  //PCF8591获取到的ADC数据
SPEAKER Speaker;//实例化Speaker
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);//实例化WS2812
APDS9960 apds = APDS9960(); //实例化apds9960




bool charge_state=1,old_charge_state=1;
bool bat_full_state=0;
int apds9960_direction=0;   //存储检测到的方向
int apds9960_flag=1;
int apds9960_motor_flag=0;

int power_button_flag=0; //电源开关机
int iic_addr[8]={}; // 用来存储iic扫描到的数据，标号为port ，数据为扫到的iic地址，没有设备值为0
int old_iic_addr[8]={};
/* 下面程序测试mpu6050模块 */
uint16_t MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //通过mpu6050获取的值 
int minVal=265; //不知为何
int maxVal=402;
double x; //用来计算角度
double y; 
double z;
/* mpu6050定义部分到这里 */

int get_encoder_data[3]={}; //编码器用

//下面变量是编码器用变量
int16_t encoder_data;
int abs_data ;

//距离变量
int vl_distance=0;
int juli=0;
//声音变量
int sound_value=0;

//下面是让led自动延时熄灭处理用函数

unsigned long duration_time=0;
unsigned long old_time=0;
int sound_led_flag = 1 ;
uint8_t b1=1,b2=1,b3=0,b4=0;//b1,b2是就地按键控制用,b3,b4是蓝牙遥控控制用
uint8_t motor_power; //[0, 100]
uint8_t dir = 1; //0:STBY 1:BW 2:FW 3:BRAKE
bool bluetooth_button_flag = false ;//主控蓝牙按键标志位

#define DELAYVAL 5 //控制ws2812的延时 
int client_huxi_value=10;
int server_huxi_value=0;


//服务器端数据定义   

#define SERVICE_UUID_IN             "bb2d8615-fcdd-45fd-bfde-f5bc3977c0e8" //执行器控制服务UUID
#define SERVICE_UUID_OUT            "bb2d8615-fcdd-45fd-bfde-f5bc3977c0e9" //传感器notify服务UUID

//#define CHARACTERISTIC_UUID_WRITE  "e1f7472c-3a41-48eb-8b43-836faec6c917" //特征uuid

#define CHARACTERISTIC_UUID_NOTIFY1 "868e76b4-97b1-4542-8a56-5a1e192eff01" //测试用，可读，值为hello ，广播蓝牙按键的值，如果按键值有变化的话
#define CHARACTERISTIC_UUID_NOTIFY2 "868e76b4-97b1-4542-8a56-5a1e192eff02" //接收客户端的传感器数据
#define CHARACTERISTIC_UUID_NOTIFY3 "868e76b4-97b1-4542-8a56-5a1e192eff03" //控制led
#define CHARACTERISTIC_UUID_NOTIFY4 "868e76b4-97b1-4542-8a56-5a1e192eff04" //控制电机

#define CHARACTERISTIC_UUID_NOTIFYA "868e76b4-97b1-4542-8a56-5a1e192eff0a" //1端口传感器值
#define CHARACTERISTIC_UUID_NOTIFYB "868e76b4-97b2-4542-8a56-5a1e192eff0b" //2端口传感器值
#define CHARACTERISTIC_UUID_NOTIFYC "868e76b4-97b3-4542-8a56-5a1e192eff0c" //3端口传感器值
#define CHARACTERISTIC_UUID_NOTIFYD "868e76b4-97b4-4542-8a56-5a1e192eff0d" //服务器端-下面复制自ble-uart
BLEServer *pServer = NULL;
//下面是第一个服务
BLECharacteristic * Notify1_Characteristic;//服务器端用来广播按键值，notify
BLECharacteristic * Notify2_Characteristic;//广播给icbricks客户端16字节传感器数据
BLECharacteristic * Notify3_Characteristic;//平板用---接收的值用来控制电机
BLECharacteristic * Notify4_Characteristic;//平板用---接收的值用来控制led灯
//下面是第二个服务
BLECharacteristic * NotifyA_Characteristic;//广播输入传感器值
BLECharacteristic * NotifyB_Characteristic;//广播输入传感器值
BLECharacteristic * NotifyC_Characteristic;//广播输入传感器值
BLECharacteristic * NotifyD_Characteristic;//广播输入传感器值



bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

//定义了一系列ble传送变量
uint8_t port_in_A[4] = {0,0,0,0}; //用来传送输入端口的数值
uint8_t port_in_B[4] = {0,0,0,0}; //用来传送输入端口的数值
uint8_t port_in_C[4] = {0,0,0,0}; //用来传送输入端口的数值
uint8_t port_in_D[4] = {0,0,0,0}; //用来传送输入端口的数值
uint8_t port_in_ALL[16]={};
uint8_t port_out_A[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //用来传送输出端口的数值,前两个端口
uint8_t port_out_B[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //用来传送输出端口的数值,后两个端口


//定义一个数组更新程序用，作用是暂存蓝牙服务器数据接收回调函数接收的数据
//16个数，每个输出端口占用4个位置，第一个位置是区分电机还是灯，电机设置为8，灯设置为9，随便起的
uint8_t BLE_SERVER_CALLBACK_DATA_MOTOR[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t BLE_SERVER_CALLBACK_DATA_LED[12] =   {0,0,0,0,0,0,0,0,0,0,0,0};

//int address = 0; //用来eeprom暂存
int encoder_LED_value =0;
int old_data2=0;

//下面变量为了解决编码器控制led灯时错乱的情况，区分哪一路单独的变量控制
int encoder_LED_value_0 =0;
int encoder_LED_value_1 =0;
int encoder_LED_value_2 =0;
int encoder_LED_value_3 =0;

int old_data_0=0;
int old_data_1=0;
int old_data_2=0;
int old_data_3=0;





//客户端数据定义
// The remote service we wish to connect to.
static BLEUUID    serviceUUID("bb2d8615-fcdd-45fd-bfde-f5bc3977c0e8");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID_1("868e76b4-97b1-4542-8a56-5a1e192eff01");//这个uuid是按键值
static BLEUUID    charUUID_2("868e76b4-97b1-4542-8a56-5a1e192eff02");//这个uuid是icbricks客户端16字节传感器数据



static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic_1;//客户端远程获取我们感兴趣的特征1
static BLERemoteCharacteristic* pRemoteCharacteristic_2;//客户端远程获取我们感兴趣的特征1
std::string sensor_value_16byte;


static BLEAdvertisedDevice* myDevice;
//客户端数据定义到这里

void server_conf(); //服务器函数声明


//主控按键直接控制点电机及led声明函数
void motor_ctrl(int ,int ,int );
void led_ctrl(int ,int  );

void bluetooth_button_handle(void);
void ring_button_handle_up(void);
void ring_button_handle_down(void);
void ble_button_handle(void);

uint8_t ble_button_up_down=0,ble_button_left_right=0,ble_button_ble=0;


//一些中间变量
int flag=1;  // 标记是主机还是从机模式
static BLEAddress *Server_BLE_Address;
String Scaned_BLE_Address; //暂存搜寻到的设备mac地址
String My_BLE_Address = ""; // 主机模式下存自己的BLE设备地址  测试 MAC地址（1）"80:7d:3a:fd:cb:06" //MAC地址（1）
String MAC_Address_From_NVS = "" ; //用来存储从nvs获取的数据
String comdata = ""; //字符串变量，用来暂存客户端从串口2接收的地址

String data1 = "";
int chongman=0;//充满标志，只要达到过充满的值就直接置位，除非断电重启



unsigned long old_waiting_serial2_data=0;

char rxValue_A[4]={};
char rxValue_B[3]={};
//中间变量到这里

unsigned long ble_c_time=0 ,ble_old_time = 0; //存储时间
int WDT_flag=0;//看门狗需要
unsigned long last_WDT_time=millis() ;//看门狗需要

// TCA搜寻，本质是使用for循环嵌套进行
int device_address[8] ={0x36,0x29,0x48,0x68,0x67,0x37,0x39,0x40} ;//这里定义了一个存储设备地址的数组


int kaiji=1;
int Volume=1;//音量
uint8_t mac_data_uint8_t[6]={};


//部分函数声明
void DRV8830_Run(uint8_t, uint8_t, uint8_t, uint8_t);
//模块驱动函数
void tcaScan(void);
void tcaSelect(uint8_t i);
void mpu_read();
void color_value(void);
byte read_sound(void);
void RGB_Config(void);
int  read_distance(void);
void ecoder_value(void);
void apds9960_value(void);

//逻辑处理函数,暂时有8个逻辑
void encoder_motor(int);
void encoder_LED(int);
void distance_motor(int);
void distance_LED(int);
void sound_motor(int);
void sound_LED(int);
void mpu_motor(int);
void mpu_LED(int);
void apds9960_LED(int);
void apds9960_motor(int);
//两个主控ble连接之后遥控的逻辑处理
void ble_encoder_motor(int);
void ble_encoder_LED(int);
void ble_distance_motor(int);
void ble_distance_LED(int);
void ble_sound_motor(int);
void ble_sound_LED(int);
void ble_mpu_motor(int);
void ble_mpu_LED(int);
void ble_apds9960_LED(int);
void ble_apds9960_motor(int);

void TaskPBle(void *pvParameters);
void getMac(char *mac_data);


//声音播放处理函数
void play_sound(const uint8_t* play_sound_data, uint16_t len , uint16_t vol) {

        for(int i=0; i<len; i++) {
            dacWrite(25, play_sound_data[i]/vol);
            delayMicroseconds(45);
            
        }
    
        for(int t=play_sound_data[len-1]/vol; t>=0; t--) {
            dacWrite(25, t);
            //delayMicroseconds(1000);
        }
    
}








//下面来自自ble-uart
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

//蓝牙sever接收回调A，需要两个AB，A是灯，b是电机
class MyCallbacks_A: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *Notify3_Characteristic) {
      std::string rxValue = Notify3_Characteristic->getValue();
      if (rxValue.length() > 0) {
        //Serial.println("****AAAAAAAAAAAAAAAAAAAAAAAAAAAA*****");
        //Serial.print("Received Value: ");
        
        for (int i = 0; i < rxValue.length(); i++){
          rxValue_A[i]=rxValue[i];
          //Serial.print(rxValue_A[i],HEX);
          //Serial.print(",");

        }

        if(rxValue_A[0]==0){
         
         BLE_SERVER_CALLBACK_DATA_LED[0]=rxValue_A[1];
         BLE_SERVER_CALLBACK_DATA_LED[1]=rxValue_A[2];
         BLE_SERVER_CALLBACK_DATA_LED[2]=rxValue_A[3];

        }
        else if (rxValue_A[0]==1)
        {
        
         BLE_SERVER_CALLBACK_DATA_LED[3]=rxValue_A[1];
         BLE_SERVER_CALLBACK_DATA_LED[4]=rxValue_A[2];
         BLE_SERVER_CALLBACK_DATA_LED[5]=rxValue_A[3];


        }
        else if(rxValue_A[0]==2){
         
         BLE_SERVER_CALLBACK_DATA_LED[6]=rxValue_A[1];
         BLE_SERVER_CALLBACK_DATA_LED[7]=rxValue_A[2];
         BLE_SERVER_CALLBACK_DATA_LED[8]=rxValue_A[3];

        }
        else if(rxValue_A[0]==3){
         
         BLE_SERVER_CALLBACK_DATA_LED[9] =rxValue_A[1];
         BLE_SERVER_CALLBACK_DATA_LED[10]=rxValue_A[2];
         BLE_SERVER_CALLBACK_DATA_LED[11]=rxValue_A[3];

        }
        else{}

        // for (int i = 0; i < 12; i++){
          
        //   Serial.print(BLE_SERVER_CALLBACK_DATA_LED[i],HEX);
        //   Serial.print(",");

        // }


        // Serial.println();
        // Serial.println("*********");
      }
    }
};
//复制的到这里


//蓝牙sever接收回调B
class MyCallbacks_B: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *Notify4_Characteristic) {
      std::string rxValue_B_br = Notify4_Characteristic->getValue();
      if (rxValue_B_br.length() > 0) {
        //Serial.println("****BBBBBBBBBBBBBBBBBBBBBBBBB*****");
        //Serial.print("Received Value_B: ");
        for (int i = 0; i < rxValue_B_br.length(); i++){
          rxValue_B[i]=rxValue_B_br[i];
          //Serial.print(rxValue_B[i],HEX);
          //Serial.print(",");

        }

        if(rxValue_B[0]==0){
         
         BLE_SERVER_CALLBACK_DATA_MOTOR[0]=rxValue_B[0];
         BLE_SERVER_CALLBACK_DATA_MOTOR[1]=rxValue_B[1];
         BLE_SERVER_CALLBACK_DATA_MOTOR[2]=rxValue_B[2];

        }
        else if (rxValue_B[0]==1)
        {
         
         BLE_SERVER_CALLBACK_DATA_MOTOR[3]=rxValue_B[0];
         BLE_SERVER_CALLBACK_DATA_MOTOR[4]=rxValue_B[1];
         BLE_SERVER_CALLBACK_DATA_MOTOR[5]=rxValue_B[2];


        }
        else if(rxValue_B[0]==2){
         
         BLE_SERVER_CALLBACK_DATA_MOTOR[6]=rxValue_B[0];
         BLE_SERVER_CALLBACK_DATA_MOTOR[7]=rxValue_B[1];
         BLE_SERVER_CALLBACK_DATA_MOTOR[8]=rxValue_B[2];

        }
        else if(rxValue_B[0]==3){
         
         BLE_SERVER_CALLBACK_DATA_MOTOR[9]=rxValue_B[0];
         BLE_SERVER_CALLBACK_DATA_MOTOR[10]=rxValue_B[1];
         BLE_SERVER_CALLBACK_DATA_MOTOR[11]=rxValue_B[2];

        }
        else{


        }

        // for (int i = 0; i < 12; i++){
          
        //   Serial.print(BLE_SERVER_CALLBACK_DATA_MOTOR[i],HEX);
        //   Serial.print(",");

        // }


        // Serial.println();
        // Serial.println("*********");
      }
    }
};







// 客户端类和函数定义
// notify通知回调，静态函数，不能被其他文件引用，防止重名
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,uint8_t* pData,size_t length,bool isNotify) 
{

    // Serial.print("Notify callback for characteristic ");
    // Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    // Serial.print(" of data length ");
    // Serial.println(length);
    // Serial.print("data: ");

    // Serial.print(pData[0],HEX);//(char*)pData
    // Serial.print(",");
    // Serial.print(pData[1],HEX);
    // Serial.print("%");
    // Serial.println(pData[2],HEX);


ble_button_up_down=pData[0];
ble_button_left_right=pData[1];
ble_button_ble=pData[2];








}



//  BLE客户端回调，包含两个函数
class MyClientCallback : public BLEClientCallbacks 
{
  void onConnect(BLEClient* pclient) 
  {

  }

  void onDisconnect(BLEClient* pclient) 
  {
     connected = false;
     //Serial.println("onDisconnect!!!!");
     
  }
};





// 连接到服务器
bool connectToServer() 
{
    //("Forming a connection to "); //形成一个连接，连接到...
    //Serial.println(myDevice->getAddress().toString().c_str()); //打印连接到的设备address
    //Serial.println(myDevice->getAddressType()); //打印连接到的设备address
    BLEClient*  pClient  = BLEDevice::createClient();//创建一个客户端
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

     //定义一个字符数组用来储存mac地址由字符串转换为字符数组
    char cArr[MAC_Address_From_NVS.length() + 1];
    //mac地址由字符串转换为字符数组
    MAC_Address_From_NVS.toCharArray(cArr,MAC_Address_From_NVS.length() + 1);
     //将字符数组形式mac地址转为uint8_t
     getMac(cArr);//执行完后mac_data_uint8_t变量就是转换完成后的mac地址

    //  for(int i=0;i<6;i++){

    //   Serial.print(mac_data_uint8_t[i],HEX);
    //   Serial.print(",");
    //    }
    //  Serial.println();

    if(pClient->connect(mac_data_uint8_t, BLE_ADDR_TYPE_PUBLIC)==1){ 

     Serial.println(" - Connected to server");
     //play_sound(ble_connect_sound,32278,Volume);

    }//如果第一次连接上了，则什么也不做
    else{//如果第一次没有连接，则再连一次

          Serial.println(" -fuck,  can not Connected to server--------error");
          BLEDevice::deinit(false); //如果为true，则释放BLE内存，并且无法再使用BLE。
          //Serial.println("zzzz");
          BLEDevice::deinit(true);

            for(int i=2; i<4; i++) {//切换到服务器模式之后全蓝
            pixels.setPixelColor(i, pixels.Color(0, 0,10 ));
            pixels.show();   
            delay(DELAYVAL); 
            }
            
            server_conf();
            connected=0;       //从机切换到主机时恢复逻辑控制功能
            deviceConnected=0; //
            flag=1;//恢复服务器模式
            return false;


          

    }

    //Serial.println(pClient->connect(myDevice));
    Serial.println(" - Connected to server");// 提示连接成功

    // 获取对远端BLE服务器中我们要获取的服务
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    
    if (pRemoteService == nullptr) //如果服务为空指针，则提示连接失败
    {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");// 没有失败则提示连接成功


    // 获取对远程BLE服务器的服务中的特征
    pRemoteCharacteristic_1 = pRemoteService->getCharacteristic(charUUID_1);
    pRemoteCharacteristic_2 = pRemoteService->getCharacteristic(charUUID_2);


    if (pRemoteCharacteristic_2 == nullptr) {//如果服务为空则提示失败
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID_2.toString().c_str());
      pClient->disconnect();
      return false;
    }
    //Serial.println(" - Found our characteristic");// 没有失败则提示连接成功

    // 如果是Read，则读取特征的值，放在这里就执行一次
    if(pRemoteCharacteristic_2->canRead()) {
      std::string value = pRemoteCharacteristic_2->readValue();
      //Serial.print("The characteristic value was: ");
      //Serial.println(value.c_str());
    }
   //如果是Notify，则回调notifyCallback，这里只获取按键的notify值
    if(pRemoteCharacteristic_1->canNotify())
      pRemoteCharacteristic_1->registerForNotify(notifyCallback);

    connected = true; // 标志位置True
}
/**
 * 扫描BLE服务器，找到第一个广播我们正在寻找的服务，注意这里是搜服务
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks 
  {
    /** 呼叫每个正在广播的BLE服务器 */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
    // 在广播的时候

    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
    
    //
    Server_BLE_Address = new BLEAddress(advertisedDevice.getAddress());
      
    Scaned_BLE_Address = Server_BLE_Address->toString().c_str();
    
    //我们已经找到了一个设备，现在让我们看看它的地址是否是我们需要的
    //读取eeprom里的地址，判断是否一致
    // address = 16;
    // Serial.print("callback_NVS.getString:  ");
    // Serial.println(NVS.getString ("myString"));

    // Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    Serial.println(" doConnect = FALSE ; doScan = FALSE;!!");
    if (Scaned_BLE_Address == MAC_Address_From_NVS)//My_BLE_Server_Address  EEPROM.readString(address)
    {
      Serial.println("1---doConnect = true ; doScan = true;!!");   
      BLEDevice::getScan()->stop();
      Serial.println("2---doConnect = true ; doScan = true;!!");  
      myDevice = new BLEAdvertisedDevice(advertisedDevice);// 实例化一个myDevice，从搜寻到的里面找的
      Serial.println("3---doConnect = true ; doScan = true;!!");  
      doConnect = true;//标志位
      Serial.println("4---doConnect = true ; doScan = true;!!");  
      doScan = true;
      
    } // Found our server

  } // onResult
}; // MyAdvertisedDeviceCallbacks


//客户端类和函数定义到这里


// 服务器端类及函数
void server_conf()
{
  //Serial.begin(9600);
  //Serial.println("Starting BLE work!");
  String BLE_Name_1="ICBricks ";
  String BLE_Name_2="";
  String BLE_Name="";

  //Serial.println("BLE_server_conf-1");//打印看一下
  BLEDevice::init(BLE_Name.c_str());
  //Serial.println("BLE_server_conf-2");//打印看一下
  My_BLE_Address=BLEDevice::getAddress().toString().c_str();  //获取自己的地址（主机模式下就可）
  //BLEDevice::deinit(false);//释放BLE内存
  //Serial.println("BLE_server_conf-3");//打印看一下
  BLE_Name_2=My_BLE_Address.substring(12,14)+My_BLE_Address.substring(15,17);            //截取mac地址最后两个十六进制数
  BLE_Name=BLE_Name_1+BLE_Name_2;
  esp_ble_gap_set_device_name(BLE_Name.c_str());//修改ble广播名称
  //Serial.println("BLE_server_conf-4");//打印看一下
  
  //esp_ble_gap_set_device_name();
  BLEServer *pServer = BLEDevice::createServer();
  //  Serial.println("BLE_server_conf-5");//打印看一下
  pServer->setCallbacks(new MyServerCallbacks());//MyServer回调
  //Serial.println("BLE_server_conf-6");//打印看一下
  //下面是创建BLE服务和特征的步骤

  //创建第一个服务
  BLEService *pService = pServer->createService(SERVICE_UUID_IN);//广播执行器
  //Serial.println("BLE_server_conf-7");//打印看一下

  //下面是创建Notify1特征部分，广播蓝牙按键的值，如果按键值有变化的话
  Notify1_Characteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_NOTIFY1,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_NOTIFY 
                  );             
  Notify1_Characteristic->addDescriptor(new BLE2902());
  //Serial.println("BLE_server_conf-8");//打印看一下
  //下面是创建Notify2特征部分，广播给icbricks客户端16字节传感器数据
  Notify2_Characteristic = pService->createCharacteristic(
                    
                    CHARACTERISTIC_UUID_NOTIFY2,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_WRITE |
                    BLECharacteristic::PROPERTY_NOTIFY 

                  );    
  Notify2_Characteristic->addDescriptor(new BLE2902());
  //Serial.println("BLE_server_conf-9");//打印看一下
  //下面是创建Notify3特征部分，接收值，接收的值用来控制led灯
  Notify3_Characteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_NOTIFY3,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_WRITE
                    
                  );    
  Notify3_Characteristic->addDescriptor(new BLE2902());
  Notify3_Characteristic->setCallbacks(new MyCallbacks_A()); //服务器端数据接收回调，用来接收执行器控制指令A段
  //Serial.println("BLE_server_conf-10");//打印看一下
  //下面是创建Notify4特征部分，接收值，接收的值用来控制电机
  Notify4_Characteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_NOTIFY4,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_WRITE
                  );    
  Notify4_Characteristic->addDescriptor(new BLE2902());
  Notify4_Characteristic->setCallbacks(new MyCallbacks_B()); ///服务器端数据接收回调，用来接收执行器控制指令B段

  //Serial.println("BLE_server_conf-11");//打印看一下

//下面是第二个服务

  BLEService *pService_2 = pServer->createService(SERVICE_UUID_OUT);///广播4个传感器
  //Serial.println("BLE_server_conf-12");//打印看一下
//下面四个特征是用来广播输入传感器值的，每个端口占用一个uuid

  NotifyA_Characteristic = pService_2->createCharacteristic(
                    
                    CHARACTERISTIC_UUID_NOTIFYA,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_NOTIFY 

                  );    
  NotifyA_Characteristic->addDescriptor(new BLE2902());
//
  //Serial.println("BLE_server_conf-13");//打印看一下
  NotifyB_Characteristic = pService_2->createCharacteristic(
                    
                    CHARACTERISTIC_UUID_NOTIFYB,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_NOTIFY 

                  );    
  NotifyB_Characteristic->addDescriptor(new BLE2902());
////
  //Serial.println("BLE_server_conf-14");//打印看一下
  NotifyC_Characteristic = pService_2->createCharacteristic(
                    
                    CHARACTERISTIC_UUID_NOTIFYC,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_NOTIFY 

                  );    
  NotifyC_Characteristic->addDescriptor(new BLE2902());
////
  //Serial.println("BLE_server_conf-15");//打印看一下
  NotifyD_Characteristic = pService_2->createCharacteristic(
                    
                    CHARACTERISTIC_UUID_NOTIFYD,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_NOTIFY 

                  );    
  NotifyD_Characteristic->addDescriptor(new BLE2902());

  //Serial.println("BLE_server_conf-16");//打印看一下
  //测试用
  Notify1_Characteristic->setValue("Hello");
  //NotifyB_Characteristic->setValue("Hello_FUCK");
  //Serial.println("BLE_server_conf-17");//打印看一下
  pService->start();
  //Serial.println("BLE_server_conf-18");//打印看一下
  pService_2->start();
  //创建BLE服务和特征的步骤到这里

  //Serial.println("BLE_server_conf-19");//打印看一下
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  //Serial.println("BLE_server_conf-20");//打印看一下
  pAdvertising->addServiceUUID(SERVICE_UUID_IN);
  //Serial.println("BLE_server_conf-21");//打印看一下
  pAdvertising->addServiceUUID(SERVICE_UUID_OUT);  
  //Serial.println("BLE_server_conf-22");//打印看一下
  pAdvertising->setScanResponse(true);
  //Serial.println("BLE_server_conf-23");//打印看一下
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  //Serial.println("BLE_server_conf-24");//打印看一下
  pAdvertising->setMinPreferred(0x12);
  //Serial.println("BLE_server_conf-25");//打印看一下
  BLEDevice::startAdvertising();
  //Serial.println("BLE_server_conf-26");//打印看一下
  //Serial.println("Characteristic defined! Now you can read it in your pho
  //Serial.println("Waiting a client connection to notify...");
  //设置服务器模式时顺便把灯变红

}


void client_conf()
{
  //Serial.begin(9600);
  //Serial.println("Starting Arduino BLE Client application...");
  //如果当前没有接收到来自串口2的地址值则锁死在当前循环
  old_waiting_serial2_data=millis();//获取一下当前的时间，用来判断超时
  //先锁定5秒钟时间用来接收串口2的数据，如果5秒内没有接收到新的地址则去取eeprom里的地址
  while(millis()-old_waiting_serial2_data<5000){ //My_BLE_Server_Address.length()<16如果字符串长度小于16则认为没有获取到地址，则循环接收数据
  //在此模式下等待接收从串口2接收到的数据
  //Serial.println("wating iic port send address!");

         for(int i=2; i<4; i++) {//切换到客户端连接之后全红
             pixels.setPixelColor(i, pixels.Color(10, 0,0 ));
             pixels.show();   
             delay(DELAYVAL); 
           }




   while (Serial2.available() > 0)  
    {
        comdata += char(Serial2.read());
        delay(2);
        
    }
   if (comdata.length()>0)
    {
       //Serial.println(comdata);
      //对接收到的字符串进行校验，如果头和尾分别是<>则截取mac部分并赋予搜索变量，同时写入nvs
      if((comdata[0]=='<')&&(comdata[18]=='>')){

       comdata=comdata.substring(1,18);
       NVS.setString ("myString", comdata);


      }
      
      else{}


       comdata = ""; //清空缓存变量
       
    }
   
    // 变量My_BLE_Server_Address在建立连接doConnect == true之后将会被清除
    // 这里也许应该加一个超时处理，有可能会卡在while这里
    //
  }
  Serial.println("cccc");

  //如果超过5秒获取串口2的数据之后就读取，一下nvs里的数据，用来对比搜寻到的设备的mac地址

  MAC_Address_From_NVS = NVS.getString ("myString");

  //BLEDevice::deinit(false); //如果为true，则释放BLE内存，并且无法再使用BLE。
  BLEDevice::deinit(true);

  delay(10);
  BLEDevice::init("fuck_esp_client");
  Serial.println("dddd");
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.


  //免搜索
  //BLEScan* pBLEScan = BLEDevice::getScan();
  //pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  //pBLEScan->setInterval(1349);
  //pBLEScan->setWindow(449);
  //pBLEScan->setActiveScan(true);
  //pBLEScan->start(8, false); //搜寻持续时间设置为7秒，如果超时则退出客户端模式

  //connectToServer();
  doConnect=true;


  Serial.println("eeee");
  
}
// 服务器端类及函数到这里









void setup(){ 



//看门狗初始化
esp_task_wdt_init(WDT_TIMEOUT, true); //启用超时，以便 ESP32 重新启动
esp_task_wdt_add(NULL); //将当前线程添加到 WDT watch 

Wire.begin();           //启动iic通信 

pixels.begin(); //初始化ws2812
//开机按键
pinMode(power_button,OUTPUT);
pinMode(power_button_ctrl,OUTPUT);
//4联按键
pinMode(ring_button_1,INPUT);
pinMode(ring_button_2,INPUT);
pinMode(ring_button_3,INPUT);
pinMode(ring_button_4,INPUT);

pinMode(power_state_detection,INPUT_PULLUP);//充电器插入
pinMode(power_value_detection,INPUT);//电量检测


pinMode(power_full_state_detection,INPUT);

//digitalWrite(model_out_power_ctrl, 1);

Speaker.begin();        //启动喇叭
//Speaker.setVolume(5);  //设置音量

Serial.begin(57600);    //串口1初始化
Serial2.begin(9600);    //此串口用来传送主机的mac地址
NVS.begin();

//下面是双核心处理相关  


xTaskCreatePinnedToCore(
    TaskPBle    
    ,  "BLEPriter"   // A name just for humans
    ,  6500  // 1024 * 5 This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);     // 这个0，表示要执行的TaskPBle是在核0中执行。


     


}



void loop(){

  //看门狗处理
  // 重置 WDT 每2s
 if (millis() - last_WDT_time >= 2000 ) {
 //Serial.println("Resetting WDT...");
 esp_task_wdt_reset();
 last_WDT_time = millis();
 Serial2.print("<"+My_BLE_Address+">");//每隔2秒，随看门狗给串口发发一次当前ble服务器的mac地址


 }


  Speaker.update(); //更新喇叭


  //开机处理逻辑
  digitalWrite(power_button,1);//用于治疗常绿异常情况
  if ((digitalRead(power_button) == 0) && (power_button_flag == 0)) 
  {
 
    
    digitalWrite(power_button_ctrl, 1);//钳住电源，不用断电
    power_button_flag = 1;
    //开机之后先亮绿色，绿色渐变

    //
    
    for(int j=0; j<8; j++) { 
      for(int i=0; i<2; i++) { 
          pixels.setPixelColor(i, pixels.Color(0,j , 0)); 
          pixels.show();   
          delay(DELAYVAL); 
       }
           //将蓝牙灯打开
        for(int i=2; i<4; i++) { 
        pixels.setPixelColor(i, pixels.Color(0, 0,j )); //初始化蓝牙灯亮蓝色
        pixels.show();   
        delay(DELAYVAL); 
      }
       
       
       //delay(10);

    }


//DacAudio.Play(&ON_sound); 

digitalWrite(power_button,1);//用于治疗常绿异常情况
while (digitalRead(power_button) == 0) { }
    Serial.println("fuck????????aaaaaa");

  
    

  play_sound(power_on_sound,18511,Volume);




   
  
  }
  
  //关机机处理逻辑
  digitalWrite(power_button,1);//用于治疗常绿异常情况
  if ((digitalRead(power_button) == 0) && (power_button_flag == 1)) {
    Serial.println("ccccccc");
    unsigned long poweroff_time=millis() ;
    digitalWrite(power_button,1);//用于治疗常绿异常情况
    while (digitalRead(power_button) == 0) //下面逻辑是为了按住一定时间之后关机，延时关机
    {   
      if(millis()-poweroff_time>1000){
          BLEDevice::deinit(false);    //如果为true，则释放BLE内存，并且无法再使用BLE，不释放的话
          power_button_flag = 0;
          digitalWrite(power_button_ctrl, 0);
              //开机之后先亮绿色，绿色渐变
          for(int j=8; j>=0; j--) { 
            for(int i=0; i<4; i++) { 
                pixels.setPixelColor(i, pixels.Color(0, j, 0)); 
                pixels.show();   
                delay(DELAYVAL); 
          }delay(10);
          }while(digitalRead(power_button) == 0){
                //如果不加下面的部分会导致关机时松开按键的时候闪一下，效果不好
                for(int i=0; i<4; i++) { 
                pixels.setPixelColor(i, pixels.Color(0, 0, 0)); 
                pixels.show();   
                delay(DELAYVAL); 
                }


          };//卡死在这里

    
        
      }

     }

    //Serial.println("OFF");
   

  } 


//刷新检测一下当前是否插入充电器,黑板子插入是高，篮板子插入是低，导致无法判断,只在插入或拔出时反应一下
 charge_state=digitalRead(power_state_detection);

//为了测试在这里区别对待标志位为0和1的情况 0为关机
if(power_button_flag == 0){
  //digitalWrite(power_button,0);


 //检测当前充满引脚的状态，黑板和篮板充满都是高
 bat_full_state=digitalRead(power_full_state_detection);

// Serial.print("bat_full_state:");
// Serial.println(bat_full_state);

//在关机状态下充电状态检测和电量指示



//vb测试:黑板：off：3340，on：3360    篮板：off：3190，on：3270//3389为8.2V，大于8.2就认为是满电了,读值是3160
  if((bat_full_state==1)||(chongman==1)){  //
    for(int i=0; i<2; i++) { 
        pixels.setPixelColor(i, pixels.Color(0, 10,0 )); 
        pixels.show();   
        delay(DELAYVAL); 
      }
     chongman=1;//充满标志，只要达到过充满，直接置1，除非断电重启会清零
  }
  else{


      for(int cr_value=0;cr_value<10;cr_value++){
        for(int i=0; i<2; i++) { 
        pixels.setPixelColor(i, pixels.Color(cr_value, 0, 0));
        pixels.show();   
        delay(DELAYVAL); 
        if(digitalRead(power_button) == 0)break;
      }
        delay(10); 
      }
      for(int cr_value=10;cr_value>0;cr_value--){
        for(int i=0; i<2; i++) { 
        pixels.setPixelColor(i, pixels.Color(cr_value, 0, 0));
        pixels.show();   
        delay(DELAYVAL); 
        if(digitalRead(power_button) == 0)break;
      }
       delay(10); 
    }




  }






//关闭蓝牙指示灯
for(int i=2; i<4; i++) { 
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();   
    delay(DELAYVAL); 
    
  }

 //Serial.println("OFF");


flag=1;//插电开关机后恢复flag变量至服务器模式
}



//为了测试在这里区别对待标志位为0和1的情况 1为开机
if(power_button_flag == 1){


tcaScan(); 
for(int i=0;i<8;i++){  //打印当前已经扫描到的地址
 //Serial.print(iic_addr[i],HEX);
}






//开机之后0，1号灯检测当前电量
 int vb=0;

//获取当前电量的模拟值,简单5次平均值滤波 
for(int vb_int_a=0;vb_int_a<5;vb_int_a++){vb=vb+analogRead(power_value_detection);}
vb=vb/5;
// Serial.print("on——vb:");
// Serial.println(vb);
 if(vb<2270){
 for(int i=0; i<2; i++) { 
    pixels.setPixelColor(i, pixels.Color(10, 0, 0)); //小于2270，红色
    pixels.show();   
    delay(DELAYVAL); 
  }

 }
 else if(vb>3180)
  {  //3389为8.2V，大于8.2就认为是满电了
    for(int i=0; i<2; i++) { 
        pixels.setPixelColor(i, pixels.Color(0, 10,0 )); //红色减淡，绿色渐亮
        pixels.show();   
        delay(DELAYVAL); 
      }

  }
  else{
    vb=map(vb,2270,3180,0,10);//2270约对应电池电压5.5v，根据分压，满电8.4v不会超过3476
    for(int i=0; i<2; i++) { 
        pixels.setPixelColor(i, pixels.Color(10-vb,vb , 0)); //红色减淡，绿色渐亮
        pixels.show();   
        delay(DELAYVAL); 
      }


 }


if(charge_state!=old_charge_state)//如果充电器接入状态有变化了，则灯光提示
{

  //GameAudio.PlayWav(&Charge_connect, false, 1.0);// 播放充电连接声音
     
  //digitalWrite(speaker_control,1);
  for(int charge_flag=0;charge_flag<10;charge_flag++){
    for(int i=0; i<2; i++) { 
    pixels.setPixelColor(i, pixels.Color( 0,0 ,charge_flag));
    pixels.show();   
    delay(DELAYVAL); 
    if(digitalRead(power_button) == 0)break;
  }
 delay(20); 
}
 for(int charge_flag=10;charge_flag>0;charge_flag--){
    for(int i=0; i<2; i++) { 
    pixels.setPixelColor(i, pixels.Color( 0, 0,charge_flag));
    pixels.show();   
    delay(DELAYVAL); 
    if(digitalRead(power_button) == 0)break;
  }
 delay(20); 
}
}




//如果没有连接，处理主控器几个按键按下时要做的事，这里先让按键都有声音，然后上下控制AB，左右控制CD

//使用for循环轮询4个输入端口


for(int ch=4;ch<8;ch++){

//如果接入了手势识别模块，则初始化手势识别模块
if((iic_addr[ch]==0x39)&&(old_iic_addr[ch]==0x00)){
    tcaSelect(ch);
    apds.init() ;
    apds.enableGestureSensor(true);
}





//下面是生成ble服务器发送传感器数据的字节，在主控互联或者与平板建立连接连接时执行
//需要在建立连接后启动传送，由一直传送测试修改为连接后再处理
//0x36编码器,0x29距离,0x48声音,0x68陀螺仪,0x67电机,0x39手势,0x40触摸灯

    switch (iic_addr[ch])
    {
    case 0x36://编码器
      {
        //下面两行不打开，打开之后会出现编码器控制不了输出，不知道什么原因，先这样用
        tcaSelect(ch); 
        ecoder_value();//获取编码器的值，执行完该函数之后会产生3个值 按键值get_encoder_data[2]，旋转值encoder_data，旋转绝对值abs_data
        //Serial.print("?????????????????");
        if     (ch==4){port_in_A[0]=0x36;port_in_A[1]=get_encoder_data[2];port_in_A[2]=encoder_data;port_in_A[3]=abs_data;}//4为第一个输入
        else if(ch==5){port_in_B[0]=0x36;port_in_B[1]=get_encoder_data[2];port_in_B[2]=encoder_data;port_in_B[3]=abs_data;}//5为第二个输入
        else if(ch==6){port_in_C[0]=0x36;port_in_C[1]=get_encoder_data[2];port_in_C[2]=encoder_data;port_in_C[3]=abs_data;}//6为第三个输入
        else if(ch==7){port_in_D[0]=0x36;port_in_D[1]=get_encoder_data[2];port_in_D[2]=encoder_data;port_in_D[3]=abs_data;}//7为第四个输入
      }
      break;
    case 0x29://距离
      {
        tcaSelect(ch); 
        juli=read_distance();
        if     (ch==4){port_in_A[0]=0x29;port_in_A[1]=juli;port_in_A[2]=0;port_in_A[3]=0;}//4为第一个输入
        else if(ch==5){port_in_B[0]=0x29;port_in_B[1]=juli;port_in_B[2]=0;port_in_B[3]=0;}//5为第二个输入
        else if(ch==6){port_in_C[0]=0x29;port_in_C[1]=juli;port_in_C[2]=0;port_in_C[3]=0;}//6为第三个输入
        else if(ch==7){port_in_D[0]=0x29;port_in_D[1]=juli;port_in_D[2]=0;port_in_D[3]=0;}//7为第四个输入


      }
      break;
    case 0x48://声音
      {
        tcaSelect(ch); 
        sound_value=read_sound();
        if     (ch==4){port_in_A[0]=0x48;port_in_A[1]=sound_value;port_in_A[2]=0;port_in_A[3]=0;}//4为第一个输入
        else if(ch==5){port_in_B[0]=0x48;port_in_B[1]=sound_value;port_in_B[2]=0;port_in_B[3]=0;}//5为第二个输入
        else if(ch==6){port_in_C[0]=0x48;port_in_C[1]=sound_value;port_in_C[2]=0;port_in_C[3]=0;}//6为第三个输入
        else if(ch==7){port_in_D[0]=0x48;port_in_D[1]=sound_value;port_in_D[2]=0;port_in_D[3]=0;}//7为第四个输入


      }
      break;
    case 0x68://陀螺仪
      {
        tcaSelect(ch); 
        mpu_read();
        if     (ch==4){port_in_A[0]=0x68;port_in_A[1]=AcX;port_in_A[2]=AcY;port_in_A[3]=AcZ;}//4为第一个输入
        else if(ch==5){port_in_B[0]=0x68;port_in_B[1]=AcX;port_in_B[2]=AcY;port_in_B[3]=AcZ;}//5为第二个输入
        else if(ch==6){port_in_C[0]=0x68;port_in_C[1]=AcX;port_in_C[2]=AcY;port_in_C[3]=AcZ;}//6为第三个输入
        else if(ch==7){port_in_D[0]=0x68;port_in_D[1]=AcX;port_in_D[2]=AcY;port_in_D[3]=AcZ;}//7为第四个输入


      }
      break;
    case 0x39://手势
      {
        tcaSelect(ch); 
        apds9960_value();//执行完此函数之后apds9960_direction变量将变为1-4
        if     (ch==4){port_in_A[0]=0x39;port_in_A[1]=apds9960_direction;port_in_A[2]=0;port_in_A[3]=0;}//4为第一个输入
        else if(ch==5){port_in_B[0]=0x39;port_in_B[1]=apds9960_direction;port_in_B[2]=0;port_in_B[3]=0;}//5为第二个输入
        else if(ch==6){port_in_C[0]=0x39;port_in_C[1]=apds9960_direction;port_in_C[2]=0;port_in_C[3]=0;}//6为第三个输入
        else if(ch==7){port_in_D[0]=0x39;port_in_D[1]=apds9960_direction;port_in_D[2]=0;port_in_D[3]=0;}//7为第四个输入


      }
      break;

      default:
      {
        if     (ch==4){port_in_A[0]=0;port_in_A[1]=0;port_in_A[2]=0;port_in_A[3]=0;}//4为第一个输入
        else if(ch==5){port_in_B[0]=0;port_in_B[1]=0;port_in_B[2]=0;port_in_B[3]=0;}//5为第二个输入
        else if(ch==6){port_in_C[0]=0;port_in_C[1]=0;port_in_C[2]=0;port_in_C[3]=0;}//6为第三个输入
        else if(ch==7){port_in_D[0]=0;port_in_D[1]=0;port_in_D[2]=0;port_in_D[3]=0;}//7为第四个输入

      }

      break;

    }


    //生成16个字节的传感器各端口值
    uint8_t port_in_ALL[16] = {port_in_A[0],port_in_A[1],port_in_A[2],port_in_A[3],port_in_B[0],port_in_B[1],port_in_B[2],port_in_B[3],port_in_C[0],port_in_C[1],port_in_C[2],port_in_C[3],port_in_D[0],port_in_D[1],port_in_D[2],port_in_D[3]}; //用来传送输入端口的数值
          


  //下面处理各种蓝牙连接，或者是没有连接的情况
  //deviceConnected变量 标志服务器模式下被连接
  //connected变量 标志客户端模式下被连接
  //如果服务器模式下蓝牙连接了，则广播4个端口传感器值
  

//如果蓝牙连接了，则接收控制数据
//因跨核心的数组变量定义问题，服务器模式下的蓝牙发送暂时放到这里
if (deviceConnected) {


        NotifyA_Characteristic->setValue(port_in_A, 4);
        NotifyB_Characteristic->setValue(port_in_B, 4);
        NotifyC_Characteristic->setValue(port_in_C, 4);
        NotifyD_Characteristic->setValue(port_in_D, 4);

        if(port_in_A[0]!=0){NotifyA_Characteristic->notify();}
        if(port_in_B[0]!=0){NotifyB_Characteristic->notify();}
        if(port_in_C[0]!=0){NotifyC_Characteristic->notify();}
        if(port_in_D[0]!=0){NotifyD_Characteristic->notify();}


        delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
        //下面是广播给icbricks客户端的
        
        Notify2_Characteristic->setValue(port_in_ALL, 16);

        delay(5); // 太快了蓝牙栈将会堵塞
        ble_button_handle();//服务器模式下处理按键逻辑








    if(iic_addr[0]==0x40){
    tcaSelect(0);
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(0);  // 第1个字节
    Wire.write(0); /* 第2个字节，亮度值*/
    Wire.write(1);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[0]);     // 第4个字节R
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[1]);  // 第5个字节G
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[2]);  // 第6个字节B
    Wire.endTransmission();
    }
 if(iic_addr[1]==0x40){
    tcaSelect(1);
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(0);  // 第1个字节
    Wire.write(0); /* 第2个字节，亮度值*/
    Wire.write(1);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[3]);     // 第4个字节R
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[4]);  // 第5个字节G
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[5]);  // 第6个字节B
    Wire.endTransmission();
    }
 if(iic_addr[2]==0x40){
    tcaSelect(2);
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(0);  // 第1个字节
    Wire.write(0); /* 第2个字节，亮度值*/
    Wire.write(1);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[6]);     // 第4个字节R
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[7]);  // 第5个字节G
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[8]);  // 第6个字节B
    Wire.endTransmission();
    }
 if(iic_addr[3]==0x40){
    tcaSelect(3);
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(0);  // 第1个字节
    Wire.write(0); /* 第2个字节，亮度值*/
    Wire.write(1);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[9]);     // 第4个字节R
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[10]);  // 第5个字节G
    Wire.write(BLE_SERVER_CALLBACK_DATA_LED[11]);  // 第6个字节B
    Wire.endTransmission();
    }










   
  if(iic_addr[0]==0x67){
    motor_ctrl(BLE_SERVER_CALLBACK_DATA_MOTOR[0],BLE_SERVER_CALLBACK_DATA_MOTOR[1],BLE_SERVER_CALLBACK_DATA_MOTOR[2]);
  }

  if(iic_addr[1]==0x67){
    motor_ctrl(BLE_SERVER_CALLBACK_DATA_MOTOR[3],BLE_SERVER_CALLBACK_DATA_MOTOR[4],BLE_SERVER_CALLBACK_DATA_MOTOR[5]);
  }


  if(iic_addr[2]==0x67){
    motor_ctrl(BLE_SERVER_CALLBACK_DATA_MOTOR[6],BLE_SERVER_CALLBACK_DATA_MOTOR[7],BLE_SERVER_CALLBACK_DATA_MOTOR[8]);
  }

  if(iic_addr[3]==0x67){
    motor_ctrl(BLE_SERVER_CALLBACK_DATA_MOTOR[9],BLE_SERVER_CALLBACK_DATA_MOTOR[10],BLE_SERVER_CALLBACK_DATA_MOTOR[11]);
  }






//bluetooth_button_handle();

}




//如果在客户端模式下蓝牙连接了，connected变量将为真
//将远程获取的传感器值和本地连接的执行器模块进行处理
else if(connected){
//对应关系
/*
客户端获取的服务器端传感器iic地址   对应TCA9548   左    右
sensor_value_16byte[0]                         7----->0
sensor_value_16byte[4]                         6----->1
sensor_value_16byte[8]                         5----->2
sensor_value_16byte[12]                        4----->3
*/
//下面对16进制地址进行减法，输入地址-输出地址，对结果进行判断
//ch在7，6，5，4之间，根据两个数列的关系进行计算28-4*ch
int ble_model_judge = sensor_value_16byte[4*ch-16]-iic_addr[7-ch];//十六进制减完之后变成10进制;//d



//蓝牙无线按键控制
if((sensor_value_16byte[8]==0)&&(sensor_value_16byte[12]==0))//如果服务器上侧两个输入口没有接传感器，则客户端可以用上下环按键控制
{

  if(ble_button_up_down==1){

    motor_ctrl(0,1,100);//port0电机转动正
    motor_ctrl(1,1,100);//port1电机转动正
    led_ctrl(0,0); //port0开灯
    led_ctrl(1,0); //port1开灯

  }    
  else if(ble_button_up_down==2){

    motor_ctrl(0,2,100);//port0电机转动反
    motor_ctrl(1,2,100);//port1电机转动反
    led_ctrl(0,0); //port0开灯
    led_ctrl(1,0); //port1开灯

  }
    
  else{
      motor_ctrl(0,0,0);//port0电机停止
      motor_ctrl(1,0,0);//port1电机停止
      led_ctrl(0,1); //port0关灯
      led_ctrl(1,1); //port1关灯

    
  }


}


if((sensor_value_16byte[0]==0)&&(sensor_value_16byte[4]==0))//如果服务器下侧两个输入口没有接传感器，则客户端可以用左右环按键控制
{

  if(ble_button_left_right==1){ 
    motor_ctrl(2,1,100);//port0电机转动正
    motor_ctrl(3,1,100);//port1电机转动正
    led_ctrl(2,0); //port0开灯
    led_ctrl(3,0); //port1开灯


  }    


  else if(ble_button_left_right==2){
    motor_ctrl(2,2,100);//port2电机转动反
    motor_ctrl(3,2,100);//port3电机转动反
    led_ctrl(2,0); //port2开灯
    led_ctrl(3,0); //port3开灯


  }    
  else{
      motor_ctrl(2,0,0);//port2电机停止
      motor_ctrl(3,0,0);//port3电机停止
      led_ctrl(2,1); //port2关灯
      led_ctrl(3,1); //port3关灯F

    
  }

  
}







switch(ble_model_judge){

case -10:{ble_encoder_LED(ch);break;}
case -49:{ble_encoder_motor(ch);break;}
case -23:{ble_distance_LED(ch);break;}
case -62:{ble_distance_motor(ch);break;}
case -31:{ble_sound_motor(ch);break;}
case 8:{ble_sound_LED(ch); break;}
case 1: {ble_mpu_motor(ch);  break;}
case 40:{ble_mpu_LED(ch);break;}
case -7:{ble_apds9960_LED(ch);break;}
case -46:{ble_apds9960_motor(ch);break;}

}


}

else //如果没有蓝牙连接，则处理逻辑控制
{


Serial.write(port_in_ALL,16);//串口输出,用于scratch互动，传输执行器参数
delay(1);
//下面对16进制地址进行减法，输入地址-输出地址，对结果进行判断
int model_judge=iic_addr[ch]-iic_addr[7-ch];//十六进制减完之后变成10进制
//Serial.print("test-pass-speed");

if((iic_addr[7]==0)&&(iic_addr[6]==0))//如果上侧两个输入口没有接传感器，则可以用上下环按键控制
{

 ring_button_handle_up();


}
if((iic_addr[5]==0)&&(iic_addr[4]==0))//如果下侧两个输入口没有接传感器，则可以用左右环按键控制
{
  ring_button_handle_down();
  
}


switch(model_judge){

case -10:{encoder_LED(ch);break;}
case -49:{encoder_motor(ch);break;}
case -23:{distance_LED(ch);break;}
case -62:{distance_motor(ch);break;}
case -31:{sound_motor(ch);break;}
case 8:{sound_LED(ch); break;}
case 1: {mpu_motor(ch);  break;}
case 40:{mpu_LED(ch);break;}
case -7:{apds9960_LED(ch);break;}
case -46:{apds9960_motor(ch);break;}

}





}

old_iic_addr[ch]=iic_addr[ch]; //

}



for(int i=0;i<8;i++){  //将地址存储数组清零
iic_addr[i]=0;
}

old_charge_state=charge_state;//更新充电器接入状态

}



}







































///////////////////////////////////////////////////////////////////////////////////////

// 核 0 的程序，蓝牙
void TaskPBle(void *pvParameters)
{
  (void) pvParameters;

  


  for(;;){
    //继承来自loop的逻辑，如果处于开机状态

    //查看task1堆栈大小
    unsigned int temp1 = uxTaskGetStackHighWaterMark(nullptr);
    //Serial.print("task1="); Serial.println(temp1);

  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());
    //定义了一个kaiji变量，当程序初次运行的时候为1，当处于开机模式且kaiji=1的时候，只运行一次
     if((kaiji==1)&&(power_button_flag == 1)) {

     server_conf();
     kaiji=0;

     }


     if(power_button_flag == 1){


      //仍然保留按键点按蓝牙按键时发送主机的地址到串口2，
      if(digitalRead(bluetooth_button)==0){
      
      //Speaker.playMusic(button_sound, 22050); 
      
      Serial2.print("<"+My_BLE_Address+">");
      

      }

    //开机之后先处理蓝牙部分，复制自合并前esp_ble_test
    ble_old_time = millis();
       //处理蓝牙按键一直按住的情况，切换工作模式

    //Serial.println("task1==============================");
    while(digitalRead(bluetooth_button)==0){

    ble_c_time=millis() ;
    Serial.println(ble_c_time-ble_old_time);

    if(ble_c_time-ble_old_time>3000)//如果大于3000ms
    {
      //BLEDevice::deinit(false); //如果为true，则释放BLE内存，并且无法再使用BLE。
      //while(digitalRead(bluetooth_button)==0){}
        Serial.println("zzzz");

          Serial.println("wwww");
          flag=!flag;
          Serial.println(flag);
            //connected=0;       //从机切换到主机时恢复逻辑控制功能
            //deviceConnected=0; //



          if(flag==1)//如果是在客户端模式且被连接着的时候，切换模式，则改为服务器模式
          {
            Serial.println("reboot----1");
            for(int i=2; i<4; i++) {//切换到服务器模式之后全蓝
            pixels.setPixelColor(i, pixels.Color(0, 0,10 ));
            pixels.show();   
            delay(DELAYVAL); 
          }
           Serial.println("reboot----2");
            delay(5);
            flag=1;
             //BLEDevice::deinit(true); // 重置BLE。
            BLEDevice::deinit(false); // 重置BLE。
           Serial.println("reboot----3");
            server_conf();
             Serial.println("reboot----4");
            delay(500);
            connected=0;       //从机切换到主机时恢复逻辑控制功能
            deviceConnected=0; //
            

          }  

          if(flag==0)//其余情况，置为客户端
          {
            Serial.println("aaaaa");


          //   for(int i=2; i<4; i++) {//切换到客户端连接之后全红
          //   pixels.setPixelColor(i, pixels.Color(10, 0,0 ));
          //   pixels.show();   
          //   delay(DELAYVAL); 
          // }
        BLEDevice::deinit(false); // 重置BLE。
          //BLEDevice::deinit(true);

            flag=0;
            client_conf();
            Serial.println("bbbbb");


           
          }



        

        }////如果按压大于3秒

     }//如果一直压住 


    
if(flag==0){ //如果是客户端

  //Serial.println("fuck_Client.");

  if (doConnect == true) {
    if (connectToServer()) {
      //Serial.println("We are now connectded to the BLE Server.");
      //在这里清除本次串口2接收的值
      
    } else {
      //Serial.println("We have failed to co nnect to the server; there is nothin more we will do.");
    doScan=true;
    
    }
    doConnect = false;
  }

  // 在客户端模式，如果已经连接到ble服务器了，则从服务器读取16个字节的传感器数据
  
  if (connected) {

    sensor_value_16byte = pRemoteCharacteristic_2->readValue();
  //   if (sensor_value_16byte.length() > 0) {
  //   Serial.print("Read sensor_value_16byte : ");
  //   //使用for循环来读取接收的值，共16个字节
  //   for (int i = 0; i < sensor_value_16byte.length(); i++)
  //   {
  //     Serial.println(sensor_value_16byte[i],HEX);
  //   }

  //  // Serial.println();
  // }

    for(int i=2; i<4; i++) {//客户端连接之后变绿 
      pixels.setPixelColor(i, pixels.Color(0, client_huxi_value,client_huxi_value ));
      pixels.show();   
      delay(DELAYVAL); 
      }
     client_huxi_value--;
     if(client_huxi_value<0)client_huxi_value=10;

    delay(5); 

  //客户端逻辑处理








  }
  
  else if(doScan){ //如果在正常状态下突然断开链接了，则执行下面的部分

    //BLEDevice::getScan()->start(0);  // 可以选择是否持续搜寻
    BLEDevice::deinit(false); // 重置BLE。
    BLEDevice::deinit(true);  
    server_conf();// 恢复到BLE主机状态
    flag=1; 


for(int i=0; i<4; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 10));
    pixels.show();   
    delay(DELAYVAL); 
  }






  }

//c处理如果处于服务器模式，扫描完成后没有连接的对象，则恢复到服务器模式
else{

    BLEDevice::deinit(false); // 重置BLE。
    BLEDevice::deinit(true);  
    server_conf();// 恢复到BLE主机状态
    flag=1; 


for(int i=0; i<4; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 10));
    pixels.show();   
    delay(DELAYVAL); 
  }




}


  
  //delay(20); // Delay 



}


if(flag==1){ //如果是服务器

/*
  Serial.println("fuck_Server.");Serial.println
  // 

  delay(20);
  //Serial.println(myDevice->getAddress().toString().c_str());
  //();//

*/


 //如果在服务器模式下，蓝牙被链接了，则执行这里
 //则蓝牙更新当前传感器状态，并且接收执行器控制


      // for(int i=2; i<4; i++) {  //如果服务器被连接了则将灯置红，此时蓝牙灯会红蓝交替闪烁
      //   pixels.setPixelColor(i, pixels.Color(10, 0, 0));
      //   pixels.show();   
      //   delay(DELAYVAL); 
      // }
if (deviceConnected) {


    for(int i=2; i<4; i++) {//客户端连接之后变绿 
      pixels.setPixelColor(i, pixels.Color(server_huxi_value, 0, server_huxi_value));
      pixels.show();   
      delay(DELAYVAL); 
      }
     server_huxi_value++;
     if(server_huxi_value>10)server_huxi_value=0;

     delay(3);



    }

 if(deviceConnected==false){

    for(int i=2; i<4; i++) {//客户端连接之后变绿 
      pixels.setPixelColor(i, pixels.Color(0, 0, 10));
      pixels.show();   
      delay(DELAYVAL); 
      }


 }    


    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        //Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;


    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

     }










     }//开机状态
    


  delay(10);

  }
 vTaskDelete(NULL);
}



///////////////////////////////////////////////////////////////////////


























// TCA通道选择
void tcaSelect(uint8_t i){
    byte error;
    Wire.beginTransmission(TCAADDR);
    Wire.write( 1<<i );
    error = Wire.endTransmission();
    switch (error) {
    case 0:
        break;
    case WIRE_ADDRESS_ERROR:
        //Serial.printf("Wire address error\n");
        break;
    case WIRE_NACKORERROR_ERROR:
        //Serial.printf("Wire NAK or ERROR\n");
        break;
    default:
        //Serial.printf("Wire unknown error: %d\n", error);
        break;
    }
}



void tcaScan(void)
{

    //Wire.begin();
    for (auto t = 0; t < 8 /*NUM_TCA*/; t++) {
        byte error;
        //Serial.printf("TCA Port # %d \n", t);// 打印当前正在扫描的PORT
        tcaSelect(t);
        for (int j = 0; j < 8; j++ ) { 
            
            Wire.beginTransmission(device_address[j]);
            error = Wire.endTransmission();
            if (!error) {                                                                        
                    //Serial.printf("  Unknown I2C device found at address 0x%02x\n", device_address[j]);
                    
                    iic_addr[t] =device_address[j] ;//;0x34
            }
            else if (error != WIRE_ADDRESS_ERROR) {
                //Serial.printf("  error I2C device error: %d, at address 0x%02x\n", error, device_address[j]);
            }
         
        }

    }


}






/* mpu数据读取函数 */
void mpu_read(void){

 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission();


 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission();
 Wire.requestFrom(MPU_addr,uint8_t(14),true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
// x轴
    if(abs(AcX)<3000){
        AcX=0 ;
    }//死区

    else if(abs(AcX)>16000){
        AcX=255;  
    }
    else if((abs(AcX)<16000)&&(abs(AcX)>3000)){
        AcX=map(abs(AcX),3000,16000,0,255);
        }
    else{}
// Y轴
    if((AcY<3000)&&(AcY>-3000)){
        AcY=127 ;
    }//死区
    else if(AcY>16000){
        AcY=255;  
    }
    else if(AcY<-16000){
        AcY=0;  
    }

    else if((AcY>=-16000)&&(AcY<=-3000)){
        AcY=map(AcY,-16000,-3000,0,126);
    }

    else if((AcY>=3000)&&(AcY<=16000)){

        AcY=map(AcY,3000,16000,128,255);

        }
    else{}



// z轴
    if(abs(AcZ)<3000){
        AcZ=0 ;
    }//死区

    else if(abs(AcZ)>16000){
        AcZ=255;  
    }
    else if((abs(AcZ)<16000)&&(abs(AcZ)>3000)){
        AcZ=map(abs(AcZ),3000,16000,0,255);
        }
    else{}








//  Serial.print(AcX);//-old_ax
//  Serial.print(" ,");
//  Serial.print(AcY); //-old_ay
//  Serial.print(" ,");
//  Serial.print(AcZ); //-old_az
//  Serial.println(" ,");
 /*
 Serial.print(GyX); 
 Serial.print(" ,");
 Serial.print(GyY); 
 Serial.print(" ,");
 Serial.print(GyZ); 
 Serial.println();

*/

 delay(1);

 }

//获取距离
int read_distance(void){

 VL53L0X dis_sensor;
 dis_sensor.setTimeout(500);
 dis_sensor.init();
 //dis_sensor.setMeasurementTimingBudget(20000);
 vl_distance = (dis_sensor.readRangeSingleMillimeters()/10)-4;//减去4为了校准误差，大约偏差，每个传感器都不一样
 if(vl_distance>125){vl_distance=124;}//限制距离不大于124，大于124数据无效
 else if(vl_distance<=0){vl_distance=0;}//防止距离出现负值
 else{}
 //Serial.println(vl_distance);
 return vl_distance;

}


//获取编码器的值


void ecoder_value(void){

Wire.requestFrom(0x36, 3);   // 从0x38地址请求3个字节数据
for(int k=0; k<3 ; k++)
 {
   get_encoder_data[k]=Wire.read(); // receive a byte as character
 }

encoder_data = (short)((get_encoder_data[0] << 8) | get_encoder_data[1]); //将传送过来的两个字节转换为一个16位有符号数
//Serial.print(":"); 
//Serial.print(get_encoder_data[2]); //按键值
//Serial.print(","); 



//////////////





//////////////
if(encoder_data<0)
{
  abs_data=0;
}
else{
  abs_data=1;
}
encoder_data=abs(encoder_data);//保证这个字节都为正值
//Serial.print(abs_data); 
//Serial.print(","); 
//Serial.println(encoder_data);        //当前旋转值


}

//获取手势传感器的值
void apds9960_value(void){



 if ( apds.isGestureAvailable() ) {
    switch ( apds.readGesture() ) {
      case DIR_UP:
        //Serial.println("UP^^^^^^^^^^^^^^^^^^^^^");
        apds9960_direction=1;
        break;
      case DIR_DOWN:
        //Serial.println("DOWN%%%%%%%%%%%%%%%%%%%");
        apds9960_direction=2;
        break;
      case DIR_LEFT:
        //Serial.println("LEFT<<<<<<<<<<<<<<<<<<<<<<<<<<");
        apds9960_direction=3;
        break;
      case DIR_RIGHT:
        //Serial.println("RIGHT>>>>>>>>>>>>>>>>>>>>>>>>>>");
        apds9960_direction=4;
        break;
      case DIR_NEAR:
        //Serial.println("NEAR???????????????????????");
        break;
      case DIR_FAR:
        //Serial.println("FAR???????????????????????????");
        break;
      default:
      ;
        //Serial.println("NONE");
    }
  }


}





// 获取pcf8591声音传感器模块的数值，3.3V最大值是169

byte read_sound(){

 Wire.beginTransmission(PCF8591);
 Wire.write(0x04);
 Wire.endTransmission();
 Wire.requestFrom(PCF8591, 5);

 adcvalue0=Wire.read(); 
 adcvalue0=Wire.read();
 adcvalue1=Wire.read();
 adcvalue2=Wire.read();
 adcvalue3=Wire.read();
 /* 
 Serial.print(adcvalue0);
 Serial.print(" ,");
 Serial.print(adcvalue1); 
 Serial.print(" ,");
 Serial.print(adcvalue2); 
 Serial.print(" ,");
 Serial.print(adcvalue3); 
 Serial.println();
 */
 return adcvalue1;

}


// 驱动DRV8830
void DRV8830_Run(uint8_t address_ic, uint8_t address_reg, uint8_t power, uint8_t Direction) {
  uint8_t voltage = (uint8_t)(57 * (power / 100.0) + 6); // 0x3F - 0x06 = 0x39(57 Dec)
  Wire.beginTransmission(address_ic);
  Wire.write(address_reg);
  Wire.write((voltage << 2) + Direction);
  Wire.endTransmission();
}




//使用编码器控制电机
void encoder_motor(int chanl){
    //屏蔽掉下面两行的原因与encoder_LED一致
    //tcaSelect(chanl); 
    //ecoder_value();//获取编码器的值，执行完该函数之后会产生3个值 按键值get_encoder_data[2]，旋转值encoder_data，旋转绝对值abs_data
    tcaSelect(7-chanl);
    if(abs_data==0){
    encoder_data=encoder_data*5;//乘4是为了避免拧的圈数过多
    if(encoder_data>100)encoder_data=100;//限制100以内
    DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, encoder_data, 1);//正转
    delay(5);
    Wire.endTransmission();

    }
    else if(abs_data==1){
      if(encoder_data==0){
        DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);//停止
        delay(5);
        Wire.endTransmission();

      }
      else{
        encoder_data=encoder_data*5;//乘3是为了避免拧的圈数过多
        if(encoder_data>100)encoder_data=100;//限制100以内
        DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, encoder_data, 2);//反转
        delay(5);
        Wire.endTransmission();
      }

      
    } 




}

//使用编码器控制彩色LED

void encoder_LED(int chanl){
    
    //下面备注的两行，是因为在广播蓝牙值的时候已经获取过编码器值了，重复可能有问题，这里暂时屏蔽掉先用着
    //tcaSelect(chanl); 
    //ecoder_value();//获取编码器的值，执行完该函数之后会产生3个值 按键值get_encoder_data[2]，旋转值encoder_data，正负标记位abs_data
    tcaSelect(7-chanl);
    Wire.beginTransmission(0x40);//40是LED的地址

    //因为存在连接多路编码器控制led的情况，如果使用一个变量就会导致一个编码器控制多个led，闪灯等情况，使用多个变量
    //第一个字节判断和产生
    if(chanl==7){

    if((old_data_0==1)&&(get_encoder_data[2]==0)) {encoder_LED_value_0=(encoder_LED_value_0+1)%2;}  // 1为从0变到1
    old_data_0=get_encoder_data[2];
    Wire.write(encoder_LED_value_0);  // 第1个字节encoder_LED_value
    }
    else if(chanl==6){
    if((old_data_1==1)&&(get_encoder_data[2]==0)) {encoder_LED_value_1=(encoder_LED_value_1+1)%2;}  // 1为从0变到1
    old_data_1=get_encoder_data[2];
    Wire.write(encoder_LED_value_1);  // 第1个字节encoder_LED_value
    }
    else if(chanl==5){
    if((old_data_2==1)&&(get_encoder_data[2]==0)) {encoder_LED_value_2=(encoder_LED_value_2+1)%2;}  // 1为从0变到1
    old_data_2=get_encoder_data[2];
    Wire.write(encoder_LED_value_2);  // 第1个字节encoder_LED_value
    }

    else if(chanl==4){
    if((old_data_3==1)&&(get_encoder_data[2]==0)) {encoder_LED_value_3=(encoder_LED_value_3+1)%2;}  // 1为从0变到1
    old_data_3=get_encoder_data[2];
    Wire.write(encoder_LED_value_3);  // 第1个字节encoder_LED_value
    }
    else{}

    if(abs_data==1){ encoder_data = encoder_data*10 ; if(encoder_data>230)encoder_data = 230;}//若为正数，则加亮度，拧的时候亮度10倍速
    if(abs_data==0){ encoder_data = -(encoder_data*4) ; if(encoder_data<-20)encoder_data = -20;}//若为负数，则加减小，拧的时候亮度4倍速
    Wire.write(encoder_data);//第2个字节
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();



}

//使用距离控制电机

void distance_motor(int chanl){

    tcaSelect(chanl); 
    juli=read_distance();
    tcaSelect(7-chanl);
    int fangxiang=0;
   //接BD端口电机正转，在FH端口电机反转
      if(chanl==7||chanl==6){
      fangxiang = 1;
      }
      else if(chanl==5||chanl==4){
      fangxiang  = 2;
      }
      else{}
  //电机正反转
    if(juli>35){
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);
    }


    
    else {juli=map(juli,0,35,100,0);
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, juli, fangxiang);
    }


    
    delay(5);
    Wire.endTransmission();

}

//使用距离控制彩色LED
void distance_LED(int chanl){

    tcaSelect(chanl); 
    juli=read_distance();
    tcaSelect(7-chanl);
    if(juli>20){juli=-20;}
    else {juli=map(juli,0,20,235,0);}
    
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(0);  // 第1个字节
    Wire.write(juli); /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();

}


//使用声音控制电机
uint8_t sound_value_flag=0;
unsigned long sound_value_time=0;
void sound_motor(int chanl){
    
    tcaSelect(chanl); 
    delay(15);//插入的瞬间值较大，稳定一会
    sound_value=read_sound();
    //Serial.println(sound_value);
    if(sound_value>50&&sound_value!=255){//拔掉声音模块的瞬间值为255
     sound_value_flag=1;
     sound_value_time=millis();
      
    }
    
    if(millis()-sound_value_time>500){sound_value_flag=0;}

    if(sound_value_flag==1)//标志位等于0时转动
    {
      tcaSelect(7-chanl);
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 100, 1);
      delay(5);
      Wire.endTransmission();
      
    }
    if(sound_value_flag==0)//标志位等于0时转动
    {
      tcaSelect(7-chanl);
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);
      delay(5);
      Wire.endTransmission();
      
    }



}


//使用声音控制彩色LED
void sound_LED(int chanl){

    tcaSelect(chanl); 
    sound_value=read_sound();
    tcaSelect(7-chanl);
    if(sound_value<=50){sound_value=-20;}//死区
    else if((sound_value>50)&&(sound_value<170)){
      sound_value=map(sound_value,4,170,-20,234);
      sound_led_flag = 0 ;
      old_time=millis();

      }
    
     duration_time = millis()-old_time;
     if(duration_time>4000){
       sound_led_flag = 1 ;

     }



    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(sound_led_flag);  // 第1个字节
    Wire.write(235); /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();

}



//使用mpu控制电机
void mpu_motor(int chanl){

    tcaSelect(chanl); 
    mpu_read();
    tcaSelect(7-chanl);

    if(AcY==127){
      AcY=0;
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, AcY, 0);
    
    }
    else if(AcY<127){
      AcY=map(AcY,0,126,100,0);  
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, AcY, 1);
    

    }
    else if(AcY>127){
      AcY=map(AcY,128,255,0,100);
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, AcY, 2);
    
    
    }
    else{}
    delay(5);
    Wire.endTransmission();


}


//使用mpu控制LED
void mpu_LED(int chanl){

    tcaSelect(chanl); 
    mpu_read();
    tcaSelect(7-chanl);
    if(AcY==127){AcY=-20;}
    else if(AcY<127){AcY=map(AcY,0,126,235,-19);  }
    else if(AcY>127){AcY=map(AcY,128,255,-19,235);}
    else{}
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(0);  // 第1个字节
    Wire.write(AcY); //AcY的值0-255，中值为127，有死区 /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();

    delay(5);
    

}



//使用apds9960控制LED
void apds9960_LED(int chanl){

    tcaSelect(chanl); 
    apds9960_value();//执行完此函数之后apds9960_direction变量将变为1-4
    tcaSelect(7-chanl);
    if(apds9960_direction==1){
      apds9960_flag=0;
    }
    else if(apds9960_direction==2){
     apds9960_flag=1;
    }
    else if(apds9960_direction==3){
     apds9960_flag=0;
    }
    else if(apds9960_direction==4){
     apds9960_flag=1;
    }
    else{
     apds9960_flag=1;
    }
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(apds9960_flag);  // 第1个字节
    Wire.write(235); /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();


}



//使用apds9960控制电机
//这里定义了两个变量，手势识别后会数字一直会在最后一个数不变，刷新时电机需要停止一下
//为了解决左右不能连续挥动控制左右瞬间切换的问题
uint8_t  old_apds9960_direction,new_apds9960_direction;
void apds9960_motor(int chanl){
    
   
    tcaSelect(chanl); 
    apds9960_value();//执行完此函数之后apds9960_direction变量将变为1-4
    new_apds9960_direction = apds9960_direction; //刷新方向值
    //Serial.println(apds9960_direction);
    delay(2);
    tcaSelect(7-chanl);
    if(apds9960_direction==1){
        if(new_apds9960_direction==old_apds9960_direction){}//如果方向值没有变化，什么也不做
        else{//如果方向变化了，则先停一下电机
          DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);
          delay(2);
          Wire.endTransmission();
        }
          apds9960_flag=1;  //控制转向
          apds9960_motor_flag=100;  //控制速度
    }
    else if(apds9960_direction==2){

        if(new_apds9960_direction==old_apds9960_direction){}
        else{
          DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);
          delay(2);
          Wire.endTransmission();
        }

        apds9960_flag=2;
        apds9960_motor_flag=100;

    }
    else if(apds9960_direction==3){
     apds9960_flag=0;
     apds9960_motor_flag=0;
    }
    else if(apds9960_direction==4){
     apds9960_flag=0;
     apds9960_motor_flag=0;
    }
    else{
     apds9960_flag=0;
     apds9960_motor_flag=0;
    }
    old_apds9960_direction = new_apds9960_direction;

    DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, apds9960_motor_flag, apds9960_flag);
    delay(5);
    Wire.endTransmission();

}



//下面是两个遥控器ble连接之后的逻辑控制函数
// 28-4*ch代表ble传来的传感器数组对应的位置
void ble_encoder_motor(int ble_chanl){

    abs_data=sensor_value_16byte[4*ble_chanl-16+3];    //获取编码器正负号标记位
    encoder_data=sensor_value_16byte[4*ble_chanl-16+2];//获取编码器旋转值

    tcaSelect(7-ble_chanl);
    if(abs_data==0){
    encoder_data=encoder_data*5;//乘3是为了避免拧的圈数过多
    if(encoder_data>100)encoder_data=100;//限制100以内
    DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, encoder_data, 1);//正转
    delay(5);
    Wire.endTransmission();

    }
    else if(abs_data==1){
      if(encoder_data==0){
        DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);//停止
        delay(5);
        Wire.endTransmission();

      }
      else{
        encoder_data=encoder_data*5;//乘3是为了避免拧的圈数过多
        if(encoder_data>100)encoder_data=100;//限制100以内
        DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, encoder_data, 2);//反转
        delay(5);
        Wire.endTransmission();
      }

      
    } 



}

void  ble_encoder_LED(int ble_chanl){
    
    abs_data=sensor_value_16byte[4*ble_chanl-16+3];    //获取编码器正负号标记位
    encoder_data=sensor_value_16byte[4*ble_chanl-16+2];//获取编码器旋转值
    get_encoder_data[2]=sensor_value_16byte[4*ble_chanl-16+1];//获取蓝牙传过来的按键值

    tcaSelect(7-ble_chanl);
    Wire.beginTransmission(0x40);//40是LED的地址
    //第一个字节判断和产生

    if(ble_chanl==7){
    /*下面是为了解决编码器控制led灯错乱的问题，使用4个变量4路*/
    if((old_data_0==1)&&(get_encoder_data[2]==0)) {encoder_LED_value_0=(encoder_LED_value_0+1)%2;}  // 1为从0变到1
    old_data_0=get_encoder_data[2];
    Wire.write(encoder_LED_value_0);  // 第1个字节encoder_LED_value
    }
    else if(ble_chanl==6){
    if((old_data_1==1)&&(get_encoder_data[2]==0)) {encoder_LED_value_1=(encoder_LED_value_1+1)%2;}  // 1为从0变到1
    old_data_1=get_encoder_data[2];
    Wire.write(encoder_LED_value_1);  // 第1个字节encoder_LED_value
    }
    else if(ble_chanl==5){
    if((old_data_2==1)&&(get_encoder_data[2]==0)) {encoder_LED_value_2=(encoder_LED_value_2+1)%2;}  // 1为从0变到1
    old_data_2=get_encoder_data[2];
    Wire.write(encoder_LED_value_2);  // 第1个字节encoder_LED_value
    }

    else if(ble_chanl==4){
    if((old_data_3==1)&&(get_encoder_data[2]==0)) {encoder_LED_value_3=(encoder_LED_value_3+1)%2;}  // 1为从0变到1
    old_data_3=get_encoder_data[2];
    Wire.write(encoder_LED_value_3);  // 第1个字节encoder_LED_value
    }
    else{}
    if(abs_data==1){ encoder_data = encoder_data*8 ; if(encoder_data>230)encoder_data = 230;}//若为正数，则加亮度，拧的时候亮度8倍速
    if(abs_data==0){ encoder_data = -(encoder_data*2) ; if(encoder_data<-20)encoder_data = -20;}//若为负数，则加减小，拧的时候亮度2倍速
    Wire.write(encoder_data);
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();

}

void ble_distance_motor(int ble_chanl){
  

    tcaSelect(ble_chanl); 
    juli=sensor_value_16byte[4*ble_chanl-16+1];
    tcaSelect(7-ble_chanl);
    if(juli>35){
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);
    }
    else {juli=map(juli,0,35,100,0);
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, juli, 1);
    }
    
    delay(5);
    Wire.endTransmission();




}
void ble_distance_LED(int ble_chanl){

    tcaSelect(ble_chanl); 
    juli=sensor_value_16byte[4*ble_chanl-16+1];
    tcaSelect(7-ble_chanl);
    if(juli>20){juli=-20;}
    else {juli=map(juli,0,20,235,0);}
    
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(0);  // 第1个字节
    Wire.write(juli); /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();


}
void ble_sound_motor(int ble_chanl){
  

    tcaSelect(ble_chanl); 
    delay(15);//插入的瞬间值较大，稳定一会
    sound_value=sensor_value_16byte[4*ble_chanl-16+1];
    if(sound_value>50&&sound_value!=255){//拔掉声音模块的瞬间值为255
     sound_value_flag=1;
     sound_value_time=millis();
      
    }
    
    if(millis()-sound_value_time>500){sound_value_flag=0;}

    if(sound_value_flag==1)//标志位等于0时转动
    {
      tcaSelect(7-ble_chanl);
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 100, 1);
      delay(5);
      Wire.endTransmission();
      
    }
    if(sound_value_flag==0)//标志位等于0时转动
    {
      tcaSelect(7-ble_chanl);
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);
      delay(5);
      Wire.endTransmission();
      
    }






}
void ble_sound_LED(int ble_chanl){
  
    tcaSelect(ble_chanl); 
    sound_value=sensor_value_16byte[4*ble_chanl-16+1];
    tcaSelect(7-ble_chanl);
    if(sound_value<=50){sound_value=-20;}//死区
    else if((sound_value>50)&&(sound_value<170)){
      sound_value=map(sound_value,4,170,-20,234);
      sound_led_flag = 0 ;
      old_time=millis();

      }
    
     duration_time = millis()-old_time;
     if(duration_time>4000){
       sound_led_flag = 1 ;

     }



    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(sound_led_flag);  // 第1个字节
    Wire.write(235); /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();


}
void ble_mpu_motor(int ble_chanl){
  
    AcY=sensor_value_16byte[4*ble_chanl-16+2];
    tcaSelect(7-ble_chanl);

    if(AcY==127){
      AcY=0;
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, AcY, 0);
    
    }
    else if(AcY<127){
      AcY=map(AcY,0,126,100,0);  
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, AcY, 1);
    

    }
    else if(AcY>127){
      AcY=map(AcY,128,255,0,100);
      DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, AcY, 2);
    
    
    }
    else{}
    delay(5);
    Wire.endTransmission();




}
void ble_mpu_LED(int ble_chanl){
  
    AcY = sensor_value_16byte[4*ble_chanl-16+2];
    tcaSelect(7-ble_chanl);
    if(AcY==127){AcY=-20;}
    else if(AcY<127){AcY=map(AcY,0,126,235,-19);  }
    else if(AcY>127){AcY=map(AcY,128,255,-19,235);}
    else{}
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(0);  // 第1个字节
    Wire.write(AcY); //AcY的值0-255，中值为127，有死区 /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();

    delay(5);




}
void ble_apds9960_LED(int ble_chanl){

    apds9960_direction = sensor_value_16byte[4*ble_chanl-16+1];
    tcaSelect(7-ble_chanl);
    if(apds9960_direction==1){
      apds9960_flag=0;
    }
    else if(apds9960_direction==2){
     apds9960_flag=1;
    }
    else if(apds9960_direction==3){
     apds9960_flag=0;
    }
    else if(apds9960_direction==4){
     apds9960_flag=1;
    }
    else{
     apds9960_flag=1;
    }
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(apds9960_flag);  // 第1个字节
    Wire.write(235); /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();



}

uint8_t  ble_old_apds9960_direction,ble_new_apds9960_direction;
void ble_apds9960_motor(int ble_chanl){
  
    apds9960_direction = sensor_value_16byte[4*ble_chanl-16+1];//执行完此函数之后apds9960_direction变量将变为1-4
    ble_new_apds9960_direction = apds9960_direction; //刷新方向值
    //Serial.println(apds9960_direction);
    delay(2);
    tcaSelect(7-ble_chanl);
    if(apds9960_direction==1){
        if(ble_new_apds9960_direction==ble_old_apds9960_direction){}//如果方向值没有变化，什么也不做
        else{//如果方向变化了，则先停一下电机
          DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);
          delay(2);
          Wire.endTransmission();
        }
          apds9960_flag=1;  //控制转向
          apds9960_motor_flag=100;  //控制速度
    }
    else if(apds9960_direction==2){

        if(ble_new_apds9960_direction==ble_old_apds9960_direction){}
        else{
          DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, 0, 0);
          delay(2);
          Wire.endTransmission();
        }

        apds9960_flag=2;
        apds9960_motor_flag=100;

    }
    else if(apds9960_direction==3){
     apds9960_flag=0;
     apds9960_motor_flag=0;
    }
    else if(apds9960_direction==4){
     apds9960_flag=0;
     apds9960_motor_flag=0;
    }
    else{
     apds9960_flag=0;
     apds9960_motor_flag=0;
    }
    ble_old_apds9960_direction = ble_new_apds9960_direction;

    DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, apds9960_motor_flag, apds9960_flag);
    delay(5);
    Wire.endTransmission();



}


















//下面是给使用主控按键控制的函数，电机控制,h是通道选择，i是正转或者反转（01），p功率（0-100）
void motor_ctrl(int h,int i,int p){
   //Serial.println("motor_ctrl_ing");
   if(iic_addr[h]==0x67){
     Wire.beginTransmission(0x67);
    tcaSelect(h);
    DRV8830_Run(DRV8830_ADDRESS, CTR_ADDRESS, p, i);//正转
    delay(5);
    Wire.endTransmission();
    }

}

//LED控制 ，形参1：地址，形参2：控制参数0开灯，1关灯
void led_ctrl(int h,int i){
   //Serial.println("led_ctrl_ing");
    if(iic_addr[h]==0x40){
    tcaSelect(h);
    Wire.beginTransmission(0x40);//39是LED的地址
    Wire.write(i);  // 第1个字节
    Wire.write(200); /* 第2个字节，亮度值*/
    Wire.write(0);  // 第3个字节,发1代表直接控制RGB的颜色
    Wire.write(0);     // 第4个字节R
    Wire.write(0);  // 第5个字节G
    Wire.write(0);  // 第6个字节B
    Wire.endTransmission();
   //Serial.print("*********************");
    }


}






//下面函数用来控制环按键的控制，分成了两个，上路和下路

void ring_button_handle_up(){

if(digitalRead(ring_button_1) == 0){//上

  motor_ctrl(0,1,100);//port0电机转动
  motor_ctrl(1,1,100);//port1电机转动
 
  led_ctrl(0,0); //port0开灯
  led_ctrl(1,0); //port1开灯
  //DacAudio.Play(&Button_press);       // 播放按键声
  //Speaker.playMusic(button_sound, 22050); 
  play_sound(button_sound,4687,Volume);


   
  while(digitalRead(ring_button_1) == 0){ }

  b1=!b1;

}
if(digitalRead(ring_button_4) == 0){
  
  motor_ctrl(0,2,100);//port0电机转动
  motor_ctrl(1,2,100);//port1电机转动
 
  led_ctrl(0,0); //port0开灯
  led_ctrl(1,0); //port1开灯

  //DacAudio.Play(&Button_press);       // 播放按键声
  //Speaker.playMusic(button_sound, 22050);  
  play_sound(button_sound,4687,Volume);

  while(digitalRead(ring_button_4) == 0){  }

  b1=!b1;
}

  if(b1){
    motor_ctrl(0,0,100);//port0电机停止
    motor_ctrl(1,0,100);//port1电机停止


  
    led_ctrl(0,1); //port0关灯
    led_ctrl(1,1); //port1关灯

  }









}







//处理环按键按下之后的动作
void ring_button_handle_down(){



if(digitalRead(ring_button_2) == 0){

  motor_ctrl(2,1,100);//port0电机转动
  motor_ctrl(3,1,100);//port1电机转动
 
  led_ctrl(2,0); //port0开灯
  led_ctrl(3,0); //port1开灯
  //DacAudio.Play(&Button_press);       // 播放按键声
 // Speaker.playMusic(button_sound, 22050); 
 play_sound(button_sound,4687,Volume);

  while(digitalRead(ring_button_2) == 0){  }

  b2=!b2;



}

if(digitalRead(ring_button_3) == 0){

  motor_ctrl(2,2,100);//port0电机转动9
  motor_ctrl(3,2,100);//port1电机转动
 
  led_ctrl(2,0); //port0开灯
  led_ctrl(3,0); //port1开灯

  //DacAudio.Play(&Button_press);       // 播放按键声
  //Speaker.playMusic(button_sound, 22050);
  play_sound(button_sound,4687,Volume);
  
  while(digitalRead(ring_button_3) == 0){  }

  b2=!b2; 



}


//对环按键状态进行判断


  if(b2){
    motor_ctrl(2,0,100);//port2电机停止
    motor_ctrl(3,0,100);//port3电机停止
  
    led_ctrl(2,1); //port0关灯
    led_ctrl(3,1); //port1关灯
    
  }




}


/*** ble链接情况下服务器发送端主控按键处理 ***/
//定义一个数组，包含3个字节数据，b3,b4,txValue,代表按键状态

/*

b3:控制1，2路，0停，1正转，2反转
b4:控制3，4路，0停，1正转，2反转

*/



uint8_t ble_button_value[3] = {0,0,0};


void ble_button_handle(){


ble_button_value[0]=b3;//蓝牙遥控用b3，b4,就地控制用b1，b2
ble_button_value[1]=b4;
ble_button_value[2]=txValue;



if(digitalRead(ring_button_1) == 0){//上


  //Speaker.playMusic(button_sound, 22050); 
  play_sound(button_sound,4687,Volume);
 
  if(b3==0)b3=1;//如果是不转，则正转
  else if(b3==1)b3=0;//如果是正转，则停止
  else if(b3==2)b3=0;//如果是反转，则停止


  ble_button_value[0]=b3;
  Notify1_Characteristic->setValue(ble_button_value, 3);
  Notify1_Characteristic->notify();
  delay(6); // 太快了蓝牙栈将会堵塞

  while(digitalRead(ring_button_1) == 0){ }



}
if(digitalRead(ring_button_4) == 0){
  

  //Speaker.playMusic(button_sound, 22050); 
  play_sound(button_sound,4687,Volume); 
 
  if(b3==0)b3=2;//如果是不转，则反转
  else if(b3==1)b3=0;//如果是正转，则停止
  else if(b3==2)b3=0;//如果是反转，则停止

  ble_button_value[0]=b3;
  Notify1_Characteristic->setValue(ble_button_value, 3);
  Notify1_Characteristic->notify();
  delay(6); // 太快了蓝牙栈将会堵塞

  while(digitalRead(ring_button_4) == 0){  }


}

//左右路
if(digitalRead(ring_button_2) == 0){


  //Speaker.playMusic(button_sound, 22050); 
  play_sound(button_sound,4687,Volume);
  
  if(b4==0)b4=1;//如果是不转，则正转
  else if(b4==1)b4=0;//如果是正转，则停止
  else if(b4==2)b4=0;//如果是反转，则停止
  ble_button_value[1]=b4;
  Notify1_Characteristic->setValue(ble_button_value,3);
  Notify1_Characteristic->notify();
  delay(6); // 太快了蓝牙栈将会堵塞

  while(digitalRead(ring_button_2) == 0){  }





}

if(digitalRead(ring_button_3) == 0){


  //Speaker.playMusic(button_sound, 22050); 
  play_sound(button_sound,4687,Volume);
  
  if(b4==0)b4=2;//如果是不转，则反转
  else if(b4==1)b4=0;//如果是正转，则停止
  else if(b4==2)b4=0;//如果是反转，则停止
  ble_button_value[1]=b4;
  Notify1_Characteristic->setValue(ble_button_value, 3);
  Notify1_Characteristic->notify();
  delay(6); // 太快了蓝牙栈将会堵塞

  while(digitalRead(ring_button_3) == 0){  }




}

//下面是蓝牙按键处理部分

//如果蓝牙按键按下了，notify的值变为0
if((digitalRead(bluetooth_button) == 0) & (bluetooth_button_flag==false))
{

        ble_button_value[2]=0;
        Notify1_Characteristic->setValue(ble_button_value,3);
        Notify1_Characteristic->notify();
        delay(6); // 太快了蓝牙栈将会堵塞

        bluetooth_button_flag=true;
}




//检测按键释放，重置标志位，notify的值变为1
if((digitalRead(bluetooth_button)==1)&(bluetooth_button_flag))  
{   
   
        ble_button_value[2]=1;
        Notify1_Characteristic->setValue(ble_button_value, 3);
        Notify1_Characteristic->notify();
        delay(6); // 太快了蓝牙栈将会堵塞
    
        bluetooth_button_flag=false;

}



}



//下面函数用来将字符串类型的mac地址转为uint8_t
void getMac(char *mac_data)
{
    char n[6][2];
        //char mac[20]="80:32:C0:AF:55:AB";
        sscanf(mac_data,"%[^:]:%[^:]:%[^:]:%[^:]:%[^:]:%[^:]",n[0],n[1],n[2],n[3],n[4],n[5]);
    
    int i, j;
    int m[2];
    
    for(i=0;i<6;i++)
    {
        for(j=0;j<2;j++)
        {
            if(n[i][j]=='0')
                m[j]=0;
            else if(n[i][j]=='1')
                m[j]=1;
            else if(n[i][j]=='2')
                m[j]=2;
            else if(n[i][j]=='3')
                m[j]=3;
            else if(n[i][j]=='4')
                m[j]=4;
            else if(n[i][j]=='5')
                m[j]=5;
            else if(n[i][j]=='6')
                m[j]=6;
            else if(n[i][j]=='7')
                m[j]=7;
            else if(n[i][j]=='8')
                m[j]=8;
            else if(n[i][j]=='9')
                m[j]=9;
            else if((n[i][j]=='a') || (n[i][j]=='A'))
                m[j]=10;
            else if((n[i][j]=='b') || (n[i][j]=='B'))
                m[j]=11;
            else if((n[i][j]=='c') || (n[i][j]=='C'))
                m[j]=12;
            else if((n[i][j]=='d') || (n[i][j]=='D'))
                m[j]=13;
            else if((n[i][j]=='e') || (n[i][j]=='E'))
                m[j]=14;
            else if((n[i][j]=='f') || (n[i][j]=='F'))
                m[j]=15;
        }
        mac_data_uint8_t[i] = m[0]<<4 | m[1];
    }
    
    //return *mac_data_uint8_t;
    //printf("0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\n",*mac_data,*(mac_data+1),*(mac_data+2),*(mac_data+3),*(mac_data+4),*(mac_data+5));
}










