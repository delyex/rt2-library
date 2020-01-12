/**
 * @file RailTuner2.h
 * @author zgc (delyex@foxmail.com)
 * @brief RailTuner2 轨道参数计算程序
 * @version 0.1
 * @date 2019-12-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <vector>
#include <string>
#include <fstream>
#include "../Laser/laser.h"
/**
 * @brief 处理综合轨检车文件
 * 
 */

namespace RailTuner2
{
/**
     * @brief 2D 激光扫描仪数量
     * 
     */
#define NUM_OF_LASER2D 4

class BaseTuner
{
public:
    /**
 * @brief 函数返回值
 */
    enum RETURN_VALUE
    {
        OK,
        ERR,
        FINISH_WITH_ERR,
        ERR_MODE,
        ERR_PATH
    };

private:
protected:
    /**
     * @brief 对数据 x 轴进行中心化和标准化
     * 
     * @param ps_Processed 处理后的数据
     * @param m 数据均值（x轴）
     * @param c 数据标准差（x轴）
     * @param ps_Original 处理前的数据
     * @param iLength 数据长度
     */
    void CentralizeAndStandardize(PROFILE_POINT *ps_Processed, double *m, double *c, PROFILE_POINT *ps_Original, int iLength);

    /**
     * @brief 计算多项式拟合曲线系数
     * 
     * @param pdCoeff // 系数向量
     * @param ps_Points // 用来拟合的点集
     * @param iLength // 点集长度
     * @param iOrder // 拟合阶次
     */
    void PolyFit(double *pdCoeff, PROFILE_POINT *ps_Points, int iLength, int iOrder);

    void CalcCurvature(double dCoeff, double dFrom, double dTo);

public:
    BaseTuner(/* args */);
    ~BaseTuner();
};

#pragma pack(push, 1)
/**
* @brief IMU 数据结构，与数据传输格式相同
* 
*/
struct IMU_FRAME
{
    /**
    * @brief 帧头1：0x99
    * 
    */
    unsigned char hdr1; // 0x99
    /**
    * @brief 帧头2：0x66
    * 
    */
    unsigned char hdr2; // 0x66
    /**
    * @brief 数据长度（不包括自身、帧头和校验和）
    * 
    */
    unsigned char ucLen; // 数据长度
    /**
    * @brief IMU 工作时间
    * 
    */
    unsigned int uiImuOpeTime; // IMU 工作时间
    /**
    * @brief UTC
    * 
    */
    unsigned int uiUtc; // UTC
    /**
    * @brief 陀螺脉冲数
    * 
    */
    // TODO: 需要改为补偿数
    union {
        /**
        * @brief 向量形式
        * 
        */
        int all[3];
        /**
        * @brief 分量形式
        * 
        */
        struct
        {
            int x;
            int y;
            int z;
        };
    } iGyro;
    /**
    * @brief 加速度计脉冲数
    * 
    */
    // TODO: 需要改为补偿数
    union {
        /**
        * @brief 向量形式
        * 
        */
        int all[3];
        /**
        * @brief 分量形式
        * 
        */
        struct
        {
            int x;
            int y;
            int z;
        };
    } iAcc;
    /**
     * @brief 陀螺温度，标度因数：16.0，即陀螺温度 = sTempr / 16.0
     * 
     */
    union {
        /**
         * @brief 向量形式
         * 
         */
        short all[3];
        /**
         * @brief 分量形式
         * 
         */
        struct
        {
            short x;
            short y;
            short z;
        };
    } sTempr;
    /**
     * @brief 工作状态字
     * 
     */
    unsigned short usOpeSts;
    /**
     * @brief 算法状态字
     * 
     */
    unsigned short usAlgoSts;
    /**
     * @brief GPS 纬度，标度因数 11930400，即 GPS 纬度 = iLatGps / 11930400
     * 
     */
    int iLatGps; //
    /**
     * @brief GPS 经度，标度因数 11930400，即 GPS 经度 = iLonGps / 11930400
     * 
     */
    int iLonGps; // GPS 经度 = iLonGps / 11930400
    /**
     * @brief GPS 高度，标度因数 131080，即 GPS 高度 = iHgtGps / 131080
     * 
     */
    int iHgtGps; //
    /**
     * @brief 里程计脉冲数
     * 
     */
    int iOdo;
    /**
     * @brief 传感器状态字
     * 
     */
    unsigned short usSensorSts; //? 这是什么？
    /**
     * @brief 北向位移
     * 
     */
    float fDistN;
    /**
     * @brief 天向位移
     * 
     */
    float fDistU;
    /**
     * @brief 东向位移
     * 
     */
    float fDistE;
    /**
     * @brief 北向速度
     * 
     */
    float fVelN;
    /**
     * @brief 天向速度
     * 
     */
    float fVelU;
    /**
     * @brief 东向速度
     * 
     */
    float fVelE;
    /**
     * @brief 横滚
     * 
     */
    float fRoll; //? 单位？
    /**
     * @brief 航向
     * 
     */
    float fHdg; //? 单位？
    /**
     * @brief 俯仰
     * 
     */
    float fPitch; //? 单位？
    /**
     * @brief 检验和，包括数据长度，累加和低 8 位
     * 
     */
    unsigned char ucChkSum;
};
#pragma pack(pop)

/**
 * @brief 输入缓冲区数据结构
 * 
 */
struct INPUT_FRAME
{
    /**
     * @brief IMU 数据帧
     * 
     */
    IMU_FRAME s_Imu;
    /**
     * @brief 2D 激光扫描仪数据帧
     * 
     */
    LASER2D_FRAME_IN s_Laser2dIn[NUM_OF_LASER2D];
};

// 实时输出数据
/**
 * @brief 轨道不平顺性参数
 * 
 */
struct TRACK_IRREGULARITIES
{
    /**
     * @brief 数据长度
     * 
     */
    unsigned int uiLen;
    /**
     * @brief IMU 工作时间
     * 
     */
    unsigned int uiImuOpeTime;
    /**
     * @brief UTC
     * 
     */
    unsigned int uiUtc;
    /**
     * @brief 速度，单位：km/h
     * 
     */
    float fSpd;
    /**
     * @brief  里程，单位：m
     * 
     */
    float fMileage;
    /**
     * @brief 现场标记里程，RFID 读出的信息
     * 
     */
    int iRfid;
    /**
     * @brief 轨枕计数
     * 
     */
    int iSleeperCnt;
    /**
     * @brief 反射板计数
     * 
     */
    int iReflectorCnt;
    /**
     * @brief 正矢计算尺度，指其后四个变量计算所用的弦长
     * 
     */
    float fVersineChord;
    /**
     * @brief 左轨轨向（正矢），1.5-42/70/200m可选，单位：mm
     * 
     */
    float fVersineHorizontalLeft;
    /**
     * @brief 右轨轨向（正矢），1.5-42/70/200m可选，单位：mm
     * 
     */
    float fVersineHorizontalRight;
    /**
     * @brief 左轨高低（正矢），1.5-42/70/200m可选，单位：mm
     * 
     */
    float fVersineVerticalLeft;
    /**
     * @brief 左轨高低（正矢），1.5-42/70/200m可选，单位：mm
     * 
     */
    float fVersineVerticalRight;
    /**
     * @brief 左轨轨向（正矢），10m，单位：mm
     * 
     */
    float fVersineHorizontalLeft10m;
    /**
     * @brief 右轨轨向（正矢），10m，单位：mm
     * 
     */
    float fVersineHorizontalRight10m;
    /**
     * @brief 左轨高低（正矢），10m，单位：mm
     * 
     */
    float fVersineVerticalLeft10m;
    /**
     * @brief 左轨高低（正矢），10m，单位：mm
     * 
     */
    float fVersineVerticalRight10m;
    /**
     * @brief 轨距
     * 
     */
    float fGauge;
    /**
     * @brief 规矩变化率
     * 
     */
    float fGaugeRate;
    /**
     * @brief 超高
     * 
     */
    float fSuperrelevation;
    /**
     * @brief 三角坑
     * 
     */
    float fTwist;
    /**
     * @brief 曲率
     * 
     */
    float fCurvature;
};
/**
* @brief 输出缓冲区数据结构
* 
*/
struct OUTPUT_FRAME
{
    /**
    * @brief 轨道不平顺性
    * 
    */
    TRACK_IRREGULARITIES s_Track;
    /**
    * @brief 校正和计算后的 2D 激光扫描仪数据，包括轨顶点和轨距点
    * 
    */
    LASER2D_FRAME_OUT s_Laser2dOut[NUM_OF_LASER2D];
};

/**
 * @brief 实时处理模块
 * 
 */
class RealTimeTuner : public BaseTuner
{
private:
    /* data */
public:
    RealTimeTuner(/* args */);
    ~RealTimeTuner();
    /**
    * @brief 启动实时计算
    * 
    * @param buf 数据输入缓冲区首地址
    * @return int 返回值
    */
    RETURN_VALUE Dispatch(INPUT_FRAME &buf);
    /**
    * @brief 提示输入了新的数据
    * 
    * @return int 
    */
    int UpdateData(void);
    /**
    * @brief 获取最新的一帧计算结果
    * 
    * @return OUTPUT_BUFFER* 计算结果缓冲区首地址
    */
    OUTPUT_FRAME *GetLastResult(void);
    /**
    * @brief 计算结果读取完毕，释放缓冲区
    * 
    * @return true 释放成功
    * @return false 释放失败
    */
    bool ReleaseOutputBuffer(void);
};

class PostTuner : public BaseTuner
{
private:
    /**
    * @brief 当前目录下的 INS 文件名
    * 
    */
    std::vector<std::string> m_strInsFilenames;

    /**
    * @brief 当前目录下的 2D 激光文件名
    * 
    */
    std::vector<std::string> m_strLaser2DFilenames[NUM_OF_LASER2D];

    /**
    * @brief 当前打开的 INS 数据文件
    * 
    */
    std::ifstream m_fileInInsCur;

    /**
    * @brief 当前打开的 2D 激光数据文件
    * 
    */
    std::ifstream m_fileInLaser2DCur[NUM_OF_LASER2D];
    /**
     * @brief 用来存储保存数据的结构体
     * 
     */
    OUTPUT_FRAME m_s_SaveData;

public:
    PostTuner();
    ~PostTuner();

public:
    /**
    * @brief 打开问文件夹作为工作区
    * 
    * @param strFolderPath  文件夹路径
    * @return int 操作结果，0 - 没有错误
    */
    int OpenFolder(std::string strFolderPath);
};

} // namespace RailTuner2
