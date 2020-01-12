/**
 * @brief 每一包 2D 激光扫描仪数据点集的头部信息
 * 
 */
struct PROFILE_POINT_SET_HEADER
{
    /**
     * @brief 时间戳
     * 
     */
    unsigned long long ullTimestamp;
    /**
     * @brief 数据帧号
     * 
     */
    unsigned long long ullFrameIndex;
    /**
     * @brief 有效点数量
     * 
     */
    unsigned int uiValidPointCount;
    /**
     * @brief 存储有效点的缓冲区首地址
     * 
     */
};

/**
 * @brief 激光扫描仪采样点
 * 
 */
struct PROFILE_POINT
{
    /**
     * @brief x 方向测距值 x-coordinate in engineering units (mm)
     * 
     */
    double x;
    /**
     * @brief z 方向测距值 z-coordinate in engineering units (mm)
     * 
     */
    double z;
};
/**
 * @brief 2D 激光扫描仪输入帧格式
 * 
 */
struct LASER2D_FRAME_IN
{
    PROFILE_POINT_SET_HEADER s_Hdr;
    PROFILE_POINT *ps_Buffer;
};

/**
 * @brief 2D 激光扫描仪输出帧格式
 * 
 */
struct LASER2D_FRAME_OUT
{
    PROFILE_POINT_SET_HEADER s_Hdr;
    /**
    * @brief 轨顶点号
    * 
    */
    unsigned int uiTopIndex;
    /**
    * @brief 轨距点号
    * 
    */
    unsigned int uiGaugeIndex;
    /**
    * @brief 存储有效点的缓冲区
    * 
    */
    PROFILE_POINT *ps_ProfileBuffer;
};