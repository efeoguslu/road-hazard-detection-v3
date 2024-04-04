#include <sys/time.h>

typedef struct 
{
    int32_t speed_data;
    uint32_t rpm;
    int32_t gear;
} st_motor_data;

typedef struct
{
    //celcius values with x100 magnificied
    //value 1765 means 17.65 celcius degrees
    int32_t indoor_temperature;
    int32_t outdoor_temperature;

}st_temperatures_data;

typedef struct 
{
    //psi values with x100 magnificied
    //value 3045 means 30.45 psi
    union 
    {
        uint32_t pressures[4];
        struct 
        {
            uint32_t p_front_left;
            uint32_t p_front_right;
            uint32_t p_rear_left;
            uint32_t p_rear_right;
        }named_;
    }data_field;
    
}st_tire_pressures_data;



namespace TLogger
{
    typedef enum{
        INFO,
        WARN,
        ERROR
    }TLOGTYPE;

    static bool start=true;
    static TLOGTYPE type = INFO;
    void TLog()
    {
        std::cout<<std::endl;
        start = true;
    }

    static std::string prep_header()
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
//        time_t now = time(0);      
        tm* ltm = localtime(&tv.tv_sec);  //can be changed to utc        
        char timeBuf[256];
        strftime(timeBuf,sizeof(timeBuf),"[%Y-%m-%d %H:%M:%S.",ltm);
        std::string header(timeBuf);
        header+=std::to_string(tv.tv_usec)+"]";
        switch (type)
        {
            case INFO:
            {
                header+="[INFO]";
                break;
            }
            case WARN:
            {
                header+="[WARN]";
                break;
            }   
            case ERROR:
            {
                header+="[ERR]";
                break;
            }
            default:
                break;
        } 
        return header;
    }

    template<typename T, typename... Args>
    void TLog(const T & val, Args&&... args)
    {
        if(start)
        {
            std::cout<<prep_header();
            start = false;
        }
        std::cout<<val;
        TLog(args...);
    }

    template<typename T, typename... Args>
    void TLogInfo(const T & val, Args&&... args)
    {
        type=INFO;
        TLog(val,args...);
    }

    template<typename T, typename... Args>
    void TLogWarn(const T & val, Args&&... args)
    {
        type=WARN;
        TLog(val,args...);
    }

    template<typename T, typename... Args>
    void TLogError(const T & val, Args&&... args)
    {
        type=ERROR;
        TLog(val,args...);
    }
}

