#include <sys/time.h>
#include <fstream>

namespace TLogger
{
    typedef enum{
        INFO,
        WARN,
        ERROR
    }TLOGTYPE;

    static bool start=true;
    static TLOGTYPE type = INFO;
    static std::ofstream logFile; // Declare an ofstream object for the log file

    // Function to initialize the log file
    void initLogFile(const std::string& filename) {
        logFile.open(filename, std::ofstream::out | std::ios::app);
        if (!logFile.is_open()) {
            std::cerr << "Failed to open log file: " << filename << std::endl;
            exit(1); // Exit the program if the file cannot be opened
        }
    }

    // Function to write log message to a file
    void writeToFile(const std::string& directoryPath, const std::string& filename, const std::string& message) {
        std::ofstream loggingFile(directoryPath + filename, std::ios::app); // Open the file in append mode
        if (loggingFile.is_open()) {
            loggingFile << message << std::endl; // Write the message                                       ! !!!//BUNU WRITETOFILE YAZAN YERLERE KOY. INIT ETMEYİ UNUTMA
            loggingFile.close(); // Close the file
        } else {
            std::cerr << "Unable to open file for writing: " << directoryPath + filename << std::endl;
        }
    }

    

    void TLog()
    {
        logFile << std::endl; // std::cout<<std::endl;
        start = true;
    }

    static std::string prep_header()
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
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

    // Specialization of TLogInfo for a single string argument
    void TLogInfo(const std::string& directoryPath, const std::string& filename, const std::string& val) {
        std::string message = prep_header() + val; // Prepare the log message
        writeToFile(directoryPath, filename, message); // Write the message to the file
    }

    // Variadic template version of TLogInfo
    template<typename T, typename... Args>
    void TLogInfo(const std::string& directoryPath, const std::string& filename, const T& val, Args&&... args) {
        std::string message = prep_header() + std::to_string(val); // Prepare the log message
        writeToFile(directoryPath, filename, message); // Write the message to the file
        TLogInfo(directoryPath, filename, args...); // Continue with the rest of the arguments if any
    }

    /*
    void TLogInfo(const T & val, Args&&... args)
    {
        type=INFO;
        TLog(val,args...);
    }
    */
    

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

