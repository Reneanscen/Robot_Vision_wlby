#ifndef _SLEEP_H_
#define _SLEEP_H_
/*******************************************************************************
* @brief 睡眠工具类
* @author jazzey
* @date 2023-8-11
********************************************************************************/

#include <unistd.h>

namespace wlby{

class SleepUtils{
public:
    static void Sleep(int microseconds){
        usleep(microseconds*1000);
    }
private:
    SleepUtils(){}
};

}






#endif