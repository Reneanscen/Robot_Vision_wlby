#ifndef _MESSAGE_REPORT_H_
#define _MESSAGE_REPORT_H_

/*******************************************************************************
* @brief 错误和警告消息打印
* @author jazzey
* @date 2023-8-8
********************************************************************************/

#include <string>
#include "logger/my_logger.h"

//错误消息打印： MESSAGE_REPORT_ERROR(错误号，错误信息)
#define MESSAGE_REPORT_ERROR(err,msg) MessageReport::reportError(err,msg)

//警告消息打印： MESSAGE_REPORT_WARNING(警告号，警告信息)
#define MESSAGE_REPORT_WARNING(warning,msg) MessageReport::reportWarning(warning,msg)

#define MESSAGE_REPORT_ASSERT(err,msg) MessageReport::reportErrorAndAssert(err,msg)

namespace wlby{

class MessageReport {
  public:
		static void reportError(int error_num, const std::string &err_msg);
        static void reportWarning(int warning_num, const std::string &warning_msg);
        static void reportErrorAndAssert(int error_num, const std::string &err_msg);

};

}

#endif