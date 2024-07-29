#include "message_report/message_report.h"
#include <cassert>

namespace wlby{

void MessageReport::reportError(int error_num, const std::string &err_msg){ 
    log__save("Error",kLogLevel_Error,kLogTarget_Stdout|kLogTarget_Filesystem, "[ Error: error code: %d , %s ]", error_num , err_msg.c_str());
}

void MessageReport::reportWarning(int warning_num, const std::string &warning_msg){
    log__save("Waring",kLogLevel_Warning,kLogTarget_Stdout|kLogTarget_Filesystem, "[ Waring: warning code: %d , %s ]", warning_num , warning_msg.c_str());
}

void MessageReport::reportErrorAndAssert(int error_num, const std::string &err_msg){
    log__save("Error",kLogLevel_Error,kLogTarget_Stdout|kLogTarget_Filesystem, "[ Error: error code: %d , %s ]", error_num , err_msg.c_str());
    assert(false);
}


}