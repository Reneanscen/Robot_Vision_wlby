#ifndef _WLBY_CHECK_H_
#define _WLBY_CHECK_H_

/*******************************************************************************
* @brief CHECK操作函数
* @author jazzey
* @date 2023-8-8
********************************************************************************/

#include <cassert>
#include "logger/my_logger.h"

namespace wlby {

#define CHECK_EQ(data1, data2)  if(data1 != data2){ \
 log__save("CHECK_Error", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem, \
  "CHECK_EQ Failed: %s:%d - %s", __FILE__,__LINE__,__PRETTY_FUNCTION__); assert(false);}

#define CHECK_GT(data1, data2)  if(data1 <= data2){ \
 log__save("CHECK_Error", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem, \
  "CHECK_GT Failed: %s:%d - %s", __FILE__,__LINE__,__PRETTY_FUNCTION__); assert(false);}

#define CHECK_GE(data1, data2)  if(data1 < data2){ \
 log__save("CHECK_Error", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem, \
  "CHECK_GE Failed: %s:%d - %s", __FILE__,__LINE__,__PRETTY_FUNCTION__); assert(false);}

#define CHECK_LT(data1, data2)  if(data1 >= data2){ \
 log__save("CHECK_Error", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem, \
  "CHECK_LT Failed: %s:%d - %s", __FILE__,__LINE__,__PRETTY_FUNCTION__); assert(false);}

#define CHECK_LE(data1, data2)  if(data1 > data2){ \
 log__save("CHECK_Error", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem, \
  "CHECK_LE Failed: %s:%d - %s", __FILE__,__LINE__,__PRETTY_FUNCTION__); assert(false);}

#define CHECK_NOT_NULL(ptr) if(ptr == nullptr || ptr == NULL || ptr == 0){ \
 log__save("CHECK_Error", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem, \
  "NOT_NULL Failed: %s:%d - %s", __FILE__,__LINE__,__PRETTY_FUNCTION__); assert(false); }

#define CHECK_TRUE(data) if(!data){ \
  log__save("CHECK_Error", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem, \
  "CHECK_TRUE Failed: %s:%d - %s", __FILE__,__LINE__,__PRETTY_FUNCTION__); assert(false); }


/* 判断浮点数 @x 是否为 0 */
#define FLOAT_IS_ZERO(x) (((x) < 0.000001) && ((x) > -0.000001))
/* 判断两个浮点数是否相等 */
#define TWO_FLOAT_EQUAL(n, m) ((fabs((n)-(m))) <= 0.000001 )

}

#endif