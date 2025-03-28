#ifndef __STRUCT_TYPEDEF_H_
#define __STRUCT_TYPEDEF_H_

//在非C语言环境下要注释掉下面这四行
#ifdef __cplusplus
 extern "C" {
#endif

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;   //如果在Linux64上跑请改为signed long

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;   //如果在Linux64上跑请改为unsigned long
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

//在非C语言环境下要注释掉下面这三行
#ifdef __cplusplus
}
#endif

#endif
