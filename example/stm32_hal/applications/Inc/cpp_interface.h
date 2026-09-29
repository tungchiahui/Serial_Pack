#ifndef CPP_INTERFACE_H
#define CPP_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

// 0：裸机；1：FreeRTOS。保留现有应用入口行为。
#define isRTOS 1

void cpp_main(void);

#ifdef __cplusplus
}
#endif

#endif
