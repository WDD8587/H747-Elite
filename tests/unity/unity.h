#ifndef UNITY_H
#define UNITY_H
#include <stdint.h>
#include <stdio.h>
void unity_init(void);
void unity_test_start(const char*n);
void unity_test_end(void);
void unity_assert_equal_int(const char*m,int e,int a);
void unity_assert_equal_float(const char*m,float e,float a,float d);
void unity_assert_true(const char*m,int c);
void unity_print_summary(void);
#endif
