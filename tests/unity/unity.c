#include "unity.h"
static int total,passed,failed;
void unity_init(void){total=0;passed=0;failed=0;}
void unity_test_start(const char*n){printf("\n[TEST] %s\n",n);}
void unity_test_end(void){}
void unity_assert_equal_int(const char*m,int e,int a){total++;if(e==a){passed++;printf("  PASS: %s (%d)\n",m,e);}else{failed++;printf("  FAIL: %s expected %d got %d\n",m,e,a);}}
void unity_assert_equal_float(const char*m,float e,float a,float d){total++;if((e-a)<d&&(a-e)<d){passed++;printf("  PASS: %s (%.3f)\n",m,e);}else{failed++;printf("  FAIL: %s expected %.3f got %.3f\n",m,e,a);}}
void unity_assert_true(const char*m,int c){total++;if(c){passed++;printf("  PASS: %s\n",m);}else{failed++;printf("  FAIL: %s is false\n",m);}}
void unity_print_summary(void){printf("\n=== %d tests: %d passed, %d failed ===\n",total,passed,failed);}
