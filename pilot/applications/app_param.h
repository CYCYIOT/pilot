#ifndef _APP_PARAM_H_
#define _APP_PARAM_H_

#include "lib_math.h"
#include "param.h"

#define PARAM_NAME_LENGTH 40

void param_init();
void param_load(void);
void param_save(void);
void param_list(void);
bool param_get(char* name,float * val);
bool param_set(char* name,float val);
bool param_set_var(char* name,float * var);
bool param_define(char* name,float val);
bool param_get_bynum(uint16_t num,char name[PARAM_NAME_LENGTH],float *val);
uint16_t param_get_total();

#endif

