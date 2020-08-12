#ifndef _APP_PARAM_CALIB_H_
#define _APP_PARAM_CALIB_H_

#include <stdbool.h>

#include "param.h"

void param_calib_init();
void param_calib_load(void);
void param_calib_save(void);
void param_calib_list(void);
bool param_calib_get(char* name,float * val);
bool param_calib_set(char* name,float val);
bool param_calib_define(char* name,float val,float * val_p);

#endif

