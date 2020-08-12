#ifndef _HAL_BK2461_H_
#define	_HAL_BK2461_H_

bool bk2461_open();
bool bk2461_read(float * rc);
bool bk2461_write(float * motor);

#endif
