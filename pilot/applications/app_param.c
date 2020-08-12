#include <malloc.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/stat.h>
#include <stdlib.h>

#include "app_param.h"
#include "app_debug.h"

#include "disk_flush.h"

#define DEBUG_ID DEBUG_ID_PARAM

#define PARAM_FILE_PATH			"/netPrivate/param"

int param_file_fd = -1;

typedef struct param_info_s{
	struct param_info_s *next;
	char name[PARAM_NAME_LENGTH];
	float val;
	float * var;
}param_info_s;

param_info_s param_head = {
	.next = NULL,
	.name = "count",
	.val = 0,
	.var = NULL,
};

struct param_info_s *end = NULL;

static float param_parse_number(const char *num)
{
	float n=0,sign=1,scale=0;int subscale=0,signsubscale=1;

	if (*num=='-') sign=-1,num++;	/* Has sign? */
	if (*num=='0') num++;			/* is zero */
	if (*num>='1' && *num<='9')	do	n=(n*10.0)+(*num++ -'0');	while (*num>='0' && *num<='9');	/* Number? */
	if (*num=='.' && num[1]>='0' && num[1]<='9') {num++;		do	n=(n*10.0)+(*num++ -'0'),scale--; while (*num>='0' && *num<='9');}	/* Fractional part? */
	if (*num=='e' || *num=='E')		/* Exponent? */
	{	
		num++;if (*num=='+') num++;	else if (*num=='-') signsubscale=-1,num++;		/* With sign? */
		while (*num>='0' && *num<='9') subscale=(subscale*10)+(*num++ - '0');	/* Number? */
	}

	n=sign*n*pow(10.0,(scale+subscale*signsubscale));	/* number = +/- number.fraction * 10^+/- exponent */
	
	return n;
}

static const char * param_text_skip(const char *in)
{
	while (in && *in && (unsigned char)*in<=32) 
		in++; 
	
	return in;
}

static int param_text_find(const char * text , char * dst , int len)
{
	int count;

	text = param_text_skip(text);
	for(count = 0 ; count < len ; count++){
		if(text[count] <= 32){
			memcpy(dst,text,count);
			return count;
		}
	}

	return 0;
}

struct param_info_s* param_list_check(char* name)
{
	struct param_info_s *ptr = NULL;
	uint8_t len;
	
	ptr = &param_head;
	len = strlen(name);
	if(len > PARAM_NAME_LENGTH){
		len = PARAM_NAME_LENGTH;
	}
	
	while((strncmp(ptr->name,name,len)) != 0 || (strlen(name) != strlen(ptr->name))){
		ptr = ptr->next;
		if(ptr == NULL){
			return NULL;
		}		
	}

	return ptr;
}

struct param_info_s* param_list_check_bynum(uint16_t num)
{
	struct param_info_s *ptr = NULL;
	
	ptr = &param_head;
	if(num > (uint16_t)param_head.val){
		return NULL;
	}
	
	while(num > 0){
		num--;
		ptr = ptr->next;
		if(ptr == NULL){
			return NULL;
		}		
	}

	return ptr;
}

bool param_define(char* name,float val)
{
	struct param_info_s *ptr = NULL;
	
	ptr = param_list_check(name);
	if(ptr == NULL){
		DEBUG(DEBUG_ID,"create:%s(%f)",name,val);
		ptr = malloc(sizeof(struct param_info_s));
		memset(ptr,0,sizeof(struct param_info_s));
		if(ptr == NULL){
			ERR(DEBUG_ID,"malloc failed");
			return false;
		}else{
			strncpy(ptr->name,name,PARAM_NAME_LENGTH);
			ptr->next	= end->next;
			ptr->val	= val;
			end->next = ptr;
			end = ptr;
			param_head.val += 1.0f;
		}
	}else{
		ERR(DEBUG_ID,"redefine:%s(%f)",name,val);
		return false;
	}
	
	return true;
}

static bool param_text_parse(const char * text , int len)
{
	char item_name[PARAM_NAME_LENGTH] = {0};
	char item_val[PARAM_NAME_LENGTH] = {0};
	int text_start = 0;

	text_start = param_text_find(&text[text_start],item_name,PARAM_NAME_LENGTH);
	if(text_start == 0){
		return false;
	}

	text_start = param_text_find(&text[text_start],item_val,PARAM_NAME_LENGTH);
	if(text_start == 0){
		return false;
	}

	//printf("param_text_parse name(%s) val(%s)\r\n",item_name,item_val);
	return param_set(item_name,param_parse_number(item_val));
}

bool param_load_file(int fd)
{
	int text_start = 0;
	int text_end = 0;
	char * text;
	struct stat fileStat;  

	if(fstat(fd, &fileStat) == -1){
		ERR(DEBUG_ID,"fstat failed");
		return false;	
	}  

	text = (char*)malloc(fileStat.st_size+1);
	read(fd,text,fileStat.st_size);
	text[fileStat.st_size]='\0';
	
	for(text_end = 0 ; text_end < fileStat.st_size ; text_end++){
		if(text[text_end] == '\n'){
			param_text_parse(&text[text_start],text_end - text_start);
			text_start = text_end + 1;
		}
	}

	free(text);

	return true;
}

bool param_save_file(int fd)
{
	struct param_info_s *ptr = NULL;
	ptr = &param_head;
	char * buf;
	
	buf = malloc(PARAM_NAME_LENGTH * 2);

	ptr = ptr->next;
	while(ptr != NULL){
		memset(buf,0,PARAM_NAME_LENGTH * 2);
		snprintf(buf,PARAM_NAME_LENGTH * 2,"%s %4.4f \r\n",ptr->name,ptr->val);
		write(fd,buf,strlen(buf));
		ptr = ptr->next;
	}

	free(buf);
	
	return true;
}

void param_list_debug(void)
{
	struct param_info_s *ptr = NULL;
	uint8_t i = 0;

	ptr = &param_head;
	while(ptr != NULL){
		DEBUG(DEBUG_ID,"%3d  %-20s  %f",i++, ptr->name,ptr->val);
		ptr = ptr->next;
	};
}

void param_list(void)
{
	struct param_info_s *ptr = NULL;
	uint8_t i = 0;

	ptr = &param_head;
	while(ptr != NULL){
		INFO(DEBUG_ID,"%3d  %-20s  %f",i++, ptr->name,ptr->val);
		ptr = ptr->next;
	};
}

void param_load(void)
{
	if(param_file_fd < 0){
		param_file_fd = open(PARAM_FILE_PATH,O_RDWR ,0);
	}
	
	if(param_file_fd > 0){
		param_load_file(param_file_fd);
		close(param_file_fd);
		param_file_fd = -1;
		INFO(DEBUG_ID,"param_load done");
	}else{
		ERR(DEBUG_ID,"param_load load failed,can not open file");
		param_save();
	}

	param_list_debug();
}

void param_save(void)
{
	if(param_file_fd < 0){
		param_file_fd = open(PARAM_FILE_PATH,O_RDWR|O_CREAT|O_TRUNC,0);
	}

	if(param_file_fd > 0){
		param_save_file(param_file_fd);
		close(param_file_fd);
		data_flush_to_disk();
		param_file_fd = -1;
		INFO(DEBUG_ID,"param_save done");
	}else{
		ERR(DEBUG_ID,"param_save save failed,can not open file");
	}
}

uint16_t param_get_total()
{
	return (uint16_t)param_head.val;
}

bool param_get_bynum(uint16_t num,char name[PARAM_NAME_LENGTH],float *val)
{
	struct param_info_s *ptr = NULL;
		
	ptr = param_list_check_bynum(num);
	if(ptr != NULL && name != NULL){
		strncpy(name,ptr->name,PARAM_NAME_LENGTH);
		*val = ptr->val;
		return true;
	}else{
		return false;
	}
}

bool param_get(char* name,float *val)
{
	struct param_info_s *ptr = NULL;
		
	ptr = param_list_check(name);
	if(ptr != NULL){
		*val = ptr->val;
		return true;
	}else{
		return false;
	}	
}

bool param_set(char* name,float val)
{
	struct param_info_s *ptr = NULL;
		
	ptr = param_list_check(name);
	if(ptr != NULL){
		DEBUG(DEBUG_ID,"param_set:%s(%f)",name,val);
		if(ptr->var != NULL){
			*ptr->var = val;
		}
		ptr->val = val;
		return true;
	}else{
		return false;
	}
}

bool param_set_var(char* name,float * var)
{
	struct param_info_s *ptr = NULL;
		
	ptr = param_list_check(name);
	if(ptr != NULL){
		DEBUG(DEBUG_ID,"param_set_var:%s",name);
		ptr->var = var;
		if(var != NULL){
			*var = ptr->val;
		}
		return true;
	}else{
		ERR(DEBUG_ID,"param_set_var failed:(%s)",name);
		return false;
	}
}

void param_init()
{
	INFO(DEBUG_ID,"init");
	end = &param_head;
	
	param_define(IMU__ROTATION_NAME		,IMU__ROTATION_DEF);
	param_define(MAG__ROTATION_NAME		,MAG__ROTATION_DEF);
	param_define(FLOW__ROTATION_NAME	,FLOW__ROTATION_DEF);

	param_define(INAV__FLOW__FUSE_POS_FLIP1_NAME		,INAV__FLOW__FUSE_POS_FLIP1_DEF);
	param_define(INAV__FLOW__FUSE_VEL_FLIP1_NAME		,INAV__FLOW__FUSE_VEL_FLIP1_DEF);
	param_define(INAV__FLOW__FUSE_BIAS_FLIP1_NAME		,INAV__FLOW__FUSE_BIAS_FLIP1_DEF);
	param_define(INAV__FLOW__FUSE_POS_FLIP2_NAME		,INAV__FLOW__FUSE_POS_FLIP2_DEF);
	param_define(INAV__FLOW__FUSE_VEL_FLIP2_NAME		,INAV__FLOW__FUSE_VEL_FLIP2_DEF);
	param_define(INAV__FLOW__FUSE_BIAS_FLIP2_NAME		,INAV__FLOW__FUSE_BIAS_FLIP2_DEF);
	param_define(INAV__FLOW__FUSE_POS_FLIP_STEP_NAME	,INAV__FLOW__FUSE_POS_FLIP_STEP_DEF);
	param_define(INAV__FLOW__FUSE_VEL_FLIP_STEP_NAME	,INAV__FLOW__FUSE_VEL_FLIP_STEP_DEF);
	param_define(INAV__FLOW__FUSE_BIAS_FLIP_STEP_NAME	,INAV__FLOW__FUSE_BIAS_FLIP_STEP_DEF);
	param_define(INAV__FLOW__FUSE_POS_TAKEOFF_NAME		,INAV__FLOW__FUSE_POS_TAKEOFF_DEF);
	param_define(INAV__FLOW__FUSE_VEL_TAKEOFF_NAME		,INAV__FLOW__FUSE_VEL_TAKEOFF_DEF);
	param_define(INAV__FLOW__FUSE_BIAS_TAKEOFF_NAME		,INAV__FLOW__FUSE_BIAS_TAKEOFF_DEF);
	param_define(INAV__FLOW__FUSE_POS_NORMAL_NAME		,INAV__FLOW__FUSE_POS_NORMAL_DEF);
	param_define(INAV__FLOW__FUSE_VEL_NORMAL_NAME		,INAV__FLOW__FUSE_VEL_NORMAL_DEF);
	param_define(INAV__FLOW__FUSE_BIAS_NORMAL_NAME		,INAV__FLOW__FUSE_BIAS_NORMAL_DEF);
	param_define(INAV__FLOW__FUSE_POS_ROTATE_NAME		,INAV__FLOW__FUSE_POS_ROTATE_DEF);
	param_define(INAV__FLOW__FUSE_VEL_ROTATE_NAME		,INAV__FLOW__FUSE_VEL_ROTATE_DEF);
	param_define(INAV__FLOW__FUSE_BIAS_ROTATE_NAME		,INAV__FLOW__FUSE_BIAS_ROTATE_DEF);
	param_define(INAV__FLOW__FUSE_POS_ACTION_NAME		,INAV__FLOW__FUSE_POS_ACTION_DEF);
	param_define(INAV__FLOW__FUSE_VEL_ACTION_NAME		,INAV__FLOW__FUSE_VEL_ACTION_DEF);
	param_define(INAV__FLOW__FUSE_BIAS_ACTION_NAME		,INAV__FLOW__FUSE_BIAS_ACTION_DEF);
	param_define(INAV__FLOW__QUALITY_WEIGHT_MIN_NAME,	INAV__FLOW__QUALITY_WEIGHT_MIN_DEF);
	param_define(INAV__FLOW__QUALITY_WEIGHT_SCALE_NAME, INAV__FLOW__QUALITY_WEIGHT_SCALE_DEF);

	param_define(INAV__RF__FUSE_VEL_NORMAL_NAME			,INAV__RF__FUSE_VEL_NORMAL_DEF);
	param_define(INAV__RF__FUSE_BIAS_NORMAL_NAME		,INAV__RF__FUSE_BIAS_NORMAL_DEF);
	param_define(INAV__RF__FUSE_VEL_FLIP_NAME			,INAV__RF__FUSE_VEL_FLIP_DEF);
	param_define(INAV__RF__FUSE_BIAS_FLIP_NAME			,INAV__RF__FUSE_BIAS_FLIP_DEF);
	param_define(INAV__RF__FUSE_VEL_NOISE_NAME			,INAV__RF__FUSE_VEL_NOISE_DEF);
	param_define(INAV__RF__FUSE_BIAS_NOISE_NAME			,INAV__RF__FUSE_BIAS_NOISE_DEF);
	param_define(INAV__RF__VEL_NOISE_NAME				,INAV__RF__VEL_NOISE_DEF);
	
	param_define(INAV__GPS__FUSE_POS_NORMAL_NAME		,INAV__GPS__FUSE_POS_NORMAL_DEF);
	param_define(INAV__GPS__FUSE_VEL_NORMAL_NAME		,INAV__GPS__FUSE_VEL_NORMAL_DEF);
	param_define(INAV__GPS__FUSE_BIAS_NORMAL_NAME		,INAV__GPS__FUSE_BIAS_NORMAL_DEF);

	param_define(INAV__BARO__FUSE_POS_ACTION_NAME		,INAV__BARO__FUSE_POS_ACTION_DEF);
	param_define(INAV__BARO__FUSE_VEL_ACTION_NAME		,INAV__BARO__FUSE_VEL_ACTION_DEF);
	param_define(INAV__BARO__FUSE_BIAS_ACTION_NAME		,INAV__BARO__FUSE_BIAS_ACTION_DEF);
	param_define(INAV__BARO__FUSE_POS_BREAK_NAME		,INAV__BARO__FUSE_POS_BREAK_DEF);
	param_define(INAV__BARO__FUSE_VEL_BREAK_NAME		,INAV__BARO__FUSE_VEL_BREAK_DEF);
	param_define(INAV__BARO__FUSE_BIAS_BREAK_NAME		,INAV__BARO__FUSE_BIAS_BREAK_DEF);		
	param_define(INAV__BARO__FUSE_POS_FLIP_NAME			,INAV__BARO__FUSE_POS_FLIP_DEF);
	param_define(INAV__BARO__FUSE_VEL_FLIP_NAME			,INAV__BARO__FUSE_VEL_FLIP_DEF);
	param_define(INAV__BARO__FUSE_BIAS_FLIP_NAME		,INAV__BARO__FUSE_BIAS_FLIP_DEF);
	param_define(INAV__BARO__FUSE_POS_TAKEOFF_NAME		,INAV__BARO__FUSE_POS_TAKEOFF_DEF);
	param_define(INAV__BARO__FUSE_VEL_TAKEOFF_NAME		,INAV__BARO__FUSE_VEL_TAKEOFF_DEF);
	param_define(INAV__BARO__FUSE_BIAS_TAKEOFF_NAME		,INAV__BARO__FUSE_BIAS_TAKEOFF_DEF);
	param_define(INAV__BARO__FUSE_POS_NORMAL_NAME		,INAV__BARO__FUSE_POS_NORMAL_DEF);
	param_define(INAV__BARO__FUSE_VEL_NORMAL_NAME		,INAV__BARO__FUSE_VEL_NORMAL_DEF);
	param_define(INAV__BARO__FUSE_BIAS_NORMAL_NAME		,INAV__BARO__FUSE_BIAS_NORMAL_DEF);
	param_define(INAV__BARO__FUSE_POS_NOISE_NAME		,INAV__BARO__FUSE_POS_NOISE_DEF);
	param_define(INAV__BARO__FUSE_VEL_NOISE_NAME		,INAV__BARO__FUSE_VEL_NOISE_DEF);
	param_define(INAV__BARO__FUSE_BIAS_NOISE_NAME		,INAV__BARO__FUSE_BIAS_NOISE_DEF);
	param_define(INAV__BARO__FUSE_POS_STEP_NAME			,INAV__BARO__FUSE_POS_STEP_DEF);
	param_define(INAV__BARO__FUSE_VEL_STEP_NAME			,INAV__BARO__FUSE_VEL_STEP_DEF);
	param_define(INAV__BARO__FUSE_BIAS_STEP_NAME		,INAV__BARO__FUSE_BIAS_STEP_DEF);
	param_define(INAV__BARO__VEL_NOISE_NAME				,INAV__BARO__VEL_NOISE_DEF);
	param_define(INAV__BARO__VEL_CHECK_ACC_FILTER_NAME	,INAV__BARO__VEL_CHECK_ACC_FILTER_DEF);
	param_define(INAV__BARO__VEL_CHECK_VAL_NAME			,INAV__BARO__VEL_CHECK_VAL_DEF);
	param_define(INAV__BARO__VEL_CHECK_TIMEOUT_NAME		,INAV__BARO__VEL_CHECK_TIMEOUT_DEF);

	param_define(PID__ATT__ROLL_P_NAME		,PID__ATT__ROLL_P_DEF);
	param_define(PID__ATT__PITCH_P_NAME		,PID__ATT__PITCH_P_DEF);
	param_define(PID__ATT__YAW_P_NAME		,PID__ATT__YAW_P_DEF);

	param_define(PID__POS__Z_P_NAME			,PID__POS__Z_P_DEF);
	param_define(PID__POS__XY_P_NAME		,PID__POS__XY_P_DEF);

	param_define(PID__VEL__XY_P_NAME		,PID__VEL__XY_P_DEF);
	param_define(PID__VEL__XY_I_NAME		,PID__VEL__XY_I_DEF);
	param_define(PID__VEL__XY_D_NAME		,PID__VEL__XY_D_DEF);
	param_define(PID__VEL__XY_D_W_NAME		,PID__VEL__XY_D_W_DEF);
	param_define(PID__VEL__XY_I_MAX_NAME	,PID__VEL__XY_I_MAX_DEF);
	param_define(PID__VEL__X_I_INIT_NAME	,PID__VEL__X_I_INIT_DEF);
	param_define(PID__VEL__Y_I_INIT_NAME	,PID__VEL__Y_I_INIT_DEF);
	param_define(PID__VEL__BREAK_XY_P_NAME	,PID__VEL__BREAK_XY_P_DEF);
	param_define(PID__VEL__BREAK_XY_I_NAME	,PID__VEL__BREAK_XY_I_DEF);
	
	param_define(PID__VEL__Z_P_NAME			,PID__VEL__Z_P_DEF);
	param_define(PID__VEL__Z_I_NAME			,PID__VEL__Z_I_DEF);
	param_define(PID__VEL__Z_D_NAME			,PID__VEL__Z_D_DEF);
	param_define(PID__VEL__Z_D_W_NAME		,PID__VEL__Z_D_W_DEF);
	param_define(PID__VEL__Z_I_MAX_NAME		,PID__VEL__Z_I_MAX_DEF);
	param_define(PID__VEL__Z_I_MIN_NAME		,PID__VEL__Z_I_MIN_DEF);
	param_define(PID__VEL__Z_I_INIT_NAME	,PID__VEL__Z_I_INIT_DEF);

	param_define(PID__RATE__FLIP_ROLL_P_NAME	,PID__RATE__FLIP_ROLL_P_DEF);
	param_define(PID__RATE__FLIP_ROLL_I_NAME	,PID__RATE__FLIP_ROLL_I_DEF);
	param_define(PID__RATE__FLIP_ROLL_D_NAME	,PID__RATE__FLIP_ROLL_D_DEF);
	param_define(PID__RATE__FLIP_PITCH_P_NAME	,PID__RATE__FLIP_PITCH_P_DEF);
	param_define(PID__RATE__FLIP_PITCH_I_NAME	,PID__RATE__FLIP_PITCH_I_DEF);
	param_define(PID__RATE__FLIP_PITCH_D_NAME	,PID__RATE__FLIP_PITCH_D_DEF);
	param_define(PID__RATE__FLIP_YAW_P_NAME		,PID__RATE__FLIP_YAW_P_DEF);
	param_define(PID__RATE__FLIP_YAW_I_NAME		,PID__RATE__FLIP_YAW_I_DEF);
	param_define(PID__RATE__FLIP_YAW_D_NAME		,PID__RATE__FLIP_YAW_D_DEF);

	param_define(PID__RATE__ROLL_P_NAME			,PID__RATE__ROLL_P_DEF);
	param_define(PID__RATE__ROLL_I_NAME			,PID__RATE__ROLL_I_DEF);
	param_define(PID__RATE__ROLL_D_NAME			,PID__RATE__ROLL_D_DEF);
	param_define(PID__RATE__ROLL_D_W_NAME		,PID__RATE__ROLL_D_W_DEF);
	param_define(PID__RATE__ROLL_I_MAX_NAME		,PID__RATE__ROLL_I_MAX_DEF);
	param_define(PID__RATE__ROLL_I_INIT_NAME	,PID__RATE__ROLL_I_INIT_DEF);
	
	param_define(PID__RATE__PITCH_P_NAME		,PID__RATE__PITCH_P_DEF);
	param_define(PID__RATE__PITCH_I_NAME		,PID__RATE__PITCH_I_DEF);
	param_define(PID__RATE__PITCH_D_NAME		,PID__RATE__PITCH_D_DEF);
	param_define(PID__RATE__PITCH_D_W_NAME		,PID__RATE__PITCH_D_W_DEF);
	param_define(PID__RATE__PITCH_I_MAX_NAME	,PID__RATE__PITCH_I_MAX_DEF);
	param_define(PID__RATE__PITCH_I_INIT_NAME	,PID__RATE__PITCH_I_INIT_DEF);
	
	param_define(PID__RATE__YAW_P_NAME			,PID__RATE__YAW_P_DEF);
	param_define(PID__RATE__YAW_I_NAME			,PID__RATE__YAW_I_DEF);
	param_define(PID__RATE__YAW_D_NAME			,PID__RATE__YAW_D_DEF);
	param_define(PID__RATE__YAW_I_MAX_NAME		,PID__RATE__YAW_I_MAX_DEF);
	param_define(PID__RATE__YAW_I_INIT_NAME		,PID__RATE__YAW_I_INIT_DEF);
	
	param_define(ATT__RATE_RP_LOW_FILTER_NAME	,ATT__RATE_RP_LOW_FILTER_DEF);
	param_define(ATT__RATE_YAW_LOW_FILTER_NAME	,ATT__RATE_YAW_LOW_FILTER_DEF);

	param_define(CON__COMM__ATT_RP_GAIN_NAME		,CON__COMM__ATT_RP_GAIN_DEF);
	param_define(CON__COMM__RATE_YAW_GAIN_NAME		,CON__COMM__RATE_YAW_GAIN_DEF);
	param_define(CON__COMM__VEL_Z_GAIN_NAME			,CON__COMM__VEL_Z_GAIN_DEF);
	param_define(CON__COMM__ATT_RP_LIMIT_NAME		,CON__COMM__ATT_RP_LIMIT_DEF);
	param_define(CON__COMM__RATE_YAW_LIMIT_NAME		,CON__COMM__RATE_YAW_LIMIT_DEF);
	param_define(CON__COMM__RATE_RP_LIMIT_NAME		,CON__COMM__RATE_RP_LIMIT_DEF);
	param_define(CON__COMM__RATE_RP_STAB_LIMIT_NAME	,CON__COMM__RATE_RP_STAB_LIMIT_DEF);
	param_define(CON__COMM__VEL_Z_LIMIT_NAME		,CON__COMM__VEL_Z_LIMIT_DEF);
	param_define(CON__COMM__VEL_Z_STAB_LIMIT_NAME	,CON__COMM__VEL_Z_STAB_LIMIT_DEF);
	param_define(CON__COMM__VEL_XY_LIMIT_NAME		,CON__COMM__VEL_XY_LIMIT_DEF);
	param_define(CON__COMM__POS_Z_LIMIT_NAME		,CON__COMM__POS_Z_LIMIT_DEF);
	param_define(CON__COMM__RF_GAIN_NAME			,CON__COMM__RF_GAIN_DEF);
	param_define(CON__COMM__RF_VEL_CHECK_NAME		,CON__COMM__RF_VEL_CHECK_DEF);

	param_define(CON__LAND__VEL_NAME				,CON__LAND__VEL_DEF);
	param_define(CON__LAND__CHECK_ACC_NAME			,CON__LAND__CHECK_ACC_DEF);
	param_define(CON__LAND__CHECK_THR_NAME			,CON__LAND__CHECK_THR_DEF);
	param_define(CON__LAND__CHECK_THR_TIME_NAME		,CON__LAND__CHECK_THR_TIME_DEF);
	
	param_define(CON__ALTHOLD__ATT_RP_LIMIT_NAME	,CON__ALTHOLD__ATT_RP_LIMIT_DEF);
	param_define(CON__ALTHOLD__ATT_RP_GAIN_NAME		,CON__ALTHOLD__ATT_RP_GAIN_DEF);
	param_define(CON__ALTHOLD__RATE_RP_LIMIT_NAME	,CON__ALTHOLD__RATE_RP_LIMIT_DEF);

	param_define(CON__TAKEOFF__SPIN_NAME			,CON__TAKEOFF__SPIN_DEF);
	param_define(CON__TAKEOFF__ALT_NAME				,CON__TAKEOFF__ALT_DEF);
	param_define(CON__TAKEOFF__VEL_LIMIT_NAME		,CON__TAKEOFF__VEL_LIMIT_DEF);
	param_define(CON__TAKEOFF__RC_CHECK_NAME		,CON__TAKEOFF__RC_CHECK_DEF);

	param_define(CON__FLIP__RISE_SPEED_NAME			,CON__FLIP__RISE_SPEED_DEF);
	param_define(CON__FLIP__BURST_ATT_LIMIT_NAME	,CON__FLIP__BURST_ATT_LIMIT_DEF);
	param_define(CON__FLIP__BURST_RATE_LIMIT_NAME	,CON__FLIP__BURST_RATE_LIMIT_DEF);
	param_define(CON__FLIP__BURST_ATT_CHECK_NAME	,CON__FLIP__BURST_ATT_CHECK_DEF);
	param_define(CON__FLIP__BURST_RATE_CHECK_NAME	,CON__FLIP__BURST_RATE_CHECK_DEF);
	param_define(CON__FLIP__BREAK_ATT_NAME			,CON__FLIP__BREAK_ATT_DEF);
	param_define(CON__FLIP__HOLD_ATT_NAME			,CON__FLIP__HOLD_ATT_DEF);
	param_define(CON__FLIP__BREAK_GAIN_NAME			,CON__FLIP__BREAK_GAIN_DEF);
	param_define(CON__FLIP__STAB_GAIN_NAME			,CON__FLIP__STAB_GAIN_DEF);
	param_define(CON__FLIP__YAW_GAIN_NAME			,CON__FLIP__YAW_GAIN_DEF);
	param_define(CON__FLIP__HOLD_TIMEOUT_NAME		,CON__FLIP__HOLD_TIMEOUT_DEF);
	param_define(CON__FLIP__STABLE_TIMEOUT_NAME		,CON__FLIP__STABLE_TIMEOUT_DEF);
	
	param_define(CON__POSHOLD__AIR_RES_BREAK_ANGLE_NAME ,CON__POSHOLD__AIR_RES_BREAK_ANGLE_DEF);
	param_define(CON__POSHOLD__AIR_RES_COFF_NAME		,CON__POSHOLD__AIR_RES_COFF_DEF);

	param_define(FS__RC_TIMEOUT_NAME		,FS__RC_TIMEOUT_DEF);
	param_define(FS__LOW_BATT_NAME			,FS__LOW_BATT_DEF);
	param_define(FS__ATT_LIMIT_NAME			,FS__ATT_LIMIT_DEF);
	param_define(FS__IMU_TEMP_LIMIT_NAME	,FS__IMU_TEMP_LIMIT_DEF);

	param_define(LOG__RUN_NAME				,LOG__RUN_DEF);
	param_define(LOG__RATE_FLOW_NAME		,LOG__RATE_FLOW_DEF);
	param_define(LOG__RATE_SENS_NAME		,LOG__RATE_SENS_DEF);
	param_define(LOG__RATE_ALT_NAME			,LOG__RATE_ALT_DEF);
	param_define(LOG__RATE_ATT_NAME			,LOG__RATE_ATT_DEF);
	param_define(LOG__RATE_AHRS_NAME		,LOG__RATE_AHRS_DEF);
	param_define(LOG__RATE_PIDRR_NAME		,LOG__RATE_PIDRR_DEF);
	param_define(LOG__RATE_PIDRP_NAME		,LOG__RATE_PIDRP_DEF);
	param_define(LOG__RATE_PIDRY_NAME		,LOG__RATE_PIDRY_DEF);
	param_define(LOG__RATE_PIDVX_NAME		,LOG__RATE_PIDVX_DEF);
	param_define(LOG__RATE_PIDVY_NAME		,LOG__RATE_PIDVY_DEF);
	param_define(LOG__RATE_PIDVZ_NAME		,LOG__RATE_PIDVZ_DEF);
	param_define(LOG__RATE_IMU_NAME			,LOG__RATE_IMU_DEF);
	param_define(LOG__RATE_BAT_NAME			,LOG__RATE_BAT_DEF);
	param_define(LOG__RATE_NAV_NAME			,LOG__RATE_NAV_DEF);
	param_define(LOG__RATE_NAV2_NAME		,LOG__RATE_NAV2_DEF);
	param_define(LOG__RATE_NAV3_NAME		,LOG__RATE_NAV3_DEF);	
	param_define(LOG__RATE_NAV4_NAME		,LOG__RATE_NAV4_DEF);
	param_define(LOG__RATE_RCI_NAME			,LOG__RATE_RCI_DEF);
	param_define(LOG__RATE_RCO_NAME			,LOG__RATE_RCO_DEF);
	param_define(LOG__RATE_BARO_NAME		,LOG__RATE_BARO_DEF);
	param_define(LOG__RATE_GPS_NAME			,LOG__RATE_GPS_DEF);
	param_define(LOG__RATE_SYSTEM_NAME		,LOG__RATE_SYSTEM_DEF);
	param_define(LOG__RATE_POSHOLD_NAME		,LOG__RATE_POSHOLD_DEF);
	param_define(LOG__RATE_RF_NAME			,LOG__RATE_RF_DEF);

	param_define(MOTOR__MAP_0_NAME			,MOTOR__MAP_0_DEF);
	param_define(MOTOR__MAP_1_NAME			,MOTOR__MAP_1_DEF);
	param_define(MOTOR__MAP_2_NAME			,MOTOR__MAP_2_DEF);
	param_define(MOTOR__MAP_3_NAME			,MOTOR__MAP_3_DEF);
	param_define(MOTOR__MAP_4_NAME			,MOTOR__MAP_4_DEF);
	param_define(MOTOR__MAP_5_NAME			,MOTOR__MAP_5_DEF);
	param_define(MOTOR__MAP_6_NAME			,MOTOR__MAP_6_DEF);
	param_define(MOTOR__MAP_7_NAME			,MOTOR__MAP_7_DEF);
	param_define(MOTOR__RP_MAX_NAME			,MOTOR__RP_MAX_DEF);
	param_define(MOTOR__YAW_MAX_NAME		,MOTOR__YAW_MAX_DEF);
	param_define(MOTOR__THR_MAX_NAME		,MOTOR__THR_MAX_DEF);
	param_define(MOTOR__OUT_MAX_NAME		,MOTOR__OUT_MAX_DEF);
	param_define(MOTOR__OUT_MIN_NAME		,MOTOR__OUT_MIN_DEF);
	
	param_define(RC__DEADZONE_ROLL_NAME		,RC__DEADZONE_ROLL_DEF);
	param_define(RC__DEADZONE_PITCH_NAME	,RC__DEADZONE_PITCH_DEF);
	param_define(RC__DEADZONE_YAW_NAME		,RC__DEADZONE_YAW_DEF);
	param_define(RC__DEADZONE_THR_NAME		,RC__DEADZONE_THR_DEF);

	param_define(RC__MODE_HIGH_NAME			,RC__MODE_HIGH_DEF);
	param_define(RC__MODE_MEDIUM_NAME		,RC__MODE_MEDIUM_DEF);
	param_define(RC__MODE_LOW_NAME			,RC__MODE_LOW_DEF);

	param_define(FLOW__DELAY_NAME	,FLOW__DELAY_DEF);
	param_define(FLOW__SCALE_NAME	,FLOW__SCALE_DEF);

	param_define(RANGEFINDER__MAX_VAL_NAME			,RANGEFINDER__MAX_VAL_DEF);
	param_define(RANGEFINDER__MIN_VAL_NAME			,RANGEFINDER__MIN_VAL_DEF);
	param_define(RANGEFINDER__ROTATE_CHECK_NAME		,RANGEFINDER__ROTATE_CHECK_DEF);

	param_define(IMU__ACC_XY_LOW_FILTER_NAME		,IMU__ACC_XY_LOW_FILTER_DEF);
	param_define(IMU__ACC_Z_LOW_FILTER_NAME			,IMU__ACC_Z_LOW_FILTER_DEF);

	param_define(IMU__GYRO_SCALE_X_NAME		,IMU__GYRO_SCALE_X_DEF);
	param_define(IMU__GYRO_SCALE_Y_NAME		,IMU__GYRO_SCALE_Y_DEF);
	param_define(IMU__GYRO_SCALE_Z_NAME		,IMU__GYRO_SCALE_Z_DEF);

	param_define(AHRS__RP_ACTION_CHECK_TIMEOUT_NAME	,AHRS__RP_ACTION_CHECK_TIMEOUT_DEF);
	param_define(AHRS__RP_ACTION_CHECK_NAME			,AHRS__RP_ACTION_CHECK_DEF);
	param_define(AHRS__RP_P_ACTION_NAME				,AHRS__RP_P_ACTION_DEF);
	param_define(AHRS__RP_I_ACTION_NAME				,AHRS__RP_I_ACTION_DEF);
	param_define(AHRS__RP_P_ARM_NAME				,AHRS__RP_P_ARM_DEF);
	param_define(AHRS__RP_I_ARM_NAME				,AHRS__RP_I_ARM_DEF);
	param_define(AHRS__YAW_P_ARM_NAME				,AHRS__YAW_P_ARM_DEF);
	param_define(AHRS__YAW_I_ARM_NAME				,AHRS__YAW_I_ARM_DEF);
	param_define(AHRS__RP_P_DISARM_NAME				,AHRS__RP_P_DISARM_DEF);
	param_define(AHRS__RP_I_DISARM_NAME				,AHRS__RP_I_DISARM_DEF);
	param_define(AHRS__YAW_P_DISARM_NAME			,AHRS__YAW_P_DISARM_DEF);
	param_define(AHRS__YAW_I_DISARM_NAME			,AHRS__YAW_I_DISARM_DEF);
}

