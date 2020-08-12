#include <malloc.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include "app_param_calib.h"
#include "app_debug.h"

#include "lib_math.h"
#include "param.h"
#include "disk_flush.h"

#define DEBUG_ID DEBUG_ID_PARAM

#define PARAM_CALIB_FILE_PATH	"/netPrivate/param_calib"

int param_calib_file_fd = -1;

#define ITEM_MAX 30

typedef struct param_info_s{
	struct param_info_s *next;
	char name[ITEM_MAX];
	float val;
	float * val_p;
}param_info_s;

param_info_s param_calib_head = {
	.next = NULL,
	.name = "count",
	.val = 0,
	.val_p = NULL,
};

struct param_info_s *param_calib_end = NULL;

static float param_calib_parse_number(const char *num)
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

static const char * param_calib_text_skip(const char *in)
{
	while (in && *in && (unsigned char)*in<=32) 
		in++; 
	
	return in;
}

static int param_calib_text_find(const char * text , char * dst , int len)
{
	int count;

	text = param_calib_text_skip(text);
	for(count = 0 ; count < len ; count++){
		if(text[count] <= 32){
			memcpy(dst,text,count);
			return count;
		}
	}

	return 0;
}

struct param_info_s* param_calib_list_check(char* name)
{
	struct param_info_s *ptr = NULL;
	uint8_t len;
	
	ptr = &param_calib_head;
	len = strlen(name);
	if(len > ITEM_MAX){
		len = ITEM_MAX;
	}
	
	while((strncmp(ptr->name,name,len)) != 0 || (strlen(name) != strlen(ptr->name))){
		ptr = ptr->next;
		if(ptr == NULL){
			return NULL;
		}		
	}

	return ptr;
}

bool param_calib_define(char* name,float val,float * val_p)
{
	struct param_info_s *ptr = NULL;
	
	ptr = param_calib_list_check(name);
	if(ptr == NULL){
		DEBUG(DEBUG_ID,"create:%s(%f)",name,val);
		ptr = malloc(sizeof(struct param_info_s));
		memset(ptr,0,sizeof(struct param_info_s));
		if(ptr == NULL){
			ERR(DEBUG_ID,"malloc failed");
			return false;
		}else{
			strncpy(ptr->name,name,ITEM_MAX);
			ptr->next	= param_calib_end->next;
			ptr->val	= val;
			ptr->val_p	= val_p;
			if(val_p != NULL){
				*val_p = ptr->val;
			}
			param_calib_end->next = ptr;
			param_calib_end = ptr;
			param_calib_head.val += 1.0f;
		}
	}else{
		ERR(DEBUG_ID,"redefine:%s(%f)",name,val);
		return false;
	}
	
	return true;
}

static bool param_calib_text_parse(const char * text , int len)
{
	char item_name[ITEM_MAX] = {0};
	char item_val[ITEM_MAX] = {0};
	int text_start = 0;

	text_start = param_calib_text_find(&text[text_start],item_name,ITEM_MAX);
	if(text_start == 0){
		return false;
	}

	text_start = param_calib_text_find(&text[text_start],item_val,ITEM_MAX);
	if(text_start == 0){
		return false;
	}

	//printf("param_text_parse name(%s) val(%s)\r\n",item_name,item_val);
	return param_calib_set(item_name,param_calib_parse_number(item_val));
}

bool param_calib_load_file(int fd)
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
			param_calib_text_parse(&text[text_start],text_end - text_start);
			text_start = text_end + 1;
		}
	}

	free(text);
	
	return true;
}

bool param_calib_save_file(int fd)
{
	struct param_info_s *ptr = NULL;
	ptr = &param_calib_head;
	char * buf;
	
	buf = malloc(ITEM_MAX * 2);

	ptr = ptr->next;
	while(ptr != NULL){
		memset(buf,0,ITEM_MAX * 2);
		snprintf(buf,ITEM_MAX * 2,"%s %4.4f \r\n",ptr->name,ptr->val);
		write(fd,buf,strlen(buf));
		ptr = ptr->next;
	}

	free(buf);
	
	return true;
}

void param_calib_list_debug(void)
{
	struct param_info_s *ptr = NULL;
	uint8_t i = 0;

	ptr = &param_calib_head;
	while(ptr != NULL){
		DEBUG(DEBUG_ID,"%3d  %-20s  %f",i++, ptr->name,ptr->val);
		ptr = ptr->next;
	};
}

void param_calib_list(void)
{
	struct param_info_s *ptr = NULL;
	uint8_t i = 0;

	ptr = &param_calib_head;
	while(ptr != NULL){
		INFO(DEBUG_ID,"%3d  %-20s  %f",i++, ptr->name,ptr->val);
		ptr = ptr->next;
	};
}

void param_calib_load(void)
{
	if(param_calib_file_fd < 0){
		param_calib_file_fd = open(PARAM_CALIB_FILE_PATH,O_RDWR ,0);
	}
	
	if(param_calib_file_fd > 0){
		param_calib_load_file(param_calib_file_fd);
		close(param_calib_file_fd);
		param_calib_file_fd = -1;
		INFO(DEBUG_ID,"param_calib_load done");
	}else{
		ERR(DEBUG_ID,"param_calib_load load failed,can not open file");
		param_calib_save();
	}

	param_calib_list_debug();
}

void param_calib_save(void)
{
	if(param_calib_file_fd < 0){
		param_calib_file_fd = open(PARAM_CALIB_FILE_PATH,O_RDWR|O_CREAT|O_TRUNC,0);
	}

	if(param_calib_file_fd > 0){
		param_calib_save_file(param_calib_file_fd);
		close(param_calib_file_fd);
		data_flush_to_disk();
		param_calib_file_fd = -1;
		INFO(DEBUG_ID,"param_calib_save done");
	}else{
		ERR(DEBUG_ID,"param_calib_save save failed,can not open file");
	}
}

bool param_calib_get(char* name,float *val)
{
	struct param_info_s *ptr = NULL;
		
	ptr = param_calib_list_check(name);
	if(ptr != NULL){
		*val = ptr->val;
		//printf("param_get:%s,%d,(%3.3f,%d)\r\n",ptr->name,ptr->type,ptr->val.f,ptr->val.i);
		return true;
	}else{
		return false;
	}	
}

bool param_calib_set(char* name,float val)
{
	struct param_info_s *ptr = NULL;
		
	ptr = param_calib_list_check(name);
	if(ptr != NULL){
		DEBUG(DEBUG_ID,"param_set:%s(%f)",name,val);
		if(ptr->val_p != NULL){
			*ptr->val_p = val;
		}
		ptr->val = val;
		return true;
	}else{
		return false;
	}
}

void param_calib_init()
{
	INFO(DEBUG_ID,"init");
	
	param_calib_end = &param_calib_head;
}

