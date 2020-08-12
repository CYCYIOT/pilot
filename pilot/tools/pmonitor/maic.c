#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main()
{
	FILE *ptr = NULL;
	char cmd[50] = "ps | grep pilot | grep -v grep | wc -l";
	int status = 0;
	char buf[10];
	int count;

	while(1)
	{
		if((ptr = popen(cmd, "r")) == NULL){
			printf("popen err \r\n");	
			continue;
		}

		memset(buf, 0, sizeof(buf));

		if((fgets(buf, sizeof(buf),ptr)) != NULL){
			count = atoi(buf);
			if(count <= 0){
				usleep(100 * 1000);
				system("chmod 777 /tmp/pilot");
				system("/tmp/pilot &");
				printf("restart pilot \r\n");
			}
		}

		pclose(ptr);

		usleep(1 * 1000 * 1000);
	}
}

