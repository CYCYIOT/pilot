#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int data_flush_to_disk(void)
{
	sync();
	system("blockdev --flushbufs /dev/mtdblock4");

	return 0;
}

