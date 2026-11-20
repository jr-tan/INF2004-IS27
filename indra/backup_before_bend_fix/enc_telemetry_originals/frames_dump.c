/*
 *	frames_dump.c: packs one frame of each kind with known values (the
 *	same C code the car runs) and prints them as hex for test_frames.py.
 *	gcc -I../../app_program/subsystem1 frames_dump.c ../../app_program/subsystem1/comms_frames.c -o frames_dump
 */
#include <stdio.h>
#include "comms_frames.h"

static void dump(const char *name, const uint8_t *b, uint32_t n)
{
	printf("%s ", name);
	for (uint32_t i = 0; i < n; i++) printf("%02x", b[i]);
	printf("\n");
}

int main(void)
{
	uint8_t b[16];
	dump("status",   b, fr_status(b, 7, 123456, 0, SYS_AVOID, 0x203, 1));
	dump("fault",    b, fr_fault(b, 70000, 0x088));
	dump("motion",   b, fr_motion(b, 1234, -110, 115, 532, 2, 1));
	dump("line",     b, fr_line(b, 999, 7, -517, 1, 'B', 2, 80, 45, 1800, 1));
	{
		uint8_t sp[5] = { 0, 12, 40, 18, 2 };
		int32_t mms = 0, bend = 0, kp = 0, pv = 0, pv4 = 9;
		uint32_t ok = fr_cmd_tune(sp, 5, &mms, &bend, &kp, &pv);
		uint32_t ok4 = fr_cmd_tune(sp, 4, &mms, &bend, &kp, &pv4);
		printf("speedcmd %d\n", (int)(ok && ok4 && bend == 40 && kp == 1800 && pv == 2 && pv4 == 0 ? mms : -1));
	}
	dump("terrain",  b, fr_terrain(b, 4321, -73, 125, 23, 271, 5));
	dump("obstacle", b, fr_obstacle(b, 65535, 12, -5, 10, 19, 21, 1, 3));
	{
		FR_RES r = { 23, 41, 13, 32, 7, 16, 2, 16, 2, 12, 1, 12, 5, 0, 187, 96 };
		dump("resources", b, fr_resources(b, 3210, &r));
	}
	dump("saturate", b, fr_motion(b, 0, 5000, -5000, 99999, 0, 0));
	return 0;
}
