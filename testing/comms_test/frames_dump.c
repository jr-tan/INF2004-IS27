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
	dump("status",   b, comms_frames_status(b, 7, 123456, 0, SYS_AVOID, 0x203, 1));
	dump("fault",    b, comms_frames_fault(b, 70000, 0x088));
	dump("motion",   b, comms_frames_motion(b, 1234, -110, 115, 532, 2, 3));
	dump("line",     b, comms_frames_line(b, 999, 7, -517, 1, 'B', 2, 80, 45, 1800, 1));
	{
		uint8_t sp[5] = { 0, 12, 40, 18, 2 };
		int32_t mms = 0, bend = 0, kp = 0, pv = 0, pv4 = 9;
		uint32_t ok = comms_frames_cmd_tune(sp, 5, &mms, &bend, &kp, &pv);
		uint32_t ok4 = comms_frames_cmd_tune(sp, 4, &mms, &bend, &kp, &pv4);
		printf("speedcmd %d\n", (int)(ok && ok4 && bend == 40 && kp == 1800 && pv == 2 && pv4 == 0 ? mms : -1));
	}
	dump("terrain",  b, comms_frames_terrain(b, 4321, -73, 125, 23, 271, 5));
	dump("obstacle", b, comms_frames_obstacle(b, 65535, 12, -5, 10, 19, 21, 1, 3));
	{
		fr_res_t r = { 23, 41, 13, 32, 7, 16, 2, 16, 2, 12, 1, 12, 5, 0, 187, 96 };
		dump("resources", b, comms_frames_resources(b, 3210, &r));
	}
	dump("saturate", b, comms_frames_motion(b, 0, 5000, -5000, 99999, 0, 0));
	{
		/* barcode map: A=L B=R C=U D=S Z=S, rest straight */
		uint8_t act[FR_MAP_LETTERS] = { 1, 2, 3, 4 }, back[FR_MAP_LETTERS], bad[FR_MAP_BYTES];
		uint32_t ok, i;
		act[25] = 4;
		dump("map_ack", b, comms_frames_map_ack(b, act, 1));
		ok = comms_frames_map_decode(b, FR_MAP_BYTES, back);		/* first 10 bytes = robot/map */
		for (i = 0; i < FR_MAP_LETTERS; i++) ok = ok && back[i] == act[i];
		ok = ok && !comms_frames_map_decode(b, FR_MAP_ACK_BYTES, back);	/* wrong length */
		for (i = 0; i < FR_MAP_BYTES; i++) bad[i] = b[i];
		bad[0] = (uint8_t)((bad[0] & ~7U) | 5U);		/* A = 5: no such action */
		back[0] = 9;
		ok = ok && !comms_frames_map_decode(bad, FR_MAP_BYTES, back) && back[0] == 9;	/* untouched */
		printf("mapdecode %d\n", (int)ok);
	}
	return 0;
}
