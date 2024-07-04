#include <cstdio>
#include <sbgCom/sbgCom.h>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

	SbgProtocolHandle protocolHandle;

	if (sbgComInit("/dev/sbg", 115200, &protocolHandle) == SBG_NO_ERROR) {
    printf("sbgComInit success\n");
  } else {
    printf("sbgComInit failed\n");
  }

  printf("hello world sbg package\n");
  return 0;
}
