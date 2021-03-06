#include <termios.h>
#include <stdio.h>
#include <sys/select.h>

#ifdef __GNUC__
/*! getch hack

  Don't use in production code!
*/
#ifdef __cplusplus
int _getch(void)
#else
extern "C" int _getch(void)
#endif
{
	struct termios oldt, newt;
	int c;

	tcgetattr(0, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON|ECHO);
	tcsetattr(0, TCSANOW, &newt);
	c = getchar();
	tcsetattr(0, TCSANOW, &oldt);
	return c;
}

/*! kbhit hack

  Don't use in production code!
*/
#ifdef __cplusplus
int _kbhit()
#else
extern "C" int _kbhit()
#endif
{
	struct termios oldt, newt;

	struct timeval tv;
	fd_set rfd;
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	FD_ZERO(&rfd);
	FD_SET(0, &rfd);

	tcgetattr(0, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON|ECHO);
	tcsetattr(0, TCSANOW, &newt);

	if (select(1, &rfd, 0, 0, &tv) == -1)
		return 0;

	tcsetattr(0, TCSANOW, &oldt);

	return (FD_ISSET(0, &rfd) != 0);
}
#endif
