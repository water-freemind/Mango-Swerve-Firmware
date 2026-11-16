/* Newlib support for the GCC build. Hardware UART traffic uses the HAL APIs. */
#include <errno.h>
#include <sys/stat.h>

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _read(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}

int _write(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}

int _lseek(int file, int offset, int direction)
{
    (void)file;
    (void)offset;
    (void)direction;
    errno = ESPIPE;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    (void)file;
    status->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int signal)
{
    (void)pid;
    (void)signal;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    while (1) {}
}
