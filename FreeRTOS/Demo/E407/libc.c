/*
 * libc.c - "external" variants of builtin functions
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

/*
 * The compiler may need to generate a pointer to a function that would
 * normally be inlined as builtin. Also, builtins that find nothing worth
 * optimizing may (apparently) call the corresponding libc function.
 *
 * For those reasons, we have to provide non-inlined versions of these
 * functions and they must no use builtins themselves.
 */


#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


void *memcpy(void *dest, const void *src, size_t n)
{
	void *ret = dest;

	while (n--)
		*(uint8_t *) dest++ = *(const uint8_t *) src++;
	return ret;
}


void *memset(void *s, int c, size_t n)
{
	void *ret = s;

	while (n--)
		*(int8_t *) s++ = c;
	return ret;
}


int memcmp(const void *s1, const void *s2, size_t n)
{
	int d;

	while (n--) {
		d = *(const uint8_t *) s1 - *(const uint8_t *) s2;
		if (d)
			return d;
		s1++;
		s2++;
	}
	return 0;
}


char *strcat(char *dest, const char *src)
{
	char *ret = dest;

	while (*dest)
		dest++;
	do *dest++ = *src;
	while (*src++);
	return ret;
}


char *strncpy(char *dest, const char *src, size_t n)
{
	char *ret = dest;

	while (n--) {
		*dest++ = *src;
		if (!*src)
			break;
		src++;
	}
	return ret;
}


int __sprintf_chk(char *s, int flag, size_t slen, const char *format, ...)
{
	va_list ap;
	int ret;

	va_start(ap, format);
	ret = __builtin___vsprintf_chk(s, flag, slen, format, ap);
	va_end(ap);
	return ret;
}


/*
 * @@@ __builtin___vsprintf_chk may loop back to __vsprintf_chk. Not sure if
 * this works.
 */

int __vsprintf_chk(char *s, int flag, size_t slen, const char *format,
    va_list ap)
{
	return __builtin___vsprintf_chk(s, flag, slen, format, ap);
}
