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


size_t strlen(const char *s)
{
	int n = 0;

	while (*s++)
		n++;
	return n;
}


static int log_calc(unsigned v, unsigned base)
{
	int n = 0;

	if (!v)
		return 1;
	while (v) {
		v /= base;
		n++;
	}
	return n;
}


static void log_put(char *s, unsigned v, unsigned base)
{
	if (!v) {
		s[-1] = '0';
		return;
	}
	while (v) {
		*--s = "0123456789abcdef"[v % base];
		v /= base;
	}
}


int __vsprintf_chk(char *s, int flag, size_t slen, const char *format,
    va_list ap)
{
//	const char *end = s+slen;
	int n = 0, len;
	int int_val;
	unsigned uint_val;
	const char *str_val;

	while (*format) {
		if (*format == '%') {
			switch (*++format) {
			case '%':
			default:
				if (s)
					*s++ = *format;
				n++;
				break;
			case 'd':
				int_val = va_arg(ap, int);
				len = int_val < 0 ? 1+log_calc(-int_val, 10) :
				    log_calc(int_val, 10);
				n += len;
				if (!s)
					break;
				if (int_val < 0) {
					*s = '-';
					log_put(s+len, -int_val, 10);
				} else {
					log_put(s+len, int_val, 10);
				}
				s += len;
				break;
			case 'p':
				/* fall through. only works on 32 bit arch */
			case 'u':
				uint_val = va_arg(ap, unsigned);
				len = log_calc(uint_val, 10);
				n += len;
				if (!s)
					break;
				s += len;
				log_put(s, uint_val, 10);
				break;
			case 's':
				str_val = va_arg(ap, const char *);
				len = strlen(str_val);
				n += len;
				if (!s)
					break;
				memcpy(s, str_val, len);
				s += len;
				break;
			case 'x':
				uint_val = va_arg(ap, unsigned);
				len = log_calc(uint_val, 16);
				n += len;
				if (!s)
					break;
				s += len;
				log_put(s, uint_val, 16);
				break;
			}
			format++;
			continue;
		}
		if (s)
			*s++ = *format;
		format++;
		n++;
	}
	if (s)
		*s = 0;
	return n;
}
