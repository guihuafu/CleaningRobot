/**
 * This is the base library for support IEC call C/C++ algorithms
 * implementation
 *
 * copyright @ Inovance.Ltd
 * author : moubo
 * date : 2025.06.05
 */
#ifndef __LIBINOPLC_H__
#define __LIBINOPLC_H__

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdarg.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

///< __DEBUG__ is used for open debug log
//< #define __DEBUG__
#define __LOGFILE            "/App/Log/inoplc.log"

///< base type define for mapping IEC variables
#define BOOL                 int8_t
#define BYTE                 int8_t
#define INT                  int16_t
#define SINT                 int8_t
#define USINT                uint8_t
#define UINT                 uint16_t
#define DINT                 int32_t
#define LINT                 int64_t
#define UDINT                uint32_t
#define ULINT                uint64_t
#define WORD                 int16_t
#define DWORD                int32_t
#define LWORD                uint64_t
#define REAL                 float
#define LREAL                double
#define POINTER              __ptr_t
//#define STRING             ///< ?
//#define TIME               ///< ?
//#define DATE               ///< ?
//#define BIT                ///< ?

#define __IOBLCOK_DEFINE     struct __attribute__((packed))

///< base log handle, just for debugging
#ifdef __DEBUG__
/**
 * @brief this routine is used for debug or info etc., you can choose LOGLEVEL env
 * for different situation.
 *
 * @param [in] func 	function name of where call the function
 * @param [in] line 	line of the file
 * @param [in] format 	log message format
 * @param [in] ... 	log message
 *
 * @return handle result
 *	@retval -1 = failed
 *	@retval  0 = success
 */
static int libinoplc_log_(const char *func, int line, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	char *logenv = getenv("LOGLEVEL");

	if (logenv != NULL && atoi(logenv) >= 1) {
		printf("%ld: %s[%d]: ", time(NULL), func, line);
		vprintf(format, args);
		printf("\n");
	}

	FILE *fp = fopen(__LOGFILE, "a+");
	if (fp != NULL) {
		fprintf(fp, "%ld: %s[%d]: ", time(NULL), func, line);
		vfprintf(fp, format, args);
		fprintf(fp, "\n");
		fclose(fp);
	}

	va_end(args);

	return 0;
}
#define libinoplc_log(format, ...)  libinoplc_log_(__FUNCTION__, __LINE__, format, ##__VA_ARGS__)
#else
#define libinoplc_log(format, ...)
#endif


#ifdef __cplusplus
}
#endif

#endif /*__LIBINOPLC_H__*/