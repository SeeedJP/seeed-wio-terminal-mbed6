#ifndef RAW_SERIAL_H_
#define RAW_SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define RAW_SERIAL_ENABLE

#if defined(RAW_SERIAL_ENABLE)
void raw_serial_init(void);
void raw_serial_write(const char *str);
void raw_serial_printf(const char *fmt, ...);

#define raw_trace() raw_serial_printf("%s:%s().L%d\r\n", __FILE__, __func__, __LINE__)

#else

#define raw_serial_init()
#define raw_serial_write(str)
#define raw_serial_printf(fmt, ...)
#define raw_trace()

#endif

#ifdef __cplusplus
}
#endif
#endif
