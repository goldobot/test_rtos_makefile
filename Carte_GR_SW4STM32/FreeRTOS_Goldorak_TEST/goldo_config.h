#ifndef __GOLDO_CONFIG_H__
#define __GOLDO_CONFIG_H__
#include <stdio.h>
#include <stdbool.h>

typedef unsigned int wint_t;
#define OK 0
#define GOLDO_ERROR 1
typedef unsigned short int uint16_t;

#define goldo_trace(message) printf("%s",__FILE__)
#define goldo_log(importance,...) printf(__VA_ARGS__)

#endif /* __GOLDO_CONFIG_H__ */
