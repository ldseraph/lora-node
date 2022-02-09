#include <stdint.h>

#define RAND_LOCAL_MAX 2147483647L
static uint32_t next = 1;

static int32_t lorawan_rand1() {
  return ((next = next * 1103515245L + 12345L) % RAND_LOCAL_MAX);
}

void lorawan_srand(uint32_t seed) {
  next = seed;
}

int32_t lorawan_rand(int32_t min, int32_t max) {
  return (int32_t)lorawan_rand1() % (max - min + 1) + min;
}
