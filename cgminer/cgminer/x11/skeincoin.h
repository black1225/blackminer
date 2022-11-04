#ifndef SKEINCOIN_H
#define SKEINCOIN_H

#include "miner.h"

extern int skeinmid(unsigned char *out, const unsigned char *in);
extern int skeincoin(unsigned char *out, const unsigned char *in, unsigned long long inlen);

#endif /* SKEIN_H */
