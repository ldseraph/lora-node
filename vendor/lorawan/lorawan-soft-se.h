#ifndef __LORAWAN_CMAC_H__
#define __LORAWAN_CMAC_H__

#define N_ROW 4
#define N_COL 4
#define N_BLOCK (N_ROW * N_COL)
#define N_MAX_ROUNDS 14

#define AES_CMAC_KEY_LENGTH 16
#define AES_CMAC_DIGEST_LENGTH 16

typedef struct {
  uint8_t ksch[(N_MAX_ROUNDS + 1) * N_BLOCK];
  uint8_t rnd;
} lorawan_aes_context;

typedef struct {
  lorawan_aes_context rijndael;
  uint8_t             x[16];
  uint8_t             m_last[16];
  uint32_t            m_n;
} lorawan_aes_cmac_ctx;

void    lorawan_cmac_setkey(lorawan_aes_cmac_ctx*, uint8_t*);
void    lorawan_cmac_update(lorawan_aes_cmac_ctx*, const uint8_t*, uint32_t);
void    lorawan_cmac_final(lorawan_aes_cmac_ctx* ctx, uint8_t digest[AES_CMAC_DIGEST_LENGTH]);
uint8_t lorawan_soft_se_aes_encrypt(const uint8_t in[N_BLOCK], uint8_t out[N_BLOCK], const lorawan_aes_context ctx[1]);
uint8_t lorawan_soft_se_aes_set_key(lorawan_aes_context* ctx, const uint8_t key[], uint8_t keylen);
void    lorawan_soft_se_derive(uint8_t* key, uint8_t* in, uint8_t* out);

#endif  // __LORAWAN_CMAC_H__