#ifndef CRYPTO_WRAP
#define CRYPTO_WRAP

void copy_mem (uint8_t *dest, const uint8_t *source, uint8_t n);
uint16_t next_lfsr(uint16_t lfsr);
void generate_code(uint8_t command, uint8_t *generated_code);
uint8_t decode_code(uint8_t *command, uint8_t *generated_code);

#endif /* CRYPTO_WRAP */
