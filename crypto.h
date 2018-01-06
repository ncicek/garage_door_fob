#ifndef CRYPTO_WRAP
#define CRYPTO_WRAP

uint16_t next_lfsr(uint16_t lfsr);
void generate_code(uint8_t command, uint8_t *generated_code);
uint8_t decode_code(uint8_t *command, uint8_t *generated_code);

#endif /* CRYPTO_WRAP */
