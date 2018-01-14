#ifndef CRYPTO_WRAP
#define CRYPTO_WRAP

uint16_t next_lfsr(uint16_t lfsr);
void generate_code(uint8_t command, uint8_t *generated_code, uint16_t time);
uint8_t decode_code(uint8_t *command, uint8_t *generated_code, uint16_t *time);
uint8_t get_chips_id();
uint16_t get_previous_lfsr(uint8_t device_id);
void save_current_lfsr(uint16_t current_lfsr, uint8_t device_id);
void load_lfsrs_into_ram();
void generate_seed();
#endif /* CRYPTO_WRAP */
