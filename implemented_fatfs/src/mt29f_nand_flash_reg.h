#include<stdio.h>
#include<stdbool.h>
#include"mt29f_nand_flash.h"





bool validate_row_address(row_address_t row); 
bool validate_column_address(column_address_t column);
ReturnType get_feature(uint8_t reg, uint8_t *data_out, uint32_t timeout);
ReturnType set_feature(uint8_t reg, uint8_t data, uint32_t timeout);
ReturnType poll_for_oip_clear(feature_reg_status_t *status_out, uint32_t timeout);
ReturnType reset(void);
ReturnType whoami(void);
ReturnType unlock_all_blocks(void);
ReturnType enable_ecc(void);
ReturnType get_ret_from_ecc_status(feature_reg_status_t status);
ReturnType page_read(row_address_t row, uint32_t timeout);
ReturnType read_from_cache(column_address_t col, uint8_t *data_out, size_t read_len, uint32_t timeout);
ReturnType program_load(column_address_t column, uint8_t *data_in, size_t write_len, uint32_t timeout);
ReturnType program_execute(row_address_t row, uint32_t timeout);
ReturnType write_enable(uint32_t timeout);
ReturnType block_erase(row_address_t row, uint32_t timeout);
ReturnType program_load_random_data(column_address_t column,uint8_t *data_in, size_t write_len, uint32_t timeout);

