/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SCREENS_H
#define __SCREENS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifndef SCREENS_INTF_RET_TYPE
#define SCREENS_INTF_RET_TYPE                      int8_t
#endif


//typedef SCREENS_INTF_RET_TYPE (*bme280_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
//typedef SCREENS_INTF_RET_TYPE (*screens_read_fptr_t)(uint8_t reg_addr);


typedef SCREENS_INTF_RET_TYPE (*screens_read_fptr_t)(uint16_t data);


struct screen_data_holder
{
	uint8_t some_number;
    uint8_t screen_index;

    /*< Read function pointer */
//    screens_read_fptr_t read;
//    screens_read_fptr_t test_function;
    screens_read_fptr_t screen_renderer[3];
};


#ifdef __cplusplus
}
#endif

#endif /* __SCREENS_H */
