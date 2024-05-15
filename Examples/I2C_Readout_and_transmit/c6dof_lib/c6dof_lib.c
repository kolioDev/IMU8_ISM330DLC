#include <stdint.h>
#include <stdint-gcc.h>
#include "./c6dof_lib.h"
#include <Wire.h>


u_int8_t c6dofimu8_read_byte(c6dofimu8_t *ctx, u_int8_t reg_addr, u_int8_t *data) {
    if (reg_addr & 0x80) {
        return C6DOFIMU8_ADDR_ERROR;
    }

    // Use Wire library to read a single byte from the sensor
    Wire.beginTransmission(ctx->device_addr); // I2C address of the sensor
    Wire.write(reg_addr);
    if(Wire.endTransmission(false) != 0){
         return C6DOFIMU8_DATA_RATE_ERROR;
    } 
    Wire.requestFrom((int)ctx->device_addr, 1); // Request 1 byte from the sensor
    *data = Wire.read();

    return C6DOFIMU8_OK;
}

u_int8_t c6dofimu8_write_byte(c6dofimu8_t *ctx, u_int8_t reg_addr, u_int8_t data) {
    if (reg_addr & 0x80) {
        return C6DOFIMU8_ADDR_ERROR;
    }

    // Use Wire library to write a single byte to the sensor
    Wire.beginTransmission(ctx->device_addr); // I2C address of the sensor
    Wire.write(reg_addr);
    Wire.write(data);
    if(Wire.endTransmission() != 0){
         return C6DOFIMU8_DATA_RATE_ERROR;
    } 

    return C6DOFIMU8_OK;
}

u_int8_t c6dofimu8_read_bytes(c6dofimu8_t *ctx, u_int8_t reg_addr, u_int8_t *data, u_int8_t len) {
    if (reg_addr & 0x80) {
        return C6DOFIMU8_ADDR_ERROR;
    }

    // Use Wire library to read multiple bytes from the sensor
    Wire.beginTransmission(ctx->device_addr); // I2C address of the sensor
    Wire.write(reg_addr);
    if(Wire.endTransmission(false) != 0){
         return C6DOFIMU8_DATA_RATE_ERROR;
    } 
    Wire.requestFrom((int)ctx->device_addr, (int)len); // Request len bytes from the sensor
    // for (int i = 0; i < len; i++) {
    //     data[i] = Wire.read();
    // }

    u_int8_t bytesRead = 0;
    while (Wire.available() && bytesRead < len) {
        data[bytesRead] = Wire.read();
        bytesRead++;
    }

    if (bytesRead != len) {
        return C6DOFIMU8_DATA_RATE_ERROR;
    }

    return C6DOFIMU8_OK;
}

u_int8_t c6dofimu8_write_bytes(c6dofimu8_t *ctx, u_int8_t reg_addr, u_int8_t *data, u_int8_t len) {
    if (reg_addr & 0x80) {
        return C6DOFIMU8_ADDR_ERROR;
    }

    // Use Wire library to write multiple bytes to the sensor
    Wire.beginTransmission(ctx->device_addr); // I2C address of the sensor
    Wire.write(reg_addr);
    for (int i = 0; i < len; i++) {
        Wire.write(data[i]);
    }

    if(Wire.endTransmission() != 0){
         return C6DOFIMU8_DATA_RATE_ERROR;
    } 

    return C6DOFIMU8_OK;
}



u_int8_t c6dofimu8_reset(c6dofimu8_t *ctx) {
    u_int8_t temp_data;
    u_int8_t err;

    err = c6dofimu8_read_byte(ctx, C6DOFIMU8_CTRL3_C_REG, &temp_data);
    if(err!=C6DOFIMU8_OK){
        return err;
    }
    temp_data |= C6DOFIMU8_SW_RESET_CMD;

    err = c6dofimu8_write_byte(ctx, C6DOFIMU8_CTRL3_C_REG, temp_data);
    if(err !=C6DOFIMU8_OK){
        return err;
    }

    delay(1000); // Delay 1 second
    return C6DOFIMU8_OK;
}

u_int8_t c6dofimu8_set_odr(c6dofimu8_t *ctx, u_int8_t gyro_odr, u_int8_t accel_odr) {
    u_int8_t odr_data[2];
    u_int8_t err;

    if ((gyro_odr > 0x0B) || (accel_odr > 0x0B)) {
        return C6DOFIMU8_DATA_RATE_ERROR;
    }

    err = c6dofimu8_read_bytes(ctx, C6DOFIMU8_CTRL1_XL_REG, odr_data, 2);
    if(err!=C6DOFIMU8_OK){
        return err;
    }
    odr_data[0] &= 0x0F;
    odr_data[1] &= 0x0F;
    odr_data[0] |= accel_odr << 4;
    odr_data[1] |= gyro_odr << 4;
    err = c6dofimu8_write_bytes(ctx, C6DOFIMU8_CTRL1_XL_REG, odr_data, 2);
    if(err!=C6DOFIMU8_OK){
        return err;
    }    

    
    return C6DOFIMU8_OK;
}

u_int8_t c6dofimu8_set_fsr(c6dofimu8_t *ctx, u_int8_t gyro_fsr, u_int8_t accel_fsr) {
    u_int8_t fsr_data[2];
    u_int8_t err;

    if ((gyro_fsr > 4) || (accel_fsr > 3)) {
        return C6DOFIMU8_FULL_SCALE_ERROR;
    }

    err = c6dofimu8_read_bytes(ctx, C6DOFIMU8_CTRL1_XL_REG, fsr_data, 2);
    if(err!=C6DOFIMU8_OK){
        return err;
    }    
    fsr_data[0] &= 0xF3;
    fsr_data[1] &= 0xF1;
    fsr_data[0] |= accel_fsr << 2;

    if (gyro_fsr == C6DOFIMU8_FS_G_125DPS) {
        fsr_data[1] |= 0x02;
    } else {
        fsr_data[1] |= gyro_fsr << 2;
    }

    err = c6dofimu8_write_bytes(ctx, C6DOFIMU8_CTRL1_XL_REG, fsr_data, 2);
    if(err!=C6DOFIMU8_OK){
        return err;
    }

    ctx->gyro_res = gyro_fsr;
    ctx->accel_res = accel_fsr;

    return C6DOFIMU8_OK;
}



u_int8_t c6dofimu8_default_cfg(byte board_addr, c6dofimu8_t *ctx, u_int8_t ODR, u_int8_t gyro_scale, u_int8_t acc_scale ) {
    ctx->device_addr = board_addr;
    u_int8_t err;

    err = c6dofimu8_reset(ctx);
    if(err!=C6DOFIMU8_OK) return err;

    err = c6dofimu8_set_odr(ctx, ODR, ODR);
    if(err!=C6DOFIMU8_OK) return err;

    err = c6dofimu8_set_fsr(ctx, gyro_scale,  acc_scale);
    if(err!=C6DOFIMU8_OK) return err;

    err = c6dofimu8_write_byte(ctx, C6DOFIMU8_INT1_CTRL_REG, C6DOFIMU8_INT_PIN_G_DRDY_FLAG_EN);
    if(err!=C6DOFIMU8_OK) return err;

    err = c6dofimu8_write_byte(ctx, C6DOFIMU8_INT2_CTRL_REG, C6DOFIMU8_INT_PIN_XL_DRDY_FLAG_EN);
    if(err!=C6DOFIMU8_OK) return err;


    return C6DOFIMU8_OK;

}



static void get_resolution ( c6dofimu8_t *ctx, double *gyro_lsb, double *accel_lsb )
{
    switch ( ctx->gyro_res )
    {
        case C6DOFIMU8_FS_G_250DPS :
        {
            *gyro_lsb = 0.00762939453125;
            break;
        }
        case C6DOFIMU8_FS_G_500DPS :
        {
            *gyro_lsb = 0.0152587890625;
            break;
        }
        case C6DOFIMU8_FS_G_1000DPS :
        {
            *gyro_lsb = 0.030517578125;
            break;
        }
        case C6DOFIMU8_FS_G_2000DPS :
        {
            *gyro_lsb = 0.06103515625;
            break;
        }
        case C6DOFIMU8_FS_G_125DPS :
        {
            *gyro_lsb = 0.003814697265625;
            break;
        }
    }

    switch ( ctx->accel_res )
    {
        case C6DOFIMU8_FS_XL_2G :
        {
            *accel_lsb = 0.00006103515625;
            break;
        }
        case C6DOFIMU8_FS_XL_16G :
        {
            *accel_lsb = 0.00048828125;
            break;
        }
        case C6DOFIMU8_FS_XL_4G :
        {
            *accel_lsb = 0.0001220703125;
            break;
        }
        case C6DOFIMU8_FS_XL_8G :
        {
            *accel_lsb = 0.000244140625;
            break;
        }
    }
}

u_int8_t c6dofimu8_get_data_raw(c6dofimu8_t *ctx, u_int8_t *out ){
    u_int8_t err = c6dofimu8_read_bytes(ctx, C6DOFIMU8_OUT_TEMP_REG, out, 14);
    if(err != C6DOFIMU8_OK){
        return err;
    }
    return C6DOFIMU8_OK;
}

u_int8_t c6dofimu8_get_data (c6dofimu8_t *ctx, t_c6dofimu8_axis *accel_out, t_c6dofimu8_axis *gyro_out, int8_t *temp_out )
{
    u_int8_t temp_data[ 14 ];
    int16_t temp;
    int16_t accel_tmp[ 3 ];
    int16_t gyro_tmp[ 3 ];
    u_int8_t tmp_idx;
    u_int8_t count;
    u_int8_t check_var;
    double res;
    double gyro_lsb;
    double accel_lsb;
    u_int8_t err; 

    // Read 14 bytes 
    // c6dofimu8_read_bytes( ctx, C6DOFIMU8_OUT_TEMP_REG, temp_data, 14 );
    // get_resolution( ctx, &gyro_lsb, &accel_lsb );
    err = c6dofimu8_read_bytes(ctx, C6DOFIMU8_OUT_TEMP_REG, temp_data, 14);
    if(err != C6DOFIMU8_OK){
        return err;
    }

  
    // for(int i=0; i<14; i++){
    //     Serial.print(temp_data[i]);
    //     Serial.print(" ");
    // }
    
    get_resolution( ctx, &gyro_lsb, &accel_lsb );
    // Serial.print("gyro lsb:   ");
    // String gyro_lsb_str = String(gyro_lsb, 6);
    // Serial.println(gyro_lsb_str);

    // Serial.print("accel lsb:   ");
    // String accel_lsb_str = String(accel_lsb, 6);
    // Serial.println(accel_lsb_str);
  


    // Copy paste from the microelectronica library
    check_var = 0;
    tmp_idx = 0;
    for ( count = 0; count < 14; count++ )
    {
        if ( count < 2 )
        {
            if ( check_var == 0 )
            {
                temp = temp_data[ count ];
                check_var = 1;
            }
            else
            {
                temp |= (int16_t)temp_data[ count ] << 8;
                check_var = 0;
            }
        }
        else if ( ( count >= 2 ) && ( count < 8 ) )
        {
            if ( check_var == 0 )
            {
                gyro_tmp[ tmp_idx ] = temp_data[ count ];
                check_var = 1;
            }
            else
            {
                gyro_tmp[ tmp_idx ] |= (int16_t)temp_data[ count ] << 8;
                tmp_idx++;
                check_var = 0;
                
                if ( count == 7 )
                {
                    tmp_idx = 0;
                }
            }
        }
        else
        {
            if ( check_var == 0 )
            {
                accel_tmp[ tmp_idx ] = temp_data[ count ];
                check_var = 1;
            }
            else
            {
                accel_tmp[ tmp_idx ] |= (int16_t)temp_data[ count ] << 8;
                tmp_idx++;
                check_var = 0;
            }
        }
    }

    res = temp;
    res /= TEMP_LSB_RES;
    res += TEMP_OFFSET;
    *temp_out = res;

    accel_out->x = accel_tmp[ 0 ];
    accel_out->x *= accel_lsb;
    accel_out->y = accel_tmp[ 1 ];
    accel_out->y *= accel_lsb;
    accel_out->z = accel_tmp[ 2 ];
    accel_out->z *= accel_lsb;

    gyro_out->x = gyro_tmp[ 0 ];
    gyro_out->x *= gyro_lsb;
    gyro_out->y = gyro_tmp[ 1 ];
    gyro_out->y *= gyro_lsb;
    gyro_out->z = gyro_tmp[ 2 ];
    gyro_out->z *= gyro_lsb;

    return C6DOFIMU8_OK;
}


uint8_t c6dofimu8_get_drdy_status(c6dofimu8_t *ctx, uint8_t bit_mask) {
    uint8_t temp_data;

    c6dofimu8_read_byte(ctx, C6DOFIMU8_STATUS_REG, &temp_data);
    temp_data &= bit_mask;

    return temp_data;
}