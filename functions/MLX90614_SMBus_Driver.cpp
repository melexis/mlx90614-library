/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "mbed.h"
#include "MLX90614_SMBus_Driver.h"

I2C i2c(p9, p10);

uint8_t Calculate_PEC(uint8_t, uint8_t);
void WaitEE(uint16_t ms);

void MLX90614_SMBusInit()
{   
    i2c.stop();
}

int MLX90614_SMBusRead(uint8_t slaveAddr, uint8_t readAddress, uint16_t *data)
{
    uint8_t sa;                           
    int ack = 0;
    uint8_t pec;                               
    char cmd = 0;
    char smbData[3] = {0,0,0};
    uint16_t *p;
    
    p = data;
    sa = (slaveAddr << 1);
    pec = sa;
    cmd = readAddress;
    
    i2c.stop();
    wait_us(5);    
    ack = i2c.write(sa, &cmd, 1, 1);
    
    if (ack != 0x00)
    {
        return -1;
    }
             
    sa = sa | 0x01;
    ack = i2c.read(sa, smbData, 3, 0);
    
    if (ack != 0x00)
    {
        return -1; 
    }          
    i2c.stop();   
    
    pec = Calculate_PEC(0, pec);
    pec = Calculate_PEC(pec, cmd);
    pec = Calculate_PEC(pec, sa);
    pec = Calculate_PEC(pec, smbData[0]);
    pec = Calculate_PEC(pec, smbData[1]);
    
    if (pec != smbData[2])
    {
        return -2;
    }
        
    *p = (uint16_t)smbData[1]*256 + (uint16_t)smbData[0];
    
    return 0;   
} 

void MLX90614_SMBusFreqSet(int freq)
{
    i2c.frequency(1000*freq);
}

int MLX90614_SMBusWrite(uint8_t slaveAddr, uint8_t writeAddress, uint16_t data)
{
    uint8_t sa;
    int ack = 0;
    char cmd[4] = {0,0,0,0};
    static uint16_t dataCheck;
    uint8_t pec;
    
    sa = (slaveAddr << 1);
    cmd[0] = writeAddress;
    cmd[1] = data & 0x00FF;
    cmd[2] = data >> 8;
    
    pec = Calculate_PEC(0, sa);
    pec = Calculate_PEC(pec, cmd[0]);
    pec = Calculate_PEC(pec, cmd[1]);
    pec = Calculate_PEC(pec, cmd[2]);
    
    cmd[3] = pec;

    i2c.stop();
    wait_us(5);    
    ack = i2c.write(sa, cmd, 4, 0);
    
    if (ack != 0x00)
    {
        return -1;
    }         
    i2c.stop();   
    
    WaitEE(10);
    
    MLX90614_SMBusRead(slaveAddr, writeAddress, &dataCheck);
    
    if ( dataCheck != data)
    {
        return -3;
    }    
    
    return 0;
}

int MLX90614_SendCommand(uint8_t slaveAddr, uint8_t command)
{
    uint8_t sa;
    int ack = 0;
    char cmd[2]= {0,0};
    uint8_t pec;
    
    if(command != 0x60 && command != 0x61)
    {
        return -5;
    }
         
    sa = (slaveAddr << 1);
    cmd[0] = command;
        
    pec = Calculate_PEC(0, sa);
    pec = Calculate_PEC(pec, cmd[0]);
       
    cmd[1] = pec;

    i2c.stop();
    wait_us(5);    
    ack = i2c.write(sa, cmd, 2, 0);
    
    if (ack != 0x00)
    {
        return -1;
    }         
    i2c.stop();   
        
    return 0;
}    

uint8_t Calculate_PEC (uint8_t initPEC, uint8_t newData)
{
    uint8_t data;
    uint8_t bitCheck;

    data = initPEC ^ newData;
    
    for (int i=0; i<8; i++ )
    {
        bitCheck = data & 0x80;
        data = data << 1;
        
        if (bitCheck != 0)
        {
            data = data ^ 0x07;
        }
        
    }
    return data;
}
    
void WaitEE(uint16_t ms)
{
    wait_ms(ms);
}    