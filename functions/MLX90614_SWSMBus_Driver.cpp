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
 /**
 * As the timings depend heavily on the MCU in use, it is recommended
 * to make sure that the proper timings are achieved. For that purpose
 * an oscilloscope might be needed to strobe the SCL and SDA signals.
 * The Wait(int) function could be modified in order to better 
 * trim the frequency. For coarse setting of the frequency or 
 * dynamic frequency change using the default function implementation, 
 * ‘freqCnt’ argument should be changed – lower value results in 
 * higher frequency.
 */
 
#include "mbed.h"
#include "MLX90614_SMBus_Driver.h"


DigitalInOut sda(p9);
DigitalOut scl(p10);

#define LOW 0;
#define HIGH 1;

#define SCL_HIGH scl = HIGH;
#define SCL_LOW scl = LOW;     
#define SDA_HIGH sda.input();
#define SDA_LOW sda.output(); \
                sda = LOW;           

int SMBusSendByte(int8_t);
void SMBusReadBytes(int, char *);
void SMBusStart(void);
void SMBusStop(void);
void SMBusRepeatedStart(void);
void SMBusSendACK(void);
void SMBusSendNack(void);
int SMBusReceiveAck(void);
void Wait(int);
uint8_t Calculate_PEC(uint8_t, uint8_t);
void WaitEE(uint16_t ms);

static int freqCnt;

void MLX90614_SMBInit()
{   
    SMBusStop();
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
    
    SMBusStop();
    Wait(freqCnt);  
    SMBusStart();
    Wait(freqCnt);
    
    ack = SMBusSendByte(sa)!=0;
    if(ack != 0)
    {
        return -1;
    } 
    
    ack = SMBusSendByte(cmd);   
    if(ack != 0)
    {
        return -1;
    }
    
    SMBusRepeatedStart();
                
    sa = sa | 0x01;
    ack = SMBusSendByte(sa);
    if(ack != 0)
    {
        return -1;
    } 
        
    SMBusReadBytes(3, smbData);                   
    if (ack != 0x00)
    {
        return -1; 
    }          
    SMBusStop();   
    
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
    freqCnt = freq>>1;
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

    SMBusStop();
    Wait(freqCnt);
    SMBusStart();
    ack = SMBusSendByte(sa);
    if (ack != 0x00)
    {
        return 1; 
    }  
    
    for(int i = 0; i<4; i++)
    {
        ack = SMBusSendByte(cmd[i]);
    
        if (ack != 0x00)
        {
            return -1;
        }  
    }           
    SMBusStop();   
    
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

    SMBusStop();
    Wait(freqCnt);
    SMBusStart();
    ack = SMBusSendByte(sa);
    if (ack != 0x00)
    {
        return 1; 
    }  
    
    for(int i = 0; i<2; i++)
    {
        ack = SMBusSendByte(cmd[i]);
    
        if (ack != 0x00)
        {
            return -1;
        }  
    }               
    
    if (ack != 0x00)
    {
        return -1;
    }         
    SMBusStop();   
        
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
    int cnt;
    for(int i = 0;i<ms;i++)
    {
        for(int j = 0;j<24000;j++)
        {
        }    
        cnt = cnt++; 
    }  
}    

int SMBusSendByte(int8_t data)
{
   int ack = 1;
   int8_t byte = data; 
   
   for(int i=0;i<8;i++)
   {
       Wait(freqCnt);
       
       if(byte & 0x80)
       {
           SDA_HIGH;
       }
       else
       {
           SDA_LOW;
       }
       Wait(freqCnt);
       SCL_HIGH;
       Wait(freqCnt);
       Wait(freqCnt);
       SCL_LOW;
       byte = byte<<1;        
   }    
   
   Wait(freqCnt);
   ack = SMBusReceiveAck();
   
   return ack; 
}

void SMBusReadBytes(int nBytes, char *dataP)
{
    char data;
    for(int j=0;j<nBytes;j++)
    {
        Wait(freqCnt);
        SDA_HIGH;    
        
        data = 0;
        for(int i=0;i<8;i++){
            Wait(freqCnt);
            SCL_HIGH;
            Wait(freqCnt);
            data = data<<1;
            if(sda == 1){
                data = data+1;  
            }
            Wait(freqCnt);
            SCL_LOW;
            Wait(freqCnt);
        }  
        
        
        SMBusSendACK();
            
        *(dataP+j) = data; 
    }    
    
}
        
void Wait(int freqCnt)
{
    int cnt;
    for(int i = 0;i<freqCnt;i++)
    {
        cnt = cnt++; 
    }    
} 

void SMBusStart(void)
{
    SDA_HIGH;
    SCL_HIGH;
    Wait(freqCnt);
    Wait(freqCnt);
    SDA_LOW;
    Wait(freqCnt);
    SCL_LOW;
    Wait(freqCnt);    
    
}

void SMBusStop(void)
{
    SCL_LOW;
    SDA_LOW;
    Wait(freqCnt);
    SCL_HIGH;
    Wait(freqCnt);
    SDA_HIGH;
    Wait(freqCnt);
} 
 
void SMBusRepeatedStart(void)
{
    SCL_LOW;
    Wait(freqCnt);
    SDA_HIGH;
    Wait(freqCnt);
    SCL_HIGH;
    Wait(freqCnt);
    SDA_LOW;
    Wait(freqCnt);
    SCL_LOW;
           
}

void SMBusSendACK(void)
{
    SDA_LOW;
    Wait(freqCnt);
    SCL_HIGH;
    Wait(freqCnt);
    Wait(freqCnt);
    SCL_LOW;
    Wait(freqCnt);
    SDA_HIGH;
    
}

void SMBusSendNack(void)
{
    SDA_HIGH;
    Wait(freqCnt);
    SCL_HIGH;
    Wait(freqCnt);
    Wait(freqCnt);
    SCL_LOW;
    Wait(freqCnt);
    SDA_HIGH;
    
}

int SMBusReceiveAck(void)
{
    int ack;
    
    SDA_HIGH;
    Wait(freqCnt);
    SCL_HIGH;
    Wait(freqCnt);
    if(sda == 0)
    {
        ack = 0;
    }
    else
    {
        ack = 1;
    }
    Wait(freqCnt);        
    SCL_LOW;
    SDA_LOW;
    
    return ack;    
}  

     