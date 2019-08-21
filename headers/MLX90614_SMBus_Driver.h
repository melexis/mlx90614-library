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
#ifndef _MLX90614_SMBus_Driver_H_
#define _MLX90614_SMBus_Driver_H_

#include <stdint.h>

    void MLX90614_SMBusInit(void);
    int MLX90614_SMBusRead(uint8_t slaveAddr,uint8_t readAddress, uint16_t *data);
    int MLX90614_SMBusWrite(uint8_t slaveAddr,uint8_t writeAddress, uint16_t data);
    int MLX90614_SendCommand(uint8_t slaveAddr,uint8_t command);
    void MLX90614_SMBusFreqSet(int freq);
#endif