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
#ifndef _MLX614_API_H_
#define _MLX614_API_H_
        
    int MLX90614_DumpEE(uint8_t slaveAddr, uint16_t *eeData);
    int MLX90614_GetTa(uint8_t slaveAddr, float *ta);
    int MLX90614_GetTo(uint8_t slaveAddr, float *to);
    int MLX90614_GetTo2(uint8_t slaveAddr, float *to2);
    int MLX90614_GetIRdata1(uint8_t slaveAddr, uint16_t *ir1);
    int MLX90614_GetIRdata2(uint8_t slaveAddr, uint16_t *ir2);
    int MLX90614_GetEmissivity(uint8_t slaveAddr, float *emissivity);
    int MLX90614_SetEmissivity(uint8_t slaveAddr, float value);
    int MLX90614_GetFIR(uint8_t slaveAddr, uint8_t *fir);
    int MLX90614_SetFIR(uint8_t slaveAddr, uint8_t value);
    int MLX90614_GetIIR(uint8_t slaveAddr, uint8_t *iir);
    int MLX90614_SetIIR(uint8_t slaveAddr, uint8_t value);
    float MLX90614_TemperatureInFahrenheit(float temperature);
    int16_t MLX90614_ConvertIRdata(uint16_t ir);
    
#endif

