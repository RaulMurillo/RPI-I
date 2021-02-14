/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Attributes State Machine */
enum characteristic
{
    CEAQ_IDX_SVC, // índice del servicio Capacity Estimation & Air Quality

    IDX_MEAS_FREQ_CHAR,   // índice de la característica Measurement Frequency
    IDX_MEAS_FREQ_VAL,    // índice del valor Measurement Frequency
    IDX_MEAS_FREQ_DESC,   // indice del descriptor de Measurement Frequency
    IDX_MEAS_FREQ_FORMAT, // índice del formato de presentacion de Measurement Frequency

    IDX_DIST_CHAR,   // índice de la característica Distance
    IDX_DIST_VAL,    // índice del valor Distance
    IDX_DIST_DESC,   // indice del descriptor de Distance
    IDX_DIST_FORMAT, // índice del formato de presentacion de Distance

    IDX_CE_ACTIVE_CHAR,   // índice de la característica CE-Active
    IDX_CE_ACTIVE_VAL,    // índice del valor CE-Active
    IDX_CE_ACTIVE_DESC,   // indice del descriptor de CE-Active
    IDX_CE_ACTIVE_FORMAT, // índice del formato de presentacion de CE-Active

    IDX_CO2_ACTIVE_CHAR,   // índice de la característica CO2-Active
    IDX_CO2_ACTIVE_VAL,    // índice del valor CO2-Active
    IDX_CO2_ACTIVE_DESC,   // indice del descriptor de CO2-Active
    IDX_CO2_ACTIVE_FORMAT, // índice del formato de presentacion de CO2-Active

    IDX_TMP_ACTIVE_CHAR,   // índice de la característica TMP-Active
    IDX_TMP_ACTIVE_VAL,    // índice del valor TMP-Active
    IDX_TMP_ACTIVE_DESC,   // indice del descriptor de TMP-Active
    IDX_TMP_ACTIVE_FORMAT, // índice del formato de presentacion de TMP-Active

    IDX_CE_CHAR,    // índice de la característica Capacity Estimation (CE)
    IDX_CE_VAL,     // índice del valor CE
    IDX_CE_DESC,    // indice del descriptor de CE
    IDX_CE_NTF_CFG, // índice de la configuración de notificaciones (CCC) de CE
    IDX_CE_FORMAT,  // índice del formato de presentacion de CE

    IDX_CO2_CHAR,    // índice de la característica CO2
    IDX_CO2_VAL,     // índice del valor CO2
    IDX_CO2_DESC,    // indice del descriptor de CO2
    IDX_CO2_NTF_CFG, // índice de la configuración de notificaciones (CCC) CO2
    IDX_CO2_FORMAT,  // índice del formato de presentacion de CO2

    IDX_TMP_CHAR,    // índice de la característica Temperature
    IDX_TMP_VAL,     // índice del valor Temperature
    IDX_TMP_DESC,    // indice del descriptor de Temperature
    IDX_TMP_NTF_CFG, // índice de la configuración de notificaciones (CCC) Temperature
    IDX_TMP_FORMAT,  // índice del formato de presentacion de Temperature

    CEAQ_IDX_NB, // número de elementos de la tabla
};
