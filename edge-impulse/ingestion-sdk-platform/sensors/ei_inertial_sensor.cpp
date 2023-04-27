/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_inertial_sensor.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/porting/ei_logging.h"


#include "qma7981.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

static float imu_data[INERTIAL_AXIS_SAMPLED];

bool ei_inertial_init(void) {

    if(qma7981_init()) {
        EI_LOGW("Failed to connect to Inertial sensor!\n");
        return false;
    }

    ei_sleep(100);
    qma7981_set_range(QMA_RANGE_2G);
    qma7981_set_badwidth(QMA_BANDWIDTH_128_HZ);

    if(ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
        EI_LOGE("Failed to register Inertial sensor!\n");
        return false;
    }

    return true;
}

float *ei_fusion_inertial_read_data(int n_samples)
{
    
    qma7981_get_acce(&imu_data[0], &imu_data[1], &imu_data[2]);
    imu_data[0] *= CONVERT_G_TO_MS2;
    imu_data[1] *= CONVERT_G_TO_MS2;
    imu_data[2] *= CONVERT_G_TO_MS2;
    
    return imu_data;
}