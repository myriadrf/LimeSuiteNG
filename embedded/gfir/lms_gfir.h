/* 
 * File:   lms_gfir.h
 * Author: ignas
 *
 * Created on January 18, 2016, 1:01 PM
 */

#ifndef LMS_GFIR_H
#define LMS_GFIR_H

#ifdef __cplusplus
extern "C" {
#endif

void GenerateFilter(int n, float w1, float w2, float a1, float a2, float* coefs);

#ifdef __cplusplus
}
#endif

#endif /* LMS_GFIR_H */
