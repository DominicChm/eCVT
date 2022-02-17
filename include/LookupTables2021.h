/*
 *  LookupTables.h - Lookup table arrays.
 *  Created by Shaina Bagri, January 2021.
 *  Released to Cal Poly Baja SAE. ;)
 */

#ifndef LookupTables2021_h
#define LookupTables2021_h

#include <Arduino.h>

// 1% Ratio Increments
static const int32_t pLookup[] = {
    48296, 47188, 46113, 45069, 44055, 43072, 42117, 41191, 40291, 39418, 38570, 37746, 36946,
    36169, 35415, 34681, 33968, 33275, 32601, 31946, 31308, 30688, 30084, 29496, 28924, 28366,
    27823, 27294, 26779, 26276, 25786, 25307, 24841, 24386, 23942, 23508, 23085, 22671, 22267,
    21873, 21487, 21110, 20742, 20381, 20028, 19683, 19346, 19015, 18692, 18375, 18065, 17762,
    17464, 17173, 16887, 16607, 16332, 16063, 15799, 15540, 15286, 15036, 14792, 14552, 14316,
    14085, 13858, 13634, 13415, 13200, 12988, 12781, 12576, 12375, 12178, 11984, 11793, 11605,
    11420, 11239, 11060, 10884, 10711, 10541, 10373, 10208, 10046, 9886, 9728, 9573, 9420, 9270,
    9121, 8975, 8831, 8689, 8549, 8411, 8276, 8142, 8009};

// 1% Ratio Increments
static const int32_t sLookup[] = {
    2120, 3278, 4386, 5446, 6461, 7433, 8364, 9257, 10114, 10936, 11725, 12484, 13212, 13914, 14588,
    15237, 15862, 16465, 17046, 17606, 18147, 18669, 19173, 19660, 20131, 20586, 21027, 21454, 21867,
    22268, 22655, 23032, 23397, 23751, 24094, 24428, 24753, 25068, 25374, 25672, 25962, 26243, 26518,
    26785, 27045, 27299, 27546, 27787, 28022, 28251, 28474, 28692, 28905, 29113, 29316, 29515, 29709,
    29898, 30083, 30264, 30442, 30615, 30785, 30951, 31113, 31272, 31428, 31581, 31730, 31877, 32020,
    32161, 32299, 32434, 32567, 32697, 32825, 32951, 33074, 33194, 33313, 33429, 33543, 33656, 33766,
    33875, 33981, 34086, 34189, 34290, 34390, 34487, 34583, 34678, 34771, 34863, 34953, 35042, 35129,
    35215, 35300};

// 1% Ratio Increments
static const int32_t cLookup[] = {
    183, 187, 190, 194, 197, 201, 205, 208, 212, 216, 219, 223, 227, 230, 234, 238, 242, 246, 249, 253,
    257, 261, 265, 269, 273, 277, 280, 284, 288, 292, 296, 300, 304, 308, 312, 316, 320, 324, 328, 332,
    336, 340, 344, 348, 352, 356, 360, 364, 368, 372, 376, 380, 384, 388, 392, 396, 400, 404, 408, 412,
    416, 420, 425, 429, 433, 437, 441, 445, 449, 453, 457, 461, 465, 469, 473, 477, 482, 486, 490, 494,
    498, 502, 506, 510, 514, 518, 523, 527, 531, 535, 539, 543, 547, 551, 555, 559, 564, 568, 572, 576,
    580};

#endif