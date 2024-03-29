//
// Created by zhpwa on 2024/1/11.
//

#ifndef _BSP_BUZ_DEFS_H_
#define _BSP_BUZ_DEFS_H_

#include "stdint.h"

const uint16_t psc_table[128] = {313,
								 295,
								 279,
								 263,
								 248,
								 234,
								 221,
								 209,
								 197,
								 186,
								 175,
								 166,
								 156,
								 147,
								 139,
								 131,
								 124,
								 117,
								 110,
								 104,
								 98,
								 93,
								 87,
								 83,
								 78,
								 73,
								 69,
								 65,
								 62,
								 58,
								 55,
								 52,
								 49,
								 46,
								 43,
								 41,
								 39,
								 36,
								 34,
								 32,
								 31,
								 29,
								 27,
								 26,
								 24,
								 23,
								 21,
								 20,
								 19,
								 18,
								 17,
								 16,
								 15,
								 14,
								 13,
								 13,
								 12,
								 11,
								 10,
								 10,
								 9,
								 9,
								 8,
								 8,
								 7,
								 7,
								 6,
								 6,
								 6,
								 5,
								 5,
								 5,
								 4,
								 4,
								 4,
								 4,
								 3,
								 3,
								 3,
								 3,
								 3,
								 2,
								 2,
								 2,
								 2,
								 2,
								 2,
								 2,
								 1,
								 1,
								 1,
								 1,
								 1,
								 1,
								 1,
								 1,
								 1,
								 1,
								 1,
								 1,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0,
								 0};

const uint16_t arr_table[128] = {65439,
								 65523,
								 65379,
								 65450,
								 65498,
								 65505,
								 65449,
								 65305,
								 65376,
								 65336,
								 65524,
								 65179,
								 65439,
								 65523,
								 65379,
								 65450,
								 65236,
								 65227,
								 65449,
								 65305,
								 65376,
								 64989,
								 65524,
								 64791,
								 65025,
								 65523,
								 65379,
								 65450,
								 64718,
								 65227,
								 64864,
								 64689,
								 64722,
								 64989,
								 65524,
								 64791,
								 64212,
								 65523,
								 65379,
								 65450,
								 63707,
								 64140,
								 64864,
								 63491,
								 64722,
								 63635,
								 65524,
								 64791,
								 64212,
								 63798,
								 63563,
								 63525,
								 63707,
								 64140,
								 64864,
								 61224,
								 62233,
								 63635,
								 65524,
								 61846,
								 64212,
								 60608,
								 63563,
								 59995,
								 63707,
								 60131,
								 64864,
								 61224,
								 57787,
								 63635,
								 60063,
								 56692,
								 64212,
								 60608,
								 57207,
								 53996,
								 63707,
								 60131,
								 56756,
								 53571,
								 50564,
								 63635,
								 60063,
								 56692,
								 53510,
								 50507,
								 47672,
								 44996,
								 63707,
								 60131,
								 56756,
								 53571,
								 50564,
								 47726,
								 45047,
								 42519,
								 40132,
								 37880,
								 35754,
								 33747,
								 63707,
								 60131,
								 56756,
								 53571,
								 50564,
								 47726,
								 45047,
								 42519,
								 40132,
								 37880,
								 35754,
								 33747,
								 31853,
								 30065,
								 28377,
								 26785,
								 25281,
								 23862,
								 22523,
								 21259,
								 20065,
								 18939,
								 17876,
								 16873,
								 15926,
								 15032,
								 14188,
								 13392};

const int noteFrequency[128] = {(int) 8.175798915643707, (int) 8.661957218027252, (int) 9.177023997418987,
								(int) 9.722718241315029, (int) 10.300861153527185, (int) 10.913382232281371,
								(int) 11.562325709738575, (int) 12.249857374429665, (int) 12.978271799373285,
								(int) 13.75,
								(int) 14.56761754744031, (int) 15.433853164253879, (int) 16.351597831287414,
								(int) 17.323914436054505, (int) 18.354047994837973, (int) 19.445436482630058,
								(int) 20.60172230705437, (int) 21.826764464562743, (int) 23.12465141947715,
								(int) 24.49971474885933, (int) 25.95654359874657, (int) 27.5, (int) 29.13523509488062,
								(int) 30.867706328507758, (int) 32.70319566257483, (int) 34.64782887210901,
								(int) 36.70809598967595, (int) 38.890872965260115, (int) 41.20344461410874,
								(int) 43.653528929125486, (int) 46.2493028389543, (int) 48.99942949771866,
								(int) 51.91308719749314, (int) 55.0, (int) 58.27047018976124, (int) 61.7354126570155,
								(int) 65.40639132514966, (int) 69.29565774421802, (int) 73.41619197935188,
								(int) 77.78174593052023, (int) 82.4068892282175, (int) 87.30705785825097,
								(int) 92.4986056779086, (int) 97.99885899543733, (int) 103.82617439498628, (int) 110.0,
								(int) 116.54094037952248, (int) 123.470825314031, (int) 130.8127826502993,
								(int) 138.59131548843604, (int) 146.83238395870376, (int) 155.56349186104046,
								(int) 164.813778456435, (int) 174.61411571650194, (int) 184.9972113558172,
								(int) 195.99771799087466, (int) 207.65234878997256, (int) 220.0,
								(int) 233.0818807590449,
								(int) 246.94165062806212, (int) 261.6255653005986, (int) 277.1826309768721,
								(int) 293.66476791740763, (int) 311.1269837220809, (int) 329.62755691286986,
								(int) 349.22823143300394, (int) 369.9944227116344, (int) 391.9954359817492,
								(int) 415.3046975799452, (int) 440.0, (int) 466.1637615180898, (int) 493.88330125612424,
								(int) 523.2511306011972, (int) 554.3652619537442, (int) 587.3295358348153,
								(int) 622.2539674441618, (int) 659.2551138257397, (int) 698.4564628660079,
								(int) 739.9888454232688, (int) 783.9908719634984, (int) 830.6093951598904, (int) 880.0,
								(int) 932.3275230361796, (int) 987.7666025122485, (int) 1046.5022612023945,
								(int) 1108.7305239074883, (int) 1174.6590716696305, (int) 1244.5079348883237,
								(int) 1318.5102276514795, (int) 1396.9129257320158, (int) 1479.9776908465376,
								(int) 1567.9817439269968, (int) 1661.2187903197807, (int) 1760.0,
								(int) 1864.6550460723593,
								(int) 1975.533205024497, (int) 2093.004522404789, (int) 2217.4610478149766,
								(int) 2349.318143339261, (int) 2489.0158697766474, (int) 2637.020455302959,
								(int) 2793.8258514640315, (int) 2959.955381693075, (int) 3135.9634878539937,
								(int) 3322.4375806395615, (int) 3520.0, (int) 3729.3100921447212,
								(int) 3951.0664100489917,
								(int) 4186.009044809578, (int) 4434.922095629955, (int) 4698.636286678519,
								(int) 4978.031739553295, (int) 5274.040910605921, (int) 5587.65170292806,
								(int) 5919.91076338615, (int) 6271.926975707992, (int) 6644.875161279119, (int) 7040.0,
								(int) 7458.6201842894425, (int) 7902.132820097983, (int) 8372.018089619156,
								(int) 8869.84419125991, (int) 9397.272573357039, (int) 9956.06347910659,
								(int) 10548.081821211843, (int) 11175.30340585612, (int) 11839.8215267723,
								(int) 12543.853951415984};
#endif //_BSP_BUZ_DEFS_H_
