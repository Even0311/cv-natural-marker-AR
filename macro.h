//
// Created by Haoran Zhi on 16/10/17.
//

#ifndef EXAMPLE_MARKERLESS_AR_MACRO_H
#define EXAMPLE_MARKERLESS_AR_MACRO_H

#define TIME_START uint64 time = ::cvGetTickCount();

#define TIME_TEST uint64 time_n = ::cvGetTickCount(); \
                    double gap = ((double)((int)(time_n - time))) / cvGetTickFrequency(); \
                    gap = gap * 1e-3;\
                    std::cout << "gap time = " << gap <<std::endl;

#endif //EXAMPLE_MARKERLESS_AR_MACRO_H
