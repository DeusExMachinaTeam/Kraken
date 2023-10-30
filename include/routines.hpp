#ifndef __KRAKEN_ROUTINES_HPP__
#define __KRAKEN_ROUTINES_HPP__

#include "stdafx.hpp"

namespace kraken::routines {
    #pragma pack(push, 1)
    struct _Redirect {
        char   op;
        size_t to;
    };
    #pragma pack(pop)

    inline void Redirect(size_t size, void* src, void* tar) {
        DWORD protection;

        if (size < sizeof(_Redirect)) {
            throw std::runtime_error("Insuffient space to apply trampoline!");
        }

        VirtualProtect(src, size, PAGE_EXECUTE_READWRITE, &protection);
        memset(src, 0xCC, size);
        _Redirect* op = (_Redirect*) src;
        op->op = 0xE9;
        op->to = (size_t) tar - (size_t) src - 5;
        VirtualProtect(src, size, protection, &protection);
    };

    inline void Override(size_t size, void* src, char* data) {
        DWORD protection;

        VirtualProtect(src, size, PAGE_EXECUTE_READWRITE, &protection);
        memcpy(src, data, size);
        VirtualProtect(src, size, protection, &protection);
    };

    inline void RemapPtr(void* src, void* tar) {
        DWORD protection;
        size_t temp = (size_t) tar;

        VirtualProtect(src, sizeof(size_t), PAGE_EXECUTE_READWRITE, &protection);
        memcpy(src, &temp, sizeof(size_t));
        VirtualProtect(src, sizeof(size_t), protection, &protection);
    };
};

#endif