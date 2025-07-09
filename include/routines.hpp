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

    inline void make_call(void* addr, void* target)
    {
        DWORD protection;
        constexpr size_t instr_size = 5;

        intptr_t rel = reinterpret_cast<intptr_t>(target) - reinterpret_cast<intptr_t>(addr) - instr_size;

        uint8_t buffer[instr_size];
        buffer[0] = 0xE8;
        *reinterpret_cast<int32_t*>(&buffer[1]) = static_cast<int32_t>(rel);

        VirtualProtect(addr, instr_size, PAGE_EXECUTE_READWRITE, &protection);
        memcpy(addr, buffer, instr_size);
        VirtualProtect(addr, instr_size, protection, &protection);
    };
};

#endif