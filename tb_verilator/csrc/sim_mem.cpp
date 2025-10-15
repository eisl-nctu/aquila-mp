// This code is modified https://github.com/riscv/riscv-isa-sim/blob/master/fesvr/elfloader.cc
// See LICENSE in https://github.com/riscv/riscv-isa-sim for license details.
#include "sim_mem.h"
#include <elf.h>
#include <string>
#include <cstring>
#include <cassert>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#define bswap from_le

//X86 machine
template<typename T> static inline T from_le(T n) { return n; }

void sim_mem_write(Vaquila_testharness_dp_ram* ram, uint32_t addr, size_t length, const void* bytes)
{
    int* data = (int *)bytes;
    //out of boundary protection?
    for (int i = 0 ; i < length ; i +=4 ) {
        ram->writeWord(addr+i, *((uint8_t *)bytes+i));
    /*     ram->writeByte(addr+i,*((unsigned char*)bytes+i+3)); */
    /*     ram->writeByte(addr+i+1,*((unsigned char*)bytes+i+2)); */
    /*     ram->writeByte(addr+i+2,*((unsigned char*)bytes+i+1)); */
    /*     ram->writeByte(addr+i+3,*((unsigned char*)bytes+i)); */
    }
}

/* load binary executable to mock memory */
void sim_mem_load_bin_program(Vaquila_testharness_dp_ram* ram, const std::string fn, uint32_t* entry)
{
    std::ifstream bin(fn, std::ios::binary | std::ios::ate);
    if (!bin.is_open()) {
        std::cerr << "failed to open " << fn << std::endl;
    }
    std::ifstream::pos_type pos = bin.tellg();
    int f_length = pos;
    char* buf = new char[f_length];
    bin.seekg(0, std::ios::beg);
    bin.read(buf, f_length);
    uint32_t *p = (uint32_t *)buf;
    uint32_t dst_addr = 0x80400000;
    *entry = 0x80400000;
    for (int i = 0; i < f_length / 4; i++) {
        ram->writeWord(dst_addr, *p);
        dst_addr += 4;
        p++;
    }
    printf("send_done\n");
}

/* load elf to mock memory */
std::map<std::string, uint64_t> sim_mem_load_program(Vaquila_testharness_dp_ram* ram, const std::string fn, uint32_t* entry)
{
    std::map<std::string, uint64_t> symbols;
    // https://codereview.stackexchange.com/questions/22901/reading-all-bytes-from-a-file
    std::ifstream bpfs(fn, std::ios::binary|std::ios::ate);
    if (!bpfs.is_open())
        return symbols;
    std::ifstream::pos_type pos = bpfs.tellg();

    int f_length = pos;

    char* buf = new char[f_length];
    bpfs.seekg(0, std::ios::beg);
    bpfs.read(buf, f_length);
    bpfs.close();

    size_t size = f_length;

    uint32_t *magic = (uint32_t *)ELFMAG;
    uint8_t *src;
    uint32_t dst_addr;

    uint32_t i, j;
    std::cout << "file size : " << size << std::endl;

    std::vector<uint8_t> zeros;

    /* Read the ELF header. */
    Elf32_Ehdr* eh = (Elf32_Ehdr*)buf;
    Elf32_Phdr* ph = (Elf32_Phdr*)(buf + bswap(eh->e_phoff));
    *entry = bswap(eh->e_entry);
    if (*((uint32_t *)eh->e_ident) == *magic) {
        std::cout << "OK, is ELF file. " << std::endl;
        for (i = 0; i < bswap(eh->e_phnum); i++) {
            if (bswap(ph[i].p_type) == PT_LOAD && bswap(ph[i].p_filesz != 0)) {
                src = (uint8_t *)(buf + bswap(ph[i].p_offset));
                dst_addr = (uint32_t)bswap(ph[i].p_paddr); 
                for (j = 0; j < bswap(ph[i].p_filesz); j+=sizeof(int)) {
                    /* *(uint32_t *)(dst_addr + j) = *(uint32_t *)(src_addr + j); */ 
                    uint32_t addr = dst_addr + j;
                    uint32_t data = *(uint32_t *)(src + j);
                    ram->writeWord(addr, data); 
                }
                while (j < bswap(ph[i].p_memsz)) {
                    /* *(uint32_t *)(dst_addr + j) = 0; */
                    uint32_t addr = dst_addr + j;
                    uint32_t data = 0;
                    ram->writeWord(addr, 0);
                    j += sizeof(int);
                }
            }
        }
    } else {
        std::cout << "\n\nMagic number = 0x" << *((uint32_t *)eh->e_ident) << std::endl;
        std::cout << "Error! Not an ELF file." << std::endl << std::endl; 
    }
    
    /* do { */
    /*     Elf32_Ehdr* eh = (Elf32_Ehdr*)buf; */
    /*     Elf32_Phdr* ph = (Elf32_Phdr*)(buf + bswap(eh->e_phoff)); */
    /*     *entry = bswap(eh->e_entry); */
    /*     assert( size >= bswap(eh->e_phoff) + bswap(eh->e_phnum)*sizeof(*ph)); */
    /*     for (unsigned i = 0; i < bswap(eh->e_phnum); i++) { */
    /*         if(bswap(ph[i].p_type) == PT_LOAD && bswap(ph[i].p_memsz)) { */
    /*             if (bswap(ph[i].p_filesz)) { */
    /*                 assert(size >= bswap(ph[i].p_offset) + bswap(ph[i].p_filesz)); */
    /*                 sim_mem_write(ram,bswap(ph[i].p_paddr), bswap(ph[i].p_filesz), (uint8_t*)buf + bswap(ph[i].p_offset)); */
    /*             } */
    /*             zeros.resize(bswap(ph[i].p_memsz) - bswap(ph[i].p_filesz)); */
    /*             sim_mem_write(ram, bswap(ph[i].p_paddr) + bswap(ph[i].p_filesz), bswap(ph[i].p_memsz) - bswap(ph[i].p_filesz), &zeros[0]); */
    /*         } */
    /*     } */
    /*     Elf32_Shdr* sh = (Elf32_Shdr*)(buf + bswap(eh->e_shoff)); */
    /*     assert(size >= bswap(eh->e_shoff) + bswap(eh->e_shnum)*sizeof(*sh)); */
    /*     assert(bswap(eh->e_shstrndx) < bswap(eh->e_shnum)); */
    /*     assert(size >= bswap(sh[bswap(eh->e_shstrndx)].sh_offset) + bswap(sh[bswap(eh->e_shstrndx)].sh_size)); */

    /*     char *shstrtab = buf + bswap(sh[bswap(eh->e_shstrndx)].sh_offset); */
    /*     unsigned strtabidx = 0, symtabidx = 0; */
    /*     for (unsigned i = 0; i < bswap(eh->e_shnum); i++) { */
    /*         unsigned max_len = bswap(sh[bswap(eh->e_shstrndx)].sh_size) - bswap(sh[i].sh_name); */
    /*         assert(bswap(sh[i].sh_name) < bswap(sh[bswap(eh->e_shstrndx)].sh_size)); */
    /*         assert(strnlen(shstrtab + bswap(sh[i].sh_name), max_len) < max_len); */
    /*         if (bswap(sh[i].sh_type) & SHT_NOBITS) continue; */
    /*         assert(size >= bswap(sh[i].sh_offset) + bswap(sh[i].sh_size)); */
    /*         if (strcmp(shstrtab + bswap(sh[i].sh_name), ".strtab") == 0) */
    /*             strtabidx = i; */
    /*         if (strcmp(shstrtab + bswap(sh[i].sh_name), ".symtab") == 0) */
    /*             symtabidx = i; */
    /*     } */
    /*     if (strtabidx && symtabidx) { */
    /*         char* strtab = buf + bswap(sh[strtabidx].sh_offset); */
    /*         Elf32_Sym* sym = (Elf32_Sym*)(buf + bswap(sh[symtabidx].sh_offset)); */
    /*         for (unsigned i = 0; i < bswap(sh[symtabidx].sh_size)/sizeof(Elf32_Sym); i++) { */
    /*             unsigned max_len = bswap(sh[strtabidx].sh_size) - bswap(sym[i].st_name); */
    /*             assert(bswap(sym[i].st_name) < bswap(sh[strtabidx].sh_size)); */
    /*             assert(strnlen(strtab + bswap(sym[i].st_name), max_len) < max_len); */
                /* symbols[strtab + bswap(sym[i].st_name)] = bswap(sym[i].st_value); */
            /* } */
        /* } */
    /* } while(0); */
    return symbols;
}

int sim_mem_dump_memory(Vaquila_testharness_dp_ram* ram, const std::string fn)
{
    std::fstream fs(fn, std::fstream::out);
    if (!fs.is_open()){
        std::cerr << "failed to open " << fn << " to write memory data.!!!" << std::endl;
        return -1;
    }
    int mem_size = ram->MEM_SIZE;
    unsigned int mem_offset = ram->MEM_OFFSET;
    std::cout << "memory size = " << mem_size << std::endl;

    for (int i = 0 ; i < mem_size; i += 4) {
        /* std::cout << "i : " << i << std::endl; */
        fs << "0x" << std::setfill('0') << std::setw(8)
            << std::right << std::hex << i + mem_offset << ":  0x";
        fs << std::setfill('0') << std::setw(2) << std::right
            << std::hex << (unsigned int)ram->mem[i / 4];
        fs << std::endl;
    }

    std::cout << "dump memory to \"" << fn << "\" finished!" << std::endl;
    fs.close();
    return 0;
}

uint32_t sim_mem_tohost_monitor(Vaquila_testharness_dp_ram* ram, uint64_t tohost_addr)
{
    uint32_t mem_offset = ram->MEM_OFFSET;
    uint32_t data = 0;
    data = (ram->mem[tohost_addr-mem_offset] & 0xff);
    data = data | (ram->mem[tohost_addr+1-mem_offset] & 0xff) << 8;
    data = data | (ram->mem[tohost_addr+2-mem_offset] & 0xff) << 16;
    data = data | (ram->mem[tohost_addr+3-mem_offset] & 0xff) << 24;
    return data;
}
