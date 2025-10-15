// =============================================================================
//  Program : uartboot.c
//  Author  : Chun-Jen Tsai
//  Date    : Nov/12/2021
// -----------------------------------------------------------------------------
//  Description:
//  This is the boot code for Aquila SoC.  Upon reset, the boot code waiting
//  for an ELF program file to be transferred from the UART port.
//  The processor will be halted when the execution is finished.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Sep/17/2022, by Chun-Jen Tsai
//    Modify the ELF loader to perform on-the-fly loading from UART without
//    using any ELF file loading buffer.
//
//  Oct/15/2022, by Chun-Jen Tsai
//    Use different schemes for loading into TCM (on-the-fly) and into
//    DRAM (buffered loading).
//
//  Aug/07/2024, by Chun-Jen Tsai
//    Fix a bug in zero-initialization of .bss sections.
//
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Yang Ming Chiao Tung Uniersity (NYCU)
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
#include <elf.h>
#include <stdint.h>
#include "io_uart.h"

int load_elf(Elf32_Ehdr *ehdr);

// ------------------------------------------------------------------------------
//  Memory Map:
//     0x00000000 ~ 0x0000FFFF: on-chip memory (64KB, boot code)
//     0x80000000 ~ 0x8FFFFFFF: main memory (code, data, heap, and stack)
//     0xC0000000 ~ 0xCFFFFFFF: I/O device area
//     0xF0000000 ~ 0xFFFFFFFF: system device area
// ------------------------------------------------------------------------------

#define LOCK_0 0x1234
uint8_t eheader[64], pheader[128];
char    *organization = "EISL@NYCU, Hsinchu, Taiwan";
int     year = 2025;
volatile unsigned int *prog_ready = (unsigned int *)0x80000000U;
volatile unsigned int *prog = (unsigned int *)0x80000010U;
int main(void)
{
    Elf32_Ehdr *ehdr = (Elf32_Ehdr *) eheader;
    uint32_t *magic = (uint32_t *) ELFMAG;
    uint32_t size;
    int idx;
    *prog_ready = 0;
    int hart_id;
    asm volatile ("csrrs %0, mhartid, x0" :"=r"(hart_id): : );
    if (hart_id == 0) 
    {
        printf("=======================================================================\n");
        printf("Copyright (c) 2019-%d, %s.\n", year, organization);
        printf("The Aquila SoC is ready.\n");
        printf("Core %d is waiting for a program to be sent from the UART ...\n", hart_id);

        // Read the ELF header.
        for (idx = 0; idx < sizeof(Elf32_Ehdr); idx++)
        {
            eheader[idx] = inbyte();
        }

        // Read the Program headers.
        for (idx = 0; idx < ehdr->e_phentsize*ehdr->e_phnum; idx++)
        {
            pheader[idx] = inbyte();
        }

        if (*((uint32_t *) ehdr->e_ident) == *magic)
        {
            *prog = ehdr->e_entry;  /* set program entry point */
            size = ehdr->e_shoff + (ehdr->e_shentsize * ehdr->e_shnum);
            load_elf(ehdr);

            printf("\nProgram entry point at 0x%x, size = 0x%x.\n", *prog, size);
            printf("-----------------------------------------------------------------------\n");

            asm volatile ("fence.i"); // force flushing of I/D caches.
            *prog_ready = LOCK_0;
            register uint32_t entry = *prog;
            asm volatile ("jalr zero, %0, 0" :: "r"(entry));
        }
        else
        {
            printf("\n\nMagic number = 0x%X\n", *((uint32_t *) ehdr->e_ident));
            printf("Error! Not an ELF file.\n\n");
        }
    }
    else
    {
        while(*prog_ready != LOCK_0);
        register uint32_t entry = *prog;
        asm volatile ("jalr zero, %0, 0" :: "r"(entry));
    }

    // Halt the processor.
    exit(0);

    return 0;
}

#define N_ZERO_SECS 8

int load_elf(Elf32_Ehdr *ehdr)
{
    Elf32_Phdr *section;
    uint32_t skip, current_byte;
    uint8_t  *dst_addr;
    int idx, jdx;

    // The info of sections to be initialized to zeros.
    uint32_t *zmem[N_ZERO_SECS];  // Addresses of the sections.
    int zsize[N_ZERO_SECS], zidx; // Sizes of the sectinos.

    current_byte = sizeof(Elf32_Ehdr) + ehdr->e_phentsize*ehdr->e_phnum;

    // Load all loadable sections of an ELF image to the destination.
    zidx = 0;
    section = (Elf32_Phdr *) pheader;
    for (idx = 0; idx < ehdr->e_phnum; idx++)
    {
        // Locate CODE and DATA sections
        if (section[idx].p_type == PT_LOAD && section[idx].p_filesz != 0)
        {
            dst_addr = (uint8_t *) section[idx].p_paddr;
            skip = section[idx].p_offset - current_byte;
            while (skip-- > 0) inbyte(), current_byte++;

            for (jdx = 0; jdx < section[idx].p_filesz; jdx++)
            {
                dst_addr[jdx] = inbyte();
                current_byte++;
            }

            // Record the memory areas that need to be zero-initialized.
            zmem[zidx] = (uint32_t *) &(dst_addr[jdx]);
            zsize[zidx] = (section[idx].p_memsz - jdx)/sizeof(int);
            if (++zidx >= N_ZERO_SECS) break;
        }
    }

    // Set the uninitialized memory areas to all zeros.
    for (idx = 0; idx < zidx; idx++)
    {
        for (jdx = 0; jdx < zsize[idx]; jdx++) zmem[idx][jdx] = 0;
    }
    return 0;
}

