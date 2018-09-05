/*
 * QEMU LSI53C895A SCSI Host Bus Adapter emulation
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the LGPL.
 */

#ifndef LSI_H
#define LSI_H

#include "qemu/osdep.h"

#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/scsi/scsi.h"

/* Maximum length of MSG IN data.  */
#define LSI_MAX_MSGIN_LEN 8

typedef struct lsi_request {
    SCSIRequest *req;
    uint32_t tag;
    uint32_t dma_len;
    uint8_t *dma_buf;
    uint32_t pending;
    int out;
    QTAILQ_ENTRY(lsi_request) next;
} lsi_request;

typedef struct {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    MemoryRegion mmio_io;
    MemoryRegion ram_io;
    MemoryRegion io_io;
    AddressSpace pci_io_as;

    int carry; /* ??? Should this be an a visible register somewhere?  */
    int status;
    /* Action to take at the end of a MSG IN phase.
       0 = COMMAND, 1 = disconnect, 2 = DATA OUT, 3 = DATA IN.  */
    int msg_action;
    int msg_len;
    uint8_t msg[LSI_MAX_MSGIN_LEN];
    /* 0 if SCRIPTS are running or stopped.
     * 1 if a Wait Reselect instruction has been issued.
     * 2 if processing DMA from lsi_execute_script.
     * 3 if a DMA operation is in progress.  */
    int waiting;
    SCSIBus bus;
    int current_lun;
    /* The tag is a combination of the device ID and the SCSI tag.  */
    uint32_t select_tag;
    int command_complete;
    QTAILQ_HEAD(, lsi_request) queue;
    lsi_request *current;

    uint32_t dsa;
    uint32_t temp;
    uint32_t dnad;
    uint32_t dbc;
    uint8_t istat0;
    uint8_t istat1;
    uint8_t dcmd;
    uint8_t dstat;
    uint8_t dien;
    uint8_t sist0;
    uint8_t sist1;
    uint8_t sien0;
    uint8_t sien1;
    uint8_t mbox0;
    uint8_t mbox1;
    uint8_t dfifo;
    uint8_t ctest2;
    uint8_t ctest3;
    uint8_t ctest4;
    uint8_t ctest5;
    uint8_t ccntl0;
    uint8_t ccntl1;
    uint32_t dsp;
    uint32_t dsps;
    uint8_t dmode;
    uint8_t dcntl;
    uint8_t scntl0;
    uint8_t scntl1;
    uint8_t scntl2;
    uint8_t scntl3;
    uint8_t sstat0;
    uint8_t sstat1;
    uint8_t scid;
    uint8_t sxfer;
    uint8_t socl;
    uint8_t sdid;
    uint8_t ssid;
    uint8_t sfbr;
    uint8_t stest1;
    uint8_t stest2;
    uint8_t stest3;
    uint8_t sidl;
    uint8_t stime0;
    uint8_t respid0;
    uint8_t respid1;
    uint32_t mmrs;
    uint32_t mmws;
    uint32_t sfs;
    uint32_t drs;
    uint32_t sbms;
    uint32_t dbms;
    uint32_t dnad64;
    uint32_t pmjad1;
    uint32_t pmjad2;
    uint32_t rbc;
    uint32_t ua;
    uint32_t ia;
    uint32_t sbc;
    uint32_t csbc;
    uint32_t scratch[18]; /* SCRATCHA-SCRATCHR */
    uint8_t sbr;
    uint32_t adder;

    /* Script ram is stored as 32-bit words in host byteorder.  */
    uint32_t script_ram[2048];
} LSIState;

#define TYPE_LSI53C810  "lsi53c810"
#define LSI53C810(obj) \
    OBJECT_CHECK(LSIState, (obj), TYPE_LSI53C810)

#define TYPE_LSI53C895A "lsi53c895a"
#define LSI53C895A(obj) \
    OBJECT_CHECK(LSIState, (obj), TYPE_LSI53C895A)

#endif
