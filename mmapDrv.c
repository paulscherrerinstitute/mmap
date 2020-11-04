#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <devLib.h>
#include <regDev.h>

#ifdef __unix
 #include <sys/mman.h>
 #include <sys/stat.h>
 #include <sys/sysmacros.h>
 #define HAVE_MMAP
#endif /*__unix */

#ifndef EPICS_3_13
 #include <errlog.h>
 #include <devLibVME.h>
 #include <epicsTypes.h>
 #include <epicsThread.h>
 #include <epicsExport.h>
 #include <epicsEvent.h>
 #include <epicsMutex.h>
 #include <epicsFindSymbol.h>
#else /* 3.13 is vxWorks only */
 #include <sysLib.h>
 #include <intLib.h>
 #include <wdLib.h>
 #include <semLib.h>
 #include <vme.h>
 #include <iv.h>
 #define atVMEA16 VME_AM_USR_SHORT_IO
 #define atVMEA24 VME_AM_STD_SUP_DATA
 #define atVMEA32 VME_AM_EXT_SUP_DATA
 #define atVMECSR VME_AM_CSR
 #define epicsEventId SEM_ID
 #define epicsEventWaitWithTimeout(s,t) semTake(s, t*sysClkRateGet())
 #define epicsEventWaitOK OK
 #define epicsEventMustCreate(i) semBCreate(SEM_Q_FIFO, i)
 #define epicsEventEmpty SEM_EMPTY
 #define epicsMutexId SEM_ID
 #define epicsMutexMustCreate() semMCreate(SEM_DELETE_SAFE|SEM_INVERSION_SAFE|SEM_Q_PRIORITY)
 #define epicsMutexMustLock(sem) semTake(sem, WAIT_FOREVER)
 #define epicsMutexUnlock(sem) semGive(sem)
#endif /* EPICS_3_13 */

#define EPICSVER EPICS_VERSION*10000+EPICS_REVISION*100+EPICS_MODIFICATION
#if EPICSVER < 31409 || (EPICSVER < 31500 && __STDC_VERSION__ < 199901L)
typedef long long epicsInt64;
typedef unsigned long long epicsUInt64;
#endif

/* Try to find dma support */
#ifdef vxWorks
#include <version.h>
#ifndef _WRS_VXWORKS_MAJOR
/* vxWorks 5 */
#define HAVE_DMA
#endif /* vxWorks 5 */
#include <dmaLib.h>
#include <semLib.h>
#include <sysLib.h>
#endif /* vxWorks */

#if defined (__GNUC__) && defined (_ARCH_PPC)
 #define SYNC __asm__("eieio;sync");
#else /* !_ARCH_PPC */
 #define SYNC
#endif /* !_ARCH_PPC */

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif /* O_CLOEXEC */

#define MAGIC 2661166104U /* crc("mmap") */

static char cvsid_mmapDrv[] __attribute__((unused)) =
    "$Id: mmapDrv.c,v 1.18 2015/06/25 14:57:42 zimoch Exp $";

#define INTR_NONE  0
#define INTR_UIO  -2

struct regDevice {
    unsigned long magic;
    const char* name;
    volatile char* localbaseaddress;
    int vmespace;
    unsigned int baseaddress;
    char* intrsource;
    int intrvector;
    int intrlevel;
    int (*intrhandler)(regDevice *device);
    void* userdata;
    IOSCANPVT ioscanpvt;
    unsigned long long intrcount;
    unsigned long long intrmissed;
    unsigned int flags;
    char* devtype;
    char* addrspace;
#ifdef HAVE_DMA
    int maxDmaSpeed;
    epicsEventId dmaComplete;
#endif /* HAVE_DMA */
};

int mmapDebug = 0;

/* Device flags */
#define ALLOW_BLOCK_TRANSFER 0x0000001
#define READONLY_DEVICE      0x0000002
#define SWAP_BYTE_PAIRS      0x0000100
#define SWAP_WORD_PAIRS      0x0000200
#define SWAP_DWORD_PAIRS     0x0000400

/******** Support functions *****************************/

typedef struct mmapIntrInfo {
    struct mmapIntrInfo* next;
    regDevice *device;
    IOSCANPVT ioscanpvt;
    int intrvector;
    int intrlevel;
} mmapIntrInfo;
static mmapIntrInfo* intrInfos = NULL;
static epicsMutexId mmapConnectInterruptLock;

void mmapInterrupt(void *arg)
{
    mmapIntrInfo *info = arg;
    regDevice *device = info->device;
    device->intrcount++;
    if (mmapDebug >= 2)
        printf("mmapInterrupt %s: count = %llu, %s\n",
            device->name, device->intrcount,
            device->intrhandler ? "calling handler" : "no handler installed");
    if (device->intrhandler)
    {
        if (device->intrhandler(device) != 0) return;
    }
    scanIoRequest(info->ioscanpvt);
}

#ifdef HAVE_MMAP

void mmapUioInterruptThread(void* arg)
{
    mmapIntrInfo *info = arg;
    regDevice *device = info->device;
    int fd;
    int n;
    epicsUInt32 intrno = 0;
    epicsUInt32 reenable = 1;
    epicsUInt32 lastnum = 0;
    char devname[9+sizeof(info->intrvector)*2+sizeof(info->intrvector)/2];

    sprintf(devname, "/dev/uio%u", info->intrvector);

    if (mmapDebug)
        printf("mmapUioInterruptThread %s: Starting interrupt handling on %s\n",
            device->name, devname);

    fd = open(devname, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        fd = open(devname, O_RDONLY | O_CLOEXEC);
        if (fd < 0)
        {
            epicsMutexUnlock(mmapConnectInterruptLock);
            errlogSevPrintf(errlogFatal,
                "mmapUioInterruptThread %s: Cannot open %s: %s\n",
                device->name, devname, strerror(errno));
            return;
        }
    }

    if (write(fd, &reenable, 4) == -1)
    {
        if (mmapDebug)
            printf("mmapUioInterruptThread %s: %s does not need re-enable\n",
                device->name, devname);
        reenable = 0;
    }
    while ((n=read(fd, &intrno, 4)) != -1)
    {
        if (mmapDebug >= 2)
            printf("mmapUioInterruptThread %s: interrupt number %u (%d bytes read)\n",
                device->name, n, intrno);

        if (lastnum && intrno != lastnum+1)
        {
            device->intrmissed++;
            if (mmapDebug >= 1)
                printf("mmapUioInterruptThread %s: missed %lld interrupts so far\n",
                    device->name, device->intrmissed);
        }
        lastnum = intrno;

        mmapInterrupt(arg);
        if (reenable) write(fd, &reenable, 4);
    }
    errlogSevPrintf(errlogFatal,
        "mmapUioInterruptThread %s: Interrupt handling %s working on %s.\n",
        device->name, intrno ? "stopped" : "not", devname);
    close(fd);
}

mmapIntrInfo *mmapConnectUioInterrupt(regDevice *device, int uionum)
{
    mmapIntrInfo *info;

    if (mmapDebug)
        printf("mmapConnectUioInterrupt %s: uionum = %d\n",
            device->name, uionum);

    info = calloc(sizeof(mmapIntrInfo), 1);
    if (info)
    {
        info->device = device;
        info->intrlevel = INTR_UIO;
        info->intrvector = uionum;
        scanIoInit(&info->ioscanpvt);
        if (info->ioscanpvt)
        {
            char threadname[9+sizeof(uionum)*2+sizeof(uionum)/2];
            sprintf(threadname, "intr-uio%u", uionum);
            if (epicsThreadCreate(threadname, epicsThreadPriorityMax,
                epicsThreadGetStackSize(epicsThreadStackSmall),
                mmapUioInterruptThread, info))
            {
                return info;
            }
        }
        free(info);
    }
    errlogSevPrintf(errlogFatal,
        "mmapConnectUioInterrupt %s: cannot start uio interrupt handler thread: %s\n",
        device->name, strerror(errno));
    return NULL;
}
#endif /* HAVE_MMAP */

mmapIntrInfo *mmapConnectVmeInterrupt(regDevice *device, int intrvector, int intrlevel)
{
    mmapIntrInfo *info;

    if (mmapDebug)
        printf("mmapConnectVmeInterrupt %s: intrvector = %d intrlevel = %d\n",
            device->name, intrvector, intrlevel);

    if (intrvector >= 256)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConnectVmeInterrupt %s: illegal VME interrupt vector number %i\n",
            device->name, intrvector);
        return NULL;
    }

    if (intrlevel < 1 || intrlevel > 7)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConnectVmeInterrupt %s: illegal VME interrupt level number %i\n",
            device->name, intrvector);
        return NULL;
    }

    info = calloc(sizeof(mmapIntrInfo), 1);
    if (info)
    {
        info->device = device;
        info->intrlevel = intrlevel;
        info->intrvector = intrvector;
        scanIoInit(&info->ioscanpvt);
        if (info->ioscanpvt)
        {
            if (devConnectInterrupt(intVME, intrvector, mmapInterrupt, info) != 0)
            {
                epicsMutexUnlock(mmapConnectInterruptLock);
                errlogSevPrintf(errlogFatal,
                    "mmapConfigure %s: cannot connect to interrupt vector %d\n",
                    device->name, intrvector);
                free(info);
                return NULL;
            }
            if (devEnableInterruptLevel(intVME, intrlevel) != 0)
            {
                epicsMutexUnlock(mmapConnectInterruptLock);
                errlogSevPrintf(errlogMajor,
                    "mmapConfigure %s: cannot enable interrupt level %d\n",
                    device->name, intrlevel);
            }
            return info;
        }
        free(info);
    }
    epicsMutexUnlock(mmapConnectInterruptLock);
    errlogSevPrintf(errlogFatal,
        "mmapConnectUioInterrupt %s: cannot initialize interrupts: %s\n",
        device->name, strerror(errno));
    return NULL;
}

void mmapReport(
    regDevice *device,
    int level)
{
    if (device && device->magic == MAGIC)
    {
        if (device->localbaseaddress)
            printf("mmap %s:0x%x @%p\n",
                device->addrspace, device->baseaddress, device->localbaseaddress);
        else
            printf("mmap %s (no map, interrupt only)\n", device->addrspace);
        if (level > 0 && device->intrvector >= 0)
        {
            printf("     Intr %s count: %llu, missed: %llu\n",
                device->intrsource, device->intrcount, device->intrmissed);
        }
        if (level > 1)
        {
            printf("     Flags: %#x\n", device->flags);
        }
    }
}

#ifdef __linux__
int mmapDevTypeToStr(unsigned int dev, char* pdevname)
{
    char devtype[32];
    FILE* devices = fopen("/proc/devices", "r");
    if (devices)
    {
        unsigned int devnum;

        fscanf(devices, "Character devices:");
        while (fscanf(devices, "%u %31s", &devnum, devtype) == 2)
        {
            if (devnum == dev)
            {
                fclose(devices);
                strcpy(pdevname, devtype);
                return 1;
            }
        }
        fclose(devices);
    }
    pdevname[0] = 0;
    return 0;
}
#endif /* __linux__ */

IOSCANPVT mmapGetInScanPvt(
    regDevice *device,
    size_t offset __attribute__((unused)),
    unsigned int dlen __attribute__((unused)),
    size_t nelm __attribute__((unused)),
    int intrvector,
    const char* user)
{
    mmapIntrInfo *info, **pinfo;
    int intrlevel = device->intrlevel;

    if (!device || device->magic != MAGIC)
    {
        errlogSevPrintf(errlogMajor,
            "mmapGetInScanPvt: illegal device handle\n");
        return NULL;
    }
    if (mmapDebug)
        printf("mmapGetInScanPvt %s: devtype = %s intrvector = %i device->intrvector = %d intrlevel = %d device->intrsource = %s\n",
            device->name, device->devtype, intrvector, device->intrvector, intrlevel, device->intrsource);

    if (intrvector < 0)
    {
        intrvector = device->intrvector;
        if (intrvector < 0) return NULL;
    }

    pinfo = &intrInfos;
again:
    while ((info=(*pinfo)) != NULL)
    {
        if (info->intrlevel == intrlevel && info->intrvector == intrvector)
            return info->ioscanpvt;
        pinfo = &info->next;
    }
    epicsMutexMustLock(mmapConnectInterruptLock);
    if (*pinfo)
    {
        /* someone added something while we were sleeping */
        epicsMutexUnlock(mmapConnectInterruptLock);
        goto again;
    }
#ifdef HAVE_MMAP
    if (intrlevel == INTR_UIO)
        info = mmapConnectUioInterrupt(device, intrvector);
#endif /* HAVE_MMAP */
    if (intrlevel >= 1 && intrlevel <= 7)
        info = mmapConnectVmeInterrupt(device, intrvector, intrlevel);
    *pinfo = info;
    epicsMutexUnlock(mmapConnectInterruptLock);
    if (!info) return NULL;
    return info->ioscanpvt;
}

#define mmapGetOutScanPvt mmapGetInScanPvt

#ifdef HAVE_DMA
static const int dmaModes [] = {DT32, DT64, DT2eVME, DT2eSST160, DT2eSST267, DT2eSST320};
static const char* dmaModeStr [] = {"BLT", "MBLT", "2eVME", "2eSST160", "2eSST267", "2eSST320"};

void mmapCancelDma(int handle)
{
    dmaRequestCancel(handle, FALSE);
}
#endif /* HAVE_DMA */


int mmapRead(
    regDevice *device,
    size_t offset,
    unsigned int dlen,
    size_t nelem,
    void* pdata,
    int prio,
    regDevTransferComplete callback,
    const char* user)
{
    volatile char* src;

    if (!device || device->magic != MAGIC)
    {
        errlogSevPrintf(errlogMajor,
            "mmapRead %s: illegal device handle\n", user);
        return -1;
    }
    if (!device->localbaseaddress)
    {
/*
        if (offset == 0)
        {
            regDevCopy(dlen, 1, device->intrcount, pdata, NULL, 0);
            return 0;
        }
*/
        errlogSevPrintf(errlogMajor,
            "mmapRead %s: device without a map\n", user);
        return -1;
    }

    src = device->localbaseaddress+offset;
#ifdef HAVE_DMA
    /* Try block transfer for long arrays */
    if (nelem >= 1024 &&                          /* inefficient for short arrays */
        (device->flags & ALLOW_BLOCK_TRANSFER) && /* card must be able to do block transfer */
        (((long)pdata|(long)src) & 0x7) == 0)     /* src and dst address must be multiple of 8 */
    {
        unsigned int addrMode;
        unsigned int dmaStatus;

        if (device->maxDmaSpeed == -1) goto noDmaRead;
        switch (device->vmespace)
        {
            case atVMEA16:
                addrMode = AM16;
                break;
            case atVMEA24:
                addrMode = AM24;
                break;
            case atVMEA32:
                addrMode = AM32;
                break;
            default:
                goto noDmaRead;
        }
        if (mmapDebug)
            printf("mmapRead %s %s: block transfer from %p to %p, 0x%"Z"x * %d bit\n",
                user, device->name, device->localbaseaddress+offset, pdata, nelem, dlen*8);
        while (1)
        {
#ifdef dmaTransferRequest_can_wait
            if (dmaTransferRequest(pdata, (unsigned char*) src, nelem*dlen,
                    addrMode, dmaModes[device->maxDmaSpeed], V2C, 100, NULL, NULL, &dmaStatus) != -1)
            {
#else /* !dmaTransferRequest_can_wait */
            int dmaHandle;
            if ((dmaHandle = dmaTransferRequest(pdata, (unsigned char*) src, nelem*dlen,
                    addrMode, dmaModes[device->maxDmaSpeed], V2C, 100,
                    (VOIDFUNCPTR)semGive, device->dmaComplete, &dmaStatus)) != -1)
            {
                if (epicsEventWaitWithTimeout(device->dmaComplete, 1.0) != epicsEventWaitOK)
                {
                    dmaRequestCancel(dmaHandle, TRUE);
                    errlogSevPrintf(errlogMajor,
                        "mmapRead %s %s: block transfer timeout.\n",
                            user, device->name);
                    return -1;
                }
#endif /* !dmaTransferRequest_can_wait */
                if (dmaStatus == DMA_BUSERR && dlen == 8 && device->maxDmaSpeed > 0)
                {
                    /* try again with a slower block transfer speed */
                    if (mmapDebug)
                        printf("mmapRead %s %s: block transfer mode %s failed. trying slower speed\n",
                            user, device->name, dmaModeStr[device->maxDmaSpeed]);
                    device->maxDmaSpeed--;
                    continue;
                }
                if (dmaStatus == DMA_DONE)
                {
                    return 0;
                }
                errlogSevPrintf(errlogMajor,
                    "mmapRead %s %s: DMA %s error (0x%x). Using normal transfer.\n",
                        user, device->name,
                        dmaStatus == DMA_PROERR ? "protocol" :
                        dmaStatus == DMA_BUSERR ? "vme" :
                        dmaStatus == DMA_CPUERR ? "pci" :
                        dmaStatus == DMA_STOP   ? "timeout" : "unknown",
                        dmaStatus);
                goto noDmaRead;
            }
            else
            {
                /* dmaTransferRequest failed because of either
                   * DMA not configured
                   * queue full
                   * unaligned access (already checked before)
                */
                if (mmapDebug)
                    printf("mmapRead %s %s: block transfer mode %s failed\n",
                        user, device->name, dmaModeStr[device->maxDmaSpeed]);
                break;
            }
        }
    }
noDmaRead:
#endif /* HAVE_DMA */
    if (mmapDebug)
        printf("mmapRead %s %s: normal transfer from %p to %p, 0x%"Z"x * %d bit\n",
            user, device->name, device->localbaseaddress+offset, pdata, nelem, dlen*8);
    regDevCopy(dlen, nelem, src, pdata, NULL, 0);
    if (device->flags & SWAP_DWORD_PAIRS && dlen >= 8)
    {
        epicsUInt64* p = pdata;
        size_t i;
        for (i = 0; i < nelem*(dlen/8); i++)
            p[i] = p[i] >> 32 | p[i] << 32;
    }
    if (device->flags & SWAP_WORD_PAIRS && dlen >= 4)
    {
        epicsUInt32* p = pdata;
        size_t i;
        for (i = 0; i < nelem*(dlen/4); i++)
            p[i] = p[i] >> 16 | p[i] << 16;
    }
    if (device->flags & SWAP_BYTE_PAIRS && dlen >= 2)
    {
        epicsUInt16* p = pdata;
        size_t i;
        for (i = 0; i < nelem*(dlen/2); i++)
            p[i] = p[i] >> 8 | p[i] << 8;
    }
    return 0;
}

int mmapWrite(
    regDevice *device,
    size_t offset,
    unsigned int dlen,
    size_t nelem,
    void* pdata,
    void* pmask,
    int prio,
    regDevTransferComplete callback,
    const char* user)
{
    volatile char* dst;

    if (!device || device->magic != MAGIC)
    {
        errlogSevPrintf(errlogMajor,
            "mmapWrite %s: illegal device handle\n", user);
        return -1;
    }
    if (device->flags & READONLY_DEVICE)
    {
        errlogSevPrintf(errlogMajor,
            "mmapWrite %s %s: device is read-only\n", user, device->name);
        return -1;
    }
    if (!device->localbaseaddress)
    {
        errlogSevPrintf(errlogMajor,
            "mmapWrite %s: device without a map\n", user);
        return -1;
    }

    dst = device->localbaseaddress+offset;
#ifdef HAVE_DMA
    /* Try block transfer for long arrays */
    if (pmask == NULL &&                           /* cannot use read-modify-write with DMA */
        nelem >= 1024 &&                          /* inefficient for short arrays */
        (device->flags & ALLOW_BLOCK_TRANSFER) && /* card must be able to do block transfer */
        (((long)pdata|(long)dst) & 0x7) == 0)     /* src and dst address must be multiple of 8 */
    {
        unsigned int addrMode;
        unsigned int dmaStatus;

        if (device->maxDmaSpeed == -1) goto noDmaWrite;
        switch (device->vmespace)
        {
            case atVMEA16:
                addrMode = atVMEA16;
                break;
            case atVMEA24:
                addrMode = atVMEA24;
                break;
            case atVMEA32:
                addrMode = atVMEA32;
                break;
            default:
                goto noDmaWrite;
        }
        if (mmapDebug)
            printf("mmapWrite %s %s: block transfer from %p to %p, 0x%"Z"x * %d bit\n",
                user, device->name, pdata, device->localbaseaddress+offset, nelem, dlen*8);
        while (1)
        {
#ifdef dmaTransferRequest_can_wait
            if ((dmaStatus = dmaTransferRequest((unsigned char*) dst, pdata, nelem*dlen,
                    addrMode, dmaModes[device->maxDmaSpeed], C2V, 100, NULL, NULL, &dmaStatus)) != -1)
            {
#else /* !dmaTransferRequest_can_wait */
            int dmaHandle;
            if ((dmaHandle = dmaTransferRequest((unsigned char*) dst, pdata, nelem*dlen,
                    addrMode, dmaModes[device->maxDmaSpeed], C2V, 100,
                    (VOIDFUNCPTR)semGive, device->dmaComplete, &dmaStatus)) != -1)
            {
                if (epicsEventWaitWithTimeout(device->dmaComplete, 1.0) != epicsEventWaitOK)
                {
                    dmaRequestCancel(dmaHandle, TRUE);
                    errlogSevPrintf(errlogMajor,
                        "mmapWrite %s %s: block transfer transfer timeout.\n",
                            user, device->name);
                    return -1;
                }
#endif /* !dmaTransferRequest_can_wait */
                if (dmaStatus == DMA_BUSERR && dlen == 8 && device->maxDmaSpeed > 0)
                {
                    /* try again with a slower block transfer speed */
                    if (mmapDebug)
                        printf("mmapWrite %s %s: block transfer mode %s failed. trying slower speed\n",
                            user, device->name, dmaModeStr[device->maxDmaSpeed]);
                    device->maxDmaSpeed--;
                    continue;
                }
                if (dmaStatus == DMA_DONE)
                {
                    return 0;
                }
                errlogSevPrintf(errlogMajor,
                    "mmapWrite %s %s: DMA %s error (0x%x). Using normal transfer.\n",
                        user, device->name,
                        dmaStatus == DMA_PROERR ? "protocol" :
                        dmaStatus == DMA_BUSERR ? "vme" :
                        dmaStatus == DMA_CPUERR ? "pci" :
                        dmaStatus == DMA_STOP   ? "timeout" : "unknown",
                        dmaStatus);
                goto noDmaWrite;
            }
            else
            {
                /* dmaTransferRequest failed because of either
                   * DMA not configured
                   * queue full
                   * unaligned access (already checked before)
                */
                if (mmapDebug)
                    printf("mmapWrite %s %s: block transfer mode %s failed\n",
                        user, device->name, dmaModeStr[device->maxDmaSpeed]);
                break;
            }
        }
    }
noDmaWrite:
#endif /* HAVE_DMA */
    if (mmapDebug)
        printf("mmapWrite %s %s: transfer from %p to %p, 0x%"Z"x * %d bit\n",
            user, device->name, pdata, dst, nelem, dlen*8);
    regDevCopy(dlen, nelem, pdata, dst, pmask, 0);
    SYNC
    return 0;
}

static regDevSupport mmapSupport = {
    mmapReport,
    mmapGetInScanPvt,
    mmapGetOutScanPvt,
    mmapRead,
    mmapWrite
};

int mmapIntAckSetBits16(regDevice *device)
{
    size_t offset = (size_t) device->userdata >> 16;
    size_t bits   = (size_t) device->userdata & 0xFFFF;
    *(epicsInt16*)(device->localbaseaddress+offset) |= bits;
    SYNC
    return 0;
}

int mmapIntAckClearBits16(regDevice *device)
{
    size_t offset = (size_t) device->userdata >> 16;
    size_t bits   = (size_t) device->userdata & 0xFFFF;
    *(epicsInt16*)(device->localbaseaddress+offset) &= ~bits;
    SYNC
    return 0;
}

#ifdef HAVE_dmaAlloc
void* mmapDmaAlloc(
    regDevice *device,
    void* old,
    size_t size)
{
    return dmaRealloc(old, size);
}
#endif /* HAVE_dmaAlloc */

/****** startup script configuration function ***********************/

int mmapConfigure(
    const char* name,
    unsigned int baseaddress,
    unsigned int size,
#ifdef vxWorks
    int addrspace,   /* int for compatibility with earliner versions */
    int intrvector,
#define ADDRSPACEFMT "%d"
#define INTRFMT "%d"
#else /* !vxWorks */
    char* addrspace,
    char* intrsource,
#define ADDRSPACEFMT "%s"
#define INTRFMT "%s"
#endif /* !vxWorks */
    int intrlevel,
    int (*intrhandler)(regDevice *),
    void* userdata)
{
    regDevice* device;
    char* localbaseaddress = NULL;
    int vmespace=-2;
    int flags=0;
    char devtype[32] = "";
#ifdef vxWorks
    char intrsource[12] = "";
#else /* !vxWorks */
    int intrvector = -1;
    struct stat sb;
#endif /* !vxWorks */

    if (name == NULL)
    {
        printf("usage: mmapConfigure(\"name\", baseaddress, size, addrspace, intrvector, ilvl)\n");
        printf("maps register block to device \"name\"\n");
        printf("\"name\" must be a unique string on this IOC\n");
#ifdef HAVE_MMAP
        printf("addrspace: device used for mapping (default: /dev/mem)\n");
#endif /* HAVE_MMAP */
#ifdef vxWorks
        printf("addrspace = 0xc, 16, 24 or 32: VME address space (0xc means CSR)"
#else /* !vxWorks */
        printf("addrspace = csr, 16, 24 or 32: VME address space"
#endif /* !vxWorks */
#ifdef HAVE_DMA
                " (+100: allow block transfer)"
#endif /* HAVE_DMA */
                "\n");
        printf("addrspace = sim: simulation on allocated memory\n");
        return 0;
    }

    if (!mmapConnectInterruptLock)
        mmapConnectInterruptLock = epicsMutexMustCreate();

#ifdef vxWorks
    if (intrvector > 0 && intrvector < 256)
        sprintf("vme vec %i", intrsource, intrvector);
    else
        intrvector = -1;
    vmespace = addrspace;
#else /* !vxWorks */
    if (intrsource && intrsource[0])
    {
        char *end;
        intrvector = strtol(intrsource, &end, 0);
        if (end == intrsource)
        {
            intrvector = -1;
#ifdef HAVE_MMAP
            if (mmapDebug)
                printf("mmapConfigure %s: intrvector = %d, intrsource = %s\n",
                    name, intrvector, intrsource);
            if (intrsource && intrsource[0])
            {
                char intrdevtype[32];

                if (mmapDebug)
                    printf("mmapConfigure %s: checking type of %s\n",
                        name, intrsource);
                if (stat(intrsource, &sb) != -1 &&
                    mmapDevTypeToStr(major(sb.st_rdev), intrdevtype))
                {
                    if (mmapDebug)
                        printf("mmapConfigure %s: %s is a %s device\n",
                            name, intrsource, intrdevtype);
                    if (strcmp(intrdevtype, "uio") == 0)
                    {
                        intrvector = minor(sb.st_rdev);
                        intrlevel = INTR_UIO;
                        if (mmapDebug)
                            printf("mmapConfigure %s: default interrupts from uio%d\n",
                                name, intrvector);
                    }
                }
                else
                {
                    errlogSevPrintf(errlogMajor,
                        "mmapConfigure %s: Don't know how to handle interrupts from %s\n",
                            name, intrsource);
                }
            }
#endif /* HAVE_MMAP */
        }
    }

    if (!addrspace || !addrspace[0]) { addrspace="/dev/mem"; }
    sscanf (addrspace, "%i", &vmespace);
    if (strcmp(addrspace, "csr") == 0) { vmespace = 0xc; }
    if (strcmp(addrspace, "sim") == 0) { vmespace = -1; }
#endif /* !vxWorks */
    if (vmespace > 0)
    {
        flags=vmespace/100;
        vmespace%=100;
        if (mmapDebug)
            printf("mmapConfigure %s: vmespace = %d\n",
                name, vmespace);
        switch (vmespace)
        {
            case 0xc:
                vmespace = atVMECSR;
                break;
            case 16:
                vmespace = atVMEA16;
                break;
            case 24:
                vmespace = atVMEA24;
                break;
            case 32:
                vmespace = atVMEA32;
                break;
            default:
                errlogSevPrintf(errlogFatal,
                    "mmapConfigure %s: illegal VME address space "
                    ADDRSPACEFMT " must be 0xc, 16, 24 or 32\n",
                    name, addrspace);
                return -1;
        }
#ifndef EPICS_3_13
        if (!pdevLibVirtualOS)
        {
            errlogSevPrintf(errlogFatal,
                "mmapConfigure %s: no VME support found on this machine\n",
                name);
            return -1;
        }

        /* just make sure that devLibInit has been called (call will fail) */
        devRegisterAddress(NULL, 0, 0, 0, NULL);

        if (pdevLibVirtualOS->pDevMapAddr(vmespace, 0, baseaddress, size, (volatile void **)(volatile char **)&localbaseaddress) != 0)
#else /* EPICS_3_13 */
        if (sysBusToLocalAdrs(vmespace, (char*)baseaddress, &localbaseaddress) != OK)
#endif /* EPICS_3_13 */
        {
            errlogSevPrintf(errlogFatal,
                "mmapConfigure %s: can't map address 0x%08x on "
                ADDRSPACEFMT " address space\n",
                name, baseaddress, addrspace);
            return -1;
        }
    }
    else
    if (vmespace == -1)
    {
        /* Simulation runs on allocated memory */
        localbaseaddress = calloc(1, size);
        if (localbaseaddress == NULL)
        {
            errlogSevPrintf(errlogFatal,
                "mmapConfigure %s: out of memory allocating %d bytes of simulated address space\n",
                name, size);
            return errno;
        }
        if (mmapDebug)
            printf("mmapConfigure %s: simulation @%p\n",
                name, localbaseaddress);
    }
#ifdef HAVE_MMAP
    else
    {
        int fd;
        unsigned long mapstart;
        size_t mapsize;
        char* p;

        if ((p = strchr(addrspace, '&')) != NULL)
        {
            *p++ = 0;
            if (strcasecmp(p, "SwapDwordPairs") == 0) flags |= SWAP_DWORD_PAIRS;
            if (strcasecmp(p, "SwapWordPairs") == 0) flags |= SWAP_WORD_PAIRS;
            if (strcasecmp(p, "SwapBytePairs") == 0) flags |= SWAP_BYTE_PAIRS;
        }

        if (mmapDebug)
            printf("mmapConfigure %s: mmap to %s\n",
                name, addrspace);

        /* round down start address to page size fo mmap() */
        mapstart = baseaddress & ~((off_t)(sysconf(_SC_PAGE_SIZE)-1));
        mapsize = size + (baseaddress - mapstart);

        /* first try to open read/write, create if necessary (and possible) */
        fd = open(addrspace, O_RDWR | O_CREAT | O_CLOEXEC, 0777);
        if (fd < 0)
        {
            /* cannot open R/W or cannot create: try to open readonly */
            fd = open(addrspace, O_RDONLY | O_CLOEXEC);
            if (fd < 0)
            {
                errlogSevPrintf(errlogFatal,
                    "mmapConfigure %s: %s: %s\n",
                    name, addrspace, strerror(errno));
                return errno;
            }
            flags |= READONLY_DEVICE;
            if (mmapDebug)
                printf("mmapConfigure %s: %s is readonly\n",
                    name, addrspace);
        }
        if (mmapDebug)
            printf("mmapConfigure %s: %s gets fd %d\n",
                name, addrspace, fd);

        /* check (regular) file size (if we cannot let's just hope for the best) and grow if necessary */
        if (fstat(fd, &sb) != -1)
        {
            if (S_ISREG(sb.st_mode) && mapsize + mapstart > (size_t)sb.st_size)
            {
                if (mmapDebug)
                    printf("mmapConfigure %s: growing %s from %lu to %lu bytes\n",
                        name, addrspace, sb.st_size, mapsize + mapstart);
                if (ftruncate(fd, mapsize + mapstart) == -1)
                {
                    if (mapstart >= (size_t)sb.st_size)
                    {
                        errlogSevPrintf(errlogFatal,
                            "mmapConfigure %s: %s too small and cannot grow (start > size): %s\n",
                            name, addrspace, strerror(errno));
                        close(fd);
                        return errno;
                    }
                    else
                    {
                        size = sb.st_size - baseaddress;
                        mapsize = sb.st_size - mapstart;
                        errlogSevPrintf(errlogMajor,
                            "mmapConfigure %s: %s too small and cannot grow, shrinking size to %u\n",
                            name, addrspace, size);
                    }
                }
            }
            if (S_ISCHR(sb.st_mode))
            {
                /* test device type if possible */
                if (mmapDevTypeToStr(major(sb.st_rdev), devtype))
                {
                    if (mmapDebug)
                        printf("mmapConfigure %s: device is %s number %d intrvector = %d\n",
                            name, devtype, minor(sb.st_rdev), intrvector);
                    if (strcmp(devtype, "uio") == 0)
                    {
                        if (intrvector < 0) intrvector = minor(sb.st_rdev);
                        intrlevel = INTR_UIO;
                        if (mmapDebug)
                            printf("mmapConfigure %s: default interrupts from uio%d\n",
                                name, intrvector);
                    }
                }
            }
        }
        else
        {
            if (mmapDebug)
                printf("mmapConfigure %s: cannot stat %s: %s\n",
                    name, addrspace, strerror(errno));
        }

        if (size)
        {
            /* map shared with other processes read/write or readonly */
            if (mmapDebug)
                printf("mmapConfigure %s: mmap(NULL, %"Z"u, %s, MAP_SHARED, %d=%s, %ld)\n",
                    name, mapsize, (flags & READONLY_DEVICE) ? "PROT_READ" : "PROT_READ|PROT_WRITE",
                    fd, addrspace, mapstart);

            localbaseaddress = mmap(NULL, mapsize,
                (flags & READONLY_DEVICE) ? PROT_READ : PROT_READ|PROT_WRITE,
                MAP_SHARED, fd, mapstart);
            if (localbaseaddress == MAP_FAILED)
            {
                errlogSevPrintf(errlogFatal,
                    "mmapConfigure %s: can't mmap %s: %s\n",
                    name, addrspace, errno == ENODEV ? "Device does not support mapping" : strerror(errno));
                return errno;
            }
            /* adjust localbaseaddress by the offset within the page */
            if (mmapDebug)
                printf("mmapConfigure %s: mmap returned %p, adjusting by %ld bytes\n",
                    name, localbaseaddress, baseaddress - mapstart);
            localbaseaddress += (baseaddress - mapstart);
        }
        /* we don't need the file descriptor any more */
        close(fd);
    }
#endif /* HAVE_MMAP */
    device = (regDevice*)malloc(sizeof(regDevice));
    if (device == NULL)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConfigure %s: out of memory\n",
            name);
        return errno;
    }
    device->magic = MAGIC;
    device->name = strdup(name);
    device->devtype = strdup(devtype);
    device->vmespace = vmespace;
    device->baseaddress = baseaddress;
    device->localbaseaddress = localbaseaddress;
    device->intrsource = intrsource && intrsource[0] ? strdup(intrsource) : NULL;
    device->intrvector = intrvector;
    device->intrlevel = intrlevel;
    device->intrhandler = intrhandler;
    device->userdata = userdata;
    device->intrcount = 0;
    device->flags = flags;
    switch (vmespace)
    {
        case -1:
            device->addrspace = "sim";
            break;
        case atVMECSR:
            device->addrspace = "CR/CSR";
            break;
        case atVMEA16:
            device->addrspace = "A16";
            break;
        case atVMEA24:
            device->addrspace = "A24";
            break;
        case atVMEA32:
            device->addrspace = "A32";
            break;
        default:
#ifndef vxWorks
            device->addrspace = strdup(addrspace);
#else /* !vxWorks */
            errlogSevPrintf(errlogFatal,
                "mmapConfigure %s: invalid addrspace %d\n",
                name, addrspace);
#endif /* !vxWorks */
    }
    if (mmapDebug)
        printf("mmapConfigure %s: vmespace = %d addrspace = %s\n",
            name, vmespace, device->addrspace);

#ifdef HAVE_DMA
    device->maxDmaSpeed=-1;
    if (vmespace > 0)
    {
        int i;
        for (i = sizeof(dmaModes)/sizeof(dmaModes[0]) - 1; i >= 0; i--)
        {
            if (dmaProtocolSupport(dmaModes[i]) == 0)
            {
                device->maxDmaSpeed = i;
                if (mmapDebug)
                    printf("mmapConfigure %s: maxDmaSpeed=%s\n",
                        name, dmaModeStr[device->maxDmaSpeed]);
                break;
            }
        }
    }
    device->dmaComplete = epicsEventMustCreate(epicsEventEmpty);
#endif /* HAVE_DMA */

    regDevRegisterDevice(name, &mmapSupport, device, size);
#ifdef HAVE_dmaAlloc
    if (vmespace > 0)
        regDevRegisterDmaAlloc(device, mmapDmaAlloc);
#endif /* HAVE_dmaAlloc */
    return 0;
}

#ifndef EPICS_3_13

#include <iocsh.h>
static const iocshArg mmapConfigureArg0 = { "name", iocshArgString };
static const iocshArg mmapConfigureArg1 = { "baseaddress", iocshArgInt };
static const iocshArg mmapConfigureArg2 = { "size", iocshArgInt };
#ifdef vxWorks
static const iocshArg mmapConfigureArg3 = { "addrspace (-1=simulation; 0xc=CSR; 16,24,32=VME,+100=block transfer)", iocshArgInt };
static const iocshArg mmapConfigureArg4 = { "intrvector", iocshArgInt };
#else /* !vxWorks */
static const iocshArg mmapConfigureArg3 = { "mapped device (sim=simulation; csr,16,24,32=VME; default:/dev/mem)", iocshArgString };
static const iocshArg mmapConfigureArg4 = { "intrsource", iocshArgString };
#endif /* !vxWorks */
static const iocshArg mmapConfigureArg5 = { "intrlevel", iocshArgInt };
static const iocshArg mmapConfigureArg6 = { "intrhandler", iocshArgString };
static const iocshArg mmapConfigureArg7 = { "userdata", iocshArgString };
static const iocshArg * const mmapConfigureArgs[] = {
    &mmapConfigureArg0,
    &mmapConfigureArg1,
    &mmapConfigureArg2,
    &mmapConfigureArg3,
    &mmapConfigureArg4,
    &mmapConfigureArg5,
    &mmapConfigureArg6,
    &mmapConfigureArg7
};

static const iocshFuncDef mmapConfigureDef =
    { "mmapConfigure", 8, mmapConfigureArgs };

static void mmapConfigureFunc (const iocshArgBuf *args)
{
    mmapConfigure(
        args[0].sval, args[1].ival, args[2].ival,
#ifdef vxWorks
        args[3].ival, args[4].ival,
#else /* !vxWorks */
        args[3].sval, args[4].sval,
#endif /* !vxWorks */
        args[5].ival,
        args[6].sval ? (int (*)(regDevice *))epicsFindSymbol(args[6].sval) : NULL,
        args[7].sval);
}

static void mmapRegistrar ()
{
    iocshRegister(&mmapConfigureDef, mmapConfigureFunc);
}

epicsExportRegistrar(mmapRegistrar);

#endif /* !EPICS_3_13 */
