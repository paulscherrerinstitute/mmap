#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <devLib.h>
#include <regDev.h>

#ifdef __unix
#include <unistd.h>
 #include <sys/mman.h>
 #include <sys/stat.h>
 #include <sys/sysmacros.h>
 #define HAVE_MMAP
#endif /*__unix */

#ifdef __linux__
 #define HAVE_UIO
 #include <glob.h>
#endif

#ifndef EPICS_3_13
 #include <errlog.h>
 #include <devLibVME.h>
 #include <epicsTypes.h>
 #include <epicsThread.h>
 #include <epicsEvent.h>
 #include <epicsMutex.h>
 #include <epicsFindSymbol.h>
 #include <epicsStdioRedirect.h>
 #include <epicsExport.h>
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
#define HAVE_DMA
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

#ifndef __GNUC__
#define __attribute__(x)
#define strcasecmp strcmp
#endif

#define MAGIC 2661166104U /* crc("mmap") */

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
#define ALLOW_DMA            0x0000001
#define BLOCK_DEVICE         0x0000002
#define MAP_DEVICE           0x0000004
#define READONLY_DEVICE      0x0000080
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
    unsigned long long intrcount;
#ifdef HAVE_UIO
    unsigned long long intrmissed;
    int uiofd;
    char uioname[1];
#endif
} mmapIntrInfo;
static mmapIntrInfo* intrInfos = NULL;
static epicsMutexId mmapConnectInterruptLock;

void mmapInterrupt(void *arg)
{
    mmapIntrInfo *info = arg;
    regDevice *device = info->device;
    info->intrcount++;
    if (mmapDebug >= 2)
        printf("mmapInterrupt %s: vector %d %s count = %llu, %s\n",
            device->name, device->intrvector,
#ifdef HAVE_UIO
            info->uioname,
#else
            "",
#endif
            info->intrcount,
            device->intrhandler ? "calling handler" : "no handler installed");
    if (device->intrhandler)
    {
        if (device->intrhandler(device) != 0) return;
    }
    scanIoRequest(info->ioscanpvt);
}

#ifdef HAVE_UIO

void mmapUioInterruptThread(void* arg)
{
    mmapIntrInfo *info = arg;
    regDevice *device = info->device;
    int fd = info->uiofd;
    int n;
    epicsUInt32 intrno = 0;
    epicsUInt32 reenable = 1;
    epicsUInt32 lastnum = 0;

    if (write(fd, &reenable, 4) == -1)
    {
        if (mmapDebug)
            printf("mmapUioInterruptThread %s: %s does not need re-enable.\n",
                device->name, info->uioname);
        reenable = 0;
    }
    while ((n=read(fd, &intrno, 4)) != -1)
    {
        if (mmapDebug >= 2)
            printf("mmapUioInterruptThread %s: Interrupt number %u (%d bytes read).\n",
                device->name, n, intrno);

        if (lastnum && intrno != lastnum+1)
        {
            info->intrmissed++;
            if (mmapDebug >= 1)
                printf("mmapUioInterruptThread %s: Missed %lld interrupts so far.\n",
                    device->name, info->intrmissed);
        }
        lastnum = intrno;

        mmapInterrupt(arg);
        if (reenable) write(fd, &reenable, 4);
    }
    errlogSevPrintf(errlogFatal,
        "mmapUioInterruptThread %s: Interrupt handling %s working on %s.\n",
        device->name, intrno ? "stopped" : "not", info->uioname);
    close(fd);
}

mmapIntrInfo *mmapConnectUioInterrupt(const char* user, regDevice *device, int uionum)
{
#define THREADNAMESTRING "Iuio"
#define GLOBSTRING       "/dev/uio{*[!0-9],}"

    mmapIntrInfo *info = NULL;
    char threadname[sizeof(THREADNAMESTRING)+sizeof(uionum)*2+sizeof(uionum)/2];
    char globpattern[sizeof(GLOBSTRING)+sizeof(info->intrvector)*2+sizeof(info->intrvector)/2];
    int globstatus;
    glob_t globresults;
    char *uioname;
    int fd;

    if (mmapDebug)
        printf("mmapConnectUioInterrupt %s %s: uionum = %d\n",
            user, device->name, uionum);

    sprintf(globpattern, GLOBSTRING "%u", uionum);

    if (mmapDebug)
        printf("mmapConnectUioInterrupt %s %s: glob \"%s\"\n",
            user, device->name, globpattern);

    globstatus = glob(globpattern, GLOB_BRACE, NULL, &globresults);
    if (globstatus == GLOB_NOMATCH) {
        errlogSevPrintf(errlogFatal,
            "mmapConnectUioInterrupt %s %s: No uio number %u found.\n",
            user, device->name, uionum);
        goto fail;
    }
    if (globstatus != 0) {
        errlogSevPrintf(errlogFatal,
            "mmapConnectUioInterrupt %s %s: glob \"%s\" failed.\n",
            user, device->name, globpattern);
        goto fail;
    }
    uioname = globresults.gl_pathv[0];
    if (mmapDebug)
        printf("mmapConnectUioInterrupt %s %s: Found %s\n",
            user, device->name, uioname);
    fd = open(uioname, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        fd = open(uioname, O_RDONLY | O_CLOEXEC);
        if (fd < 0) {
            errlogSevPrintf(errlogFatal,
                "mmapConnectUioInterrupt %s %s: Cannot open %s: %s\n",
                user, device->name, uioname, strerror(errno));
            goto fail;
        }
        if (mmapDebug)
            printf("mmapConnectUioInterrupt %s %s: %s is readonly.\n",
                user, device->name, uioname);
    }

    info = calloc(sizeof(mmapIntrInfo) + strlen(uioname), 1);
    if (!info) {
        errlogSevPrintf(errlogFatal,
            "mmapConnectUioInterrupt %s %s: Out of memory.\n",
            user, device->name);
        goto fail;
    }
    info->device = device;
    info->intrlevel = INTR_UIO;
    info->intrvector = uionum;
    info->uiofd = fd;
    strcpy(info->uioname, uioname);

    scanIoInit(&info->ioscanpvt);
    if (!info->ioscanpvt) {
        errlogSevPrintf(errlogFatal,
            "mmapConnectUioInterrupt %s %s: scanIoInit failed: %s\n",
            user, device->name, strerror(errno));
        goto fail;
    }

    sprintf(threadname, THREADNAMESTRING "%u", uionum);
    if (mmapDebug)
        printf("mmapConnectUioInterrupt %s %s: Starting interrupt thread %s for %s.\n",
            user, device->name, threadname, uioname);

    if (!epicsThreadCreate(threadname, epicsThreadPriorityMax,
        epicsThreadGetStackSize(epicsThreadStackSmall),
        mmapUioInterruptThread, info))
    {
        errlogSevPrintf(errlogFatal,
            "mmapConnectUioInterrupt %s %s: epicsThreadCreate failed: %s\n",
            user, device->name, strerror(errno));
        goto fail;
    }
    globfree(&globresults);
    return info;
fail:
    free(info);
    globfree(&globresults);
    return NULL;
}
#endif /* HAVE_UIO */

mmapIntrInfo *mmapConnectVmeInterrupt(const char* user, regDevice *device, int intrvector, int intrlevel)
{
    mmapIntrInfo *info;

    if (intrvector > 0xff)
    {
        intrlevel = intrvector >> 8;
        intrvector &= 0xff;
    }

    if (mmapDebug)
        printf("mmapConnectVmeInterrupt %s %s: intrvector=%#x intrlevel = %d\n",
            user, device->name, intrvector, intrlevel);

    if (!intrlevel)
    {
        errlogSevPrintf(errlogMajor,
            "mmapConnectVmeInterrupt %s %s: Warning: No VME interrupt level given. Interrupts for vector %#x may not be enabled.\n",
            user, device->name, intrvector);
    }
    else if (intrlevel < 1 || intrlevel > 7)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConnectVmeInterrupt %s %s: Invalid VME interrupt level %i.\n",
            user, device->name, intrlevel);
        return NULL;
    }
    if (intrvector < 1 || intrlevel > 255)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConnectVmeInterrupt %s %s: Invalid VME interrupt vector %#x.\n",
            user, device->name, intrlevel);
        return NULL;
    }

    info = calloc(sizeof(mmapIntrInfo), 1);
    if (!info) {
        errlogSevPrintf(errlogFatal,
            "mmapConnectUioInterrupt %s %s: Out of memory.\n",
            user, device->name);
        return NULL;
    }

    info->device = device;
    info->intrlevel = intrlevel;
    info->intrvector = intrvector;
    scanIoInit(&info->ioscanpvt);

    if (!info->ioscanpvt)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConnectVmeInterrupt %s %s: scanIoInit failed: %s\n",
            user, device->name, strerror(errno));
        free(info);
        return NULL;
    }

    if (devConnectInterrupt(intVME, intrvector, mmapInterrupt, info) != 0)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConnectVmeInterrupt %s %s: Cannot connect to VME interrupt vector %#x.\n",
            user, device->name, intrvector);
        free(info);
        return NULL;
    }

    if (intrlevel && devEnableInterruptLevel(intVME, intrlevel) != 0)
    {
        errlogSevPrintf(errlogMajor,
            "mmapConnectVmeInterrupt %s %s: Warning: Cannot enable VME interrupt level %d.\n",
            user, device->name, intrlevel);
    }
    return info;
}

void mmapReport(
    regDevice *device,
    int level)
{
    if (device && device->magic == MAGIC)
    {
        if (device->localbaseaddress)
            printf("mmap %s:0x%x @%p",
                device->addrspace, device->baseaddress, device->localbaseaddress);
        else
            printf("mmap %s (no map)", device->addrspace);

        if (device->intrvector >= 0)
            printf(" intr=%s", device->intrsource);
        if (device->flags & ALLOW_DMA)
            printf(" dma");
        if (device->flags & READONLY_DEVICE)
            printf(" ro");
        if (device->flags & SWAP_BYTE_PAIRS)
            printf(" sb");
        if (device->flags & SWAP_WORD_PAIRS)
            printf(" sw");
        if (device->flags & SWAP_DWORD_PAIRS)
            printf(" sd");
        printf("\n");
        if (level > 0)
        {
            mmapIntrInfo *info;
            for(info = intrInfos; info; info = info->next)
            {
                if (info->device == device)
                {
#ifdef HAVE_UIO
                    if (info->intrlevel == INTR_UIO)
                        printf("    intr %d (%s) count: %llu, missed: %llu\n",
                            info->intrvector, info->uioname,
                            info->intrcount, info->intrmissed);
                    else
#endif
                    printf("    intr %d level %d count: %llu\n",
                            info->intrvector, info->intrlevel,
                            info->intrcount);
                }
            }
        }
        if (level > 1)
        {
            printf("     flags: %#x\n", device->flags);
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
    mmapIntrInfo *info;
    mmapIntrInfo *volatile *pinfo;
    int intrlevel = device->intrlevel;

    if (!device || device->magic != MAGIC)
    {
        errlogSevPrintf(errlogMajor,
            "mmapGetInScanPvt %s: Invalid device handle\n", user);
        return NULL;
    }
    if (mmapDebug)
        printf("mmapGetInScanPvt %s: %s devtype=%s intrvector=%d=%#x default-intrvector=%d intrlevel=%d default-intrsource=%s\n",
            user, device->name, device->devtype, intrvector, intrvector, device->intrvector, intrlevel, device->intrsource);

    if (intrvector < 0)
    {
        intrvector = device->intrvector;
        if (intrvector < 0)
        {
            errlogSevPrintf(errlogMajor,
                "mmapGetInScanPvt %s: Cannot do I/O Intr without interrupt vector defined.\n",
                user);
            return NULL;
        }
    }

    pinfo = &intrInfos;
    while (1) {
        while ((info=(*pinfo)) != NULL)
        {
            if (info->intrlevel == intrlevel && info->intrvector == intrvector)
                return info->ioscanpvt;
            pinfo = &info->next;
        }
        /* Lock before adding */
        epicsMutexMustLock(mmapConnectInterruptLock);
        /* has someone added something while we were sleeping ? */
        if (!*pinfo) break;
        epicsMutexUnlock(mmapConnectInterruptLock);
    }
#ifdef HAVE_UIO
    info = mmapConnectUioInterrupt(user, device, intrvector);
    if (!info)
#endif /* HAVE_UIO */
    info = mmapConnectVmeInterrupt(user, device, intrvector, intrlevel);
    *pinfo = info;
    epicsMutexUnlock(mmapConnectInterruptLock);
    if (!info)
        return NULL;
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
            "mmapRead %s: Invalid device handle.\n", user);
        return -1;
    }
    if (!device->localbaseaddress)
    {
        errlogSevPrintf(errlogMajor,
            "mmapRead %s %s: Device has no memory map.\n", user, device->name);
        return -1;
    }

    src = device->localbaseaddress+offset;
    if (pdata == src)
    {
        if (mmapDebug)
            printf("mmapRead %s %s: Direct map, no copy needed.\n",
                user, device->name);
        return 0;
    }
#ifdef HAVE_DMA
    /* Try DMA for long arrays */
    if (nelem >= 1024 &&                          /* inefficient for short arrays */
        (device->flags & ALLOW_DMA) &&            /* hardware must be able to do DMA */
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
            printf("mmapRead %s %s: DMA transfer from %p to %p, 0x%"Z"x * %d bit\n",
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
                        "mmapRead %s %s: DMA timeout.\n",
                            user, device->name);
                    return -1;
                }
#endif /* !dmaTransferRequest_can_wait */
                if (dmaStatus == DMA_BUSERR && dlen == 8 && device->maxDmaSpeed > 0)
                {
                    /* try again with a slower DMA speed */
                    if (mmapDebug)
                        printf("mmapRead %s %s: DMA mode %s failed. Trying slower speed.\n",
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
                    printf("mmapRead %s %s: DMA mode %s failed.\n",
                        user, device->name, dmaModeStr[device->maxDmaSpeed]);
                break;
            }
        }
    }
noDmaRead:
#endif /* HAVE_DMA */
    if (mmapDebug)
        printf("mmapRead %s %s: Normal transfer from %p to %p, 0x%"Z"x * %d bit\n",
            user, device->name, device->localbaseaddress+offset, pdata, nelem, dlen*8);
    regDevCopy(dlen, nelem, src, pdata, NULL, 0);
    if (device->flags & SWAP_DWORD_PAIRS)
    {
        epicsUInt64* p = pdata;
        size_t i;
        for (i = 0; i < nelem*dlen/8; i++)
            p[i] = p[i] >> 32 | p[i] << 32;
    }
    if (device->flags & SWAP_WORD_PAIRS)
    {
        epicsUInt32* p = pdata;
        size_t i;
        for (i = 0; i < nelem*dlen/4; i++)
            p[i] = p[i] >> 16 | p[i] << 16;
    }
    if (device->flags & SWAP_BYTE_PAIRS)
    {
        epicsUInt16* p = pdata;
        size_t i;
        for (i = 0; i < nelem*dlen/2; i++)
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
            "mmapWrite %s: Invalid device handle.\n", user);
        return -1;
    }
    if (device->flags & READONLY_DEVICE)
    {
        errlogSevPrintf(errlogMajor,
            "mmapWrite %s %s: Device is read-only.\n", user, device->name);
        return -1;
    }
    if (!device->localbaseaddress)
    {
        errlogSevPrintf(errlogMajor,
            "mmapWrite %s %s: Device has no memory map.\n", user, device->name);
        return -1;
    }

    dst = device->localbaseaddress+offset;
    if (pdata == dst)
    {
        if (mmapDebug)
            printf("mmapWrite %s %s: Direct map, no copy needed.\n",
                user, device->name);
        return 0;
    }
#ifdef HAVE_DMA
    /* Try DMA for long arrays */
    if (pmask == NULL &&                          /* cannot use read-modify-write with DMA */
        nelem >= 1024 &&                          /* inefficient for short arrays */
        (device->flags & ALLOW_DMA) &&            /* hardware must be able to do DMA */
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
            printf("mmapWrite %s %s: DMA transfer from %p to %p, 0x%"Z"x * %d bit\n",
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
                        "mmapWrite %s %s: DMA timeout.\n",
                            user, device->name);
                    return -1;
                }
#endif /* !dmaTransferRequest_can_wait */
                if (dmaStatus == DMA_BUSERR && dlen == 8 && device->maxDmaSpeed > 0)
                {
                    /* try again with a slower DMA speed */
                    if (mmapDebug)
                        printf("mmapWrite %s %s: DMA mode %s failed. Trying slower speed.\n",
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
                    printf("mmapWrite %s %s: DMA mode %s failed.\n",
                        user, device->name, dmaModeStr[device->maxDmaSpeed]);
                break;
            }
        }
    }
noDmaWrite:
#endif /* HAVE_DMA */
    if (mmapDebug)
        printf("mmapWrite %s %s: Transfer from %p to %p, 0x%"Z"x * %d bit\n",
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
    int vmespace = -2;
    int flags = 0;
    char devtype[32] = "";
#ifdef vxWorks
    char intrsource[32] = "";
#else /* !vxWorks */
    int intrvector = -1;
#ifdef HAVE_MMAP
    struct stat sb;
#endif
    int missingIntrSevr = errlogFatal;
#endif /* !vxWorks */

    if (name == NULL)
    {
        printf("usage: mmapConfigure(\"name\", baseaddress, size, addrspace, intrvector, intrlevel)\n");
        printf("maps register/memory block to device \"name\"\n");
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
                " (+100: allow DMA for large arrays)"
#endif /* HAVE_DMA */
                " (+200: enable regDev block mode)"
                "\n");
        printf("addrspace = sim: simulation on allocated memory\n");
        return 0;
    }

    if (!mmapConnectInterruptLock)
        mmapConnectInterruptLock = epicsMutexMustCreate();

#ifdef vxWorks
    if (intrvector > 0 && intrvector < 256)
        sprintf(intrsource, "VME vector %x level %i", intrvector, intrlevel);
    else
        intrvector = -1;
    vmespace = addrspace;
#else /* !vxWorks */
    if (addrspace) {
        char* thisflag;
        char* nextflag;
        nextflag = strchr(addrspace, '&');
        if (nextflag) *nextflag++ = 0;
        while (nextflag) {
            thisflag = nextflag;
            nextflag = strpbrk(thisflag, "&|,;+ ");
            if (nextflag) *nextflag++ = 0;
            if (strcasecmp(thisflag, "SwapDwordPairs") == 0) flags ^= SWAP_DWORD_PAIRS;
            else if (strcasecmp(thisflag, "SwapWordPairs") == 0) flags ^= SWAP_WORD_PAIRS;
            else if (strcasecmp(thisflag, "SwapBytePairs") == 0) flags ^= SWAP_BYTE_PAIRS;
            else if (strcasecmp(thisflag, "SwapWords") == 0) flags ^= SWAP_BYTE_PAIRS;
            else if (strcasecmp(thisflag, "SwapDWords") == 0) flags ^= SWAP_BYTE_PAIRS|SWAP_WORD_PAIRS;
            else if (strcasecmp(thisflag, "SwapQWords") == 0) flags ^= SWAP_BYTE_PAIRS|SWAP_WORD_PAIRS|SWAP_DWORD_PAIRS;
#ifdef HAVE_DMA
            else if (strcasecmp(thisflag, "dma") == 0) flags |= ALLOW_DMA;
#endif
            else if (strcasecmp(thisflag, "block") == 0) flags |= BLOCK_DEVICE;
            else if (strcasecmp(thisflag, "map") == 0) flags |= MAP_DEVICE|BLOCK_DEVICE;
            else fprintf(stderr, "Unknown flag %s\n", thisflag);
        }
    }
    if (!intrsource || !intrsource[0])
    {
        intrsource = addrspace;
        if (size > 0) missingIntrSevr = errlogMajor;
    }

    if (intrsource && intrsource[0])
    {
        char *end;
        intrvector = strtol(intrsource, &end, 0);
        if (end == intrsource)
        {
            intrvector = -1;
#ifdef __linux__
            char intrdevtype[32];

            if (mmapDebug)
                printf("mmapConfigure %s: intrvector = %d, intrsource = %s\n",
                    name, intrvector, intrsource);

            if (mmapDebug)
                printf("mmapConfigure %s: checking type of %s\n",
                    name, intrsource);
            if (stat(intrsource, &sb) < 0)
            {
                if (missingIntrSevr == errlogFatal) {
                    errlogSevPrintf(errlogFatal,
                        "mmapConfigure %s: Can't read device type of %s: %s\n",
                        name, intrsource, strerror(errno));
                    return errno;
                }
            }
            else if (!S_ISCHR(sb.st_mode))
            {
                if (missingIntrSevr == errlogFatal) {
                    errlogSevPrintf(errlogFatal,
                        "mmapConfigure %s: %s is not a device file. No interrupt support.\n",
                        name, intrsource);
                    return -1;
                }
            }
            else if (!mmapDevTypeToStr(major(sb.st_rdev), intrdevtype))
            {
                errlogSevPrintf(missingIntrSevr,
                    "mmapConfigure %s: Can't translate device type %lu of %s to string.\n",
                    name, (unsigned long)sb.st_rdev, intrsource);
                if (missingIntrSevr == errlogFatal) return errno;
            }
            else
            {
                if (mmapDebug)
                    printf("mmapConfigure %s: %s is a %s device.\n",
                        name, intrsource, intrdevtype);
                if (strcmp(intrdevtype, "uio") == 0)
                {
                    intrvector = minor(sb.st_rdev);
                    intrlevel = INTR_UIO;
                    if (mmapDebug)
                        printf("mmapConfigure %s: default interrupts from uio%d.\n",
                            name, intrvector);
                }
                else
                {
                    if (missingIntrSevr == errlogFatal)
                    {
                        errlogSevPrintf(errlogFatal,
                            "mmapConfigure %s: Don't know how to handle interrupts from %s.\n",
                                name, intrsource);
                        return errno;
                    }
                }
            }
#endif /* __linux__ */
        }
    }
    if (!addrspace || !addrspace[0]) { addrspace="/dev/mem"; }
    sscanf (addrspace, "%i", &vmespace);
    if (strcmp(addrspace, "csr") == 0) { vmespace = 0xc; }
    if (strcmp(addrspace, "sim") == 0) { vmespace = -1; }
#endif /* !vxWorks */
    if (size > 0) {
        if (vmespace > 0)
        {
            flags |= vmespace/100;
            vmespace %= 100;
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
                        "mmapConfigure %s: invalid VME address space "
                        ADDRSPACEFMT " must be 0xc, 16, 24 or 32.\n",
                        name, addrspace);
                    return -1;
            }
            strcat(devtype, "VME");
#ifndef EPICS_3_13
            if (!pdevLibVirtualOS)
            {
                errlogSevPrintf(errlogFatal,
                    "mmapConfigure %s: No VME support found on this machine.\n",
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
                    "mmapConfigure %s: Cannot map address 0x%08x on "
                    ADDRSPACEFMT " address space.\n",
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
                    "mmapConfigure %s: Out of memory allocating %d bytes of simulated address space.\n",
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
                    printf("mmapConfigure %s: %s is readonly.\n",
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
                        printf("mmapConfigure %s: Growing %s from %lu to %lu bytes.\n",
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
                                "mmapConfigure %s: %s too small and cannot grow, shrinking size to %u.\n",
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
                            printf("mmapConfigure %s: Device is %s number %d intrvector = %d.\n",
                                name, devtype, minor(sb.st_rdev), intrvector);
                        if (strcmp(devtype, "uio") == 0)
                        {
                            if (intrvector < 0) intrvector = minor(sb.st_rdev);
                            intrlevel = INTR_UIO;
                            if (mmapDebug)
                                printf("mmapConfigure %s: Default interrupts from uio%d.\n",
                                    name, intrvector);
                        }
                    }
                }
            }
            else
            {
                if (mmapDebug)
                    printf("mmapConfigure %s: Cannot stat %s: %s\n",
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
                        "mmapConfigure %s: Cannot mmap %s: %s\n",
                        name, addrspace, errno == ENODEV ? "Device does not support mapping." : strerror(errno));
                    return errno;
                }
                /* adjust localbaseaddress by the offset within the page */
                if (mmapDebug)
                    printf("mmapConfigure %s: mmap returned %p, adjusting by %ld bytes.\n",
                        name, localbaseaddress, baseaddress - mapstart);
                localbaseaddress += (baseaddress - mapstart);
            }
            /* we don't need the file descriptor any more */
            close(fd);
        }
    #endif /* HAVE_MMAP */
    }

    if ((flags & MAP_DEVICE) && (flags & (MAP_DEVICE|SWAP_BYTE_PAIRS|SWAP_WORD_PAIRS|SWAP_DWORD_PAIRS)))
    {
        errlogSevPrintf(errlogFatal,
            "mmapConfigure %s: Swapping is incompatible with mapping.\n", name);
        return -1;
    }

    device = (regDevice*)calloc(sizeof(regDevice),1);
    if (device == NULL)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConfigure %s: Out of memory.\n",
            name);
        return errno;
    }
    device->magic = MAGIC;
    device->name = strdup(name);
    device->devtype = strdup(devtype);
    device->vmespace = vmespace;
    device->baseaddress = baseaddress;
    device->localbaseaddress = localbaseaddress;
    device->intrsource =
#ifndef vxWorks
            intrsource &&
#endif
            intrsource[0] ? strdup(intrsource) : NULL;
    device->intrvector = intrvector;
    device->intrlevel = intrlevel;
    device->intrhandler = intrhandler;
    device->userdata = userdata;
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
                "mmapConfigure %s: Invalid addrspace %d.\n",
                name, addrspace);
#endif /* !vxWorks */
    }
    if (mmapDebug)
        printf("mmapConfigure %s: vmespace = %d addrspace = %s\n",
            name, vmespace, device->addrspace);

    regDevRegisterDevice(name, &mmapSupport, device, size);
#ifdef HAVE_dmaAlloc
    if (vmespace > 0)
        regDevRegisterDmaAlloc(device, mmapDmaAlloc);
#endif /* HAVE_dmaAlloc */

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
        localbaseaddress = NULL; /* no direct map when doing DMA */
    }
    device->dmaComplete = epicsEventMustCreate(epicsEventEmpty);
#endif /* HAVE_DMA */

    if (flags & BLOCK_DEVICE)
    {
        if (!(flags & MAP_DEVICE))
            localbaseaddress = NULL;
        regDevMakeBlockdevice(device, REGDEV_BLOCK_READ | REGDEV_BLOCK_WRITE, REGDEV_NO_SWAP, localbaseaddress);
    }
    return 0;
}

#ifndef EPICS_3_13
epicsExportAddress(int, mmapDebug);

#include <iocsh.h>
static const iocshArg mmapConfigureArg0 = { "name", iocshArgString };
static const iocshArg mmapConfigureArg1 = { "baseaddress", iocshArgInt };
static const iocshArg mmapConfigureArg2 = { "size", iocshArgInt };
#ifdef vxWorks
static const iocshArg mmapConfigureArg3 = { "addrspace (-1=simulation; 0xc=CSR; 16,24,32=VME,+100=dma,+200=blockDevice)", iocshArgInt };
static const iocshArg mmapConfigureArg4 = { "intrvector", iocshArgInt };
#else /* !vxWorks */
static const iocshArg mmapConfigureArg3 = { "mapped device (default:/dev/mem, sim=simulation; csr,16,24,32=VME; &option)", iocshArgString };
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
