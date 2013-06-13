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
#define HAVE_MMAP
#endif /*__unix */

#ifdef __vxworks
#include <sysLib.h>
#include <intLib.h>
#include <wdLib.h>
#include <semLib.h>
#define HAVE_VME
#include <vme.h>
#include <iv.h>
#define atVMEA16 VME_AM_USR_SHORT_IO
#define atVMEA24 VME_AM_STD_SUP_DATA
#define atVMEA32 VME_AM_EXT_SUP_DATA
#include <version.h>
#ifdef RUNTIME_VERSION /* VxWorks 5.5+ */
#define HAVE_DMA
#include <dmaLib.h>
#endif /* VxWorks 5.5+ */
#else /*__vxworks */
#include <devLib.h>
#endif /* __vxworks */

#ifdef EPICS_3_14
#define HAVE_VME
#include <epicsTypes.h>
#include <epicsThread.h>
#endif

#if defined (__GNUC__) && defined (_ARCH_PPC)
#define SYNC __asm__("eieio;sync");
#else
#define SYNC
#endif

#define MAGIC 2661166104U /* crc("mmap") */

static char cvsid_mmapDrv[] __attribute__((unused)) =
    "$Id: mmapDrv.c,v 1.10 2013/06/13 15:50:23 zimoch Exp $";

struct regDevice {
    unsigned long magic;
    const char* name;
    volatile char* localbaseaddress;
    int vmespace;
    unsigned int baseaddress;
    unsigned int size;
    unsigned int intrvector;
    unsigned int intrlevel;
    int (*intrhandler)(regDevice *device);
    void* userdata;
    IOSCANPVT ioscanpvt;
    int intrcount;
    int flags;
#ifdef HAVE_DMA
    int maxDmaSpeed;
    SEM_ID dmaComplete;
    WDOG_ID dmaWatchdog;
#endif
};

int mmapDebug = 0;

/* Device flags */
#define ALLOW_BLOCK_TRANSFER 0x0000001
#define READONLY_DEVICE      0x0000002

/******** Support functions *****************************/ 

void mmapReport(
    regDevice *device,
    int level)
{
    if (device && device->magic == MAGIC)
    {
        printf("mmap driver: %d bytes @ %p - %p\n",
            device->size, device->localbaseaddress,
            device->localbaseaddress+device->size-1);
        if (level > 1)
        {
            printf("       Interrupt count: %d\n", device->intrcount);
        }
    }
}

IOSCANPVT mmapGetInScanPvt(
    regDevice *device,
    size_t offset)
{
    if (!device || device->magic != MAGIC)
    { 
        errlogSevPrintf(errlogMajor,
            "mmapGetInScanPvt: illegal device handle\n");
        return NULL;
    }
    return device->ioscanpvt;
}

#define mmapGetOutScanPvt mmapGetInScanPvt

#ifdef HAVE_DMA
static const int dmaModes [] = {DT64, DT2eVME, DT2eSST160, DT2eSST267, DT2eSST320};

void mmapCancelDma(int handle)
{
    dmaRequestCancel(handle, FALSE);
}
#endif


int mmapRead(
    regDevice *device,
    size_t offset,
    unsigned int dlen,
    size_t nelem,
    void* pdata,
    int prio)
{
    volatile char* src;

    if (!device || device->magic != MAGIC)
    {
        errlogSevPrintf(errlogMajor,
            "mmapRead: illegal device handle\n");
        return -1;
    }
    regDevCheckOffset("mmapRead", device->name, offset, dlen, nelm, device->size);
    
    src = device->localbaseaddress+offset;
#ifdef HAVE_DMA
    /* Try block transfer for long arrays */
    if (nelem >= 1024 &&                          /* inefficient for short arrays */
        (device->flags & ALLOW_BLOCK_TRANSFER) && /* card must be able to do block transfer */
        (((long)pdata|(long)src) & 0x7) == 0)     /* src and dst address must be multiple of 8 */
    {
        UINT32 addrMode;
        UINT32 dataWidth;
        int dmaHandle;
        unsigned int dmaStatus;
        
        switch (dlen)
        {
            case 1:
                dataWidth = DT8;
                break;
            case 2:
                dataWidth = DT16;
                break;
            case 4:
                dataWidth = DT32;
                break;
            case 8:
                if (device->maxDmaSpeed == -1) goto noDmaRead;
                dataWidth = dmaModes[device->maxDmaSpeed];
                break;
            default:
                goto noDmaRead;
        }
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
        if (mmapDebug >= 1) printf ("mmapRead %s: block transfer from %p to %p, %d * %d bit\n",
            device->name, device->localbaseaddress+offset, pdata, nelem, dlen*8);
        while (1)
        {
            if ((dmaHandle = dmaTransferRequest(pdata, (unsigned char*) src, nelem*dlen,
                    addrMode, dataWidth, V2C, 100,
                    (VOIDFUNCPTR)semGive, device->dmaComplete, &dmaStatus)) != ERROR)
            {
                wdStart(device->dmaWatchdog, sysClkRateGet(), (FUNCPTR)mmapCancelDma, dmaHandle);
                if (semTake(device->dmaComplete, 10*sysClkRateGet()) == ERROR)
                {
                    wdCancel(device->dmaWatchdog);
                    dmaRequestCancel(dmaHandle, TRUE);
                    errlogSevPrintf(errlogMajor,
                        "mmapRead %s: DMA transfer timeout.\n",
                            device->name);
                    return ERROR;
                }
                wdCancel(device->dmaWatchdog);
                if (dmaStatus == DMA_BUSERR && dlen == 8 && device->maxDmaSpeed > 0)
                {
                    /* try again with a slower DMA speed */
                    dataWidth = dmaModes[--device->maxDmaSpeed];
                    continue;
                }
                if (dmaStatus == DMA_DONE)
                {
                    return 0;
                }
                errlogSevPrintf(errlogMajor,
                    "mmapRead %s: DMA %s error (0x%x). Using normal transfer.\n",
                        device->name,
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
                if (mmapDebug >= 1) printf ("mmapRead %s: block transfer mode %x failed\n",
                    device->name, dataWidth);
                break;
            }
        }
    }
noDmaRead:    
#endif /* HAVE_DMA */ 
    if (mmapDebug >= 1) printf ("mmapRead %s: normal transfer from %p to %p, %d * %d bit\n",
        device->name, device->localbaseaddress+offset, pdata, nelem, dlen*8);
    regDevCopy(dlen, nelem, src, pdata, NULL, 0);
    return 0;
}

int mmapWrite(
    regDevice *device,
    size_t offset,
    unsigned int dlen,
    size_t nelem,
    void* pdata,
    void* pmask,
    int prio)
{    
    volatile char* dst;

    if (!device || device->magic != MAGIC)
    {
        errlogSevPrintf(errlogMajor,
            "mmapWrite: illegal device handle\n");
        return -1;
    }
    if (device->flags & READONLY_DEVICE)
    {
        errlogSevPrintf(errlogMajor,
            "mmapWrite: device is read-only\n");
        return -1;
    }
    if (offset > device->size ||
        offset+dlen*nelem > device->size)
    {
        errlogSevPrintf(errlogMajor,
            "mmapWrite: address out of range\n");
        return -1;
    }
    if (!device || device->magic != MAGIC
        || offset+dlen*nelem > device->size) return -1;
    dst = device->localbaseaddress+offset;
#ifdef HAVE_DMA
    /* Try block transfer for long arrays */
    if (nelem >= 1024 &&                          /* inefficient for short arrays */
        (device->flags & ALLOW_BLOCK_TRANSFER) && /* card must be able to do block transfer */
        (((long)pdata|(long)dst) & 0x7) == 0)     /* src and dst address must be multiple of 8 */
    {
        UINT32 addrMode;
        UINT32 dataWidth;
        int dmaHandle;
        unsigned int dmaStatus;
        
        switch (dlen)
        {
            case 1:
                dataWidth = DT8;
                break;
            case 2:
                dataWidth = DT16;
                break;
            case 4:
                dataWidth = DT32;
                break;
            case 8:
                if (device->maxDmaSpeed == -1) goto noDmaWrite;
                dataWidth = dmaModes[device->maxDmaSpeed];
                break;
            default:
                goto noDmaWrite;
        }
        switch (device->vmespace)
        {
            case 16:
                addrMode = atVMEA16;
                break;
            case 24:
                addrMode = atVMEA24;
                break;
            case 32:
                addrMode = atVMEA32;
                break;
            default:
                goto noDmaWrite;
        }
        if (mmapDebug >= 1) printf ("mmapWrite %s: block transfer from %p to %p, %d * %d bit\n",
            device->name, pdata, device->localbaseaddress+offset, nelem, dlen*8);
        while (1)
        {
            if ((dmaHandle = dmaTransferRequest((unsigned char*) dst, pdata, nelem*dlen,
                    addrMode, dataWidth, C2V, 100,
                    (VOIDFUNCPTR)semGive, device->dmaComplete, &dmaStatus)) != ERROR)
            {
                wdStart(device->dmaWatchdog, sysClkRateGet(), (FUNCPTR)mmapCancelDma, dmaHandle);
                if (semTake(device->dmaComplete, 10*sysClkRateGet()) == ERROR)
                {
                    wdCancel(device->dmaWatchdog);
                    dmaRequestCancel(dmaHandle, TRUE);
                    errlogSevPrintf(errlogMajor,
                        "mmapWrite %s: DMA transfer timeout.\n",
                            device->name);
                    return ERROR;
                }
                wdCancel(device->dmaWatchdog);
                if (dmaStatus == DMA_BUSERR && dlen == 8 && device->maxDmaSpeed > 0)
                {
                    /* try again with a slower DMA speed */
                    dataWidth = dmaModes[--device->maxDmaSpeed];
                    continue;
                }
                if (dmaStatus == DMA_DONE)
                {
                    return 0;
                }
                errlogSevPrintf(errlogMajor,
                    "mmapWrite %s: DMA %s error (0x%x). Using normal transfer.\n",
                        device->name,
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
                if (mmapDebug >= 1) printf ("mmapWrite %s: block transfer mode %x failed\n",
                    device->name, dataWidth);
                break;
            }
        }
    }
noDmaWrite:    
#endif /* HAVE_DMA */ 
    if (mmapDebug >= 1) printf ("mmapWrite %s: transfer from %p to %p, %d * %d bit\n",
        device->name, pdata, dst, nelem, dlen*8);
    regDevCopy(dlen, nelem, pdata, dst, NULL, 0);
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

void mmapInterrupt(void *arg)
{
    regDevice *device = (regDevice *)arg;
    device->intrcount++;
    if (device->intrhandler)
    {
        if (device->intrhandler(device) != 0) return;
    }
    scanIoRequest(device->ioscanpvt);
}

/****** startup script configuration function ***********************/

int mmapConfigure(
    const char* name,
    unsigned int baseaddress,
    unsigned int size,
#ifdef __vxworks
    int addrspace,
#define ADDRSPACEFMT "%d"
#else
    char* addrspace,
#define ADDRSPACEFMT "%s"
#endif
    unsigned int intrvector,
    unsigned int intrlevel,
    int (*intrhandler)(regDevice *device),
    void* userdata)
{
    regDevice* device;
    char* localbaseaddress;
    int vmespace=0;
    int flags=0;

    if (name == NULL || size == 0)
    {
        printf("usage: mmapConfigure(\"name\", baseaddress, size, addrspace)\n");
        printf("maps register block to device \"name\"");
        printf("\"name\" must be a unique string on this IOC\n");
#ifdef HAVE_MMAP
        printf("addrspace: device used for mapping (default: /dev/mem)\n");
#endif
#ifdef HAVE_VME
        printf("addrspace = 16, 24 or 32: VME address space"
#ifdef HAVE_DMA
                " (+100: allow block transfer)"
#endif
                "\n");
#endif
        printf("addrspace = sim: simulation on allocated memory\n");
        return 0;
    }
#ifdef HAVE_VME        
#ifdef __vxworks
    vmespace = addrspace;
#else
    if (!addrspace || !addrspace[0]) { addrspace="/dev/mem"; }
    if (sscanf (addrspace, "%d", &vmespace) == 1)
#endif
    {
        flags=vmespace/100;
        vmespace%=100;
        switch (vmespace)
        {
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
                    ADDRSPACEFMT " must be 16, 24 or 32\n",
                    name, addrspace);
                return -1;
        }
#ifdef __vxworks
        if (sysBusToLocalAdrs(vmespace, (char*)baseaddress, &localbaseaddress) != OK)
#else
        if ((*pdevLibVirtualOS->pDevMapAddr) (vmespace, 0, baseaddress, size, (volatile void **)(volatile char **)&localbaseaddress) != 0)
#endif /*__vxworks*/
        {
            errlogSevPrintf(errlogFatal,
                "mmapConfigure %s: can't map address 0x%08x on "
                ADDRSPACEFMT " address space\n",
                name, baseaddress, addrspace);
            return -1;
        }
    }
#endif /*HAVE_VME*/
    if (vmespace == -1
#ifndef __vxworks
        || strcmp(addrspace, "sim") == 0
#endif
    )
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
    }
#ifdef HAVE_MMAP
    else 
    {
        int fd;
        int first = 1;
        double sleep = 0.1;

        do {
            fd = open(addrspace, O_RDWR | O_SYNC);
            if (fd >= 0)
            {
                if (sleep > 0.1) epicsThreadSleep(1);
                localbaseaddress = mmap(NULL, size,
                    PROT_READ|PROT_WRITE, MAP_SHARED,
                    fd, baseaddress);
                close(fd);
            }
            else
            {
                flags |= READONLY_DEVICE;
                fd = open(addrspace, O_RDONLY | O_SYNC);
                if (fd >= 0)
                {
                    if (sleep > 0.1) epicsThreadSleep(1);
                    localbaseaddress = mmap(NULL, size,
                        PROT_READ, MAP_SHARED,
                        fd, baseaddress);
                    close(fd);
                }
            }
            if (fd < 0)
            {
                if (errno != ENOENT)
                {
                    errlogSevPrintf(errlogFatal,
                        "mmapConfigure %s: can't open %s: %s\n",
                        name, addrspace, strerror(errno));
                    return errno;
                }
                if (first)
                {
                    first = 0;
                    errlogSevPrintf(errlogInfo,
                        "mmapConfigure %s: can't open %s: %s\nI will retry later...\n",
                        name, addrspace, strerror(errno));
                }
                epicsThreadSleep(sleep);
                if (sleep < 10) sleep *= 1.1;
            }
        } while (fd < 0);
        if (localbaseaddress == MAP_FAILED || localbaseaddress == NULL)
        {
            errlogSevPrintf(errlogFatal,
                "mmapConfigure %s: can't mmap %s: %s\n",
                name, addrspace, strerror(errno));
            return errno;
        }
    }
#endif      
    device = (regDevice*)malloc(sizeof(regDevice));
    if (device == NULL)
    {
        errlogSevPrintf(errlogFatal,
            "mmapConfigure %s: out of memory\n",
            name);
        return errno;
    }
    device->magic = MAGIC;
    device->name = name;
    device->size = size;
    device->vmespace = vmespace;
    device->baseaddress = baseaddress;
    device->localbaseaddress = localbaseaddress;
    device->intrlevel = intrlevel;
    device->intrvector = intrvector;
    device->intrhandler = intrhandler;
    device->userdata = userdata;
    device->ioscanpvt = NULL;
    device->intrcount = 0;
    device->flags = flags;
    
#ifdef HAVE_DMA
    device->maxDmaSpeed=-1;
    {
        int i;
        for (i = sizeof(dmaModes)/sizeof(dmaModes[0]); i >= 0; i--)
        {
            if (dmaProtocolSupport(dmaModes[i]) == OK)
            {
                device->maxDmaSpeed = i;
                break;
            }
        }
    }
    device->dmaComplete = semBCreate(SEM_Q_FIFO, SEM_EMPTY);
    device->dmaWatchdog = wdCreate();
#endif /* HAVE_DMA */

    if (intrvector)
    {
        if (vmespace <= 0)
        {
            errlogSevPrintf(errlogFatal,
                "mmapConfigure %s: interrupts not supported on addrspace " ADDRSPACEFMT "\n",
                name, addrspace);
            return -1;
        }
        if (intrlevel < 1 || intrlevel > 7)
        {
            errlogSevPrintf(errlogFatal, 
                "mmapConfigure %s: illegal interrupt level %d must be 1...7\n",
                name, intrlevel);
            return -1;
        }
#ifdef HAVE_VME
        if (devConnectInterrupt(intVME, intrvector, mmapInterrupt, device) != 0)
        {
            errlogSevPrintf(errlogFatal,
                "vmemapConfigure %s: cannot connect to interrupt vector %d\n",
                name, intrvector);
            return -1;
        }
        if (devEnableInterruptLevel(intVME, intrlevel) != 0)
        {
            errlogSevPrintf(errlogFatal,
                "vmemapConfigure %s: cannot enable interrupt level %d\n",
                name, intrlevel);
            return -1;
        }
        scanIoInit(&device->ioscanpvt);
#endif /* HAVE_VME */
    }
    regDevRegisterDevice(name, &mmapSupport, device);
    return 0;
}

#ifdef EPICS_3_14

#include <iocsh.h>
static const iocshArg mmapConfigureArg0 = { "name", iocshArgString };
static const iocshArg mmapConfigureArg1 = { "baseaddress", iocshArgInt };
static const iocshArg mmapConfigureArg2 = { "size", iocshArgInt };
#ifdef __vxworks
static const iocshArg mmapConfigureArg3 = { "addrspace (-1=simulation;16,24,32=VME,+100=block transfer)", iocshArgInt };
#else
static const iocshArg mmapConfigureArg3 = { "mapped device (sim=simulation, default:/dev/mem)", iocshArgString };
#endif
static const iocshArg mmapConfigureArg4 = { "intrvector (default:0)", iocshArgInt };
static const iocshArg mmapConfigureArg5 = { "intrlevel (default:0)", iocshArgInt };
static const iocshArg mmapConfigureArg6 = { "intrhandler (default:NULL)", iocshArgString };
static const iocshArg mmapConfigureArg7 = { "userdata (default:NULL)", iocshArgString };
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
#ifdef __vxworks
        args[3].ival,
#else
        args[3].sval,
#endif
        args[4].ival, args[5].ival, NULL, NULL);
}

static void mmapRegistrar ()
{
    iocshRegister(&mmapConfigureDef, mmapConfigureFunc);
}

epicsExportRegistrar(mmapRegistrar);

#endif
