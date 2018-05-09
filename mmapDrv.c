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
 #define HAVE_MMAP
#endif /*__unix */

#ifdef EPICS_3_14
 #include <errlog.h>
 #include <devLibVME.h>
 #include <epicsTypes.h>
 #include <epicsThread.h>
 #include <epicsExport.h>
 #include <epicsEvent.h>
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
#endif /* else EPICS_3_14 */

/* Try to find dma support */
#ifdef vxWorks
#define HAVE_DMA
#include <dmaLib.h>
#include <semLib.h>
#include <sysLib.h>
#endif /* vxWorks */

#if defined (__GNUC__) && defined (_ARCH_PPC)
 #define SYNC __asm__("eieio;sync");
#else
 #define SYNC
#endif

#define MAGIC 2661166104U /* crc("mmap") */

static char cvsid_mmapDrv[] __attribute__((unused)) =
    "$Id: mmapDrv.c,v 1.18 2015/06/25 14:57:42 zimoch Exp $";

struct regDevice {
    unsigned long magic;
    const char* name;
    volatile char* localbaseaddress;
    int vmespace;
    unsigned int baseaddress;
    unsigned int intrvector;
    unsigned int intrlevel;
    int (*intrhandler)(regDevice *device);
    void* userdata;
    IOSCANPVT ioscanpvt;
    int intrcount;
    int flags;
#ifdef HAVE_DMA
    int maxDmaSpeed;
    epicsEventId dmaComplete;
#endif
    char* addrspace;
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
        printf("mmap %s:0x%x @%p\n",
            device->addrspace, device->baseaddress, device->localbaseaddress);
        if (level > 0)
        {
            printf("        Interrupt level %d vector %d count: %d\n",
                device->intrlevel, device->intrvector, device->intrcount);
        }
    }
}

IOSCANPVT mmapGetInScanPvt(
    regDevice *device,
    size_t offset,
    unsigned int dlen,
    size_t nelem,
    int intvec,
    const char* user)
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
static const int dmaModes [] = {DT32, DT64, DT2eVME, DT2eSST160, DT2eSST267, DT2eSST320};
static const char* dmaModeStr [] = {"BLT", "MBLT", "2eVME", "2eSST160", "2eSST267", "2eSST320"};

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
        if (mmapDebug >= 1) printf ("mmapRead %s %s: block transfer from %p to %p, 0x%"Z"x * %d bit\n",
            user, device->name, device->localbaseaddress+offset, pdata, nelem, dlen*8);
        while (1)
        {
#ifdef dmaTransferRequest_can_wait
            if (dmaTransferRequest(pdata, (unsigned char*) src, nelem*dlen,
                    addrMode, dmaModes[device->maxDmaSpeed], V2C, 100, NULL, NULL, &dmaStatus) != -1)
            {           
#else
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
#endif
                if (dmaStatus == DMA_BUSERR && dlen == 8 && device->maxDmaSpeed > 0)
                {
                    /* try again with a slower block transfer speed */
                    if (mmapDebug >= 1)
                        printf ("mmapRead %s %s: block transfer mode %s failed. trying slower speed\n",
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
                if (mmapDebug >= 1) printf ("mmapRead %s %s: block transfer mode %s failed\n",
                    user, device->name, dmaModeStr[device->maxDmaSpeed]);
                break;
            }
        }
    }
noDmaRead:    
#endif /* HAVE_DMA */ 
    if (mmapDebug >= 1) printf ("mmapRead %s %s: normal transfer from %p to %p, 0x%"Z"x * %d bit\n",
        user, device->name, device->localbaseaddress+offset, pdata, nelem, dlen*8);
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
        if (mmapDebug >= 1) printf ("mmapWrite %s %s: block transfer from %p to %p, 0x%"Z"x * %d bit\n",
            user, device->name, pdata, device->localbaseaddress+offset, nelem, dlen*8);
        while (1)
        {
#ifdef dmaTransferRequest_can_wait
            if ((dmaStatus = dmaTransferRequest((unsigned char*) dst, pdata, nelem*dlen,
                    addrMode, dmaModes[device->maxDmaSpeed], C2V, 100, NULL, NULL, &dmaStatus)) != -1)
            {           
#else
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
#endif
                if (dmaStatus == DMA_BUSERR && dlen == 8 && device->maxDmaSpeed > 0)
                {
                    /* try again with a slower block transfer speed */
                    if (mmapDebug >= 1)
                        printf ("mmapWrite %s %s: block transfer mode %s failed. trying slower speed\n",
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
                if (mmapDebug >= 1) printf ("mmapWrite %s %s: block transfer mode %s failed\n",
                    user, device->name, dmaModeStr[device->maxDmaSpeed]);
                break;
            }
        }
    }
noDmaWrite:    
#endif /* HAVE_DMA */ 
    if (mmapDebug >= 1) printf ("mmapWrite %s %s: transfer from %p to %p, 0x%"Z"x * %d bit\n",
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

void mmapInterrupt(void *arg)
{
    regDevice *device = (regDevice *)arg;
    device->intrcount++;
    if (mmapDebug >= 1) printf ("mmapInterrupt %s: count = %d, %s\n",
        device->name, device->intrcount, device->intrhandler ? "calling handler" : "no handler installed");
    if (device->intrhandler)
    {
        if (device->intrhandler(device) != 0) return;
    }
    scanIoRequest(device->ioscanpvt);
}

#ifdef HAVE_dmaAlloc
void* mmapDmaAlloc(
    regDevice *device,
    void* old,
    size_t size)
{
    return dmaRealloc(old, size);
}
#endif

/****** startup script configuration function ***********************/

int mmapConfigure(
    const char* name,
    unsigned int baseaddress,
    unsigned int size,
#ifdef __vxworks
    int addrspace,   /* int for compatibility with earliner versions */
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
    int vmespace=-2;
    int flags=0;

    if (name == NULL || size == 0)
    {
        printf("usage: mmapConfigure(\"name\", baseaddress, size, addrspace, ivec, ilvl)\n");
        printf("maps register block to device \"name\"\n");
        printf("\"name\" must be a unique string on this IOC\n");
#ifdef HAVE_MMAP
        printf("addrspace: device used for mapping (default: /dev/mem)\n");
#endif
#ifdef __vxworks
        printf("addrspace = 0xc, 16, 24 or 32: VME address space (0xc means CSR)"
#else
        printf("addrspace = csr, 16, 24 or 32: VME address space"
#endif
#ifdef HAVE_DMA
                " (+100: allow block transfer)"
#endif
                "\n");
        printf("addrspace = sim: simulation on allocated memory\n");
        return 0;
    }
#ifdef __vxworks
    vmespace = addrspace;
#else
    if (!addrspace || !addrspace[0]) { addrspace="/dev/mem"; }
    sscanf (addrspace, "%i", &vmespace);
    if (strcmp(addrspace, "csr") == 0) { vmespace = 0xc; }
    if (strcmp(addrspace, "sim") == 0) { vmespace = -1; }
#endif
    if (vmespace > 0)
    {
        flags=vmespace/100;
        vmespace%=100;
        if (mmapDebug) printf ("mmapConfigure %s: vmespace = %d\n",
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
#ifdef EPICS_3_14
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
#else
        if (sysBusToLocalAdrs(vmespace, (char*)baseaddress, &localbaseaddress) != OK)
#endif
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
        if (mmapDebug) printf ("mmapConfigure %s: simulation @%p\n",
            name, localbaseaddress);
    }
#ifdef HAVE_MMAP
    else 
    {
        struct stat sb;
        int fd;
        unsigned long mapstart;
        size_t mapsize;
        
        if (mmapDebug) printf ("mmapConfigure %s: mmap to %s\n",
            name, addrspace);
            
        /* round down start address to page size fo mmap() */
        mapstart = baseaddress & ~((off_t)(sysconf(_SC_PAGE_SIZE)-1));
        mapsize = size + (baseaddress - mapstart);

        /* first try to open read/write, create if necessary (and possible) */
        fd = open(addrspace, O_RDWR | O_CREAT, 0777);
        if (fd < 0)
        {
            /* cannot open R/W or cannot create: try to open readonly */
            fd = open(addrspace, O_RDONLY);
            if (fd < 0)
            {
                errlogSevPrintf(errlogFatal,
                    "mmapConfigure %s: %s: %s\n",
                    name, addrspace, strerror(errno));
                return errno;
            }
            flags |= READONLY_DEVICE;
            if (mmapDebug) printf ("mmapConfigure %s: %s is readonly\n",
                name, addrspace);
        }
        
        /* check (regular) file size (if we cannot let's just hope for the best) and grow if necessary */
        if (fstat(fd, &sb) != -1)
        {
            if ((sb.st_mode & S_IFREG) && mapsize + mapstart > (size_t)sb.st_size)
            {
                if (mmapDebug) printf ("mmapConfigure %s: growing %s from %lu to %lu bytes\n",
                    name, addrspace, sb.st_size, mapsize + mapstart);
                if (ftruncate(fd, mapsize + mapstart) == -1)
                {
                    errlogSevPrintf(errlogFatal,
                        "mmapConfigure %s: %s too small and cannot grow: %s\n",
                        name, addrspace, strerror(errno));
                    close(fd);
                    return errno;
                }
            }
        }
        else
            if (mmapDebug) printf ("mmapConfigure %s: cannot stat %s: %s\n",
                name, addrspace, strerror(errno));

        /* map shared with other processes read/write or readonly */
        if (mmapDebug) printf ("mmapConfigure %s: mmap(NULL, %"Z"u, %s, MAP_SHARED, %d (%s), %ld)\n",
            name, mapsize, (flags &= READONLY_DEVICE) ? "PROT_READ" : "PROT_READ|PROT_WRITE",
            fd, addrspace, mapstart);
            
        localbaseaddress = mmap(NULL, mapsize,
            (flags &= READONLY_DEVICE) ? PROT_READ : PROT_READ|PROT_WRITE,
            MAP_SHARED, fd, mapstart);

        /* we don't need the file descriptor any more */
        close(fd);
        
        if (localbaseaddress == MAP_FAILED || localbaseaddress == NULL)
        {
            errlogSevPrintf(errlogFatal,
                "mmapConfigure %s: can't mmap %s: %s\n",
                name, addrspace, strerror(errno));
            return errno;
        }
        
        /* adjust localbaseaddress by the offset within the page */
        if (mmapDebug) printf ("mmapConfigure %s: mmap returned %p, adjusting by %ld bytes\n",
            name, localbaseaddress, baseaddress - mapstart);
        localbaseaddress += (baseaddress - mapstart);
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
    device->name = strdup(name);
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
#ifndef __vxworks
        default:
            device->addrspace = strdup(addrspace);
#endif
    }
    if (mmapDebug) printf ("mmapConfigure %s: vmespace = %d addrspace = %s\n",
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
                if (mmapDebug) printf ("mmapConfigure %s: maxDmaSpeed=%s\n",
                    name, dmaModeStr[device->maxDmaSpeed]);
                break;
            }
        }
    }
    device->dmaComplete = epicsEventMustCreate(epicsEventEmpty);
#endif /* HAVE_DMA */

    if (intrvector)
    {
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
    }
    regDevRegisterDevice(name, &mmapSupport, device, size);
#ifdef HAVE_dmaAlloc
    if (vmespace > 0)
        regDevRegisterDmaAlloc(device, mmapDmaAlloc);
#endif
    return 0;
}

#ifdef EPICS_3_14

#include <iocsh.h>
static const iocshArg mmapConfigureArg0 = { "name", iocshArgString };
static const iocshArg mmapConfigureArg1 = { "baseaddress", iocshArgInt };
static const iocshArg mmapConfigureArg2 = { "size", iocshArgInt };
#ifdef __vxworks
static const iocshArg mmapConfigureArg3 = { "addrspace (-1=simulation; 0xc=CSR; 16,24,32=VME,+100=block transfer)", iocshArgInt };
#else
static const iocshArg mmapConfigureArg3 = { "mapped device (sim=simulation; csr,16,24,32=VME; default:/dev/mem)", iocshArgString };
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
    { "mmapConfigure", 6, mmapConfigureArgs };
    
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
