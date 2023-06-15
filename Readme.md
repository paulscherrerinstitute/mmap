# EPICS mmap driver

The mmap driver is a low level driver for the
[regDev](https://github.com/paulscherrerinstitute/regdev) device support.
It provides generic access to many types of memory mapped address ranges,
such as VME address spaces, Linux
[uio](https://www.kernel.org/doc/html/latest/driver-api/uio-howto.html)
devices, PCI memory resources, or shared memory files.
Such a memory range is "mapped" to a named regDev "device", so that regDev
can be used to access the address range via standard EPICS record types.

The mmap driver supports interrupts on VME and uio, allowing records to be
processed using `I/O Intr` scanning.

## Prerequisites and Limitations

The driver expects either a VxWorks system with a VME bus or
a POSIX compatible `mmap()` function.
The use of uio devices requires Linux.

DMA is curently only supported for the VME bus on VxWorks, but requires
PSI specific DMA support in the VxWorks kernel.

The driver has never been tested on other operating systems, such as
RTEMS, MacOS or Windows. I do not expect it to work or even compile on such
systems and do not care to support it.

## Configuration

For historical reasons, configuration on VxWorks differs a bit from Linux.
Sorry for the awkward order of arguments.

### Linux
```
  mmapConfigure name, baseaddress, size, addrspace, intrsource, intrlevel
```

 * `name` (string) must be unique among all regDev devices. It is used
   in the i/o links of the EPICS records.
 * `baseaddress` (unsigned int) is the start of the mapped address range,
   that is the address within the address space that gets mapped to offset 0.
 * `size` (unsigned int) is the length of the memory map in bytes.
 * `addrspace` (optional string) is the path of a file that can be `mmap()`ed.
    The default is `/dev/mem`.  Other possibilities could be for example
    uio devices like `/dev/uio*`, shared memory files like `/dev/shm/*` or
    pci resources like `/sys/bus/pci/devices/*/resource*`.
    The IOC needs permission to `mmap()` the file.

    Or it can be one of `csr`, `16`, `24` or `32` for VME address spaces
    or `sim` for a `calloc()`ed simulated device.

    Additional options can be appended to `addrspace` using separators
    `&`, `|`, `,`, `;`, `+` or ` ` (space). (The use of `,` or ` ` requires
    to quote the `addrspace` argument. Options are not case sensitive.)
     * `SwapDWordPairs`: swaps `0x0123456789abcdef` to `0x89abcdef01234567`
     * `SwapWordPairs`:  swaps `0x0123456789abcdef` to `0x45670123cdef89ab`
     * `SwapBytePairs`:  swaps `0x0123456789abcdef` to `0x23016745ab89efcd`
     * `SwapWords`:      swaps `0x0123456789abcdef` to `0x23016745ab89efcd`
     * `SwapDWords`:     swaps `0x0123456789abcdef` to `0x67452301efcdab89`
     * `SwapQWords`:     swaps `0x0123456789abcdef` to `0xefcdab8967452301`
     * `block`:          transfer the whole address space in one block
     * `dma`:            uses dma for large arrays and block mode
     * `map`:            allows to map arrays directly into device space
 * `intrsource` (optional string) can be a uio file like `/dev/uio0` or
    a VME interrupt vector rumber 1..255. The default is to use the same
    as `addrspace` if that is a uio device, or no interrupts otherwise.
 * `intrlevel` (optional unsigned int) is a VME interrupt level, 1..7

### VxWorks
```
  mmapConfigure "name", baseaddress, size, addrspace, intrvector, intrlevel, intrhandler, userdata
```

The arguments are the same as for Linux, except:

 * `addrspace` (unsigned int) is a code for the address space:
    * `16`, `24` or `32` for the VME A16, A24, or A32 address spaces.
    * `0xc` for the VME CR/CSR address space.
    * `-1` for a simulated memory device (`calloc()`ed)
    * add `100` to allow DMA
    * add `200` to enable regDev "block mode"
 * `intrvector` (optional unsigned int) is a VME interrupt vector, 1..0xff
 * `intrhandler` (optional function pointer) is a custom interrupt handler
 * `userdata` (optional integer) is an argument to the custom interrupt handler
 
### Block mode

The block mode is an optimization for transfer efficiency. In this mode,
regDev will transfer (read or write) the complete address space beween device
and a buffer in RAM when a record with `PRIO=HIGH` is processed.
After the transfer, records with `SCAN=I/O Intr` and the same transfer
direction (read/write) will be processed if their `PRIO` is not `HIGH` and if
they do not use interrupt triggering.
Records with `PRIO` not set to `HIGH` interact only with that buffer in RAM
and never with the device directly.

### DMA

For longer arrays and block mode, DMA can be more efficient than using the CPU
to transfer the data. For VME, DMA translates to BLT, MBLT or 2eSST mode,
using the fastest mode available.
Because of the overhead involved to set up the DMA, it is only used for arrays
with at least 1024 elements.

### Mapped arrays

The record types aai and aao allow to have their array data directly mapped to
the device address space, thus avoiding to copy the data into the record. This
requires that the data does not need to be swapped, masked, inverted, scaled,
interlaced or otherwise modified. Before EPICS 7, mapped arrays could not have
dynamic offsets.

Be aware that in this case the record content may change at any time, whenever
the device content changes, without the record being processed, even while the
record content is sent out on the network, e.g. with Channel Access, leading
to inconsitent data.

### Interrupt handling

Records using `I/O Intr` scanning will be processed upon interrupts
in one of the callback threads depending on their `PRIO` field.
Currently interrupts are supported on VME using an interrupt vector number
in the range 0x01..0xff as `intrsource` and (optionally) setting an `intrlevel`
in the range 1..7, or using a uio device as `intrsource`. This is the default
if `addrspace` is a uio device.

In addition, a record can specify the interrupt vector (and optionally level)
in the `V` parameter of its i/o link. Such settings overwrite the defaults
specified in `mmapConfigure` and can even be used if no interrupt source has
been configured in `mmapConfigure`.
The integer value of the `V=$(N)` parameter is first interpreted as a uio
device: /dev/uio`$(N)` (Linux only). If that is not a valid uio device, it is
interpreted as a VME interrupt vector number (and optionally an interrupt
level): `$(N)&0xff` overwrites the default interrupt vector and `$(N)>>8`,
if not 0, overwrites the default interrupt level.

The specified VME interrupt level is enabled by the first record using it, if
it is not 0. While it is possible to connect to a VME interrupt vector without
specifying an interrupt level, the interrupt may not be enabled and thus the
records may never process.

On VxWorks, it is possible to configure a user supplied interrupt handler that
is called in interrupt context before processing the records. There are a few
pre-defined handlers available:

 * `mmapIntAckSetBits16` sets bits `userdata & 0xffff` in offset `userdata >> 16`.
 * `mmapIntAckClearBits16` clears bits `userdata & 0xffff` in offset `userdata >> 16`.

---
Dirk Zimoch \<dirk.zimoch@psi.ch\>
