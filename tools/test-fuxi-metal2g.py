#!/usr/bin/env python3
"""Compile the real NC lookup/cache-sync/argv code with strict physmap traps."""
from pathlib import Path
import importlib.util
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('submit', ROOT/'tools/test-fuxi-adreno.py')
submit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(submit)


def main():
    adapter = (ROOT/'xnu/fuxi-adreno/fuxi_adreno.c').read_text()
    declarations = re.search(r'struct fg_window.*?static uint64_t fg_mapped_bytes;', adapter, re.S)[0]
    patch = (ROOT/'xnu/patches/0101-fuxi-gpu-nc-cache-sync.patch').read_text()
    body = ''.join(line[1:]+'\n' for line in patch.splitlines()
                   if line.startswith((' ', '+')) and not line.startswith('+++'))
    body = body[body.index('{'):body.index('\n}')+2]
    cache = 'void pmap_sync_page_attributes_phys(ppnum_t pp)\n'+body
    lookup = submit.function(adapter, 'fuxi_gpu_nc_pointer') + submit.function(adapter, 'fg_pointer_range')
    zero = submit.function((ROOT/'shim/fuxi-probe/gpu.c').read_text(), 'g_zero')
    zero = re.sub(r'__asm__ volatile\([^;]*\);', '(void)0;', zero)
    lookup += '#define G_PTR_RANGE(pa, bytes) fg_pointer_range(pa, bytes)\n' + zero
    prefix = submit.PANEL_DEFAULTS + r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint32_t ppnum_t;
typedef uintptr_t vm_offset_t;
#define FUXI 1
#define PAGE_SHIFT 14
#define PAGE_SIZE (1u<<PAGE_SHIFT)
#define TRUE 1
#define FALSE 0
#define FG_SCRATCH UINT64_C(0x9fe000000)
#define FG_SCRATCH_SIZE 0x2000000u
static const uint64_t gPhysBase=UINT64_C(0x8c0000000), gPhysSize=UINT64_C(0x13e000000);
static unsigned phys_calls, nc_calls;
static uint64_t last;
static void panic(const char *fmt, ...) { (void)fmt; exit(3); }
static void flush_dcache(uintptr_t addr, unsigned bytes, int phys) {
 assert(bytes==PAGE_SIZE); last=addr;
 if (phys) {
  /* phystokv_range must NEVER receive a carve-out or truncated RAM PA. */
  if (addr<gPhysBase || addr-gPhysBase>=gPhysSize) exit(4);
  ++phys_calls;
 } else ++nc_calls;
}
'''
    model = r'''
int main(int argc, char **argv) {
 (void)argv;
 if (argc>2) { fg_windows[7].va=0x1000000000; g_zero(FG_SCRATCH+FG_SCRATCH_SIZE-8,16); return 9; }
 if (argc>1) { pmap_sync_page_attributes_phys(0xc4f40000u>>PAGE_SHIFT); return 9; }
 const uint64_t ram=UINT64_C(0x8c4f40000);
 pmap_sync_page_attributes_phys(ram>>PAGE_SHIFT);
 assert(phys_calls==1 && last==ram && !nc_calls);
 assert(!fuxi_gpu_nc_pointer(FG_SCRATCH,1)); /* unpublished */
 assert(!fg_mapped_bytes);
 for (unsigned i=0;i<sizeof(fg_windows)/sizeof(*fg_windows);++i) {
  struct fg_window *w=&fg_windows[i]; w->va=UINT64_C(0x1000000000)+i*UINT64_C(0x4000000);
  assert(fuxi_gpu_nc_pointer(w->pa,w->size)==w->va);
  assert(fuxi_gpu_nc_pointer(w->pa+w->size-1,1)==w->va+w->size-1);
  assert(!fuxi_gpu_nc_pointer(w->pa+w->size-1,2));
  assert(!fuxi_gpu_nc_pointer(w->pa,w->size+1));
  assert(!fuxi_gpu_nc_pointer(w->pa,0));
  pmap_sync_page_attributes_phys(w->pa>>PAGE_SHIFT);
  assert(last==w->va && phys_calls==1 && nc_calls==i+1);
 }
 static unsigned char storage[FG_SCRATCH_SIZE];
 memset(storage,0x5a,sizeof(storage)); fg_windows[7].va=(uintptr_t)storage;
 g_zero(FG_SCRATCH+1232,32);
 for (unsigned i=0;i<sizeof(storage);++i) assert(storage[i]==(i>=1232 && i<1264 ? 0 : 0x5a));
 assert(!fuxi_gpu_nc_pointer(UINT64_MAX-1,8));
 assert(!fuxi_gpu_nc_pointer(0xc4f40000,16384));
 struct fg_window *w=&fg_windows[0]; w->va=UINTPTR_MAX-8;
 assert(!fuxi_gpu_nc_pointer(w->pa,16));
 puts("PASS: full-width BO PA; all NC apertures bypass physmap; unpublished/edge/overflow rejection");
}
'''
    tool = (ROOT/'tools/fuxi-gpu-abi-test.cpp').read_text()
    argv = submit.function(tool, 'test_arguments')
    argv_model = r'''
int main() {
 const char *names[]={"fuxi-gpu-abi-test","/usr/local/bin/fuxi-gpu-abi-test",
     "peakpowermanagerd","/usr/libexec/IOMFB_FDR_Loader","fuxi-gpu-abi-test-other",""};
 for (unsigned i=0;i<sizeof(names)/sizeof(*names);++i) {
  char *args[]={const_cast<char *>(names[i]),const_cast<char *>("--watchdog"),nullptr};
  assert(test_arguments(1,args)==0);
  assert(test_arguments(2,args)==(i<2 ? 1 : 0));
  args[1]=const_cast<char *>("--k1-limits");
  assert(test_arguments(2,args)==(i<2 ? 2 : 0));
  args[1]=const_cast<char *>("--launchd_boot");
  assert(test_arguments(2,args)==(i<2 ? -1 : 0));
  args[2]=args[1];args[1]=const_cast<char *>("--watchdog");
  assert(test_arguments(3,args)==(i<2 ? -1 : 0));
 }
 assert(test_arguments(0,nullptr)==0);
 puts("PASS: exec-swap host argv ignored; watchdog and usage restricted to explicit self invocation");
}
'''
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        for name, code in [('cache', prefix+declarations+lookup+cache+model),
                           ('argv', '#include <assert.h>\n#include <string.h>\n#include <stdio.h>\n'+argv+argv_model)]:
            source = d/(name+'.cpp'); source.write_text(code)
            exe = d/name
            subprocess.run(['clang++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', str(source), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            if name=='cache':
                assert subprocess.run([str(exe),'unknown']).returncode==3
                assert subprocess.run([str(exe),'zero','edge']).returncode==3
                old = code.replace(cache, 'void pmap_sync_page_attributes_phys(ppnum_t pp) {\n'
                                   ' flush_dcache((vm_offset_t)(pp << PAGE_SHIFT),PAGE_SIZE,TRUE);\n}')
                source.write_text(old)
                subprocess.run(['clang++','-std=c++17',str(source),'-o',str(exe)],check=True)
                assert subprocess.run([str(exe)]).returncode==4, 'old code must hit strict physmap trap'
                print('PASS: regression reproduces metal2f PA truncation with original helper')
    # Audit the canonical ABI, present and shared helper code. Physical accesses
    # must use NC aliases; pmap_find_phys only translates wired source KVAs to PAs.
    for path in list((ROOT/'xnu/fuxi-adreno').glob('*')) + [ROOT/'shim/fuxi-probe/gpu.c']:
        if path.suffix not in ('.c','.cpp','.inc','.h'): continue
        code = re.sub(r'/\*.*?\*/|//[^\n]*','',path.read_text(),flags=re.S)
        assert not re.search(r'\b(?:phystokv\w*|ml_static_(?:ptovirt|vtop)|ml_phys_\w+|bzero_phys\w*|bcopy_phys\w*)\s*\(',code), path
    print('PASS: ABI/client/present/shared sources contain no physical-to-physmap access helpers')


if __name__=='__main__':
    main()
