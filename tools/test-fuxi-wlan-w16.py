#!/usr/bin/env python3
"""W16 production C + host MHI peer; no firmware, network or device access.

Capacity expectations: ath12k v6.12 mhi.c, BSD-3-Clause-Clear; The Linux
Foundation (2020-2021), Qualcomm Innovation Center, Inc. (2021-2024).
Peer geometry is selected from test inputs, independently of driver macros.
"""
from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('w15', ROOT/'tools/test-fuxi-wlan-w15.py')
w15 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(w15)

# Retain W15's exact INI/CFG/MODE wire peer and synthetic download endpoint.
peer = w15.TESTS.split('int main(void) {', 1)[0]
peer = peer.replace('static unsigned dev_cmd,', r'''
static unsigned peer_n(void) { return w16_arg==1 || w16_arg==3 ? 64 : 16; }
static unsigned peer_mtu(void) { return peer_n()==64 ? 8192 : 4096; }
static unsigned peer_rxoff(void) { return peer_n()*16; }
static unsigned peer_rxbuf(void) { return 0x1000+peer_n()*8192; }
static unsigned cfg_event;
static unsigned dev_cmd,''')
peer = peer.replace('len<=4096', 'len<=peer_mtu()')
peer = peer.replace('wl_ipcr.iova+0x100;', 'wl_ipcr.iova+peer_rxoff();')
peer = peer.replace('wl_ipcr.va+0x100+16*dev_rx', 'wl_ipcr.va+peer_rxoff()+16*dev_rx')
peer = peer.replace('tre->d0==4096', 'tre->d0==peer_mtu()')
peer = peer.replace('wl_ipcr.iova+0x11000', 'wl_ipcr.iova+peer_rxbuf()')
peer = peer.replace('tre->ptr+4096', 'tre->ptr+peer_mtu()')
peer = peer.replace('next(dev_rx,16)', 'next(dev_rx,peer_n())')
peer = peer.replace('next(dev_tx,16)', 'next(dev_tx,peer_n())')
# Emulate device state writes as raw bytes at the independently specified ABI.
peer = peer.replace('commands++; if(op==18)',
                    '((uint8_t *)wl_contexts.va)[ch*44]=op==18?2:3;\n        commands++; if(op==18)')
peer = peer.replace('assert(step<3 && wl14_ready && m3_requests==1);', r'''
        assert(step<3 && wl14_ready && m3_requests==1);
        if(step==1)cfg_event=ne;
        /* Every variant retains the exact audited W15 requests. */
        if(step==2) {
            assert(wl_tx_read!=wl_tx_write);
            assert(wl_channel(20)->len==peer_n()*16);
            assert(wl_channel(21)->base==wl_ipcr.iova+peer_rxoff());
            assert(wl_channel(21)->len==peer_n()*16);
            for(unsigned i=0;i<2;i++) {
                const uint8_t *raw=(void *)(wl_contexts.va+(20+i)*44);
                assert(get32(raw)==0x202 && get32(raw+4)==i+1 && get32(raw+8)==1);
                assert(get32(raw+12)==wl_ipcr.iova+(i?peer_rxoff():0) && !get32(raw+16));
                assert(get32(raw+20)==peer_n()*16 && !get32(raw+24));
            }
            unsigned rxdb=0;
            for(unsigned j=cfg_event;j<ne;j++)if(events[j].kind=='W' && events[j].pa==WL_BAR0+0x3a8)rxdb++;
            assert(rxdb==(w16_arg==2 || w16_arg==3?3:2));
            if(w16_arg==2 || w16_arg==3)assert(strstr(logbuf,"before MODE re-ring posted RX"));
        }
''')

TESTS = peer + r'''
static unsigned writes_since(unsigned from,uint64_t pa) {
    unsigned n=0;for(unsigned i=from;i<ne;i++)if(events[i].kind=='W' && events[i].pa==pa)n++;
    return n;
}
static void boot16(unsigned variant,unsigned failure,bool keep) {
    w16_arg=variant;boot15(failure,keep);
    assert(wl16_variant==variant && wl16_large==(peer_n()==64));
    assert(wl16_redrive==(variant==2 || variant==3));
    assert(wl_ipcr.size==((peer_rxbuf()+peer_n()*peer_mtu()+16383)&~16383u));
}
static void readonly16(void) {
    uint8_t ctx[16384];memcpy(ctx,(void *)wl_contexts.va,sizeof(ctx));
    uint8_t *ipcr=malloc(wl_ipcr.size);assert(ipcr);memcpy(ipcr,(void *)wl_ipcr.va,wl_ipcr.size);
    unsigned before=ne;track_reads=true;nreads=0;wl16_snapshot("test read only");track_reads=false;
    assert(ne==before && !nreads && !memcmp(ctx,(void *)wl_contexts.va,sizeof(ctx)));
    assert(!memcmp(ipcr,(void *)wl_ipcr.va,wl_ipcr.size));free(ipcr);
}
static void notifications16(unsigned variant) {
    boot16(variant,0,true);assert(!finish_step());
    unsigned rx=wl_rx_read,tx=wl_tx_read,rxw=wl_rx_write;
    /* An empty TX ring must never be kicked; RX still owns posted buffers.
     * Notifications have no completed TRE pointer and must not reclaim one. */
    unsigned before=ne;
    for(unsigned ch=20;ch<=21;ch++)for(unsigned cc=5;cc<=6;cc++)
        dev_event(1,0,cc<<24,(ch<<24)|0x220000);
    assert(!wl_w11_events());
    assert(wl_rx_read==rx && wl_tx_read==tx && wl_rx_write==rxw);
    assert(wl16_oob[0]==1 && wl16_oob[1]==1 && wl16_dbmode[0]==1 && wl16_dbmode[1]==1);
    assert(!writes_since(before,WL_BAR0+0x3a0));
    assert(writes_since(before,WL_BAR0+0x3a8)==(wl16_redrive?2:0));
    /* Lost TX doorbell: publish one harmless QRTR control while the peer is
     * paused, then recover only by an OOB notification with queued TX. */
    dev_busy=true;assert(!wl_qrtr_control(7,7,1,0x4000));dev_busy=false;
    tx=wl_tx_read;assert(wl_tx_read!=wl_tx_write);before=ne;
    dev_event(1,0,5u<<24,0x14000000|0x220000);
    assert(!wl_w11_events());
    assert(writes_since(before,WL_BAR0+0x3a0)==(wl16_redrive?1:0));
    if(wl16_redrive)assert(wl_tx_read==wl_tx_write);
    else {assert(wl_tx_read==tx);dev_work();assert(!wl_w11_events());}
    /* Reproduce the suspicious RP shape. Snapshot must report it without
     * changing context, advancing rings, or using RP as a doorbell address. */
    wl_channel(20)->rp=0x100000001ull;wl_channel(21)->rp=0x100000002ull;
    readonly16();assert(strstr(logbuf,"rp-in-ring 0"));
    assert(!wl_w11_events());
    /* Doorbell access faults fail closed and preserve allocations until reset. */
    if(wl16_redrive) {
        mode=20;dev_event(1,0,6u<<24,0x15000000|0x220000);
        assert(wl_w11_events() && !wl_w11_ok && !frees);
        mode=0;fw_faults=0;clear_error();
    }
    contain();
}
int main(void) {
    /* qrtr 6 (W20) and 7 (W23) carry the W15 knobs as well. */
    for(int q=0;q<=7;q++)for(int v=-1;v<=5;v++) {
        qrtr_arg=q;w16_arg=v;wl_probe_configure();
        assert(wl_variant_invalid==((q>=5&&q<=7) && (v<0 || v>4)));
        assert(wl16_variant==((q>=5&&q<=7) && v>=0 && v<=4?(unsigned)v:0));
        if(q<5)assert(!wl16_large && !wl16_redrive);
    }
    for(unsigned v=0;v<=4;v++)for(unsigned keep=0;keep<2;keep++) {
        boot16(v,0,keep);int e=finish_step();if(e)fprintf(stderr,"%s",logbuf);
        assert(!e && seen15==3 && wl12_txn==15 && wl_w11_ok);
        assert(observe_ticks==200 && late_indications==3);
        assert(wl_channel(20)->cfg==0x202 && wl_channel(21)->cfg==0x202);
        check_no_ce();readonly16();
        if(v)assert(strstr(logbuf,"channels started/RX posted") && strstr(logbuf,"CFG reply/before MODE") && strstr(logbuf,"W16 MODE queued") && strstr(logbuf,"W16 MODE reply"));
        else assert(!strstr(logbuf,"W16"));
        /* Wrap both IPCR rings repeatedly. Large variant also handles a
         * complete 8192-byte QRTR packet and an immediate 40-packet burst. */
        if(v) {
            uint8_t packet[8192]={0};put32(packet,1);put32(packet+4,1);
            put32(packet+8,7);put32(packet+12,42);put32(packet+20,peer_mtu()-32);
            put32(packet+24,1);put32(packet+28,0x4000);
            packet[32]=4;put16(packet+35,0x7777);put16(packet+37,peer_mtu()-39);
            packet[39]=0x71;put16(packet+40,peer_mtu()-42);
            dev_rx_packet(packet,peer_mtu());assert(!wl_w11_events());
            if(peer_n()==64) {
                for(unsigned j=0;j<40;j++)ind(0x7777,NULL,0);
                assert(!wl_w11_events());
            }
            for(unsigned j=0;j<140;j++) {
                assert(!wl_qrtr_control(7,7,1,0x4000));ind(0x7777,NULL,0);
                assert(!wl_w11_events());
            }
            readonly16();
        }
        if(keep){unsigned before=ne;wl_finish(28);assert(ne==before && !frees);contain();}
        else {wl_finish(28);assert(frees==allocations);dev_on=false;}
    }
    for(unsigned v=1;v<=4;v++) {
        notifications16(v);
        /* Silent MODE: one bounded wait, no retry/CE or free before contain. */
        boot16(v,22,true);assert(finish_step() && seen15==3 && !wl_w11_ok);
        assert(strstr(logbuf,"MODE pending 1000 ms") && strstr(logbuf,"W16 MODE timeout"));
        assert(strstr(logbuf,"pending RX QMI headers 0 (snapshot only)"));
        assert(!observe_ticks && !frees);check_no_ce();contain();
        /* Negative MODE reply and failed provisioning retain containment. */
        boot16(v,21,true);assert(finish_step() && seen15==3 && !wl_w11_ok);contain();
        boot16(v,0,true);io_case=9;assert(finish_step() && !seen15);contain();
        boot16(v,22,true);assert(finish_step());fail_bm=fail_perst=true;
        wl5_abort(28);assert(!frees && wl_ipcr.va && wl14_m3.va);
        fail_bm=fail_perst=false;contain();
    }
    puts("W16 legacy selectors, independent 64x8192 geometry/burst/wrap, exact QMI, OOB/DB_MODE recovery, read-only context diagnostics, failures and quarantine passed");
    return 0;
}
'''


def main():
    assert 'python3 tools/test-fuxi-wlan-w16.py' in (ROOT/'tools/ci/build.sh').read_text()
    m6 = w15.w14.w12.w11.w10.m6o.m6
    m6.PREAMBLE = m6.PREAMBLE.replace('static bool PE_parse_boot_argn(',
                                     'static int w16_arg;\nstatic bool PE_parse_boot_argn(')
    m6.PREAMBLE = m6.PREAMBLE.replace('if(!strcmp(name,"fuxiwlanw16")) { assert(size==sizeof(int)); return false; }',
                                     'if(!strcmp(name,"fuxiwlanw16")) { assert(size==sizeof(int)); memcpy(out,&w16_arg,size); return true; }')
    w15.TESTS = TESTS
    w15.main()


if __name__ == '__main__':
    main()
