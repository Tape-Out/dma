"""dma 的行为测试台：给它一块预先装好数据的存储，让它搬一段，再核对。

顺带验数组译码——`src`/`dst`/`len` 三个数组步长 16、元素宽 4，
译码只查外层范围的话它们互相盖着，写 dst 会写进 src。

结构上有一条要守：**存储只由服务规则写**。测试序列若也去写它，两条规则抢
同一个写口，而服务规则每拍都要驱动 always_enabled 的 ready/resp、必然更紧急，
bsc 就把序列规则丢掉——表现是超时。所以源区数据用 mkRegFileLoad 预先装好，
序列规则只读不写。
"""
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
out.mkdir(parents=True, exist_ok=True)

SRC_BASE = 0x0000_0100
DST_BASE = 0x0000_0200
NWORDS = 4
DEPTH = 256

img = []
for i in range(DEPTH):
    if SRC_BASE // 4 <= i < SRC_BASE // 4 + NWORDS:
        img.append(0xA5A50000 + (i - SRC_BASE // 4))
    else:
        img.append(0xDEADBEEF)
hexf = out / "dma_mem.hex"
hexf.write_text("\n".join(f"{v:08X}" for v in img) + "\n", encoding="utf-8")

(out / "DmaTb.bsv").write_text(f'''package DmaTb;

import RegFile::*;
import RegIf::*;
import Dma::*;

// 由 tb/mkdmatb.py 生成，勿手改。

Bit#(12) rCTRL = 12'h000;
Bit#(12) rSRC0 = 12'h100;
Bit#(12) rDST0 = 12'h104;
Bit#(12) rLEN0 = 12'h108;
Bit#(12) rSRC1 = 12'h110;   // 第二个通道，步长 16
Bit#(12) rISTA = 12'h400;

Bit#(32) srcBase = 32'h{SRC_BASE:08X};
Bit#(32) dstBase = 32'h{DST_BASE:08X};
Integer  nwords  = {NWORDS};

typedef enum {{ Cfg, CheckMap, Go, Wait, Verify, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkDmaTb(Empty);
  DmaIfc#(12, 32, 2) d <- mkDma(DmaCfg {{ none: ? }});
  // 源区数据预先装好，测试序列于是只读不写，不跟服务规则抢写口
  RegFile#(Bit#(8), Bit#(32)) mem <- mkRegFileLoad("{hexf}", 0, {DEPTH - 1});

  Reg#(Phase)    ph  <- mkReg(Cfg);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);

  // 存储只归这一条规则写
  rule serve;
    let r = d.mem.req;
    Bit#(8)  i = truncate(r.addr >> 2);
    Bit#(32) old = mem.sub(i);
    if (d.mem.valid && r.write) mem.upd(i, applyStrb(old, r.wdata, r.wstrb));
    d.mem.ready(d.mem.valid);
    d.mem.resp(d.mem.valid, RegRsp {{ rdata: old, err: False }});
  endrule

  rule tick;
    cyc <= cyc + 1;
    if (cyc > 20000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(12) a, Bit#(32) v) = action
    let _ <- d.regs.access(RegReq {{ addr: a, write: True,
                                     wdata: v, wstrb: 4'hF }});
  endaction;

  rule cfg (ph == Cfg);
    case (s)
      0: wr(rSRC0, srcBase);
      1: wr(rDST0, dstBase);
      2: wr(rSRC1, 32'hFFFF_FFFF);   // 第二通道写个显眼的，验数组不互相盖
      default: begin ph <= CheckMap; s <= 0; end
    endcase
    if (s < 3) s <= s + 1;
  endrule

  // 写第二通道不该动到第一通道
  rule checkMap (ph == CheckMap);
    let x <- d.regs.access(RegReq {{ addr: rSRC0, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    if (x.rdata != srcBase) begin
      $display("FAIL channel 0 source is %08h, want %08h (arrays overlap)",
               x.rdata, srcBase);
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= Go;
    s  <= 0;
  endrule

  rule go (ph == Go);
    case (s)
      0: wr(rLEN0, fromInteger(nwords * 4));
      1: wr(rCTRL, 32'h1);           // en
      default: begin ph <= Wait; s <= 0; end
    endcase
    if (s < 2) s <= s + 1;
  endrule

  rule waitDone (ph == Wait);
    let x <- d.regs.access(RegReq {{ addr: rISTA, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[0] == 1) begin ph <= Verify; s <= 0; end
  endrule

  rule verify (ph == Verify);
    Bit#(32) got  = mem.sub(truncate((dstBase >> 2) + zeroExtend(s)));
    Bit#(32) want = 32'hA5A50000 + zeroExtend(s);
    Bool wrong = False;
    if (got != want) begin
      $display("FAIL word %0d: got %08h want %08h", s, got, want);
      wrong = True;
    end
    if (wrong) bad <= True;
    if (s + 1 == fromInteger(nwords)) ph <= Done;
    else s <= s + 1;
  endrule

  rule fin (ph == Done);
    if (bad) $display("FAILED");
    else $display("PASS dma: moved %0d words, and the channel arrays do not overlap",
                  nwords);
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
''', encoding="utf-8")
print(f"  dma 行为测试台就位，源区 {NWORDS} 字预装在 {hexf.name}")
