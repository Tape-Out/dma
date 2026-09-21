"""dma 的行为测试台：给它一块预先装好数据的存储，让它搬一段，再核对。

顺带验数组译码——`src`/`dst`/`len` 三个数组步长 16、元素宽 4，
译码只查外层范围的话它们互相盖着，写 dst 会写进 src。

后半段验出错：访存回错时要置 `esta`、清字节数、不报完成、不写目的地；源或目的不是
字对齐时一次访存也不许发。访存模型在 0x1000 往上没有东西，回错，并逐拍记访存次数。

认矩阵：`channels` 决定例化几个通道。只有一个通道时「写第二通道不该动到第一
通道」无从谈起，那一步去掉——拿同一个通道冒充等于什么也没验。

结构上有一条要守：**存储只由服务规则写**。测试序列若也去写它，两条规则抢
同一个写口，而服务规则每拍都要驱动 always_enabled 的 ready/resp、必然更紧急，
bsc 就把序列规则丢掉——表现是超时。所以源区数据用 mkRegFileLoad 预先装好，
序列规则只读不写。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
ch = int((cfg.get("knobs") or {}).get("channels", 2))

# 通道少于两个就没有「另一个通道」可以互相盖
second = ('{second}'
          if ch >= 2 else '      2: noAction;')

SRC_BASE = 0x0000_0100
DST_BASE = 0x0000_0200
# 尾巴不足一字的那次搬到这里，背景值必须原封不动
DST2_BASE = 0x0000_0300
# 出错与不对齐的那几次都以这里为目的地，背景值必须原封不动
DST3_BASE = 0x0000_0340
# 访存模型在这里往上回错
HOLE = 0x0000_1000
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

ERRS = '\n  // ---- 出错与对齐 ----\n  // 读的那一头回错：不许报完成、不许写目的地；esta 置位、字节数清零、irq 抬起\n  rule errA (ph == ErrA);\n    case (s)\n      0: wr(rISTA, 32\'h1);\n      1: wr(rSRC0, hole);\n      2: wr(rDST0, dst3Base);\n      3: wr(rLEN0, 8);\n      default: begin ph <= ErrW; end\n    endcase\n    if (s < 4) s <= s + 1; else s <= 0;\n  endrule\n\n  rule errW (ph == ErrW);\n    if (s < 200) s <= s + 1; else begin ph <= ErrB; s <= 0; end\n  endrule\n\n  rule errB (ph == ErrB);\n    let x <- d.regs.access(RegReq { addr: rESTA, write: False, wdata: 0, wstrb: 4\'hF });\n    if (x.rdata[0] != 1) begin\n      $display("FAIL a read that answered with a bus error did not set esta: %08h", x.rdata);\n      bad <= True;\n    end\n    ph <= ErrC;\n  endrule\n\n  rule errC (ph == ErrC);\n    let x <- d.regs.access(RegReq { addr: rISTA, write: False, wdata: 0, wstrb: 4\'hF });\n    if (x.rdata[0] != 0) begin\n      $display("FAIL a move whose read hit a bus error was reported as done");\n      bad <= True;\n    end\n    ph <= ErrD;\n  endrule\n\n  rule errD (ph == ErrD);\n    let x <- d.regs.access(RegReq { addr: rLEN0, write: False, wdata: 0, wstrb: 4\'hF });\n    if (x.rdata != 0) begin\n      $display("FAIL a move that hit a bus error left len at %0d, so it runs again", x.rdata);\n      bad <= True;\n    end\n    ph <= ErrE;\n  endrule\n\n  rule errE (ph == ErrE);\n    Bit#(32) w = mem.sub(truncate(dst3Base >> 2));\n    Bool wrong = False;\n    if (w != 32\'hDEADBEEF) begin\n      $display("FAIL a move that failed on its first read still wrote the destination: %08h", w);\n      wrong = True;\n    end\n    if (!d.irq) begin\n      $display("FAIL esta is set but irq is low");\n      wrong = True;\n    end\n    if (wrong) bad <= True;\n    ph <= ErrF;\n    s  <= 0;\n  endrule\n\n  // 写的那一头回错\n  rule errF (ph == ErrF);\n    case (s)\n      0: wr(rESTA, 32\'h1);\n      1: wr(rSRC0, srcBase);\n      2: wr(rDST0, hole);\n      3: wr(rLEN0, 4);\n      default: begin ph <= ErrG; end\n    endcase\n    if (s < 4) s <= s + 1; else s <= 0;\n  endrule\n\n  rule errG (ph == ErrG);\n    if (s < 200) s <= s + 1; else begin ph <= ErrH; s <= 0; end\n  endrule\n\n  rule errH (ph == ErrH);\n    let x <- d.regs.access(RegReq { addr: rESTA, write: False, wdata: 0, wstrb: 4\'hF });\n    if (x.rdata[0] != 1) begin\n      $display("FAIL a write that answered with a bus error did not set esta: %08h", x.rdata);\n      bad <= True;\n    end\n    ph <= ErrI;\n  endrule\n\n  rule errI (ph == ErrI);\n    let x <- d.regs.access(RegReq { addr: rISTA, write: False, wdata: 0, wstrb: 4\'hF });\n    if (x.rdata[0] != 0) begin\n      $display("FAIL a move whose write hit a bus error was reported as done");\n      bad <= True;\n    end\n    ph <= AlnA;\n    s  <= 0;\n  endrule\n\n  // 地址不是字对齐：一次访存也不许发，esta 置位\n  rule alnA (ph == AlnA);\n    case (s)\n      0: wr(rESTA, 32\'h1);\n      1: begin wr(rSRC0, srcBase + 1); reqs[1] <= 0; end\n      2: wr(rDST0, dst3Base);\n      3: wr(rLEN0, 4);\n      default: begin ph <= AlnW; end\n    endcase\n    if (s < 4) s <= s + 1; else s <= 0;\n  endrule\n\n  rule alnW (ph == AlnW);\n    if (s < 200) s <= s + 1; else begin ph <= AlnB; s <= 0; end\n  endrule\n\n  // 访存次数与存储分两条规则读：一条规则里既读 reqs 的口 1（排在服务规则之后）\n  // 又读 mem（排在服务规则写之前），与每拍都跑的服务规则成环，整条被挡掉\n  rule alnB (ph == AlnB);\n    if (reqs[1] != 0) begin\n      $display("FAIL a source address that is not word aligned still made %0d bus requests", reqs[1]);\n      bad <= True;\n    end\n    ph <= AlnB2;\n  endrule\n\n  rule alnB2 (ph == AlnB2);\n    Bit#(32) w = mem.sub(truncate(dst3Base >> 2));\n    if (w != 32\'hDEADBEEF) begin\n      $display("FAIL a source address that is not word aligned still wrote the destination: %08h", w);\n      bad <= True;\n    end\n    ph <= AlnC;\n  endrule\n\n  rule alnC (ph == AlnC);\n    let x <- d.regs.access(RegReq { addr: rESTA, write: False, wdata: 0, wstrb: 4\'hF });\n    if (x.rdata[0] != 1) begin\n      $display("FAIL a source address that is not word aligned did not set esta");\n      bad <= True;\n    end\n    ph <= AlnD;\n    s  <= 0;\n  endrule\n\n  rule alnD (ph == AlnD);\n    case (s)\n      0: wr(rESTA, 32\'h1);\n      1: begin wr(rSRC0, srcBase); reqs[1] <= 0; end\n      2: wr(rDST0, dst3Base + 2);\n      3: wr(rLEN0, 4);\n      default: begin ph <= AlnW2; end\n    endcase\n    if (s < 4) s <= s + 1; else s <= 0;\n  endrule\n\n  rule alnW2 (ph == AlnW2);\n    if (s < 200) s <= s + 1; else begin ph <= AlnE; s <= 0; end\n  endrule\n\n  rule alnE (ph == AlnE);\n    if (reqs[1] != 0) begin\n      $display("FAIL a destination address that is not word aligned still made %0d bus requests", reqs[1]);\n      bad <= True;\n    end\n    ph <= AlnF;\n  endrule\n\n  rule alnF (ph == AlnF);\n    let x <- d.regs.access(RegReq { addr: rESTA, write: False, wdata: 0, wstrb: 4\'hF });\n    if (x.rdata[0] != 1) begin\n      $display("FAIL a destination address that is not word aligned did not set esta");\n      bad <= True;\n    end\n    ph <= EstaClr;\n  endrule\n\n  rule estaClr (ph == EstaClr);\n    wr(rESTA, 32\'h1);\n    ph <= EstaW;\n    s  <= 0;\n  endrule\n\n  rule estaW (ph == EstaW);\n    if (s < 4) s <= s + 1; else ph <= EstaChk;\n  endrule\n\n  rule estaChk (ph == EstaChk);\n    if (d.irq) begin\n      $display("FAIL esta was cleared but irq stays high");\n      bad <= True;\n    end\n    ph <= Done;\n  endrule\n'

(out / f"Dma{label}Tb.bsv").write_text(f'''package Dma{label}Tb;

import RegFile::*;
import RegIf::*;
import Dma::*;

// 由 tb/mkdmatb.py 生成，勿手改。
// 这一点：channels={ch}

Bit#(12) rCTRL = 12'h000;
Bit#(12) rSRC0 = 12'h100;
Bit#(12) rDST0 = 12'h104;
Bit#(12) rLEN0 = 12'h108;
Bit#(12) rSRC1 = 12'h110;   // 第二个通道，步长 16
Bit#(12) rISTA = 12'h400;
Bit#(12) rESTA = 12'h404;

Bit#(32) srcBase = 32'h{SRC_BASE:08X};
Bit#(32) dstBase = 32'h{DST_BASE:08X};
Bit#(32) dst2Base = 32'h{DST2_BASE:08X};
Bit#(32) dst3Base = 32'h{DST3_BASE:08X};
Bit#(32) hole     = 32'h{HOLE:08X};
Integer  nwords  = {NWORDS};

typedef enum {{ Cfg, CheckMap, Go, Wait, Stop, CheckStop, Verify,
               Part, WaitPart, CheckPart, ErrA, ErrW, ErrB, ErrC, ErrD, ErrE, ErrF, ErrG,
               ErrH, ErrI, AlnA, AlnW, AlnB, AlnB2, AlnC, AlnD, AlnW2, AlnE, AlnF,
               EstaClr, EstaW, EstaChk, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkDma{label}Tb(Empty);
  DmaIfc#(12, 32, {ch}) d <- mkDma(DmaCfg {{ none: ? }});
  // 源区数据预先装好，测试序列于是只读不写，不跟服务规则抢写口
  RegFile#(Bit#(8), Bit#(32)) mem <- mkRegFileLoad("{hexf}", 0, {DEPTH - 1});

  Reg#(Phase)    ph  <- mkReg(Cfg);
  Reg#(Bit#(8))  s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);
  // 访存次数：服务规则写口 0，检查规则读写口 1
  Reg#(Bit#(16)) reqs[2] <- mkCReg(2, 0);

  // 存储只归这一条规则写
  rule serve;
    let r = d.mem.req;
    Bit#(8)  i = truncate(r.addr >> 2);
    Bit#(32) old = mem.sub(i);
    Bool miss = r.addr >= hole;
    if (d.mem.valid && r.write && !miss) mem.upd(i, applyStrb(old, r.wdata, r.wstrb));
    if (d.mem.valid) reqs[0] <= reqs[0] + 1;
    d.mem.ready(d.mem.valid);
    d.mem.resp(d.mem.valid, RegRsp {{ rdata: miss ? 0 : old, err: miss }});
  endrule

  rule tick;
    cyc <= cyc + 1;
    if (cyc > 40000) begin
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
    if (x.rdata[0] == 1) begin ph <= Stop; s <= 0; end
  endrule

  // 搬完就该停下。挑通道的条件是「字节数不为零」，而搬完之后没有任何人把它清掉——
  // 同一个描述符会被反复执行：中断反复置位、目的地被同一份数据反复覆盖、总线一直
  // 被占着。数据核对那一关看不出来，因为搬的每一遍内容都一样。
  rule stopping (ph == Stop);
    if (s == 0) wr(rISTA, 32'h1);      // 先把刚才那一次清掉
    if (s > 200) begin ph <= CheckStop; s <= 0; end
    else s <= s + 1;
  endrule

  rule checkStop (ph == CheckStop);
    let x <- d.regs.access(RegReq {{ addr: rISTA, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[0] == 1) begin
      $display("FAIL the channel ran again on its own after finishing");
      bad <= True;
    end
    ph <= Verify;
    s  <= 0;
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
    if (s + 1 == fromInteger(nwords)) begin ph <= Part; s <= 0; end
    else s <= s + 1;
  endrule

  // 字节数不是四的倍数：最后一拍只该写剩下的那几个字节。整字写出去会把目的地
  // 后面的字节一起踩掉，而搬运本身「看起来是对的」——被踩的那几个字节没人核对。
  rule part (ph == Part);
    case (s)
      0: wr(rDST0, dst2Base);
      1: wr(rLEN0, 6);           // 六个字节：一整字加半字
      default: noAction;
    endcase
    if (s > 2) begin ph <= WaitPart; s <= 0; end
    else s <= s + 1;
  endrule

  rule waitPart (ph == WaitPart);
    let x <- d.regs.access(RegReq {{ addr: rISTA, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[0] == 1) begin ph <= CheckPart; s <= 0; end
  endrule

  rule checkPart (ph == CheckPart);
    Bit#(32) w0 = mem.sub(truncate(dst2Base >> 2));
    Bit#(32) w1 = mem.sub(truncate((dst2Base >> 2) + 1));
    Bool wrong = False;
    if (w0 != 32'hA5A50000) begin
      $display("FAIL partial move, first word is %08h want a5a50000", w0);
      wrong = True;
    end
    // 背景是 deadbeef，只有低两个字节该被换掉
    if (w1 != 32'hDEAD0001) begin
      $display("FAIL a six byte move wrote past the end: %08h want dead0001", w1);
      wrong = True;
    end
    if (wrong) bad <= True;
    ph <= ErrA;
    s  <= 0;
  endrule
{ERRS}
  rule fin (ph == Done);
    if (bad) $display("FAILED");
    else $display("PASS dma: moved %0d words, kept inside a partial tail, stopped, the channel arrays do not overlap, and a bus error or a misaligned address sets esta without moving anything", nwords);
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
''', encoding="utf-8")
print(f"  dma 行为测试台就位：channels={ch}，源区 {NWORDS} 字预装在 {hexf.name}")
