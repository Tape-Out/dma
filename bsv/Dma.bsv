package Dma;

import Vector::*;
import ConfigReg::*;
import RegIf::*;
import DmaRegs::*;

// 本包不认识任何总线：控制口是中立的 RegIf，搬数据的那一口是中立的 RegManager，
// 接哪种总线由 wrap 或装配决定。
typedef struct {
  Bit#(0) none;
} DmaCfg;

typedef enum { Idle, Read, Write } Step deriving (Bits, Eq, FShow);

interface DmaIfc#(numeric type aw, numeric type dw, numeric type channels);
  interface RegIf#(aw, dw) regs;
  // 搬数据要自己发访存。装配还没有仲裁多个发起方，所以这一口先引到顶层，
  // 由外面接——这也正是「独立可流片」该有的样子。
  interface RegManager#(32, 32) mem;
  (* always_ready *) method Bool irq;
endinterface

module mkDma#(DmaCfg cfg)(DmaIfc#(aw, dw, channels))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 12, aw), Add#(_b, 1, dw),
              Add#(_c, 32, dw), Add#(_d, 24, dw), Add#(_e, 8, dw),
              Add#(_f, channels, 8), Log#(TAdd#(channels, 1), _g));

  DmaRegsIfc#(aw, dw, channels) r <- mkDmaRegs;

  Reg#(Step)                    step <- mkConfigReg(Idle);
  Reg#(Bit#(TLog#(TAdd#(channels, 1)))) ch <- mkReg(0);
  Reg#(Bit#(32))                cur  <- mkReg(0);   // 已搬字节
  Reg#(Bit#(32))                data <- mkReg(0);

  Wire#(Bool)          rdy  <- mkBypassWire;
  Wire#(Bool)          rspV <- mkBypassWire;
  Wire#(RegRsp#(32))   rspX <- mkBypassWire;

  // 轮到哪个通道：编号小的先走，够简单也够可预期
  rule pick (step == Idle && r.ctrl_en == 1 && r.ctrl_rst == 0);
    Bit#(TLog#(TAdd#(channels, 1))) sel = 0;
    Bool any = False;
    for (Integer i = 0; i < valueOf(channels); i = i + 1)
      if (!any && r.len[i] != 0) begin
        sel = fromInteger(i);
        any = True;
      end
    if (any) begin
      ch   <= sel;
      cur  <= 0;
      step <= Read;
    end
  endrule

  rule reset_all (r.ctrl_rst == 1);
    step <= Idle;
    cur  <= 0;
  endrule

  rule advance (step != Idle && rspV);
    if (step == Read) begin
      data <= rspX.rdata;
      step <= Write;
    end else begin
      // 一次搬一个字。搬完这一通道就置位状态位，软件写一清除。
      Bit#(32) next = cur + 4;
      if (next >= zeroExtend(r.len[ch])) begin
        Bit#(8) hit = 0;
        hit[ch] = 1;
        r.ista_set(hit);
        // 字节数清零，这一路才算做完。挑通道看的就是它不为零——不清的话
        // 同一个描述符会被反复执行：中断反复置位、目的地被同一份数据反复
        // 覆盖、总线一直被占着，而数据核对完全看不出来，每一遍内容都一样。
        r.len_in(ch, 0);
        step <= Idle;
      end else begin
        cur  <= next;
        step <= Read;
      end
    end
  endrule

  interface regs = r.regs;
  interface RegManager mem;
    method Bool valid = step != Idle;
    method RegReq#(32, 32) req;
      // 尾巴不足一整字时只使能剩下的那几个字节。字节数是字节数，而搬运一次走一字，
      // 整字写出去会把目的地后面的字节一起踩掉——而且核对搬运结果完全看不出来，
      // 被踩的那几个字节不在任何人的检查范围里。
      Bit#(32) rem  = zeroExtend(r.len[ch]) - cur;
      Bit#(4)  tail = (4'b0001 << rem[1:0]) - 1;
      return RegReq {
        addr:  (step == Read ? r.src[ch] : r.dst[ch]) + cur,
        write: step == Write,
        wdata: data,
        wstrb: (step == Write && rem < 4) ? tail : 4'hF };
    endmethod
    method Action ready(Bool v); rdy._write(v); endmethod
    method Action resp(Bool v, RegRsp#(32) x);
      rspV._write(v);
      rspX._write(x);
    endmethod
  endinterface
  method Bool irq = r.ista != 0;
endmodule

endpackage
