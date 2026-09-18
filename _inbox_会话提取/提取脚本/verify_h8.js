/* 用 H8 的验收判据自动核对串口日志，逐条给 PASS/FAIL。
   用法：node verify_h8.js <日志路径> */
const fs = require('fs');
const path = process.argv[2];
const chunks = [];
for (const l of fs.readFileSync(path, 'utf8').split(/\r?\n/)) {
    if (!l.includes('"dir":"rx"')) continue;
    let o; try { o = JSON.parse(l); } catch { continue; }
    if (o.text) chunks.push(o.text);
}
const all = chunks.join('');

const re = /\[ALT\] req=(\d+) mode=(\d+) st=(\d+) sw=0x([0-9A-Fa-f]+) ct=0x([0-9A-Fa-f]+) gnd=(\d+) stick=(\d+) rate=([+\-0-9.]+) h_sp=([0-9.]+) h=([0-9.]+) v=(\d+) gt=(\d+)ms thr=(\d+)/g;
const R = [];
let m;
while ((m = re.exec(all)) !== null) {
    R.push({
        req: +m[1], mode: +m[2], st: +m[3], sw: parseInt(m[4], 16), ct: parseInt(m[5], 16),
        gnd: +m[6], stick: +m[7], rate: +m[8], h_sp: +m[9], h: +m[10], v: +m[11], gt: +m[12], thr: +m[13],
    });
}
console.log(`样本数 = ${R.length}   （打印周期 500ms ⇒ 约 ${(R.length * 0.5).toFixed(0)} 秒）\n`);

let pass = 0, fail = 0, skip = 0;
function verdict(name, ok, detail, skipped) {
    if (skipped) { console.log(`[ 未测 ] ${name}\n         ${detail}`); skip++; return; }
    if (ok) { console.log(`[ 通过 ] ${name}\n         ${detail}`); pass++; }
    else { console.log(`[ 失败 ] ${name}\n         ${detail}`); fail++; }
}

/* ① 普通模式：thr = 1000 + 0.5*stick
   ⚠️ 不能只比"当前杆量的映射值"：杆在扫动时限速器会滞后（上升 800、下降 300µs/s，
      而打印周期 500ms ⇒ 最多滞后 400/150µs）。物理上正确的判据是
      "thr 必须落在【上一拍映射值, 本拍映射值】之间"。
   另外单独统计"杆量连续两拍不变"的样本，那些必须精确相等。 */
{
    const rows = R.filter(r => r.mode === 0 && (r.ct & 1));
    const map = s => 1000 + 0.5 * s;
    let outOfBand = 0, worstOut = null, stable = 0, stableBad = 0;
    for (let i = 1; i < R.length; i++) {
        const a = R[i - 1], b = R[i];
        if (a.mode !== 0 || b.mode !== 0 || !(a.ct & 1) || !(b.ct & 1)) continue;
        const lo = Math.min(map(a.stick), map(b.stick)) - 2;
        const hi = Math.max(map(a.stick), map(b.stick)) + 2;
        if (b.thr < lo || b.thr > hi) { outOfBand++; if (!worstOut) worstOut = { a, b, lo, hi }; }
        if (a.stick === b.stick) { stable++; if (Math.abs(b.thr - map(b.stick)) > 2) stableBad++; }
    }
    const s690 = rows.filter(r => r.stick === 690).map(r => r.thr);
    verdict('① 普通模式油门映射与改动前一致（thr = 1000 + 0.5×stick）',
        rows.length > 0 && outOfBand === 0 && stableBad === 0,
        rows.length === 0 ? '没有普通模式+解锁的样本' :
            `${rows.length} 个样本；限速带外 ${outOfBand} 个` +
            (worstOut ? `（例 stick ${worstOut.a.stick}→${worstOut.b.stick}，thr ${worstOut.a.thr}→${worstOut.b.thr}，允许 ${worstOut.lo}~${worstOut.hi}）` : '') +
            `；杆量连续两拍不变的 ${stable} 个样本中不精确的 ${stableBad} 个` +
            `；杆 690 → thr=${[...new Set(s690)].join(',')}（改动前也是 1345）`);
}

/* ② 定高 + 解锁 + 杆最低 + 停手 -> thr = 1000（走的是新代码路径） */
{
    const rows = R.filter(r => r.mode === 1 && (r.ct & 1) && r.st === 4 && r.stick <= 50);
    const bad = rows.filter(r => r.thr > 1010);
    verdict('② 定高+解锁+杆最低+停手 → thr=1000（真正的 GetThrottleUs 路径）',
        rows.length > 0 && bad.length === 0,
        rows.length === 0 ? '没有这种样本' :
            `${rows.length} 个样本全部 thr=1000（thr 值集合 {${[...new Set(rows.map(r => r.thr))].join(',')}}）` +
            `，其中 ct=0x7（已解锁）的样本 ${rows.filter(r => (r.ct & 0xF) === 0x7).length} 个`);
}

/* ③ 定高 + 停手 + 满杆：h_sp 涨过 4cm 之前 thr 必须恒 1000 */
{
    const rows = R.filter(r => r.mode === 1 && r.st === 4 && r.stick >= 650 && r.h_sp < 3.9);
    const bad = rows.filter(r => r.thr > 1010);
    const crossed = R.filter(r => r.mode === 1 && r.stick >= 650 && r.h_sp >= 4.0 && r.h_sp < 6.0);
    verdict('③ 定高下推杆：h_sp 涨过 4cm 之前 thr 恒 1000（电机不转）',
        rows.length > 0 && bad.length === 0,
        rows.length === 0 ? '没有 h_sp<4cm 且满杆的样本' :
            `${rows.length} 个样本全部 thr=1000（最大 stick=${Math.max(...rows.map(r => r.stick))}）` +
            `；越过 4cm 后立刻转 st=1 thr=1100 的样本 ${crossed.length} 个` +
            `（h_sp ${Math.min(...crossed.map(r => r.h_sp))}~${Math.max(...crossed.map(r => r.h_sp))}cm，thr 集合 {${[...new Set(crossed.map(r => r.thr))].join(',')}}）`);
}

/* ★ 核心不变式：定高模式下 thr 永不超过 1100 */
{
    const rows = R.filter(r => r.mode === 1);
    const over = rows.filter(r => r.thr > 1100);
    const thrs = [...new Set(rows.map(r => r.thr))].sort((a, b) => a - b);
    verdict('★ 不变式：定高模式下 thr 恒 ≤ 1100（任何杆量、任何高度）',
        rows.length > 0 && over.length === 0,
        `${rows.length} 个定高样本，thr 只出现过 {${thrs.join(',')}}；超过 1100 的 ${over.length} 个`);
}

/* ④ 控制启用：st=1 时 thr=1100，且与杆量无关 */
{
    const rows = R.filter(r => r.mode === 1 && r.st === 1);
    const thrs = [...new Set(rows.map(r => r.thr))].sort((a, b) => a - b);
    const sticks = [...new Set(rows.map(r => r.stick))].sort((a, b) => a - b);
    verdict('④ 控制启用（st=1）时 thr=1100，且不跟杆走',
        rows.length > 0 && thrs.every(t => t === 1100),
        rows.length === 0 ? '没有 st=1 的样本' :
            `${rows.length} 个样本，thr 集合 {${thrs.join(',')}}；涉及 ${sticks.length} 种杆量（${sticks[0]}~${sticks[sticks.length - 1]}）`);
}

/* ⑤ 杆回中段：h_sp 冻结 */
{
    const rows = R.filter(r => r.mode === 1 && r.st === 1 && r.rate === 0);
    const hs = [...new Set(rows.map(r => r.h_sp))];
    verdict('⑤ 杆在中段（rate=0）时 h_sp 冻结不回落',
        rows.length > 0 && hs.length <= 2,
        rows.length === 0 ? '没有中段悬停样本' :
            `${rows.length} 个样本，h_sp 只出现过 {${hs.join(',')}}cm，thr 集合 {${[...new Set(rows.map(r => r.thr))].join(',')}}`);
}

/* ⑥ 杆拉到底 -> 500ms 去抖后重新进停手 */
{
    const back = R.filter((r, i) => r.mode === 1 && r.st === 4 && i > 0 && R[i - 1].st === 1);
    verdict('⑥ 杆拉到底 → 重新进停手（500ms 去抖）',
        back.length > 0 && back.every(r => r.gt >= 500),
        back.length === 0 ? '没有 st 1→4 的转换' :
            `${back.length} 次转换，进停手那一刻 gt=${[...new Set(back.map(r => r.gt))].join(',')}ms` +
            `，对应 stick=${[...new Set(back.map(r => r.stick))].join(',')}`);
}

/* ⑦ 上锁 -> thr=1000 */
{
    const rows = R.filter(r => r.mode === 1 && !(r.ct & 1));
    const bad = rows.filter(r => r.thr !== 1000);
    verdict('⑦ 上锁 → thr 立即 1000',
        rows.length > 0 && bad.length === 0,
        `${rows.length} 个未解锁的定高样本，thr 集合 {${[...new Set(rows.map(r => r.thr))].join(',')}}`);
}

/* ⑨ 模式切换前后油门连续 */
{
    const sw = [];
    for (let i = 1; i < R.length; i++) {
        if (R[i].mode !== R[i - 1].mode) sw.push({ from: R[i - 1], to: R[i] });
    }
    const jumps = sw.filter(s => Math.abs(s.to.thr - s.from.thr) > 20);
    verdict('⑨ 模式切换前后油门连续',
        sw.length > 0 && jumps.length === 0,
        sw.length === 0 ? '没有模式切换' :
            `${sw.length} 次模式切换，油门跳变 >20µs 的 ${jumps.length} 次` +
            (jumps.length ? `（例：${jumps[0].from.mode}→${jumps[0].to.mode}，${jumps[0].from.thr}→${jumps[0].to.thr}）` : ''));
}

/* ⑧ 举到 1.3m 以上 */
{
    const high = R.filter(r => r.h > 5);
    verdict('⑧ 举到 1.3m 以上（v=0、sw 掉两位、st 仍 4、thr 仍 1000）',
        false,
        high.length === 0
            ? `日志里 h 最大只有 ${Math.max(...R.map(r => r.h))}cm —— 飞机全程在地面上，这一条【还没做】`
            : `有 ${high.length} 个 h>5cm 的样本，max h=${Math.max(...R.map(r => r.h))}cm`,
        high.length === 0);
}

console.log(`\n===== 通过 ${pass} / 失败 ${fail} / 未测 ${skip} =====`);
console.log('\n地面读数统计（停手期间）：');
{
    const g = R.filter(r => r.gnd === 1).map(r => r.h);
    if (g.length) {
        g.sort((a, b) => a - b);
        console.log(`  h = ${g[0]} ~ ${g[g.length - 1]} cm，中位 ${g[Math.floor(g.length / 2)]}cm，均值 ${(g.reduce((a, b) => a + b, 0) / g.length).toFixed(2)}cm（样本 ${g.length}）`);
    }
}
console.log('\n推杆期间 h_sp 的最大值：' + Math.max(...R.map(r => r.h_sp)) + 'cm（飞机没离地，是台架特有的现象）');
