/* 核对 2026-09-15 起的打印格式 [V] 行，并按 **H14** 的验收判据逐条给结论。
   用法：node verify_v.js <审计日志.jsonl> [更多日志...]
   日志位置：C:\Users\fool\.dsh\serial-console\logs\

   [V] 行的字段（18 项，四段）：
     [V] raw=<mm> | h=<cm> vz=<cm/s> v=<0|1> | req mode st sw ct stick h_sp gt fs fr | vzsp corr i thr

   ⚠️ 版本对应关系（搞错了会得到"解析不到任何 [V] 行"）：
     v0.19 及更早   没有 `vzsp=`       ⇒ 用 git 历史里那一版 verify_v.js
     v0.18 及更早   没有 `fs=` / `fr=` ⇒ 同上
     v0.15 及更早   没有 `i=`          ⇒ 同上
     v0.16 之前     是 [TOF]+[ALT] 两行 ⇒ 用 verify_h8.js
     本脚本         v0.20 起（H13/H14：字段没变，只是判据增加了）

   ── 判据陷阱清单（H8~H13 合计踩了十次，全在这里）────────────────
   ① 判据不能拿"本拍杆量算出的映射值"直接比 thr：杆扫动时限速器会滞后
      （上升 800、下降 300µs/s，打印周期 500ms ⇒ 最多滞后 150µs）。
      正确判据是"thr 必须落在【上一拍映射值, 本拍映射值】之间"。
   ② 判据必须考虑**钳位**：任何"应该变化 X"的断言，先问"这段行程够不够"。
   ③ `corr=0` / `vz` 印成 `0.0` 的样本**不带符号信息**，按符号分类的判据必须排除它们。
   ④ 判"积分/斜坡"的方向时，不能拿两个端点的**瞬时值**去比（端点之间可能有方向反转或丢行）。
   ⑤ **"单调不增"只对下降段成立** —— 套到所有状态上是错的：st=1 时慢层正在积分，
      油门本来就该会上升。
   ⑥ 计时类判据要记得"**进入失效的那一拍不累加计时**"，所以 2 秒要 501 拍才满。
   ⑦ ⭐ H13 起**负方向的大误差不能随便造**：`h_est` 一旦超过 125cm 会先触发**高度越界失效**
      （st=6 外环冻结），量到的是冻结值而不是限幅值。那是正确行为，但会误报判据。
   ⑧ ⭐ H13 起"符号"的参照物变了：内环的误差是 `vz − vzsp`（**不是** `vz − 0`）。
      拿 `vz` 当参照在 vzsp 较大时会误报。外环的误差是 `h_sp − h`。
   ⑨ ⭐⭐ H14 起"解手"是**边沿**（上一拍杆在低位、这一拍推过 200），**不是电平**。
      所以日志里出现"杆已经 >200 而 st 仍是 4"是**正常的**（锁着时推的杆、或杆没经过低位
      就推上去）—— 这是刻意设计，**不要**写成失败判据。
   ⑩ ⭐⭐ H14 起 `st=4` 段的**长度**有意义：只持续 ≤1 秒、且两侧都是 `st=1`
      ⇒ 解手后被判回停手 = **抖**（本次最危险的错误）。
      正常的"落地 → 再起飞"里 `st=4` 会持续更久（要去抖 + 飞行员重新推杆）。
*/
const fs = require('fs')

const RE = /\[V\] raw=(\d+)mm \| h=(-?[\d.]+)cm vz=([+-][\d.]+)cm\/s v=(\d) \| req=(\d+) mode=(\d+) st=(\d+) sw=0x([0-9A-Fa-f]+) ct=0x([0-9A-Fa-f]+) stick=(\d+) h_sp=([\d.]+) gt=(\d+)ms fs=(\d+)ms fr=(\d+) \| vzsp=([+-][\d.]+) corr=([+-][\d.]+) i=([\d.]+) thr=(\d+)/g

/* H13 的常量（改 vertical_control.h 就必须改这里） */
const OUT_DEAD = 2.0        // VERTICAL_OUTER_DEADBAND_CM
const OUT_MAX = 20.0        // VERTICAL_OUTER_VZ_SP_MAX_CMS

/* H12 的常量（改 vertical_control.h 就必须改这里） */
const THR_MIN = 1150        // VERTICAL_CONTROL_THROTTLE_MIN_US
const THR_MAX = 1650        // VERTICAL_CONTROL_THROTTLE_MAX_US
const I_START = 1250        // VERTICAL_THROTTLE_START_US
const CORR_LIM = 50         // VERTICAL_INNER_CORR_LIMIT_US
const PWM_MIN = 1000        // MOTOR_PWM_MIN_US
const ATT_BYPASS = 1100     // MIXER_PWM_EFFECTIVE_MIN_US（thr 一旦 ≤ 它，姿态环被旁路）
const MAP_MAX_DEFAULT = 2000 // 满杆油门默认值
const FS_HOLD_MS = 2000     // VERTICAL_FS_HOLD_MS
const FS_RAMP = 30          // VERTICAL_FS_RAMP_US_PER_S
const PRINT_PERIOD_S = 0.5  // tofTask 的打印周期

/* 状态码 → 名字 */
const ST_NAME = { 0: '普通', 1: '定高在干活', 4: '停手', 5: '已进未使能', 6: '失效保持', 7: '失效下降' }
const FR_NAME = { 0: '无', 1: 'ToF读数失效', 2: '姿态超限', 3: '高度越界' }

const R = []
let rawText = ''
for (const p of process.argv.slice(2)) {
  for (const line of fs.readFileSync(p, 'utf8').split(/\r?\n/)) {
    if (!line.includes('"dir":"rx"')) continue
    let o
    try { o = JSON.parse(line) } catch { continue }
    if (o.text) rawText += o.text
  }
}
let m
while ((m = RE.exec(rawText)) !== null) {
  R.push({
    raw: +m[1], h: +m[2], vz: +m[3], v: +m[4],
    req: +m[5], mode: +m[6], st: +m[7],
    sw: parseInt(m[8], 16), ct: parseInt(m[9], 16),
    stick: +m[10], hsp: +m[11], gt: +m[12], fsMs: +m[13], fr: +m[14],
    vzsp: +m[15], corr: +m[16], i: +m[17], thr: +m[18],
  })
}

console.log(`日志 ${process.argv.length - 2} 份，rx 文本 ${rawText.length} 字符`)
console.log(`[V] 样本 ${R.length}（打印周期 500ms ⇒ 约 ${(R.length * 0.5).toFixed(0)} 秒）`)
if (R.length === 0) {
  console.log('\n没有解析到任何 [V] 行。可能原因：')
  console.log('  ① 固件是 v0.18 或更早（没有 fs=/fr= 字段）—— 那一版请用 git 历史里的旧脚本；')
  console.log('  ② 固件是 v0.15 或更早（没有 i= 字段）—— 同上；')
  console.log('  ③ 固件是 v0.16 之前的 [TOF]+[ALT] 两行格式 —— 用 verify_h8.js；')
  console.log('  ④ 日志文件选错了。')
  process.exit(1)
}
const distinct = new Set()
for (const r of R) distinct.add(`${r.req}|${r.mode}|${r.st}|${r.sw}|${r.ct}`)
console.log(`不同的 (req|mode|st|sw|ct) 组合 ${distinct.size} 种\n`)

let pass = 0, fail = 0, skip = 0
function verdict(name, state, detail) {   // state: true / false / null(未测)
  const tag = state === true ? '[ 通过 ]' : state === false ? '[ 失败 ]' : '[ 未测 ]'
  console.log(`${tag} ${name}\n         ${detail}`)
  if (state === true) pass++; else if (state === false) fail++; else skip++
}

/* 把样本切成"连续同 st"的段，返回 [{st, from, to}]（下标区间，左闭右开） */
function runs() {
  const out = []
  let i = 0
  while (i < R.length) {
    let j = i
    while (j < R.length && R[j].st === R[i].st) j++
    out.push({ st: R[i].st, from: i, to: j })
    i = j
  }
  return out
}

/* ── 0 串位自检：格式串与实参错位是 varargs 唯一查不出来的错 ── */
{
  const bad = []
  for (const r of R) {
    if (!(r.raw >= 0 && r.raw <= 1400)) bad.push(`raw=${r.raw}`)
    if (!(r.h >= -50 && r.h <= 200)) bad.push(`h=${r.h}`)
    if (![0, 1, 4, 5, 6, 7].includes(r.st)) bad.push(`st=${r.st}`)
    if (r.sw > 0xF || r.ct > 0xF) bad.push(`sw/ct=0x${r.sw.toString(16)}/0x${r.ct.toString(16)}`)
    if (r.stick > 1000) bad.push(`stick=${r.stick}`)
    if (!(r.thr >= PWM_MIN && r.thr <= MAP_MAX_DEFAULT)) bad.push(`thr=${r.thr}`)
    if (!(r.i >= THR_MIN - 0.1 && r.i <= THR_MAX + 0.1)) bad.push(`i=${r.i}`)
    if (!(r.corr >= -CORR_LIM - 0.1 && r.corr <= CORR_LIM + 0.1)) bad.push(`corr=${r.corr}`)
    if (!(r.fr >= 0 && r.fr <= 3)) bad.push(`fr=${r.fr}`)
    if (!(r.vzsp >= -OUT_MAX - 0.01 && r.vzsp <= OUT_MAX + 0.01)) bad.push(`vzsp=${r.vzsp}`)
    if (r.fsMs > 600000) bad.push(`fs=${r.fsMs}`)
    if (bad.length > 8) break
  }
  verdict('★ 字段串位自检（量程/取值域都合理）', bad.length === 0,
    bad.length === 0
      ? `${R.length} 个样本全部合理（raw 0~1400、h -50~200、st∈{0,1,4,5,6,7}、sw/ct≤0xF、stick≤1000、` +
        `thr ${PWM_MIN}~${MAP_MAX_DEFAULT}、i ${THR_MIN}~${THR_MAX}、|corr|≤${CORR_LIM}、fr∈{0..3}）\n` +
        `         ⭐ i 的窄量程 + fs/fr 的位置就是**串位检查**：任何一个字段串进 i 都会立刻越界`
      : `可疑字段值：${bad.slice(0, 8).join('  ')}`)
}

/* ── ① 普通模式：thr = 1000 + (满杆−1000)×stick/1000 ──
   ⚠️ 带检的两个端点必须是「上一拍**实际** thr」与「本拍映射值」，**不能是两个映射值**：
      杆从满位收回时，thr 会从 2000 以 **300µs/s** 衰减，中途必然高于本拍映射值。 */
{
  const rows = R.filter(r => r.mode === 0 && (r.ct & 1))
  const ks = rows.filter(r => r.stick >= 300).map(r => (r.thr - 1000) / r.stick).sort((a, b) => a - b)
  const k = ks.length ? ks[Math.floor(ks.length / 2)] : NaN
  const map = s => 1000 + k * s
  let outOfBand = 0, stable = 0, stableBad = 0, worst = null
  for (let i = 1; i < R.length; i++) {
    const a = R[i - 1], b = R[i]
    if (a.mode !== 0 || b.mode !== 0 || !(a.ct & 1) || !(b.ct & 1)) continue
    if (Number.isNaN(k)) break
    const lo = Math.min(a.thr, map(b.stick)) - 2
    const hi = Math.max(a.thr, map(b.stick)) + 2
    if (b.thr < lo || b.thr > hi) { outOfBand++; if (!worst) worst = { a, b, lo, hi } }
    if (a.stick === b.stick && Math.abs(a.thr - map(a.stick)) <= 2) {
      stable++
      if (Math.abs(b.thr - map(b.stick)) > 2) stableBad++
    }
  }
  const atZero = rows.filter(r => r.stick === 0)
  const zeroReached = atZero.some(r => r.thr === 1000) || atZero.length === 0
  const zeroTooLow = atZero.filter(r => r.thr < 1000).length
  const corrNonZero = rows.filter(r => r.corr !== 0).length
  const iNotStart = rows.filter(r => Math.abs(r.i - I_START) > 0.1).length
  const fsNotZero = rows.filter(r => r.fsMs !== 0 || r.fr !== 0).length
  const inferredMax = Number.isNaN(k) ? NaN : 1000 + k * 1000
  verdict('① 普通模式 thr = 1000 + (满杆−1000)×stick/1000 + corr 恒 0 + 无失效',
    rows.length > 0 && !Number.isNaN(k) && outOfBand === 0 && stableBad === 0 &&
      corrNonZero === 0 && zeroReached && zeroTooLow === 0 && iNotStart === 0 && fsNotZero === 0,
    rows.length === 0 ? '没有"普通模式 + 已解锁"的样本'
      : `${rows.length} 个样本；反推斜率 k = ${Number.isNaN(k) ? '—' : k.toFixed(4)}` +
        `（⇒ 满杆 ${Number.isNaN(inferredMax) ? '—' : inferredMax.toFixed(0)}µs，期望 ${MAP_MAX_DEFAULT}）` +
        `；杆 0 的 ${atZero.length} 个样本中"曾经到过 1000" ${zeroReached ? '有' : '没有 ⚠️'}、低于 1000 的 ${zeroTooLow} 个；` +
        `限速带外 ${outOfBand} 个` +
        (worst ? `（例 stick ${worst.a.stick}→${worst.b.stick}，thr ${worst.a.thr}→${worst.b.thr}，允许 ${worst.lo.toFixed(0)}~${worst.hi.toFixed(0)}）` : '') +
        `；杆量连续两拍不变且已到位的 ${stable} 个中不精确的 ${stableBad} 个；` +
        `普通模式下 corr≠0 的 ${corrNonZero} 个、i≠1250 的 ${iNotStart} 个、有失效标记的 ${fsNotZero} 个` +
        (Math.abs(inferredMax - MAP_MAX_DEFAULT) > 20
          ? `\n         ⚠️ 反推满杆 = ${inferredMax.toFixed(0)}µs，不是默认 ${MAP_MAX_DEFAULT} —— 检查遥控/Flash 里的 MAX THROTTLE`
          : ''))
}

/* ── ② 停手：st=4 → thr=1000、corr=0、i=1250、fs=0 ──
   ⚠️ thr 允许出现过渡值：可能从失效下降段的任意值（最高 1650）降回 1000。 */
{
  const rows = R.filter(r => r.mode === 1 && r.st === 4)
  const badC = rows.filter(r => Math.abs(r.corr) > 0.05)
  const badI = rows.filter(r => Math.abs(r.i - I_START) > 0.1)
  const badT = rows.filter(r => r.thr < PWM_MIN || r.thr > THR_MAX)
  const badFs = rows.filter(r => r.fr !== 0)
  const thrSet = [...new Set(rows.map(r => r.thr))].sort((a, b) => a - b)
  const n1000 = rows.filter(r => r.thr === 1000).length
  verdict('② 停手（st=4）→ corr=0、i=1250、thr 回到 1000、fr=0',
    rows.length > 0 && badC.length === 0 && badI.length === 0 && badT.length === 0 && badFs.length === 0 && n1000 > 0,
    rows.length === 0 ? '没有 st=4 的样本'
      : `${rows.length} 个样本；corr 非 0 的 ${badC.length} 个；i 非 1250 的 ${badI.length} 个；` +
        `fr 非 0 的 ${badFs.length} 个（⚠️ 停手不是失效，**不许**带原因码）；` +
        `thr 集合 {${thrSet.join(',')}}（其中 1000 有 ${n1000} 个${thrSet.length > 1 ? '，其余是回落过渡值' : ''}）；越界 ${badT.length} 个`)
}

/* ── ③ 上锁 → thr=1000、corr=0、i=1250、fs=0 ── */
{
  const rows = R.filter(r => r.mode === 1 && !(r.ct & 1))
  const bad = rows.filter(r => r.thr !== 1000 || Math.abs(r.corr) > 0.05 || Math.abs(r.i - I_START) > 0.1 || r.fr !== 0)
  verdict('③ 上锁（ct 无 ARMED 位）→ thr=1000、corr=0、i=1250、fr=0',
    rows.length > 0 && bad.length === 0,
    rows.length === 0 ? '没有未解锁的定高样本'
      : `${rows.length} 个样本，thr 集合 {${[...new Set(rows.map(r => r.thr))].join(',')}}，` +
        `i 集合 {${[...new Set(rows.map(r => r.i))].join(',')}}，越界 ${bad.length} 个`)
}

/* ── ④ 控制启用：st=1 → thr 落在 [1150, 1650]、corr ∈ [−50,+50] ──
   ⚠️ 退出停手那几拍 thr 从 1000 **经速率限制器爬上来**，中途必然低于 1150，
      那是过渡值不是越界（打印周期 500ms，爬升要 0.19s ≈ 不到 1 拍，通常看不到）。 */
{
  const rows = R.filter(r => r.st === 1)
  const overHard = rows.filter(r => r.thr > THR_MAX || r.thr < PWM_MIN)
  const corrOut = rows.filter(r => Math.abs(r.corr) > CORR_LIM + 0.1)
  const frBad = rows.filter(r => r.fr !== 0)
  const stuck = []
  for (const run of runs()) {
    if (run.st !== 1) continue
    for (let k = run.from + 8; k < run.to; k++) if (R[k].thr < THR_MIN) stuck.push(R[k])
  }
  const sticks = [...new Set(rows.map(r => r.stick))].sort((a, b) => a - b)
  verdict(`④ 控制启用（st=1）→ thr 稳定后落在 ${THR_MIN}~${THR_MAX}、|corr| ≤ ${CORR_LIM}、fr=0`,
    rows.length > 0 && overHard.length === 0 && corrOut.length === 0 && frBad.length === 0 && stuck.length === 0,
    rows.length === 0 ? '没有 st=1 的样本（推杆 >650 并保持约 1.4s 才会出现）'
      : `${rows.length} 个样本；thr 范围 ${Math.min(...rows.map(r => r.thr))}~${Math.max(...rows.map(r => r.thr))}；` +
        `硬越界(超出 ${PWM_MIN}~${THR_MAX}) ${overHard.length} 个；|corr|>${CORR_LIM} 的 ${corrOut.length} 个；` +
        `fr≠0 的 ${frBad.length} 个（⚠️ st=1 表示一切正常，**不许**带原因码）；` +
        `跳过每段开头 8 拍后低于 ${THR_MIN} 的 ${stuck.length} 个` +
        (stuck.length ? `（例 thr=${stuck.slice(0, 5).map(r => r.thr).join(',')}）` : '') +
        `；涉及 ${sticks.length} 种杆量（${sticks[0]}~${sticks[sticks.length - 1]}）`)
}

/* ── ⑤⑥ ⭐⭐ 快层符号：corr 必须与**内环的误差**（vz − vzsp）反号 ──
   ⚠️ H13 起参照物是 `vz − vzsp` 而**不是** `vz`：内环的控制律是
      `corr = Kp_v × (vz_sp − vz)`，所以它的符号由 `vz_sp − vz` 决定。
      H9~H12 期间 `vzsp ≡ 0`，两者等价；H13 之后 `vzsp` 可以到 ±20cm/s，
      拿 `vz` 当参照在 vzsp 较大的段会**误报**（踩过一次）。
   ⚠️ 两个必须排除的样本类别（都是**打印分辨率**造成的假失败）：
     ① `corr = 0`：0 不带符号信息；
     ② 误差印成 0.0（真实 |误差| < 0.05）：`corr*err` 恒为 0，会被算成"同号"。
   ⇒ 判据只在**两个量都印成非 0** 的样本上做。 */
{
  const errOf = r => r.vz - r.vzsp
  const zeroCorr = R.filter(r => r.st === 1 && Math.abs(r.corr) <= 0.05).length
  const zeroErr = R.filter(r => r.st === 1 && Math.abs(r.corr) > 0.05 && Math.abs(errOf(r)) < 0.05).length
  const act = R.filter(r => r.st === 1 && r.v === 1 && Math.abs(r.corr) > 0.05 && Math.abs(errOf(r)) >= 0.05)
  const strong = act.filter(r => Math.abs(errOf(r)) >= 3.0)
  const weak = act.filter(r => Math.abs(errOf(r)) < 3.0)
  const oppS = strong.filter(r => r.corr * errOf(r) < 0).length
  const oppW = weak.filter(r => r.corr * errOf(r) < 0).length
  const sameS = strong.length - oppS
  const sameW = weak.length - oppW
  const ratio = act.filter(r => Math.abs(r.corr) < CORR_LIM - 0.05).map(r => r.corr / errOf(r))
  ratio.sort((a, b) => a - b)
  const med = ratio.length ? ratio[ratio.length / 2] : NaN
  const total = strong.length + weak.length
  const detail = total === 0
    ? `没有可判的样本（st=1 且 corr 与 (vz−vzsp) 都非 0）。（排除：corr=0 共 ${zeroCorr} 个，误差印成 0.0 共 ${zeroErr} 个）`
    : `强信号（|vz−vzsp|≥3cm/s）${strong.length} 个：反号 ${oppS}、同号 ${sameS}；` +
      `弱信号 ${weak.length} 个：反号 ${oppW}、同号 ${sameW}；` +
      `排除 corr=0 共 ${zeroCorr} 个、误差印成 0.0 共 ${zeroErr} 个；` +
      (Number.isNaN(med)
        ? `等效 Kp：全部样本都顶在 ±${CORR_LIM} 限幅上，估不出来 —— ⚠️ 先确认遥控里 vz_kp 是 5`
        : `等效 Kp ≈ ${Math.abs(med).toFixed(1)} µs/(cm/s)，符号${med < 0 ? '✅ 正确（反号）' : '❌ **反了（同号 = 正反馈）**'}；` +
          `⚠️ H13 起这个数应该接近遥控里设的 vz_kp；明显偏大说明多数样本顶在限幅上`)
  verdict('⑤⑥ ⭐⭐ 快层符号：corr 与 (vz − vzsp) 反号（写反 = 正反馈 = 一直往上飞）',
    total === 0 ? null : (sameS === 0 && sameW === 0), detail)
}

/* ── ⭐⭐ 慢层符号：i 的变化方向必须与**内环的误差**（vz − vzsp）相反 ──
   i 是**悬停油门估计器**。它的积分律是 `di = vz_ki × (vz_sp − vz) × dt`：
     飞机相对目标在上升（vz > vzsp）⇒ 不需要那么大推力 ⇒ i 必须**下降**；
     飞机相对目标在下沉（vz < vzsp）⇒ 需要更大推力  ⇒ i 必须**上升**。
   ⚠️ H13 起参照物同样是 `vz − vzsp`（不是 `vz`）。
   ⚠️ 严格档：**要求相邻两条打印的误差同号**。两个必须排除的干扰：
     ① 方向反转（打印的 vz 只是端点瞬时值，i 取决于 500ms 内的平均）；
     ② 丢行/乱码（两端之间不是 0.5s）。 */
{
  const errOf = r => r.vz - r.vzsp
  let judged = 0
  const viol = []
  let ups = 0, downs = 0
  for (let k = 1; k < R.length; k++) {
    const a = R[k - 1], b = R[k]
    if (a.st !== 1 || b.st !== 1) continue
    if (Math.abs(errOf(a)) < 1.0 || Math.abs(errOf(b)) < 1.0) continue
    if (errOf(a) * errOf(b) <= 0) continue
    const di = b.i - a.i
    if (Math.abs(di) < 0.5) continue
    judged++
    if (errOf(b) > 0) ups++; else downs++
    if (di * errOf(b) > 0) viol.push({ di: di.toFixed(2), err: errOf(b).toFixed(2), i: b.i })
  }
  const iMin = Math.min(...R.map(r => r.i))
  const iMax = Math.max(...R.map(r => r.i))
  verdict('⭐⭐ 慢层符号：i 与 (vz − vzsp) 反向',
    judged === 0 ? null : viol.length === 0,
    judged === 0
      ? `没有可判的样本。⚠️ 要么没有"匀速移动飞机"，要么误差一直落在死区内`
      : `严格档（相邻两条误差同号、|误差|≥1、|Δi|≥0.5）：可判 ${judged} 个（下沉 ${ups}、上抬 ${downs}）：同号 ${viol.length} 个` +
        (viol.length ? `（例 Δi=${viol[0].di} 而 vz−vzsp=${viol[0].err}）← **符号反了**` : ' ✅') +
        `；观测到的 i 范围 ${iMin}~${iMax}`)
}

/* ── ⭐ 慢层有界 ── */
{
  const rows = R.filter(r => r.st === 1 || r.st === 6 || r.st === 7)
  const out = rows.filter(r => r.i < THR_MIN - 0.1 || r.i > THR_MAX + 0.1)
  verdict(`⭐ 慢层有界：i ∈ [${THR_MIN}, ${THR_MAX}]（积分与输出同一把锁）`,
    rows.length === 0 ? null : out.length === 0,
    rows.length === 0 ? '没有相关样本'
      : `${rows.length} 个样本，i 范围 ${Math.min(...rows.map(r => r.i))}~${Math.max(...rows.map(r => r.i))}，越界 ${out.length} 个`)
}

/* ══════════════════════════════════════════════════════════════════════
   ★★★ H13 新增：外环（高度误差 → 目标垂速）
   ══════════════════════════════════════════════════════════════════════ */

/* ── H13-① ⭐⭐ 外环符号 + 数值：vzsp ≈ h_kp × (h_sp − h) ──
   ⚠️ 这是本环节唯一会真正导致失控的错误（写反 ⇒ 高度偏低反而命令下降 ⇒ 越掉越低），
      与 H9/H10 那两个减号同族。
   ⚠️ 反推 h_kp 只在**没顶限幅**的样本上做，否则比值没有意义。 */
{
  const act = R.filter(r => r.st === 1 && Math.abs(r.hsp - r.h) > OUT_DEAD && Math.abs(r.vzsp) < OUT_MAX - 0.05)
  const signBad = act.filter(r => (r.hsp - r.h) * r.vzsp <= 0).length
  const ks = act.map(r => r.vzsp / (r.hsp - r.h)).sort((a, b) => a - b)
  const k = ks.length ? ks[ks.length >> 1] : NaN
  verdict('H13-① ⭐⭐ 外环符号：vzsp 与 (h_sp − h) 同号（写反 = 正反馈 = 越掉越低）',
    act.length === 0 ? null : signBad === 0,
    act.length === 0 ? '没有可判的样本（st=1 且误差出死区、且 vzsp 没顶限幅）'
      : `${act.length} 个样本：同号 ${act.length - signBad} 个、**反号 ${signBad} 个**；` +
        `反推 h_kp ≈ ${Number.isNaN(k) ? '—' : k.toFixed(3)}（遥控里应设 0.3）`)
}

/* ── H13-② ⭐ 死区：|h_sp − h| ≤ 2cm ⇒ vzsp 恰好 0 ── */
{
  const inDead = R.filter(r => r.st === 1 && Math.abs(r.hsp - r.h) <= OUT_DEAD)
  const bad = inDead.filter(r => r.vzsp !== 0).length
  verdict(`H13-② ⭐ 死区：st=1 且 |h_sp − h| ≤ ${OUT_DEAD}cm ⇒ vzsp 恰好 0`,
    inDead.length === 0 ? null : bad === 0,
    inDead.length === 0 ? '没有落在死区内的样本'
      : `${inDead.length} 个样本中不为 0 的 ${bad} 个` + (bad ? '  ← **死区没生效**' : ' ✅'))
}

/* ── H13-③ 不变式：普通 / 停手 / 未解锁 / 门禁没过 ⇒ vzsp 恒 0 ── */
{
  const rows = R.filter(r => r.st !== 1 && r.st !== 6 && r.st !== 7)
  const bad = rows.filter(r => r.vzsp !== 0).length
  verdict('H13-③ ★ 不变式：普通 / 停手 / 未解锁 / 门禁没过 ⇒ vzsp 恒 0',
    rows.length === 0 ? null : bad === 0,
    `${rows.length} 个样本（st∈{0,4,5}）中非 0 的 ${bad} 个` +
    `\n         （⚠️ st=6/7 不在本判据里：失效期间外环是**冻结**的，vzsp 会保持失效前的值）`)
}

/* ── H13-④ ⭐ 起飞助力：命令爬升（vzsp>2）而飞机还没动（|vz|<1）⇒ i 必须上涨 ──
   这是"定高模式能起飞"的机器化表达：慢层的误差是 `vz_sp − vz`，
   地面上 vz≈0 ⇒ 误差 ≈ vz_sp > 0 ⇒ i 上涨 ⇒ 约 5~6 秒后油门到悬停值。
   ⚠️ 只在"两个端点都 vzsp>2 且 |vz|<1"的相邻对上判，排除爬升/下沉造成的干扰。 */
{
  let judged = 0, bad = 0
  for (let k = 1; k < R.length; k++) {
    const a = R[k - 1], b = R[k]
    if (a.st !== 1 || b.st !== 1) continue
    if (!(a.vzsp > 2 && b.vzsp > 2)) continue
    if (Math.abs(a.vz) >= 1 || Math.abs(b.vz) >= 1) continue
    judged++
    if (b.i < a.i) bad++
  }
  verdict('H13-④ ⭐ 起飞助力：命令爬升（vzsp>2）而飞机没动（|vz|<1）⇒ i 上涨',
    judged === 0 ? null : bad === 0,
    judged === 0 ? '没有可判的样本（需要"推杆但飞机还没离地"的那一段）'
      : `${judged} 个样本中 i 反而下降的 ${bad} 个` +
        (bad ? '  ← **慢层的误差可能还在用 0**' : ' ✅（= 起飞前"憋油门"）'))
}

/* ══════════════════════════════════════════════════════════════════════
   ★★★ H12 新增：失效处理
   ══════════════════════════════════════════════════════════════════════ */

/* ── H12-① ⭐⭐ 失效保持（st=6）：thr **恒定**，且原因只能是 1/3 ──
   ⚠️ 恒定判据必须按**连续段**做：段的头一拍与上一段（st=1）同值，之后一拍不动。 */
{
  const segs = runs().filter(x => x.st === 6)
  let badConst = 0, badFr = 0, badThr = 0
  const examples = []
  for (const seg of segs) {
    const first = R[seg.from].thr
    for (let k = seg.from; k < seg.to; k++) {
      if (R[k].thr !== first) { badConst++; if (examples.length < 3) examples.push(`thr ${first}→${R[k].thr}`) }
      if (R[k].fr !== 1 && R[k].fr !== 3) badFr++
      if (R[k].thr < THR_MIN || R[k].thr > THR_MAX) badThr++
    }
  }
  const frSet = [...new Set(segs.flatMap(x => R.slice(x.from, x.to).map(r => r.fr)))].sort()
  verdict('H12-① ⭐⭐ 失效保持（st=6）→ thr 恒定、fr ∈ {1 ToF, 3 高度}、thr ∈ [1150,1650]',
    segs.length === 0 ? null : (badConst === 0 && badFr === 0 && badThr === 0),
    segs.length === 0 ? '没有 st=6 的样本（拔 ToF 线 / 举到 1.25m 才会出现）'
      : `${segs.length} 段；thr 变化的样本 ${badConst} 个` +
        (examples.length ? `（例 ${examples.join('、')}）` : '') +
        `；fr 越界的 ${badFr} 个；thr 越界 ${badThr} 个；出现过的 fr={${frSet.join(',')}}` +
        `\n         ⭐ "恒定"就是"短暂失效时飞机什么都没发生"的机器化表达，也是本环节最核心的一条`)
}

/* ── H12-② ⭐⭐ 失效下降（st=7）：thr 单调不增、斜率 ≈30µs/s、地板 1000 ──
   ⚠️ **必须把 fr=2（姿态超限）的段排除**：那一种是不保持、直接 1000，
      段内压根没有 30µs/s 的斜坡，混在一起算斜率会得出一个没意义的数。 */
{
  const segs = runs().filter(x => x.st === 7)
  let badMono = 0, badRange = 0, badFr = 0
  const rates = []
  const instant = []
  for (const seg of segs) {
    let noRise = true
    for (let k = seg.from + 1; k < seg.to; k++) {
      if (R[k].thr > R[k - 1].thr) noRise = false
      if (R[k].thr < PWM_MIN || R[k].thr > THR_MAX) badRange++
      if (R[k].fr !== 1 && R[k].fr !== 2 && R[k].fr !== 3) badFr++
    }
    if (!noRise) badMono++
    const fr = R[seg.from].fr
    const dur = (seg.to - seg.from - 1) * PRINT_PERIOD_S
    if (fr !== 2 && dur > 3.0) {
      rates.push((R[seg.from].thr - R[seg.to - 1].thr) / dur)
    }
    if (fr === 2) instant.push({ thr: R[seg.from].thr, st6: false })
  }
  /* fr=2 单独一句话：它必须是"立即 1000"，不许先保持 */
  const attSegs = segs.filter(x => R[x.from].fr === 2)
  const attBadThr = attSegs.filter(x => R[x.from].thr !== 1000).length
  const attHeld = runs().filter(x => x.st === 6 && R[x.from].fr === 2).length
  verdict('H12-② ⭐⭐ 失效下降（st=7）→ thr 单调不增、∈[1000,1650]、斜率 ≈30µs/s（fr≠2 的段）',
    segs.length === 0 ? null : (badMono === 0 && badRange === 0 && badFr === 0),
    segs.length === 0 ? '没有 st=7 的样本'
      : `${segs.length} 段；thr 回升的段 ${badMono} 个；越界样本 ${badRange} 个；fr 越界 ${badFr} 个` +
        (rates.length
          ? `；${rates.length} 段够长可估斜率，测得 ${rates.map(r => r.toFixed(1)).join(', ')} µs/s（期望 ${FS_RAMP}）`
          : '；没有足够长的段可估斜率（每段要 >3 秒）') +
        `\n         ⭐ 斜率 30 远低于 motor_test 的下降限速 300 ⇒ **限速器不参与**，实测就当等于 ${FS_RAMP}µs/s`)
  verdict('H12-③ ⭐ 姿态超限（fr=2）→ **完全不进 st=6**，直接 1000',
    attSegs.length === 0 && attHeld === 0 ? null : (attHeld === 0 && attBadThr === 0),
    (attSegs.length === 0 && attHeld === 0) ? '没有 fr=2 的样本（台架把飞机歪到 30° 才会出现）'
      : `fr=2 的 st=7 段 ${attSegs.length} 个、st=6 段 ${attHeld} 个（**应为 0**）；` +
        `进入那一拍 thr≠1000 的 ${attBadThr} 个` +
        `\n         ⭐ 这是用户 2026-09-15 的决定："姿态超 25° 当撞击炸机、不管了"，所以跳过保持段直接停桨`)
}

/* ── H12-④ 守卫④：没飞过就不许锁存（地面上重新解锁 + 读数坏着）──
   ⚠️ 日志切片会让这条误报（日志从中间开始就可能看不到 st=1），所以只在
      "**全程一个 st=1 都没有，却有 st=6/7**" 这种明确矛盾时才判失败。 */
{
  const act = R.filter(r => r.st === 1).length
  const fsRows = R.filter(r => r.st === 6 || r.st === 7)
  verdict('H12-④ ⭐ 守卫④：st=6/7 必须发生在"飞过"之后（地面上重新解锁不许锁存）',
    fsRows.length === 0 ? null : (act > 0),
    fsRows.length === 0 ? '没有 st=6/7 的样本'
      : `st=1 样本 ${act} 个、st=6/7 样本 ${fsRows.length} 个` +
        (act === 0 ? '  ← ⚠️ **全程没有 st=1 却出现了失效处理**，说明"没飞过也锁存了"，必须查' : '  ✅') +
        `\n         ⚠️ 日志从中间切片时这条可能看不出问题，要人工确认失效之前确实在飞`)
}

/* ── H12-⑤ 不变式：fr≠0 ⟺ st∈{6,7} ── */
{
  const bad = R.filter(r => (r.fr !== 0) !== (r.st === 6 || r.st === 7))
  const frCount = {}
  for (const r of R) frCount[r.fr] = (frCount[r.fr] || 0) + 1
  verdict('H12-⑤ ★ 不变式：fr≠0 ⟺ st∈{6,7}（只有失效状态才带原因码）',
    bad.length === 0,
    `fr 分布 {${Object.entries(frCount).map(([k, v]) => `${FR_NAME[k] || k}:${v}`).join(', ')}}；矛盾样本 ${bad.length} 个`)
}

/* ── ⑦ 静止时 corr 的抖动幅度（判 Kp 还能不能往上加）──
   ⚠️ H13 起"静止"必须是"**没在动 且 没被命令动**"：`vzsp` 可以不为 0，
      那时飞机虽然暂时 `|vz|<0.5`（典型：还在地面上、正准备起飞），
      内环却在为"命令的速度"出满力（corr 顶 ±50）。
      把它算进"静止"会误判成"Kp 偏大或噪声偏大" —— 踩过一次。 */
{
  const still = R.filter(r => r.st === 1 && r.v === 1 && Math.abs(r.vz) < 0.5 && Math.abs(r.vzsp) < 0.5)
  const cs = still.map(r => r.corr)
  if (cs.length >= 10) {
    const mean = cs.reduce((a, b) => a + b, 0) / cs.length
    const sd = Math.sqrt(cs.reduce((a, b) => a + (b - mean) ** 2, 0) / cs.length)
    const vzSd = Math.sqrt(still.reduce((a, r) => a + r.vz ** 2, 0) / still.length)
    const eff = vzSd > 0.01 ? (sd / vzSd) : NaN
    const maxAbs = Math.max(...cs.map(Math.abs))
    const dIs = []
    for (let k = 1; k < R.length; k++) {
      if (R[k].st !== 1 || R[k - 1].st !== 1) continue
      if (Math.abs(R[k].vz) >= 0.5 || Math.abs(R[k - 1].vz) >= 0.5) continue
      dIs.push(R[k].i - R[k - 1].i)
    }
    const dI = dIs.length ? Math.sqrt(dIs.reduce((a, d) => a + d * d, 0) / dIs.length) : NaN
    verdict('⑦ 静止（|vz|<0.5）时 corr 的抖动幅度 + i 的漂移',
      cs.length >= 10 && maxAbs < CORR_LIM - 0.05,
      `${cs.length} 个样本：corr 均值 ${mean.toFixed(2)}µs、标准差 ${sd.toFixed(2)}µs、` +
      `幅度（max|corr|）${maxAbs.toFixed(1)}µs；` +
      `同时刻 |vz| 的 RMS ${vzSd.toFixed(2)}cm/s ⇒ 等效 Kp ≈ ${Number.isNaN(eff) ? '—' : eff.toFixed(1)}；` +
      `i 的逐拍变化 RMS ${Number.isNaN(dI) ? '—' : dI.toFixed(2)}µs/拍（静止时应当很小）` +
      (maxAbs >= CORR_LIM - 0.05 ? `  ⚠️ 已顶到 ±${CORR_LIM} 限幅，说明 Kp 偏大或噪声偏大` : ''))
  } else {
    verdict('⑦ 静止时 corr 的抖动幅度', null, `静置样本只有 ${cs.length} 个，不够统计`)
  }
}

/* ── ⑧ 杆拉到底 → 重新进停手 ── */
{
  const back = []
  for (let i = 1; i < R.length; i++) if (R[i].st === 4 && R[i - 1].st === 1) back.push(R[i])
  verdict('⑧ st 1→4 转换（杆拉到底后回停手）',
    back.length === 0 ? null : back.every(r => r.gt >= 500),
    back.length === 0 ? '没有 st 1→4 的转换'
      : `${back.length} 次转换，进入那拍 gt=${[...new Set(back.map(r => r.gt))].join(',')}ms、stick=${[...new Set(back.map(r => r.stick))].join(',')}`)
}

/* ══════════════════════════════════════════════════════════════════════
   H14（停手改成「推杆即解手」，2026-09-15 实施 v0.22）
   ══════════════════════════════════════════════════════════════════════ */

/* ── H14-① ⭐ 解手那一拍：st 4→1 时 stick 必须 > 200（推杆边沿门槛）──
   判据是 `stick > VERTICAL_GROUND_EXIT_STICK`（200）—— 含不含等号是这类 bug 的老地方。 */
{
  const rs = runs()
  const releases = []
  for (let k = 1; k < rs.length; k++) if (rs[k - 1].st === 4 && rs[k].st === 1) releases.push(R[rs[k].from])
  const low = releases.filter(r => r.stick <= 200)
  verdict('H14-① ⭐ 解手（st 4→1）那一拍 stick > 200（推杆门槛）',
    releases.length === 0 ? null : low.length === 0,
    releases.length === 0 ? '日志里没有 st 4→1 的转换（这次没解过手）'
      : `${releases.length} 次解手，那一拍 stick ∈ {${[...new Set(releases.map(r => r.stick))].join(',')}}` +
        (low.length ? ` ← **有 ${low.length} 次 stick ≤200 却解手了**（门槛写错了？）` : ' ✅'))
}

/* ── H14-② ⭐⭐ 推杆解手不抖：st 不许在 1 与 4 之间**快速来回翻** ──
   本次改动最容易出的错就是它：起飞保护的清除条件若还留着"变化率 < 0"，
   而杆在下降段时变化率正好是负的 ⇒ 解手 500ms 后又被判回停手 ⇒ 电机转一下停一下。
   判据：某个 st=4 段**只持续 ≤2 个样本（≤1 秒）**且前后都是 st=1 ⇒ 抖。
   ⚠️ 真实的"落地 → 再起飞"不会被误判：停手至少要 500ms 去抖 + 飞行员重新推杆。 */
{
  const rs = runs()
  const hasGround = rs.some(r => r.st === 4)
  const thrash = []
  for (let k = 1; k < rs.length - 1; k++) {
    const g = rs[k]
    if (g.st !== 4 || (g.to - g.from) > 2) continue
    if (rs[k - 1].st === 1 && rs[k + 1].st === 1) thrash.push({ at: g.from, stick: R[g.from].stick })
  }
  verdict('H14-② ⭐⭐ 推杆解手不抖（st 不许在 1↔4 之间每 1 秒来回翻）',
    hasGround ? (thrash.length === 0) : null,
    !hasGround ? '日志里没有 st=4 的段（没进过停手）'
      : `${rs.length} 个状态段，其中「停手只持续 ≤1 秒且两侧都是 st=1」的 ${thrash.length} 处` +
        (thrash.length
          ? `（样本下标 ${thrash.slice(0, 5).map(x => x.at).join(',')}，stick ${[...new Set(thrash.map(x => x.stick))].join(',')}）← **这就是抖**`
          : ' ✅'))
}

/* ── H14-③ ⭐ 推杆之后电机必须转：st=1 且 stick>200 ⇒ thr ≠ 1000 ──
   这条直接对应本环节的目的（"杆在下降段/中间段推着，电机也要转"）。
   ⚠️ 必须**跳过每个 st=1 段开头 8 拍**：解手那一拍油门还在从 1000 经速率限制器往上爬
      （实测过渡值 1000/1003/1055/1100/1192/1230），那几拍 thr 低是**正常的**。
      第一版忘了跳，于是"正确组"自己把这条判失败了 —— 与判据 ④ / ★ 的写法保持一致。 */
{
  const pushed = []
  for (const run of runs()) {
    if (run.st !== 1) continue
    for (let k = run.from + 8; k < run.to; k++) if (R[k].stick > 200) pushed.push(R[k])
  }
  const dead = pushed.filter(r => r.thr <= 1000)
  verdict('H14-③ ⭐ 推杆解手后（st=1 且 stick>200，跳过限速器过渡）thr 不许是 1000（电机必须转）',
    pushed.length === 0 ? null : dead.length === 0,
    pushed.length === 0 ? '没有「st=1 稳态且 stick>200」的样本'
      : `${pushed.length} 个样本里 thr≤1000 的有 ${dead.length} 个` +
        (dead.length ? `（stick ${[...new Set(dead.map(r => r.stick))].slice(0, 5).join(',')}）` : ' ✅'))
}

/* ── ⑨ 举到 1.2m 以上 → sw 掉 HEIGHT_OK ── */
{
  const hi = R.filter(r => r.h > 120)
  const lost = hi.filter(r => !(r.sw & 0x08))
  const hiAny = R.filter(r => r.h > 5)
  verdict('⑨ 举到 1.2m 以上 → sw 掉 HEIGHT_OK（位 3）',
    hi.length === 0 ? null : lost.length === hi.length,
    hi.length === 0 ? `h 最大只有 ${R.length ? Math.max(...R.map(r => r.h)) : '—'}cm —— 没举起来（离地样本 ${hiAny.length} 个）`
      : `h>120cm 的 ${hi.length} 个样本中掉了 HEIGHT_OK 的 ${lost.length} 个（sw=${[...new Set(hi.map(r => '0x' + r.sw.toString(16)))].join(',')}）`)
}

/* ── ★ 不变式 ── */
{
  const alt = R.filter(r => r.mode === 1)
  const over = alt.filter(r => r.thr > THR_MAX || r.thr < PWM_MIN)
  verdict(`★ 不变式：定高模式下 thr ∈ [${PWM_MIN}, ${THR_MAX}]`,
    alt.length > 0 && over.length === 0,
    `${alt.length} 个定高样本，thr 范围 ${alt.length ? Math.min(...alt.map(r => r.thr)) : '—'}~${alt.length ? Math.max(...alt.map(r => r.thr)) : '—'}，越界 ${over.length} 个`)

  /* ⭐⭐ 最关键的安全不变式：正常控制时地板必须高于姿态环旁路线 */
  const active = []
  for (const run of runs()) {
    if (run.st !== 1) continue
    for (let k = run.from + 8; k < run.to; k++) active.push(R[k])
  }
  const belowBypass = active.filter(r => r.thr <= ATT_BYPASS)
  verdict(`⭐⭐ 不变式：st=1 稳定后 thr 永不 ≤ ${ATT_BYPASS}（姿态环永不被旁路）`,
    active.length === 0 ? null : belowBypass.length === 0,
    active.length === 0 ? '没有"st=1 且已跳过每段开头 8 拍"的样本'
      : `${active.length} 个稳态样本中，thr ≤ ${ATT_BYPASS} 的有 ${belowBypass.length} 个` +
        (belowBypass.length ? `（最低 ${Math.min(...belowBypass.map(r => r.thr))}）← **地板失效**` : ' ✅'))
}

console.log(`\n===== 通过 ${pass} / 失败 ${fail} / 未测 ${skip} =====`)
console.log('\n状态分布：')
const byState = new Map()
for (const r of R) {
  const k = `mode=${r.mode} st=${r.st}${ST_NAME[r.st] ? '(' + ST_NAME[r.st] + ')' : ''} sw=0x${r.sw.toString(16).toUpperCase()} ct=0x${r.ct.toString(16).toUpperCase()} v=${r.v}${r.fr ? ' fr=' + r.fr + '(' + FR_NAME[r.fr] + ')' : ''}`
  byState.set(k, (byState.get(k) || 0) + 1)
}
for (const [k, n] of [...byState.entries()].sort((a, b) => b[1] - a[1])) console.log(`  x${String(n).padStart(4)}  ${k}`)
console.log(`\n失效计时 fs 出现过的值：{${[...new Set(R.map(r => r.fsMs))].sort((a, b) => a - b).slice(0, 25).join(', ')}${new Set(R.map(r => r.fsMs)).size > 25 ? ', ...' : ''}}`)
console.log(`corr 出现过的值：{${[...new Set(R.map(r => r.corr))].sort((a, b) => a - b).slice(0, 30).join(', ')}${new Set(R.map(r => r.corr)).size > 30 ? ', ...' : ''}}`)
console.log(`thr  出现过的值：{${[...new Set(R.map(r => r.thr))].sort((a, b) => a - b).slice(0, 40).join(', ')}${new Set(R.map(r => r.thr)).size > 40 ? ', ...' : ''}}`)
