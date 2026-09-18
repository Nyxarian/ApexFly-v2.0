/* 生成合成的 [V] 日志，自测 verify_v.js（**H12 版**，含 fs= / fr=）。
   用法：node gen_fake_v_log.js <输出目录>

   生成六份，其中三份是"正确"，三份是**故意的对照组**：

     fake_v_ok.jsonl       全部正确                       → 期望全过
     fake_v_bad.jsonl      **快层**符号写反               → 期望 ⑤⑥ 失败
     fake_v_badslow.jsonl  **慢层**符号写反               → 期望「慢层符号」失败
     fake_v_badhold.jsonl  st=6 期间 thr 在漂             → 期望 H12-① 失败
     fake_v_badramp.jsonl  st=7 期间 thr 回升             → 期望 H12-② 失败
     fake_v_badatt.jsonl   姿态超限先进了 st=6            → 期望 H12-③ 失败

   ⭐ 为什么必须有对照组：只有"正确的组也能过"是不够的 ——
      还必须证明脚本**真的有本事抓到写错的那种**，否则它只是个永远说 OK 的摆设。
   ⚠️ 合成数据必须包含真实系统里的滞后/钳位/丢行，否则自测是在自欺
      （H10 那次就是合成日志太"干净"，所以自测覆盖不到限速衰减那类判据错误）。
*/
const fs = require('fs')
const path = require('path')
const outDir = process.argv[2] || '.'

const KP = 20      // 遥控里设的 vz_kp
const KI = 5       // vz_ki
const I0 = 1300    // 慢层起始值（刻意不取 1250，留出双向行程）
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v))

function line(o) {
  return `[V] raw=${o.raw}mm | h=${o.h.toFixed(1)}cm vz=${o.vz >= 0 ? '+' : '-'}${Math.abs(o.vz).toFixed(1)}cm/s v=${o.v} | ` +
    `req=${o.req} mode=${o.mode} st=${o.st} sw=0x${o.sw.toString(16).toUpperCase()} ct=0x${o.ct.toString(16).toUpperCase()} ` +
    `stick=${o.stick} h_sp=${o.hsp.toFixed(1)} gt=${o.gt}ms fs=${o.fs}ms fr=${o.fr} | ` +
    `vzsp=${o.vzsp >= 0 ? '+' : '-'}${Math.abs(o.vzsp).toFixed(1)} ` +
    `corr=${o.corr >= 0 ? '+' : '-'}${Math.abs(o.corr).toFixed(1)} i=${o.i.toFixed(1)} thr=${o.thr}`
}

/* fault: null | 'fast' | 'slow' | 'hold' | 'ramp' | 'attHold' */
function build(fault) {
  const L = []
  /* ⚠️ 必须像固件一样把 corr 钳到 ±CORR_LIM、把 thr 钳到 [1150,1650]：
     第一版没钳，生成出来的 corr 有 ±120 的样本，于是"**正确组**"自己把
     字段自检和判据 ④ 判失败了 —— 合成数据必须满足固件的不变式，否则自测没有意义。 */
  const CORR_LIM = 50
  const push = o => {
    const c = clamp(o.corr, -CORR_LIM, CORR_LIM)
    L.push(line({ vzsp: 0, ...o, corr: c, thr: o.thr }))
  }
  const pushThr = o => {
    const c = clamp(o.corr, -CORR_LIM, CORR_LIM)
    L.push(line({ vzsp: 0, ...o, corr: c, thr: Math.round(clamp(o.thr, 1150, 1650)) }))
  }
  /* ⚠️ H13 之后 corr 必须由 **内环的误差 (vz − vzsp)** 推出来，不能只看 vz：
     `corr = Kp_v × (vz_sp − vz) = −Kp_v × (vz − vz_sp)`。
     第一版还在用 `corr = −Kp_v × vz`（vzsp≡0 时代的写法），于是"正确组"自己
     把 ⑤⑥ 和「慢层符号」判失败了 —— 合成数据必须与**当前的**控制律一致。 */
  const corrOf = (v, vzsp) => (fault === 'fast' ? KP * (v - vzsp) : -KP * (v - vzsp))
  /* ── 外环（H13）：vzsp = h_kp × (h_sp − h)，死区 2cm、限幅 ±20 ──
     fault 注入：'outer' 把符号写反；'dead' 让死区失效（界内也出力）。 */
  const OUT_KP = 0.3, OUT_DEAD = 2.0, OUT_MAX = 20.0
  const vsp = (hsp, h) => {
    const e = hsp - h
    let v = (Math.abs(e) <= OUT_DEAD) ? ((fault === 'dead') ? 0.5 : 0) : clamp(OUT_KP * e, -OUT_MAX, OUT_MAX)
    if (fault === 'outer') v = -v
    return v
  }
  /* 一条"控制启用 + 读数坏"的样本（st=6 保持段） */
  const holdRow = (fsMs, thr, i, fr) => ({
    raw: 340, h: 34.0, vz: 0.2, v: 0, req: 1, mode: 1, st: 6, sw: 0xC, ct: 0xD,
    stick: 1000, hsp: 20.0, gt: 0, fs: fsMs, fr, corr: 0, i, thr,
  })

  /* ── 普通模式：杆扫动 + 稳定段（corr=0、i=1250、fs=0、fr=0）── */
  for (const s of [0, 100, 200, 300, 400, 500, 600, 690, 800, 1000, 1000, 1000]) {
    push({ raw: 28, h: 2.8, vz: -0.2, v: 1, req: 0, mode: 0, st: 0, sw: 0xF, ct: 0xF, stick: s, hsp: 0, gt: 0, fs: 0, fr: 0, corr: 0, i: 1250, thr: 1000 + s })
  }
  /* ⭐ 收杆回落：thr 以 300µs/s 衰减 ⇒ 每拍打印（500ms）掉 150µs。
     这一段是专门为判据 ① 加的：它必须能正确处理"thr 高于本拍映射值"的过渡。 */
  for (const t of [2000, 1850, 1700, 1550, 1400, 1250, 1100, 1000]) {
    push({ raw: 28, h: 2.8, vz: -0.2, v: 1, req: 0, mode: 0, st: 0, sw: 0xF, ct: 0xF, stick: 0, hsp: 0, gt: 0, fs: 0, fr: 0, corr: 0, i: 1250, thr: t })
  }

  /* ── 定高停手（含限速器回落的过渡值）── */
  push({ raw: 28, h: 2.8, vz: -2.0, v: 1, req: 1, mode: 1, st: 4, sw: 0xF, ct: 0x7, stick: 0, hsp: 0, gt: 500, fs: 0, fr: 0, corr: 0, i: 1250, thr: 1218 })
  for (let k = 0; k < 7; k++) {
    push({ raw: 28, h: 2.8, vz: -2.0, v: 1, req: 1, mode: 1, st: 4, sw: 0xF, ct: 0x7, stick: 0, hsp: 0, gt: 500, fs: 0, fr: 0, corr: 0, i: 1250, thr: 1000 })
  }

  /* ── 进入控制启用：前 8 拍是限速器过渡（判据 ④ 会跳过每段开头 8 拍）── */
  for (const t of [1000, 1003, 1055, 1100, 1192, 1230, 1250, 1250]) {
    push({ raw: 28, h: 3.4, vz: 0.1, v: 1, req: 1, mode: 1, st: 1, sw: 0xC, ct: 0xF, stick: 1000, hsp: 6.0, gt: 0, fs: 0, fr: 0, vzsp: vsp(6.0, 3.4), corr: 0, i: 1250, thr: t })
  }

  /* ── 静止（小噪声）：给 ⑦ 用。
        ⚠️ h_sp 必须**等于** h（=3.4）⇒ 误差 0 ⇒ vzsp=0 ⇒ 才算真的"静止"；
           写成 hsp=6.0 的话 vzsp=0.78，而 ⑦ 的判据要求 |vzsp|<0.5 —— 会漏掉这些样本。 ── */
  const still = [0.3, -0.4, 0.2, -0.3, 0.1, -0.2, 0.4, -0.35, 0.25, -0.15, 0.3, -0.2]
  for (const v of still) {
    const vs = vsp(3.4, 3.4)
    pushThr({ raw: 28, h: 3.4, vz: v, v: 1, req: 1, mode: 1, st: 1, sw: 0xC, ct: 0xF, stick: 1000, hsp: 3.4, gt: 0, fs: 0, fr: 0, vzsp: vs, corr: corrOf(v, vs), i: I0, thr: I0 + corrOf(v, vs) })
  }

  /* ── 匀速移动飞机：i 与 (vz − vzsp) 反向（正确的组）。
        h=4.5 ⇒ 误差 1.5cm **落在死区内** ⇒ vzsp=0 ── */
  let i = I0
  for (const v of [6, -6, 6, -6]) {
    const vs = vsp(6.0, 4.5)
    const dir = (fault === 'slow') ? Math.sign(v - vs) : -Math.sign(v - vs)
    for (let k = 0; k < 8; k++) {
      i = clamp(i + dir * 15, 1150, 1650)
      pushThr({ raw: 45, h: 4.5, vz: v, v: 1, req: 1, mode: 1, st: 1, sw: 0xC, ct: 0xF, stick: 1000, hsp: 6.0, gt: 0, fs: 0, fr: 0, vzsp: vs, corr: corrOf(v, vs), i, thr: i + corrOf(v, vs) })
    }
  }

  /* ── ⭐ 起飞段（H13-④ 的靶子）：推杆把 h_sp 抬到 10~15cm，飞机仍在地面（h=3.0、vz≈0）
        ⇒ vzsp > 2 且 **i 必须一路涨**（这就是"起飞前憋油门 5~6 秒"）
        对照组 'takeoff'：让 i 反而下降 —— 那是"慢层的误差还在用 0"的典型症状
        ⚠️ iT 必须**接着上一段的末值**往下走：凭空跳一下会被「慢层符号」判成符号错，
           而那是合成数据的问题、不是产品代码的问题（第一版就是这么误报的）。 ── */
  let iT = i
  for (let k = 0; k < 6; k++) {
    const hspT = 10.0 + k * 1.2
    const vs = vsp(hspT, 3.0)
    iT += (fault === 'takeoff') ? -8 : 8
    pushThr({ raw: 30, h: 3.0, vz: 0.0, v: 1, req: 1, mode: 1, st: 1, sw: 0xC, ct: 0xF, stick: 1000, hsp: hspT, gt: 0, fs: 0, fr: 0, vzsp: vs, corr: corrOf(0.0, vs), i: iT, thr: iT + corrOf(0.0, vs) })
  }
  const iLock = i

  /* ── ⭐ 失效保持：ToF 掉了，thr **锁存不变**，fs 涨到 2000 ── */
  const lock = Math.round(clamp(iLock + 0, 1150, 1650))
  for (let k = 0; k < 5; k++) {
    /* 对照组 'hold'：让 thr 在保持段里漂（每拍 −3µs）—— 正确实现绝不允许 */
    const thr = (fault === 'hold') ? lock - k * 3 : lock
    push(holdRow(k * 500, thr, iLock, 1))
  }

  /* ── 失效下降：从锁存值按 30µs/s 降到 1000 ⇒ 每拍打印掉 15µs ── */
  let thr = lock
  for (let k = 0; k < 35 && thr > 1000; k++) {
    thr = Math.max(1000, lock - (k + 1) * 15 + 2000 * 0)   // 从锁存值起算
    /* 对照组 'ramp'：第 10 拍开始回升 —— 正确实现绝不允许 */
    const shown = (fault === 'ramp' && k > 10) ? thr + (k - 10) * 20 : thr
    push({ raw: 340, h: 34.0, vz: -0.4, v: 0, req: 1, mode: 1, st: 7, sw: 0xC, ct: 0xD, stick: 1000, hsp: 20.0, gt: 0, fs: 2000 + (k + 1) * 500, fr: 1, corr: 0, i: iLock, thr: shown })
  }
  for (let k = 0; k < 3; k++) {
    push({ raw: 300, h: 30.0, vz: -0.2, v: 0, req: 1, mode: 1, st: 7, sw: 0xC, ct: 0xD, stick: 1000, hsp: 20.0, gt: 0, fs: 20000 + k * 500, fr: 1, corr: 0, i: iLock, thr: 1000 })
  }

  /* ── ⭐ 短暂失效 + 恢复：st=6 一小段后回 st=1（无缝，i 不丢）── */
  push(holdRow(0, lock, iLock, 1))
  push(holdRow(500, lock, iLock, 1))
  /* ⚠️ 这四拍是"失效恢复后又回到 st=1"，外环重新出力：
         h_sp=20、h=30 ⇒ 误差 −10cm ⇒ vzsp 必须是 **−3**（不能留 0）。
         第一版忘了给它们 vzsp，于是 H13-① 把"误差为负而 vzsp=0"算成符号错而误报。 */
  for (let k = 0; k < 4; k++) {
    const vs = vsp(20.0, 30.0)
    push({ raw: 300, h: 30.0, vz: 0.2, v: 1, req: 1, mode: 1, st: 1, sw: 0xC, ct: 0xF, stick: 1000, hsp: 20.0, gt: 0, fs: 0, fr: 0, vzsp: vs, corr: corrOf(0.2, vs), i: iLock, thr: lock })
  }

  /* ── ⭐ 姿态超限：**不进 st=6**，直接 st=7 且 thr=1000 ──
     对照组 'attHold'：先插两拍 st=6 —— 那是错的（用户定的是"炸机、立刻停桨"） */
  if (fault === 'attHold') {
    push(holdRow(0, lock, iLock, 2))
    push(holdRow(500, lock, iLock, 2))
  }
  for (let k = 0; k < 4; k++) {
    push({ raw: 300, h: 30.0, vz: -0.5, v: 1, req: 1, mode: 1, st: 7, sw: 0xC, ct: 0xB, stick: 1000, hsp: 20.0, gt: 0, fs: k * 500, fr: 2, corr: 0, i: 1250, thr: 1000 })
  }
  /* ── 落地 → 回停手 ── */
  for (let k = 0; k < 4; k++) {
    push({ raw: 28, h: 2.8, vz: -0.5, v: 1, req: 1, mode: 1, st: 4, sw: 0xE, ct: 0x7, stick: 0, hsp: 0, gt: 500, fs: 0, fr: 0, corr: 0, i: 1250, thr: 1000 })
  }
  /* ── 举到 130cm → sw 掉 HEIGHT_OK ── */
  for (let k = 0; k < 4; k++) {
    push({ raw: 1300, h: 130.0, vz: -1.0, v: 1, req: 1, mode: 1, st: 4, sw: 0x7, ct: 0x6, stick: 0, hsp: 0, gt: 500, fs: 0, fr: 0, corr: 0, i: 1250, thr: 1000 })
  }

  return L.map((t, idx) => JSON.stringify({ seq: idx + 1, dir: 'rx', source: 'B', text: t + '\r\n' })).join('\n') + '\n'
}

const groups = [
  ['fake_v_ok.jsonl', null, '全部正确 → 期望全过'],
  ['fake_v_bad.jsonl', 'fast', '快层符号写反 → 期望 ⑤⑥ 失败'],
  ['fake_v_badslow.jsonl', 'slow', '慢层符号写反 → 期望「慢层符号」失败'],
  ['fake_v_badhold.jsonl', 'hold', 'st=6 期间 thr 在漂 → 期望 H12-① 失败'],
  ['fake_v_badramp.jsonl', 'ramp', 'st=7 期间 thr 回升 → 期望 H12-② 失败'],
  ['fake_v_badatt.jsonl', 'attHold', '姿态超限先进了 st=6 → 期望 H12-① 与 H12-③ 失败'],
  ['fake_v_badouter.jsonl', 'outer', '**外环符号写反** → 期望 H13-① 失败'],
  ['fake_v_baddead.jsonl', 'dead', '**死区内 vzsp 不为 0** → 期望 H13-② 失败'],
  ['fake_v_badtakeoff.jsonl', 'takeoff', '**vzsp>2 时 i 反而下降** → 期望 H13-④ 失败'],
]
for (const [name, fault, desc] of groups) {
  fs.writeFileSync(path.join(outDir, name), build(fault))
  console.log(`  已生成 ${name.padEnd(22)} ${desc}`)
}
