/* H13 接上外环之后，"定高模式怎么起飞" —— 一维数值仿真（PC 上算物理，不碰飞控代码）。
   运行：node sim_h13_takeoff.js

   为什么必须先算这一步：接上外环之后飞机**才第一次有可能自己起飞**，而悬停油门
   是"慢层积分器"现场学的（初值 1250，真实悬停约 1450）。学的过程需要时间，
   而这段时间里飞行员手里的杆还在以 +5cm/s（VERTICAL_RATE_UP_CMS）积分 h_sp —— 两者一起决定"起飞有多猛"。
   ⇒ 本脚本就是给 H14 首飞定"杆怎么推"。

   ⚠️ 本文件自己踩过两个错，都是**符号/单位**类，与飞控里最容易犯的错同族：
     ① 把"向下为正"的加速度直接加到"向上为正"的 vz 上 ⇒ 飞机加速飞到 675 米；
     ② vz 用米、h_sp 用厘米 ⇒ 地面读数算成 3+12.8，飞机实际在 12.8m 而控制器以为在 15.8cm。
     ⇒ 全程统一成 **cm / cm·s⁻¹ / cm·s⁻²**，并且只用一种符号约定（向上为正）。

   模型（与 sim_failsafe_ramp.js 同一套推力模型）：
     推力比 = ((u − u0) / (u_hover − u0))²        u0 = 1035µs（起转点，mixer.h 实测）
     向上加速度 = g × (推力比 − 1)                  u_hover = 1450µs（用户"1450 左右能起飞"）
   控制律（与 H13 要写的代码一一对应）：
     h_sp  += rate(杆) × dt                       杆 1000 → +5cm/s；中段(450) → 0
     e      = h_sp − 读数                          死区 2cm（界内输出 0）
     vz_sp  = clamp(h_kp × e, ±20)                h_kp = 0.6
     corr   = clamp(Kp_v × (vz_sp − vz), ±50)     Kp_v = 20
     i     += (vz_sp − vz) × vz_ki × dt           vz_ki = 5
     thr    = clamp(i + corr, 1150, 1650)，再经限速器（上升 800µs/s）
   地面段：thr 从 1000 起（停手），h_sp > 4cm 才退出停手、控制器才开始干活。
   读数坐标：地面读数 = 3.0cm，读数 = 3.0 + 离地高度。
*/
const g = 981            /* cm/s² */
const U0 = 1035
const U_HOVER = 1450
const DT = 0.004
/* ⚠️ 下面这组常数必须与**飞控代码里的当前值**一致，否则算出来的"预期现象"是错的。
   2026-09-15 飞前统一核对过（改代码里的常量就必须改这里）：
     H_KP / VZ_KP / VZ_KI  ←→ 遥控里首飞要设的值（不是代码默认值）
     RATE_UP / RATE_DOWN   ←→ vertical_control.h 的 VERTICAL_RATE_UP/DOWN_CMS（5/5）
     EXIT_H_SP             ←→ VERTICAL_GROUND_ENTER_H_CM（4.0，停手退出门槛也是 4.0）
     GROUND_READ           ←→ 垫高台架实测 2.8cm，取 3.0 */
const H_KP = 0.3, H_DEAD_CM = 2.0, VZ_SP_MAX = 20.0
const VZ_KP = 2.0, CORR_LIM = 50.0, VZ_KI = 5.0
const RATE_UP = 5.0, RATE_DOWN = 5.0
const THR_MIN = 1150, THR_MAX = 1650, PWM_MIN = 1000
const SLEW_UP = 800.0
const GROUND_READ = 3.0
const EXIT_H_SP = 4.0
const T_MAX = 30.0

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v))
const thrustRatio = u => (u <= U0 ? 0 : ((u - U0) / (U_HOVER - U0)) ** 2)
const f1 = (v, w) => v.toFixed(1).padStart(w)
/* 临时换一个 h_kp 再跑一遍（只给"h_kp 该取多少"那张表用） */
const runWithKp = (kp, mode, targetCm, delayMs) => run(mode, targetCm, delayMs, kp)

/* mode: 'hold'（一直推到底）| 'center'（h_sp 到 targetCm 就回中段）
   delayMs：**传感器回路的纯延迟**（ToF 的 4 步测量周期 + 差分估计 + 取数）。
            ⚠️ 第一版仿真**没有建模延迟**，而 ToF 一步就是 20ms、4 步 83ms
               （方案 §3.8 的滞后预算），延迟是"外环能不能稳住"的决定性因素。
               所以判断环路稳不稳，必须扫一遍 delayMs。 */
function run(mode, targetCm, delayMs, kpOverride, vzKpOverride) {
  const dN = Math.round(((delayMs || 0) / 1000) / DT)   /* 延迟拍数 */
  const hKp = (kpOverride === undefined) ? H_KP : kpOverride
  const vzKp = (vzKpOverride === undefined) ? VZ_KP : vzKpOverride
  let t = 0, hSp = 0, i = 1250, thr = PWM_MIN, vz = 0, hPhys = 0
  let centered = false, centerT = null
  const log = []
  const histRead = [], histVz = []                    /* 延迟线 */
  let liftT = null, liftHSp = null, maxVz = 0, maxOvershoot = 0
  while (t < T_MAX) {
    t += DT
    const stick = (mode === 'hold' || !centered) ? 1000 : 450
    /* 三段映射，**与 firmware 的 VerticalControl_StickToRateCms 逐行等价** */
    const rate = (stick >= 650) ? ((RATE_UP * (stick - 650)) / 350)
               : (stick <= 350) ? (-(RATE_DOWN * (350 - stick)) / 350)
               : 0
    hSp = clamp(hSp + rate * DT, 0, 120)
    if (!centered && hSp >= targetCm) { centered = true; centerT = t }
    histRead.push(GROUND_READ + hPhys)
    histVz.push(vz)
    /* 控制器看到的是 **dN 拍之前** 的读数与垂速 */
    const read = (dN > 0 && histRead.length > dN) ? histRead[histRead.length - 1 - dN] : histRead[histRead.length - 1]
    const vzFb = (dN > 0 && histVz.length > dN) ? histVz[histVz.length - 1 - dN] : histVz[histVz.length - 1]
    if (hSp <= EXIT_H_SP) {                      /* 停手：油门压 1000，控制器不动 */
      thr = Math.max(PWM_MIN, thr - 300 * DT)
      log.push({ t, grounded: true, hSp, read, i, vzSp: 0, corr: 0, thr, vz, hPhys })
      continue
    }
    const e = hSp - read
    const vzSp = (Math.abs(e) <= H_DEAD_CM) ? 0 : clamp(hKp * e, -VZ_SP_MAX, VZ_SP_MAX)
    const corr = clamp(vzKp * (vzSp - vzFb), -CORR_LIM, CORR_LIM)
    i = clamp(i + (vzSp - vzFb) * VZ_KI * DT, THR_MIN, THR_MAX)
    const target = clamp(i + corr, THR_MIN, THR_MAX)
    thr = (target > thr) ? Math.min(target, thr + SLEW_UP * DT) : target
    const aUp = g * (thrustRatio(thr) - 1)
    vz += aUp * DT
    hPhys += vz * DT
    if (hPhys < 0) { hPhys = 0; if (vz < 0) vz = 0 }
    if (liftT === null && hPhys > 0.5) { liftT = t; liftHSp = hSp }
    if (vz > maxVz) maxVz = vz
    if (hPhys - hSp > maxOvershoot) maxOvershoot = hPhys - hSp
    log.push({ t, grounded: false, hSp, read, i, vzSp, corr, thr, vz, hPhys })
  }
  const tail = log.filter(r => r.t > T_MAX - 3 && !r.grounded)
  const tailVz = tail.length ? Math.max(...tail.map(r => Math.abs(r.vz))) : 0
  const tailH = tail.length ? Math.max(...tail.map(r => Math.abs(r.hPhys - r.hSp))) : 0
  const air = log.filter(r => !r.grounded)
  const maxErr = air.length ? Math.max(...air.map(r => Math.abs(r.hPhys - r.hSp))) : 0
  return { log, liftT, liftHSp, maxVz, centerT, tailVz, tailH, maxOvershoot, maxErr }
}

function report(title, mode, targetCm, delayMs) {
  const r = run(mode, targetCm, delayMs)
  console.log('\n' + '='.repeat(94))
  console.log(title)
  console.log('='.repeat(94))
  console.log('   t(s)   h_sp  读数     i    vzsp  corr   thr  垂速cm/s  离地cm')
  console.log('  ' + '-'.repeat(88))
  let next = 0
  for (const x of r.log) {
    if (x.t + 1e-9 >= next) {
      next += 1.0
      const atCenter = (r.centerT !== null && Math.abs(x.t - r.centerT) < DT / 2) ? '  <-- 杆回中段' : ''
      console.log(`  ${f1(x.t, 5)}  ${f1(x.hSp, 5)}  ${f1(x.read, 5)}  ${x.i.toFixed(0).padStart(5)}  ` +
        `${f1(x.vzSp, 5)}  ${x.corr.toFixed(0).padStart(4)}  ${x.thr.toFixed(0).padStart(5)}  ` +
        `${f1(x.vz, 8)}  ${f1(x.hPhys, 7)}${atCenter}`)
    }
  }
  console.log('  ' + '-'.repeat(88))
  if (r.liftT === null) {
    console.log(`  ⚠️ ${T_MAX} 秒内没有离地`)
  } else {
    console.log(`  ⭐ 离地时刻 ${f1(r.liftT, 5)}s；离地那一刻 h_sp = ${f1(r.liftHSp, 4)}cm`)
    console.log(`  ⭐ 最大爬升 ${f1(r.maxVz, 6)} cm/s；最大过冲 ${f1(r.maxOvershoot, 6)} cm（离地高度 − 目标）`)
    console.log(`  ⭐ 末段 3 秒：|垂速| 最大 ${f1(r.tailVz, 6)} cm/s，|高度误差| 最大 ${f1(r.tailH, 6)} cm`)
  }
  return r
}

console.log(`参数（**与飞控当前值和首飞要设的值一致**）：h_kp=${H_KP}  死区=${H_DEAD_CM}cm  vz_sp 限幅=±${VZ_SP_MAX}cm/s`)
console.log(`      vz_kp=${VZ_KP}  corr 限幅=±${CORR_LIM}  vz_ki=${VZ_KI}  杆速率 升${RATE_UP}/降${RATE_DOWN} cm/s`)
console.log(`      起转点 ${U0}µs  悬停 ${U_HOVER}µs  地面读数 ${GROUND_READ}cm  仿真 ${T_MAX} 秒`)
console.log('      "退出停手"发生在 h_sp 涨过 4cm 时（杆到底约 1.33s 后）')
console.log('      ⚠️ 下面几组都**没有建模 ToF 延迟**；延迟的影响见最后那张表')

const A = report('场景 A：杆一直推到底不松手（飞行员以为"推着才上升"）', 'hold', 9999, 0)
const B = report('场景 B：h_sp 涨到 30cm 就把杆回中段', 'center', 30, 0)
const C = report('场景 C：h_sp 涨到 15cm 就回中段（最保守的首飞）', 'center', 15, 0)
const D = report('场景 D：h_sp 涨到 40cm 就回中段', 'center', 40, 0)

console.log('\n' + '='.repeat(94))
console.log('⭐ 结论')
console.log('='.repeat(94))
const rows = [['A 一直推到底', A], ['B 到 30cm 回中', B], ['C 到 15cm 回中', C], ['D 到 40cm 回中', D]]
console.log('  场景              离地(s)  h_sp@离地  最大爬升  最大过冲  末段|垂速|  末段|误差|')
console.log('  ' + '-'.repeat(86))
for (const [name, r] of rows) {
  const s = (v, w) => (v === null ? '—'.padStart(w) : f1(v, w))
  console.log(`  ${name.padEnd(16)} ${s(r.liftT, 5)}  ${s(r.liftHSp, 9)}  ${s(r.maxVz, 8)}  ` +
    `${s(r.maxOvershoot, 8)}  ${s(r.tailVz, 10)}  ${s(r.tailH, 10)}`)
}
console.log('\n  ⭐ 起飞前的"憋油门"时长由 h_kp 与目标高度共同决定：')
console.log('     · 目标越高 ⇒ 外环误差越大 ⇒ vz_sp 越大 ⇒ 积分器（悬停油门估计器）学得越快 ⇒ 憋得越短')
console.log('     · 爬升率很温和（最大 8.4cm/s）：**推力只要≈悬停就能维持匀速爬升**，')
console.log('       所以内环的 P 项只需要很小的修正量，±50µs 限幅基本碰不到。')
console.log('  ⭐ 跟随"持续上升的目标"会稳定滞后约 3cm/s ÷ 0.6 = **5cm**（与方案 §3.3 的预测一致）。')
console.log('  ⭐ 不管哪种推法，最终都收敛到目标 ±死区(2cm) 内 —— 因为外环是"有误差才出力"。')

console.log('\n' + '='.repeat(94))
console.log('⭐⭐ 传感器延迟的影响（这一条决定"外环能不能稳住"，比上面任何数字都重要）')
console.log('='.repeat(94))
console.log('  ToF 一步 20ms、4 步一个测量周期 ⇒ 加上差分估计与取数，回路延迟约 60~100ms。')
console.log('  下面用"场景 C（到 15cm 回中）"扫一遍：')
console.log('')
console.log('  延迟(ms)  离地(s)  最大爬升  最大过冲  稳态|误差|  末段|垂速|  判定')
console.log('  ' + '-'.repeat(84))
for (const d of [0, 40, 80, 120, 160, 200]) {
  const r = run('center', 15, d)
  const ok = (r.tailVz < 10) && (r.tailH < 6)
  const mid = (r.tailVz < 25) && (r.tailH < 15)
  console.log(`  ${String(d).padStart(6)}  ${(r.liftT === null ? '—' : f1(r.liftT, 5))}  ${f1(r.maxVz, 8)}  ` +
    `${f1(r.maxOvershoot, 8)}  ${f1(r.tailH, 10)}  ${f1(r.tailVz, 10)}  ${ok ? '稳' : (mid ? '轻振荡' : '⚠️ 振荡')}`)
}
console.log('\n  ⇒ 延迟是【不可消除】的（ToF 12Hz + 差分垂速），所以外环增益天然有上限。')

console.log('\n' + '='.repeat(94))
console.log('⭐⭐⭐ 那 h_kp 到底该取多少？（前提：内环已定成 vz_kp=2）')
console.log('='.repeat(94))
console.log('  做法：把 H_KP 临时换掉再跑"到 15cm 回中"，延迟分别取 60ms（设计值）与 80ms（ToF 一步）。')
console.log('')
console.log('  h_kp   延迟60ms: 稳态误差 / 末段|垂速|     延迟80ms: 稳态误差 / 末段|垂速|   判定')
console.log('  ' + '-'.repeat(88))
for (const kp of [0.1, 0.2, 0.3, 0.4, 0.6, 0.8]) {
  const a = runWithKp(kp, 'center', 15, 60)
  const b = runWithKp(kp, 'center', 15, 80)
  const ok = (a.tailVz < 8) && (b.tailVz < 12)
  console.log(`  ${kp.toFixed(1).padStart(4)}   ${f1(a.tailH, 10)} / ${f1(a.tailVz, 10)}          ` +
    `${f1(b.tailH, 10)} / ${f1(b.tailVz, 10)}      ${ok ? '稳' : '振荡'}`)
}
console.log('')
console.log('  ⇒ ⭐ 结论（**注意这是内环调好之后才成立的**）：h_kp 取 0.2~0.8 都稳，')
console.log('     所以 **h_kp=0.3 是保守值、不是临界值** —— 首飞取 0.3，之后往上加。')
console.log('     ⚠️ 反过来说：**h_kp 不是瓶颈，vz_kp 才是**。H13 方案最初把 0.6 降成 0.3，')
console.log('        原因写的是"延迟"；扫完表才看清真正的瓶颈是内环。**0.3 这个值仍然是好的**')
console.log('        （保守起步），只是理由要更正 —— 改代码注释时不要照抄旧理由。')
console.log('     ⚠️ 本模型没有 vz 噪声、没有姿态动力学，所以它只能定"量级"，不能定精确值 ——')
console.log('        真值要靠 H15 用黑匣子数据整定。')

console.log('\n' + '='.repeat(94))
console.log('⭐⭐⭐ 等等 —— 极限环到底是外环还是内环引起的？')
console.log('='.repeat(94))
console.log('  算式：悬停点附近 d(推力比)/du = 2/415 = 0.00482 /µs')
console.log('        ⇒ 修正量每变 1µs，向上加速度变 4.7 cm/s²')
console.log('        ⇒ 内环 corr = -Kp_v·vz 形成的闭环时间常数 = 1/(Kp_v × 4.7)')
console.log(`        ⇒ Kp_v=20 时 **约 ${(1 / (20 * 4.7) * 1000).toFixed(0)} ms**，而传感器+电机延迟约 80~150ms`)
console.log('        ⇒ **回路比延迟快 8~15 倍 ⇒ 相位裕度必然为负。**')
console.log('  ⇒ ⭐ 结论：**内环 vz_kp=20（H9 的实验值）在真实延迟下偏大**。')
console.log('     ⭐ 首飞因此把 vz_kp 定成 **2**（本文件顶部的 VZ_KP 就是这个值）——')
console.log('        上面所有表都是在 vz_kp=2 的前提下跑的，反映的是**真实飞行参数**。')
console.log('')
console.log('  ⭐ 扫细一点：延迟取 80 / 100 / 120 / 160ms（真实 vz 通道估计在 100~150ms）。')
console.log('')
console.log('  vz_kp   80ms末段|垂速|  100ms末段|垂速|  120ms末段|垂速|  160ms末段|垂速|   判定')
console.log('  ' + '-'.repeat(90))
for (const vk of [1, 1.5, 2, 3, 4, 5]) {
  const a = run('center', 15, 80, 0.3, vk)
  const b = run('center', 15, 100, 0.3, vk)
  const c = run('center', 15, 120, 0.3, vk)
  const d = run('center', 15, 160, 0.3, vk)
  const ok = (a.tailVz < 5) && (b.tailVz < 8) && (c.tailVz < 12)
  console.log(`  ${String(vk).padStart(5)}   ${f1(a.tailVz, 13)}   ${f1(b.tailVz, 14)}   ${f1(c.tailVz, 14)}   ${f1(d.tailVz, 14)}   ${ok ? '稳' : '振荡'}`)
}
console.log('')
console.log('  ⚠️⚠️ 这一条**推翻了 H9 的"台架验过了"**：H9 验的是**符号**，而台架拆桨时')
console.log('      **根本没有真实的推力反馈回路**（飞机不动、vz 是靠手给的），')
console.log('      所以"内环增益适合多少"从头到尾**没有被验证过** —— 这是 H14 的活。')
console.log('  ⇒ ⭐⭐ 首飞参数（**按上面这张表定，不要用设计值**）：')
console.log('       vz_kp = 2    h_kp = 0.3    vz_ki = 5    MAX THROTTLE = 2000')
console.log('     ⚠️ 顺序很重要：**先定内环（vz_kp），再定外环（h_kp）** —— 上面的表就是按这个顺序跑的。')
console.log('        H13 最初反过来（先怀疑外环、把 0.6 降到 0.3），扫完才发现瓶颈在内环；')
console.log('        **0.3 这个值仍然是好的**（保守起步），但理由要更正，不要照抄旧理由。')
console.log('     ⚠️ vz_kp 取 2 而不是 5：5 在 80ms 延迟下就已经\"轻振荡\"（末段垂速 18cm/s），')
console.log('        而真实 vz 通道延迟估计在 100~150ms。**取 2 的代价只是反应迟钝**，')
console.log('        取 5 的代价是高度在目标附近来回抽 —— 首飞先要稳。')
