/* "ToF 失效后把油门从悬停值缓慢降到 1000" 到底会发生什么 —— 一维数值仿真。
   ⚠️ 这不是飞控代码，只是在 PC 上算物理，用来给"斜坡取多快"提供依据。

   模型（与机体质量无关，因为除以了重力）：
     推力比 = ((u − u0) / (u_hover − u0))²      ← 螺旋桨推力 ∝ 转速²、转速 ∝ (油门−起转点)
     a/g   = 1 − 推力比                          ← 向下为正
     起转点 u0 = 1035µs（mixer.h 实测：M3 稳定起转 1030、其余 1035）
     悬停   u_hover = 1420µs（据用户"1450 左右能起飞"倒推）
   对照：u=1420 → a=0；u=1300 → 0.53g；u=1200 → 0.82g；u≤1035 → 1g（自由落体）

   用法：node sim_failsafe_ramp.js
*/
const g = 9.81
const U0 = 1035
const U_HOVER = 1420
const U_MIN = 1000

function accel(u) {
  if (u <= U0) return g
  const r = (u - U0) / (U_HOVER - U0)
  return g * (1 - r * r)
}

/* rateUsPerS: 油门下降斜率（µs/s）；Infinity = 立刻到 1000（= 现在的行为）
   holdS: 先保持悬停多久再开始降（0 = 立刻开始降）
   h0: 起始高度 m */
function run(rateUsPerS, holdS, h0, useLinearThrust) {
  const dt = 0.001
  let u = U_HOVER
  let v = 0
  let h = h0
  let t = 0
  const acc = (uu) => {
    if (useLinearThrust) {
      if (uu <= U0) return g
      const r = (uu - U0) / (U_HOVER - U0)
      return g * (1 - Math.min(1, r))
    }
    return accel(uu)
  }
  while (h > 0 && t < 60) {
    t += dt
    if (t > holdS) {
      if (rateUsPerS === Infinity) u = U_MIN
      else {
        u -= rateUsPerS * dt
        if (u < U_MIN) u = U_MIN
      }
    }
    const a = acc(u)
    v += a * dt
    h -= v * dt
  }
  return { tImpact: t, vImpact: v, uAtImpact: u }
}

console.log('='.repeat(84))
console.log('ToF 失效、油门从 1420 降到 1000 —— 落地速度与耗时（起始高度 1.00m）')
console.log('='.repeat(84))
console.log('斜坡(µs/s)   先保持   落地耗时   落地速度   落地时油门   相当于从多高自由落体')
console.log('-'.repeat(84))

const cases = [
  [Infinity, 0, '现在的行为：立刻停车'],
  [1000, 0, ''],
  [500, 0, ''],
  [200, 0, ''],
  [100, 0, ''],
  [50, 0, ''],
  [20, 0, ''],
  [100, 1.0, '先保持 1s 再降'],
  [50, 1.0, ''],
  [20, 1.0, ''],
]
for (const [rate, hold, note] of cases) {
  const r = run(rate, hold, 1.0, false)
  const hEq = (r.vImpact * r.vImpact) / (2 * g)   // 等价自由落体高度
  const rateTxt = rate === Infinity ? '  立刻' : String(rate).padStart(6)
  console.log(`${rateTxt}      ${hold.toFixed(1)}s    ${r.tImpact.toFixed(2)}s     ${r.vImpact.toFixed(2)} m/s    ${Math.round(r.uAtImpact)}µs      ${(hEq * 100).toFixed(1)} cm${note ? '   ← ' + note : ''}`)
}

console.log('\n' + '='.repeat(84))
console.log('起始高度 0.50m（用户说"最多也就 1 米"）')
console.log('='.repeat(84))
console.log('斜坡(µs/s)   先保持   落地耗时   落地速度   落地时油门   相当于从多高自由落体')
console.log('-'.repeat(84))
for (const [rate, hold] of [[Infinity, 0], [200, 0], [100, 0], [50, 0], [20, 0], [100, 1.0], [50, 1.0]]) {
  const r = run(rate, hold, 0.5, false)
  const hEq = (r.vImpact * r.vImpact) / (2 * g)
  const rateTxt = rate === Infinity ? '  立刻' : String(rate).padStart(6)
  console.log(`${rateTxt}      ${hold.toFixed(1)}s    ${r.tImpact.toFixed(2)}s     ${r.vImpact.toFixed(2)} m/s    ${Math.round(r.uAtImpact)}µs      ${(hEq * 100).toFixed(1)} cm`)
}

console.log('\n' + '='.repeat(84))
console.log('灵敏度检查：把推力模型从"平方"换成最保守的"线性"（下降更慢）')
console.log('='.repeat(84))
console.log('（两个模型给出的落地速度差多少，说明这个结论对模型有多敏感）\n')
console.log('斜坡(µs/s)   平方模型 v   线性模型 v    1m 高度')
for (const rate of [Infinity, 200, 100, 50, 20]) {
  const a = run(rate, 0, 1.0, false).vImpact
  const b = run(rate, 0, 1.0, true).vImpact
  console.log(`${(rate === Infinity ? '立刻' : rate).toString().padStart(6)}        ${a.toFixed(2)} m/s     ${b.toFixed(2)} m/s`)
}

console.log('\n' + '='.repeat(84))
console.log('⭐ 两个关键结论（不看模型细节也成立）')
console.log('='.repeat(84))
const now = run(Infinity, 0, 1.0, false)
const slow = run(50, 0, 1.0, false)
console.log(` ① 落地速度只由"重力 vs 推力"决定，不由斜率决定：`)
console.log(`      立刻停车      -> ${now.tImpact.toFixed(2)}s 落地、${now.vImpact.toFixed(2)} m/s`)
console.log(`      50µs/s 慢降   -> ${slow.tImpact.toFixed(2)}s 落地、${slow.vImpact.toFixed(2)} m/s`)
console.log(`      动能比 = (${slow.vImpact.toFixed(2)}/${now.vImpact.toFixed(2)})² = ${((slow.vImpact / now.vImpact) ** 2).toFixed(2)}  ⇒ 慢降把冲击能量降到约 ${Math.round((slow.vImpact / now.vImpact) ** 2 * 100)}%`)
console.log(` ② 在 1m 高度下，**飞机落地时油门还没降到 1000**（上表"落地时油门"那一列）：`)
for (const rate of [Infinity, 500, 200, 100, 50, 20]) {
  const r = run(rate, 0, 1.0, false)
  console.log(`      ${(rate === Infinity ? '立刻' : rate + 'µs/s').padStart(8)} -> 落地时油门 ${Math.round(r.uAtImpact)}µs`)
}
console.log(`\n  ⇒ "降到 1000" 这个终值在低空场景里基本看不到，斜率实际控制的是"前段还有多少推力"。`)

/* ══════════════════════════════════════════════════════════════════════════
   ★ 2026-09-15 用户拍板的参数：**先保持 2.0 秒，再以 20 或 30µs/s 下降**
   ⚠️ 模型假设：失效那一刻飞机**正在悬停**（油门 1420、v=0）。
      如果失效时它本来就在下沉，保持段的"冻结油门"保持不了高度，结果会更差。
   ══════════════════════════════════════════════════════════════════════════ */
console.log('\n' + '='.repeat(88))
console.log('★ 用户定的参数：保持 2.0s 之后再降（斜率 20 / 30 / 50µs/s）')
console.log('='.repeat(88))
console.log('起始     斜坡      落地耗时   落地速度   落地时油门   等价自由落体   落地后桨还转多久')
console.log('-'.repeat(88))
for (const h0 of [1.0, 0.5]) {
  for (const rate of [20, 30, 50]) {
    const r = run(rate, 2.0, h0, false)
    const hEq = (r.vImpact * r.vImpact) / (2 * g)
    const spin = (r.uAtImpact - U_MIN) / rate
    console.log(`${h0.toFixed(2)}m   ${String(rate).padStart(4)}µs/s   ${r.tImpact.toFixed(2)}s     ${r.vImpact.toFixed(2)} m/s    ${Math.round(r.uAtImpact)}µs      ${(hEq * 100).toFixed(1)} cm        ${spin.toFixed(1)}s`)
  }
  const imm = run(Infinity, 0, h0, false)
  console.log(`${h0.toFixed(2)}m    立刻      ${imm.tImpact.toFixed(2)}s     ${imm.vImpact.toFixed(2)} m/s    ${Math.round(imm.uAtImpact)}µs      ${((imm.vImpact ** 2) / (2 * g) * 100).toFixed(1)} cm         0.0s   ← 现状`)
}
console.log('\n  ⇒ "落地后桨还转多久" = (落地时油门 − 1000) / 斜坡。')
console.log('     它**只在你不上锁时才存在** —— 你上锁（!armed 分支）就立刻停桨。')
console.log('     但它决定了"撞地后电机还带电多久"，是斜坡取多慢的真实代价。')

console.log('\n' + '='.repeat(88))
console.log('★ 姿态超限（>25°）如果也走"2s 保持 + 20µs/s"会怎样')
console.log('='.repeat(88))
{
  /* 姿态超限 ≈ 已经撞了。飞机通常在半秒内就落地，所以 2s 保持段全程都在地上。 */
  const rate = 20
  const u0 = U_HOVER
  const held = 2.0
  const rampS = (u0 - U_MIN) / rate
  console.log(`  假设撞地那一刻油门是 ${u0}µs（悬停值）：`)
  console.log(`    保持段        ${held.toFixed(1)}s（油门锁在 ${u0}µs ⇒ **桨全速转**）`)
  console.log(`    下降段        ${rampS.toFixed(1)}s（${u0} → ${U_MIN}，斜率 ${rate}µs/s）`)
  console.log(`    ⇒ 从撞地到桨完全停转：约 **${(held + rampS).toFixed(0)} 秒**`)
  console.log(`  对照——现状（立刻 1000）： **0 秒**（撞地瞬间桨就停）`)
  console.log(`\n  ⚠️ 而"姿态超 25°"按你的说法是**判定炸机、不管了**。`)
  console.log(`     "不管了"的本意是赶紧停桨；2s+20µs/s 的效果正相反：桨还转 ${(held + rampS).toFixed(0)} 秒。`)
  console.log(`     这 ${(held + rampS).toFixed(0)} 秒里飞机已经躺在地上，桨在转 ⇒ 蹭地、翻跟头、电调发热。`)
}
