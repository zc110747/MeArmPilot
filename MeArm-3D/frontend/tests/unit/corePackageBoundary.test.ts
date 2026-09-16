/**
 * **Core → Robot Package 的 import 边界**（Phase 2 步④ 新增）。
 *
 * ## 这份测试在防什么
 *
 * Phase 2 把"型号专属的实现"搬进了各自的包（`robot-package/<id>/kinematics/`），
 * 并给 Core 留了一条**正确的路**：
 *
 * ```text
 *   RobotRegistry.loadRobot(id).kinematics.inverse(...)   ← 统一 IKResult 形状
 * ```
 *
 * 但**旧的路没有被封住** —— `robot-package/` 就在仓库里，Core 里任何一个文件
 * 都能写 `import ... from '../../robot-package/mearm-v1/kinematics/ik'`。
 * 而且这条退化的路**不会报错**：类型对得上、测试跑得过、构建打得出来。
 *
 * 它的真实代价是"抽象失效"：Core 一旦认识了某个型号的原生符号，
 * 「新增一台机器人 = 丢一个包」这个承诺就变成了空话 ——
 * 每接一台新机器人，都要回到 Core 里加分支。
 *
 * ⇒ 所以这里把**全部**"Core → 包"的 import 列出来，**白名单外一律失败**。
 * 它的价值不在今天（今天只有一条），而在于**明天别人加第二条时会响**。
 *
 * ## 白名单：一条**刻意留下**的倒置
 *
 * `src/store/robotStore.ts` 直接 import 了 MeArm 解析解的 `solveIk` 与四个原生类型。
 * 按分层这不该存在，但**这一轮不修** —— 因为它暴露的是原生诊断
 * （`candidates` / `azimuth` / `relativeAngle` / `joint`），而统一 `IKResult`
 * 刻意不带这些字段：改成走引擎会**减少**界面信息，属于行为变更，
 * 与「MeArm 冻结优先 / 抽象前后逐位一致」冲突。
 *
 * 处置路径已登记（见 `robotStore.ts` 的注释与 `docs/decisions.md`）：
 * 给 `IKResult` 加可选 `diagnostics` 袋子，包内适配器填充，Core 只透传。
 * 届时本白名单要**跟着变空** —— 那一步完成前，这里就是它的看门人。
 *
 * ## 一个刻意的实现细节：先剥注释再匹配
 *
 * Core 里大量注释**在讨论** `robot-package/...`（它们是文档，不是依赖）。
 * 不剥注释的话这份测试会被自己的文档顶红，然后被人用"再加一条豁免"糊过去。
 * 剥法是近似（正则，不认识字符串里的 `//`），但对"路径出现在 import 说明符里"
 * 这个判据足够 —— 而且**宁可多报**（多报会被看见并修正），不可漏报。
 */
import { describe, expect, it } from 'vitest';

/** 仓库相对路径 → 原文（glob key 是否带 `../` 前缀是 Vite 的实现细节，故用标记定位） */
const CORE_SOURCES = import.meta.glob('../../src/**/*.{ts,tsx,vue}', {
  eager: true,
  query: '?raw',
  import: 'default',
}) as Record<string, string>;

const MARKER = 'src/';

/** glob key（`../../src/store/robotStore.ts`）→ 仓库相对路径（`frontend/src/store/robotStore.ts`） */
function relPathOf(key: string): string {
  const at = key.lastIndexOf(`/${MARKER}`);
  if (at < 0) throw new Error(`源码 glob 的 key 不符合预期：${key}`);
  return `frontend/${key.slice(at + 1)}`;
}

/** 剥掉块注释与行注释。近似实现 —— 宁可多报，不可漏报（见文件头）。 */
function stripComments(code: string): string {
  return code.replace(/\/\*[\s\S]*?\*\//g, ' ').replace(/^\s*\/\/.*$/gm, ' ');
}

/**
 * 白名单：允许从包里 import 的 Core 文件 → 允许的符号。
 *
 * `null` 表示"这个文件里的包 import 全部豁免"（不做逐符号核对）——
 * 目前没有这样的条目，它是给"将来某条边确实需要整包透传"留的形状。
 */
const ALLOWED: Record<string, readonly string[] | null> = {
  'frontend/src/store/robotStore.ts': [
    // MeArm 解析解的原生入口（行为依赖，见文件头"白名单"一节）
    'solveIk',
    // ★ 几何真值：安全参数的前置检查**必须**与 solveIk 用同一个 `d`。
    //   曾经在 store 里自己重算过 —— 因为把 `radial` 取成了 `wrist.parentLink`
    //   （= forearm_link 80）而不是 `toolOffset[0]`（= 40），同一个目标点算出
    //   两个 `d`（92.159 vs 111.542，差 19.4mm）且不报错，症状是"改小外径后
    //   本该可达的点被判越界"。宁可扩这条边，也不要第二个口径。
    'ikGeometry',
    'wristSagittal',
    // 四种原生类型：IK 的分支 / 偏好 / 失败原因 / 原生返回形状
    'IkBranch',
    'IkPreference',
    'IkReason',
    'IkResult',
  ],
};

interface PackageImport {
  /** 仓库相对路径 */
  readonly file: string;
  /** import 说明符原文 */
  readonly specifier: string;
  /** 具名导入的符号（已去掉 `type `），仅 `import { ... } from` 形式能取到 */
  readonly names: readonly string[];
}

/**
 * 匹配 `import ... from '<spec>'`（含 `import type`）与 `import('<spec>')`。
 *
 * 子句用 `[^;]*?` 而不是 `[\s\S]*?`：**禁止跨过分号**。否则遇到
 * `import './x.css';`（无 `from`）时正则会一路吃下后续若干条语句，
 * 把不相干的符号算进 `names`，报出一堆假阳性。
 */
const FROM_RE = /import\s+(?:type\s+)?([^;]*?)\s*from\s*['"]([^'"]+)['"]/g;
const DYNAMIC_RE = /import\s*\(\s*['"]([^'"]+)['"]\s*\)/g;
const NAMED_RE = /\{([\s\S]*?)\}/;

function collect(): PackageImport[] {
  const out: PackageImport[] = [];
  for (const [key, raw] of Object.entries(CORE_SOURCES)) {
    const code = stripComments(raw);
    const file = relPathOf(key);

    const push = (specifier: string, clause: string): void => {
      if (!specifier.includes('robot-package/')) return;
      const named = NAMED_RE.exec(clause);
      const names = named
        ? named[1]!
            .split(',')
            .map((s) => s.trim().replace(/^type\s+/, ''))
            .filter((s) => s.length > 0)
        : [];
      out.push({ file, specifier, names });
    };

    for (const m of code.matchAll(FROM_RE)) push(m[2]!, m[1]!);
    for (const m of code.matchAll(DYNAMIC_RE)) push(m[1]!, '');
  }
  return out;
}

const PACKAGE_IMPORTS = collect();

describe('Core 不得依赖 robot-package（白名单外一律失败）', () => {
  it('glob 确实扫到了 Core 源码（否则下面所有断言都会"空集通过"）', () => {
    expect(Object.keys(CORE_SOURCES).length).toBeGreaterThan(50);
    expect(relPathOf('../../src/store/robotStore.ts')).toBe(
      'frontend/src/store/robotStore.ts',
    );
  });

  it('"Core → 包"的 import 只允许出现在白名单文件里', () => {
    const offenders = PACKAGE_IMPORTS.map((i) => i.file).filter((f) => !(f in ALLOWED));
    expect(
      [...new Set(offenders)].sort(),
      'Core 里出现了新的"直连包"import。请改走 ' +
        'RobotRegistry.loadRobot(id).kinematics（统一形状）；' +
        '若确实必须直连，请把它登记进 ALLOWED 并写清理由。',
    ).toEqual([]);
  });

  it('白名单里的文件确实还在用它（防止白名单腐烂成"永久豁免")', () => {
    const seen = new Set(PACKAGE_IMPORTS.map((i) => i.file));
    for (const file of Object.keys(ALLOWED)) {
      expect(seen.has(file), `${file} 已不再直连包，请从 ALLOWED 里删掉`).toBe(true);
    }
  });

  it('robotStore 只取白名单里的符号（多取一个都会悄悄扩大耦合面）', () => {
    const entries = PACKAGE_IMPORTS.filter((i) => i.file === 'frontend/src/store/robotStore.ts');
    expect(entries).toHaveLength(1);
    const allowed = ALLOWED['frontend/src/store/robotStore.ts'];
    expect(allowed, 'robotStore 的白名单不该是 null').not.toBeNull();
    const extra = entries[0]!.names.filter((n) => !allowed!.includes(n));
    expect(extra, 'robotStore 从包里多取了符号；要么改走统一形状，要么更新 ALLOWED').toEqual(
      [],
    );
    // 反向：白名单里的每个符号都该真的被用（否则白名单在虚报"必要"）
    const missing = allowed!.filter((n) => !entries[0]!.names.includes(n));
    expect(missing, 'ALLOWED 里列了但实际没 import 的符号').toEqual([]);
  });

  it('包内实现不再从 Core 的旧位置出现（搬迁后不许留下"第二条路"）', () => {
    const stale = PACKAGE_IMPORTS.filter((i) =>
      /robot\/kinematics\/(ik|mearm)/.test(i.specifier),
    );
    expect(stale, '还在 import Core 里的旧版 IK/引擎路径；它们已经搬进包里').toEqual([]);
  });
});
