#!/usr/bin/env python3
"""check_srdf_states.py —— 静态校验 SRDF 命名状态与 URDF 是否一致（不启动 ROS、不需要真机）。

为什么需要它
    SRDF 写错的代价很隐蔽：关节名写错 → 那段被忽略；group_state 漏写某个关节 →
    MoveIt 把它当 0；关节值超限 → 规划时才报错，或者真机上先动一下再中止。
    这个脚本只解析两份 XML，1 秒内把这几类问题抓出来。

用法
    pixi run python src/test/check_srdf_states.py
    pixi run python src/test/check_srdf_states.py --srdf <path> --urdf <path>

退出码
    0 = 全过（可能带警告）；1 = 有错误；2 = 只有警告。
"""
from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET

# 与真机一致的余量要求
REV_MARGIN = 0.02   # rad，约 1.15°
PRI_MARGIN = 0.001  # m

WARN, ERR = "warn", "error"


def workspace_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))       # .../src/test
    return os.path.dirname(os.path.dirname(here))           # 工作区根目录


def rel(path: str) -> str:
    try:
        r = os.path.relpath(path, workspace_root())
    except ValueError:
        return path
    return path if r.startswith("..") else r


def load_urdf(path: str):
    """返回 (joints, links)。joints: name -> {type, lower, upper, mimic, parent, child}"""
    root = ET.parse(path).getroot()
    links = {link.get("name") for link in root.findall("link")}
    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        lim = j.find("limit")
        lower = upper = None
        if lim is not None:
            if lim.get("lower") is not None:
                lower = float(lim.get("lower"))
            if lim.get("upper") is not None:
                upper = float(lim.get("upper"))
        mimic = j.find("mimic")
        joints[name] = {
            "type": j.get("type"),
            "lower": lower,
            "upper": upper,
            "mimic": mimic.get("joint") if mimic is not None else None,
            "parent": j.find("parent").get("link") if j.find("parent") is not None else None,
            "child": j.find("child").get("link") if j.find("child") is not None else None,
        }
    return joints, links


def load_srdf(path: str):
    root = ET.parse(path).getroot()
    groups = {}
    for g in root.findall("group"):
        groups[g.get("name")] = {j.get("name") for j in g.findall("joint")}
    states = []
    for g in root.findall("group_state"):
        states.append({
            "name": g.get("name"),
            "group": g.get("group"),
            "joints": [(j.get("name"), float(j.get("value"))) for j in g.findall("joint")],
        })
    end_effectors = [
        {"name": e.get("name"), "parent_link": e.get("parent_link"),
         "group": e.get("group"), "parent_group": e.get("parent_group")}
        for e in root.findall("end_effector")
    ]
    virtual_joints = [
        {"name": v.get("name"), "parent_frame": v.get("parent_frame"), "child_link": v.get("child_link")}
        for v in root.findall("virtual_joint")
    ]
    disable = [(d.get("link1"), d.get("link2")) for d in root.findall("disable_collisions")]
    return groups, states, end_effectors, virtual_joints, disable


def check(srdf_path: str, urdf_path: str):
    issues = []   # (level, msg)

    def add(level, msg):
        issues.append((level, msg))

    if not os.path.isfile(srdf_path):
        add(ERR, f"找不到 SRDF：{srdf_path}")
    if not os.path.isfile(urdf_path):
        add(ERR, f"找不到 URDF：{urdf_path}")
    if any(lvl == ERR for lvl, _ in issues):
        return issues, None

    if "/install/" in os.path.abspath(srdf_path):
        add(WARN, "校验的是 install/ 里的副本；源码在 src/dm_armx_config/config/，"
                  "确保你改的是源码那份（改完要 colcon build）")

    joints, links = load_urdf(urdf_path)
    groups, states, ees, vjs, disable = load_srdf(srdf_path)
    srdf_virtual = {v["name"] for v in vjs}

    def movable_joints(members):
        """group 里真正要写进 group_state 的关节：可动、且不是 mimic。"""
        return {j for j in members
                if joints.get(j, {}).get("type") in ("revolute", "prismatic", "continuous")
                and not joints.get(j, {}).get("mimic")}

    # ── 1. group 成员 ──
    for gname, members in groups.items():
        for jname in sorted(members):
            if jname in joints:
                continue
            if jname in srdf_virtual:
                add(WARN, f"group '{gname}' 含虚拟关节 '{jname}'（fixed，不在 URDF 里）；"
                          "建议从 group 移除，命名状态里也不需要它")
            else:
                add(ERR, f"group '{gname}' 引用了 URDF 里不存在的关节 '{jname}'")
        fixed_in_group = {j for j in members if joints.get(j, {}).get("type") == "fixed"}
        if fixed_in_group:
            add(WARN, f"group '{gname}' 里混了 fixed 关节：{', '.join(sorted(fixed_in_group))}"
                      "（不影响规划，但状态里写它没有意义）")
        if not movable_joints(members):
            add(WARN, f"group '{gname}' 没有可动关节，MoveIt 里无法规划")

    # ── 2. group_state ──
    seen_states = set()
    for st in states:
        key = (st["group"], st["name"])
        if key in seen_states:
            add(ERR, f"group_state 重复：group={st['group']} name={st['name']}")
        seen_states.add(key)

        if st["group"] not in groups:
            add(ERR, f"group_state '{st['name']}' 指向不存在的 group '{st['group']}'")
            continue

        members = groups[st["group"]]
        required = movable_joints(members)
        state_joints = {j for j, _ in st["joints"]}

        for jname, _ in st["joints"]:
            if jname not in joints:
                if jname in srdf_virtual:
                    add(WARN, f"group_state '{st['name']}' 写了虚拟关节 '{jname}'，应删掉")
                else:
                    add(ERR, f"group_state '{st['name']}' 引用了不存在的关节 '{jname}'")
            elif joints[jname].get("mimic"):
                add(WARN, f"group_state '{st['name']}' 写了 mimic 关节 '{jname}'；"
                          "mimic 由驱动关节带着动，不该单独给值")
            elif jname not in members:
                add(ERR, f"group_state '{st['name']}' 里的 '{jname}' 不属于 group '{st['group']}'")

        missing = required - state_joints
        if missing:
            add(ERR, f"group_state '{st['name']}' 漏写了 {', '.join(sorted(missing))}"
                      "（MoveIt 会把漏写的关节当 0，不报错）")

        for jname, value in st["joints"]:
            info = joints.get(jname)
            if not info or info["type"] not in ("revolute", "prismatic", "continuous"):
                continue
            lo, hi = info["lower"], info["upper"]
            margin = REV_MARGIN if info["type"] != "prismatic" else PRI_MARGIN
            if lo is not None and value < lo - 1e-9:
                add(ERR, f"group_state '{st['name']}'：{jname}={value:+.5f} 低于下限 {lo:+.5f}")
            elif hi is not None and value > hi + 1e-9:
                add(ERR, f"group_state '{st['name']}'：{jname}={value:+.5f} 高于上限 {hi:+.5f}")
            else:
                gap = min(value - lo if lo is not None else 1e9,
                          hi - value if hi is not None else 1e9)
                if gap < margin - 1e-9:
                    add(WARN, f"group_state '{st['name']}'：{jname}={value:+.5f} 离限位只有 {gap:.4f}"
                              f"（要求 ≥{margin}），真机零位噪声可能被判越界")

    # ── 3. end_effector / virtual_joint / disable_collisions ──
    for e in ees:
        if e["parent_link"] not in links:
            add(ERR, f"end_effector '{e['name']}' 的 parent_link '{e['parent_link']}' 不在 URDF")
        if e["group"] not in groups:
            add(ERR, f"end_effector '{e['name']}' 的 group '{e['group']}' 不存在")
        if e["parent_group"] not in groups:
            add(ERR, f"end_effector '{e['name']}' 的 parent_group '{e['parent_group']}' 不存在")

    for v in vjs:
        if v["child_link"] not in links:
            add(ERR, f"virtual_joint '{v['name']}' 的 child_link '{v['child_link']}' 不在 URDF")

    seen_pairs = set()
    for a, b in disable:
        for name in (a, b):
            if name not in links:
                add(ERR, f"disable_collisions 引用了不存在的 link '{name}'")
        if a == b:
            add(ERR, f"disable_collisions 自己和自己：'{a}'")
        pair = tuple(sorted((a, b)))
        if pair in seen_pairs:
            add(WARN, f"disable_collisions 重复：{a} <-> {b}")
        seen_pairs.add(pair)

    return issues, {"joints": joints, "groups": groups, "states": states}


def print_report(issues, data):
    if data is None:
        for level, msg in issues:
            print(f"[{level.upper():5s}] {msg}")
        return

    joints, groups, states = data["joints"], data["groups"], data["states"]

    print("=" * 78)
    print("SRDF 命名状态一览")
    print("=" * 78)
    for st in states:
        members = groups.get(st["group"], set())
        required = {j for j in members
                    if joints.get(j, {}).get("type") in ("revolute", "prismatic", "continuous")
                    and not joints.get(j, {}).get("mimic")}
        values = dict(st["joints"])
        cols = [f"{j}={values[j]:+.4f}" if j in values else f"{j}=MISSING"
                for j in sorted(required, key=lambda s: (len(s), s))]
        print(f"  [{st['group']}/{st['name']}]")
        print("    " + "  ".join(cols))
        gaps = []
        for jname, v in st["joints"]:
            info = joints.get(jname) or {}
            if info.get("type") not in ("revolute", "prismatic"):
                continue
            lo, hi = info.get("lower"), info.get("upper")
            gaps.append((min(v - lo if lo is not None else 1e9,
                             hi - v if hi is not None else 1e9), jname))
        if gaps:
            gap, jname = min(gaps)
            print(f"    最小限位余量：{gap:.4f}（{jname}）")

    errors = [m for lvl, m in issues if lvl == ERR]
    warns = [m for lvl, m in issues if lvl == WARN]
    print()
    print(f"结论：{len(errors)} 个错误，{len(warns)} 个警告")
    for level, msg in issues:
        print(f"  [{level.upper():5s}] {msg}")
    if not issues:
        print("  [OK   ] 全部通过")


def main() -> int:
    root = workspace_root()
    ap = argparse.ArgumentParser(description="静态校验 SRDF 命名状态")
    ap.add_argument("--srdf", default=os.path.join(root, "src/dm_armx_config/config/DM_Armx.srdf"))
    ap.add_argument("--urdf", default=os.path.join(root, "src/dm_armx_description/urdf/arm.urdf"),
                    help="活 URDF（bringup 里 xacro include 的就是它）")
    args = ap.parse_args()

    print(f"URDF: {rel(args.urdf)}")
    print(f"SRDF: {rel(args.srdf)}")
    print()

    issues, data = check(args.srdf, args.urdf)
    print_report(issues, data)

    if any(lvl == ERR for lvl, _ in issues):
        return 1
    if any(lvl == WARN for lvl, _ in issues):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
