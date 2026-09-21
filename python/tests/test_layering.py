"""分层约束 —— 通用几何不许依赖风团领域。

这个文件里**没有业务断言**，只有依赖方向。它存在的理由是一次真实的返工：最初
``scene.py``（现 :mod:`wheal.geometry`）从 ``analyze.py`` import ``accumulate``、
从 ``metrics.py`` import ``AnalyzeParams``。后果是“通用 2.5D 建模”只是口号——
任何复用都要先把风团研究模块拖进来。拆开之后，这种依赖很容易在下一次“顺手
用一下那个函数”里悄悄长回来，所以用测试钉住。

**这是 AST 层面的检查，不是运行时 import 检查。** 运行时检查做不到：Python 导入
子模块必然先执行包的 ``__init__.py``，而 ``__init__.py`` 为了导出 API 会导入所有
模块，于是无论分层对不对，``sys.modules`` 里都躺着 ``wheal.analyze``。

**允许集合是精确匹配，不是“包含”。** 新增一条包内依赖必须改这张表，也就是
必须想一下它该不该存在——这比“只要不含 analyze 就行”有用得多。
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

WHEAL_DIR = Path(__file__).resolve().parents[1] / "wheal"

#: 通用层各模块**允许**依赖的包内模块。多一个就挂。
GENERIC_LAYER: dict[str, set[str]] = {
    "stats": set(),
    "accumulate": {"frames", "stats"},
    "geometry": {"accumulate", "intrinsics"},
}

#: 风团领域模块。通用层出现其中任何一个都说明分层已经破了。
DOMAIN_MODULES = {"analyze", "baseline", "metrics", "sweep", "report", "synthetic", "cli"}


def _package_imports(module: str) -> set[str]:
    """模块的包内相对 import 目标（``from .x import y`` 里的 ``x``）。"""
    tree = ast.parse((WHEAL_DIR / f"{module}.py").read_text(encoding="utf-8"))
    return {
        node.module.split(".")[-1]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module and node.level == 1
    }


@pytest.mark.parametrize(("module", "allowed"), sorted(GENERIC_LAYER.items()))
def test_generic_module_only_depends_on_what_it_may(module, allowed):
    found = _package_imports(module)
    unexpected = found - allowed
    assert not unexpected, (
        f"{module}.py 新增了对 {sorted(unexpected)} 的依赖。"
        f"如果这是有意的，把它加进 GENERIC_LAYER；如果不是，说明通用层正在长回领域依赖。"
    )


@pytest.mark.parametrize("module", sorted(GENERIC_LAYER))
def test_generic_layer_never_reaches_into_the_domain(module):
    """单列一条直白的断言：错误信息要一眼能看懂，不用去比对允许集合。"""
    reachable = _package_imports(module) & DOMAIN_MODULES
    assert not reachable, f"{module}.py 依赖了风团领域模块 {sorted(reachable)}"


def test_the_allow_table_covers_every_generic_module():
    """防止有人加了通用模块却忘了纳入这张表——那样它就不受任何约束。"""
    generic = {"stats", "accumulate", "geometry"}
    assert generic == set(GENERIC_LAYER)


def test_stats_is_a_true_leaf():
    """``stats`` 是依赖图的叶子；一旦它依赖别人，两条调用链会开始互相牵扯。"""
    assert _package_imports("stats") == set()


def test_the_parser_would_notice_a_regression():
    """反向验证这个检查本身有效：analyze 确实是依赖 metrics 的。

    没有这一条，``_package_imports`` 返回空集合（比如 AST 解析写错、相对 import
    的 ``level`` 判断写错）也会让上面所有断言通过，而检查实际上什么都没做。
    """
    assert "metrics" in _package_imports("analyze")
