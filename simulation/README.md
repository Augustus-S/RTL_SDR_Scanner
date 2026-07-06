# Signal Simulation

该目录按调制体制归类仿真代码：

| 目录 | 内容 |
| --- | --- |
| `fm/` | WBFM 立体声广播调制、IQ 保存、非相干解调和立体声恢复 |
| `am/` | 常规 AM 调制、IQ 保存、包络解调和音频恢复 |

运行示例：

```bash
python3 simulation/fm/fm_modulation.py
python3 simulation/fm/fm_demodulation.py
python3 simulation/am/am_modulation.py
python3 simulation/am/am_demodulation.py
```

使用 Pixi 环境时：

```bash
pixi run sim-fm-mod
pixi run sim-fm-demod
pixi run sim-am-mod
pixi run sim-am-demod
```

每个子目录都有独立的 `figures/` 和对应的原理说明文档。
