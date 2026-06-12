# FM Demodulation Algorithm

本文档说明 `example/fm_broadcast_player.cpp` 中当前使用的 FM 广播解调算法。该示例实现的是宽带 FM（WBFM）解调：从 RTL-SDR 读取 unsigned 8-bit 交织 IQ 数据，解调得到 48 kHz、16-bit PCM 音频。默认 `--stereo on` 输出双声道立体声；使用 `--stereo off` 输出兼容单声道 `L+R`。默认 `--audio off` 时，PCM 写入标准输出；使用 `--audio on` 时，PCM 通过 ALSA 写入系统默认播放设备。

## 1. 信号链路

当前实现的处理链路为：

```text
RTL-SDR unsigned 8-bit IQ
  -> offset tuning 数字搬移
  -> IQ 归一化
  -> 120 kHz 复基带信道低通 FIR
  -> 相位差 FM 鉴频
  -> 100 kHz 复合基带低通 FIR
  -> 5 倍降采样到 240 kHz
  -> 15 kHz 音频低通 FIR
  -> 5 倍降采样到 48 kHz
  -> DC blocker
  -> 去加重 de-emphasis
  -> 音量缩放与限幅
  -> signed 16-bit PCM
```

示例中的主要固定参数：

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| SDR 采样率 | `1200000` Hz | RTL-SDR 复数 IQ 采样率 |
| 音频采样率 | `48000` Hz | 输出 PCM 采样率 |
| 复合基带采样率 | `240000` Hz | 鉴频后第一级降采样输出 |
| 总降采样因子 | `25` | `5 * 5` |
| 默认调谐偏移 | `250000` Hz | 让目标电台避开 RTL-SDR 中心 DC spur |
| RF 信道低通截止频率 | `120000` Hz | 鉴频前保留 WBFM 信道 |
| 复合基带低通截止频率 | `100000` Hz | 鉴频后第一级抗混叠滤波 |
| 音频低通截止频率 | `15000` Hz | 保留 FM 广播单声道音频主信号 |
| 立体声导频 | `19000` Hz | 用于恢复 38 kHz 副载波 |
| 立体声副载波 | `38000` Hz | 解调 `L-R` 差信号 |
| RF FIR tap 数 | `97` | 鉴频前复基带信道滤波器长度 |
| 复合基带 FIR tap 数 | `97` | 第一级降采样抗混叠滤波器长度 |
| 音频 FIR tap 数 | `161` | 第二级音频低通滤波器长度 |
| FM 最大频偏 | `75000` Hz | 广播 WBFM 常用最大频偏 |
| 默认去加重时间常数 | `50` us | 可用 `--deemphasis` 修改 |

## 2. Offset Tuning

RTL-SDR 是零中频接收机，中心频率附近常有 DC spur 和 I/Q 直流偏置。直接把 FM 电台调到中心频率时，DC spur 会落在 FM 载波附近，容易导致鉴频后出现强噪声或失真。

当前程序默认启用 offset tuning：

$$
F_\text{tuner} = F_\text{station} + F_\text{offset}
$$

其中：

$$
F_\text{offset} = 250{,}000
$$

这样目标电台会出现在复基带的：

$$
-F_\text{offset}
$$

随后程序用数字混频把它搬回 0 Hz：

$$
x_\text{mixed}[n] =
x_\text{raw}[n] \cdot e^{j 2\pi F_\text{offset} n / F_s}
$$

后续 FM 鉴频使用的是 $x_\text{mixed}[n]$。

可以通过命令行关闭 offset tuning：

```bash
build/fm_broadcast_player --freq 100.0 --offset off --audio on
```

但 RTL-SDR 接收 WBFM 时通常建议保持默认的 `--offset on`。

## 3. IQ 数据归一化

RTL-SDR 输出的原始 IQ 数据是 unsigned 8-bit，排列为：

$$
\{I_0, Q_0, I_1, Q_1, \ldots, I_n, Q_n\}
$$

每个分量的取值范围为：

$$
I_n, Q_n \in [0, 255]
$$

代码中将其中心化并归一化到近似区间 $[-1, 1]$：

$$
i[n] = \frac{I_n - 127.5}{127.5}
$$

$$
q[n] = \frac{Q_n - 127.5}{127.5}
$$

于是得到复基带信号：

$$
x[n] = i[n] + j q[n]
$$

其中 $j$ 是虚数单位。

## 4. FM 相位差鉴频

FM 信号的信息体现在瞬时频率变化上，而瞬时频率可以由复信号相位的一阶差分得到。

复基带信号可以写成：

$$
x[n] = A[n] e^{j\phi[n]}
$$

其中 $\phi[n]$ 是瞬时相位。相邻采样点之间的相位差为：

$$
\Delta \phi[n] = \phi[n] - \phi[n-1]
$$

代码中没有直接对每个采样点调用 `atan2(q, i)` 再相减，而是先计算：

$$
z[n] = x[n] \cdot x^*[n-1]
$$

其中 $x^*[n-1]$ 是上一采样点的共轭。展开可得：

$$
z[n] =
(i[n] + j q[n]) (i[n-1] - j q[n-1])
$$

其实部和虚部分别为：

$$
\operatorname{Re}\{z[n]\} =
i[n]i[n-1] + q[n]q[n-1]
$$

$$
\operatorname{Im}\{z[n]\} =
q[n]i[n-1] - i[n]q[n-1]
$$

相位差通过 `atan2` 计算：

$$
\Delta \phi[n] =
\operatorname{atan2}
\left(
\operatorname{Im}\{z[n]\},
\operatorname{Re}\{z[n]\}
\right)
$$

代入代码中的变量，对应为：

$$
\Delta \phi[n] =
\operatorname{atan2}
\left(
q[n]i[n-1] - i[n]q[n-1],
i[n]i[n-1] + q[n]q[n-1]
\right)
$$

相位差和瞬时频率偏移之间的关系为：

$$
\Delta f[n] =
\frac{F_s}{2\pi} \Delta \phi[n]
$$

其中 $F_s$ 是 SDR IQ 采样率。为了把 FM 鉴频输出归一化为大致的音频幅度，代码除以最大频偏 $\Delta f_\text{max}$：

$$
d[n] =
\frac{\Delta f[n]}{\Delta f_\text{max}}
=
\frac{F_s}{2\pi \Delta f_\text{max}} \Delta \phi[n]
$$

当前代码中：

$$
F_s = 1{,}200{,}000
$$

$$
\Delta f_\text{max} = 75{,}000
$$

因此鉴频输出为：

$$
d[n] =
\frac{1{,}200{,}000}{2\pi \cdot 75{,}000}
\Delta \phi[n]
$$

## 5. FIR 低通滤波

FM 广播的复合基带中包含单声道 `L+R` 音频、19 kHz 立体声导频、23-53 kHz 的 `L-R` 双边带抑制载波信号和其他附加业务。当前示例默认解码立体声；关闭立体声时，只保留约 15 kHz 以下的 `L+R` 主信道作为单声道输出。

低通 FIR 滤波器使用 windowed-sinc 方式生成。理想低通的离散冲激响应为：

$$
h_\text{ideal}[m] =
\begin{cases}
2 f_c, & m = 0 \\
\frac{\sin(2\pi f_c m)}{\pi m}, & m \ne 0
\end{cases}
$$

其中归一化截止频率为：

$$
f_c = \frac{F_c}{F_s}
$$

当前：

$$
F_c = 15{,}000
$$

$$
F_s = 1{,}200{,}000
$$

所以：

$$
f_c = \frac{15{,}000}{1{,}200{,}000}
$$

代码使用 Hamming 窗：

$$
w[n] = 0.54 - 0.46 \cos\left(\frac{2\pi n}{N-1}\right)
$$

其中 $N = 161$ 是 FIR tap 数。最终 FIR 系数为：

$$
h[n] = h_\text{ideal}[n - M] \cdot w[n]
$$

其中：

$$
M = \frac{N - 1}{2}
$$

为了使直流增益为 1，代码对 FIR 系数做归一化：

$$
h_\text{norm}[n] =
\frac{h[n]}{\sum_{k=0}^{N-1} h[k]}
$$

滤波输出为卷积：

$$
y[n] =
\sum_{k=0}^{N-1}
h_\text{norm}[k] d[n-k]
$$

## 6. 降采样

SDR 输入采样率为 1.2 MHz，而音频输出采样率为 48 kHz，因此总降采样因子为：

$$
R = \frac{1{,}200{,}000}{48{,}000} = 25
$$

当前代码不再一次性抽取 25 倍，而是分成两级：

$$
R_1 = 5,\quad F_\text{composite} = \frac{1{,}200{,}000}{5} = 240{,}000
$$

$$
R_2 = 5,\quad F_a = \frac{240{,}000}{5} = 48{,}000
$$

第一级在鉴频后用 100 kHz 低通抑制混叠，再从 1.2 MHz 降到 240 kHz。第二级用 15 kHz 音频低通，再从 240 kHz 降到 48 kHz。

代码每处理 25 个 SDR 采样输出 1 个音频采样：

$$
y_a[m] = y[mR]
$$

其中 $y_a[m]$ 是 48 kHz 音频采样序列。

注意：当前实现把低通 FIR 放在 1.2 MHz 采样率下执行，然后再抽取。因为低通截止频率为 15 kHz，低于输出音频 Nyquist 频率：

$$
\frac{48{,}000}{2} = 24{,}000 \text{ Hz}
$$

所以可以抑制主要混叠。

## 7. DC Blocker

FM 鉴频和硬件偏置可能引入直流分量，听感上表现为低频偏置或削波风险。当前代码使用一阶 DC blocker：

$$
v[m] = y_a[m] - y_a[m-1] + r v[m-1]
$$

其中：

$$
r = 0.995
$$

$r$ 越接近 1，低频截止越低，直流消除越慢。

## 8. 去加重 De-emphasis

FM 广播发射端通常会进行预加重，提高高频音频能量；接收端需要做去加重恢复频响并降低高频噪声。

当前代码使用一阶低通形式的去加重：

$$
s[m] = s[m-1] + \alpha (v[m] - s[m-1])
$$

其中：

$$
\alpha =
\frac{1}{1 + \tau F_a}
$$

$F_a$ 是音频采样率，$\tau$ 是去加重时间常数。代码默认：

$$
F_a = 48{,}000
$$

$$
\tau = 50 \times 10^{-6}
$$

因此：

$$
\alpha =
\frac{1}{1 + 50 \times 10^{-6} \cdot 48{,}000}
$$

用户可以通过命令行参数选择不同去加重时间，例如：

```bash
build/fm_broadcast_player --freq 100.0 --deemphasis 75 --audio on
```

## 9. 音量缩放、限幅与 PCM 输出

去加重后的音频样本乘以音量系数：

$$
a[m] = G \cdot s[m]
$$

其中 $G$ 是 `--volume` 参数，默认值为：

$$
G = 0.8
$$

然后限幅到 $[-1, 1]$：

$$
c[m] =
\min(1, \max(-1, a[m]))
$$

最后转换为 signed 16-bit little-endian PCM：

$$
p[m] = \operatorname{round}(32767 \cdot c[m])
$$

其中：

$$
p[m] \in [-32767, 32767]
$$

默认情况下，程序把 PCM 数据写入标准输出：

```text
stdout: signed 16-bit little-endian, stereo interleaved, 48000 Hz
```

因此可以使用：

```bash
build/fm_broadcast_player --freq 100.0 | aplay -r 48000 -f S16_LE -c 2
```

单声道输出需要关闭立体声：

```bash
build/fm_broadcast_player --freq 100.0 --stereo off | aplay -r 48000 -f S16_LE -c 1
```

如果使用内置播放功能：

```bash
build/fm_broadcast_player --freq 100.0 --audio on
```

程序会打开 ALSA 的 `"default"` 播放设备，并写入同样格式的 PCM：

```text
ALSA default device: signed 16-bit little-endian, stereo interleaved, 48000 Hz
```

## 10. 符号含义对照表

| 符号 | 含义 | 当前代码中的对应含义 |
| --- | --- | --- |
| $I_n$ | 第 $n$ 个原始 I 分量 | RTL-SDR unsigned 8-bit I byte |
| $Q_n$ | 第 $n$ 个原始 Q 分量 | RTL-SDR unsigned 8-bit Q byte |
| $i[n]$ | 归一化后的 I 分量 | `(I - 127.5) / 127.5` |
| $q[n]$ | 归一化后的 Q 分量 | `(Q - 127.5) / 127.5` |
| $x[n]$ | 复基带 IQ 采样 | $i[n] + j q[n]$ |
| $F_\text{station}$ | 目标 FM 电台频率 | `--freq` 参数 |
| $F_\text{tuner}$ | RTL-SDR 实际调谐频率 | `--offset on` 时为 $F_\text{station}+250000$ |
| $F_\text{offset}$ | offset tuning 偏移频率 | `kTuningOffsetHz = 250000` |
| $x^*[n]$ | $x[n]$ 的复共轭 | 上一 IQ 采样的共轭 |
| $A[n]$ | 瞬时幅度 | FM 解调中不直接使用 |
| $\phi[n]$ | 瞬时相位 | 由 IQ 采样隐含表示 |
| $\Delta \phi[n]$ | 相邻采样点相位差 | `atan2(imag, real)` |
| $z[n]$ | 相邻 IQ 采样共轭乘积 | $x[n]x^*[n-1]$ |
| $\operatorname{Re}\{z[n]\}$ | $z[n]$ 的实部 | `i * prevI + q * prevQ` |
| $\operatorname{Im}\{z[n]\}$ | $z[n]$ 的虚部 | `q * prevI - i * prevQ` |
| $F_s$ | SDR IQ 采样率 | `kSdrSampleRate = 1200000` |
| $F_a$ | 音频输出采样率 | `kAudioSampleRate = 48000` |
| $\Delta f[n]$ | 瞬时频率偏移 | 由相位差换算得到 |
| $\Delta f_\text{max}$ | FM 最大频偏 | `kFmDeviationHz = 75000` |
| $d[n]$ | 归一化鉴频输出 | discriminator output |
| $F_c$ | 音频低通截止频率 | `kAudioCutoffHz = 15000` |
| $f_c$ | 归一化低通截止频率 | $F_c / F_s$ |
| $N$ | FIR tap 数 | `kFirTaps = 161` |
| $M$ | FIR 中心索引 | $(N - 1) / 2$ |
| $w[n]$ | Hamming 窗函数 | FIR 设计中的窗口 |
| $h[n]$ | FIR 滤波器系数 | windowed-sinc 系数 |
| $h_\text{norm}[n]$ | 归一化 FIR 系数 | DC 增益归一化后的系数 |
| $y[n]$ | FIR 低通输出 | 降采样前音频基带 |
| $R$ | 降采样因子 | `kDecimation = 25` |
| $y_a[m]$ | 降采样后的音频样本 | 48 kHz audio sample |
| $r$ | DC blocker 反馈系数 | `0.995` |
| $v[m]$ | DC blocker 输出 | 去直流后的音频 |
| $\tau$ | 去加重时间常数 | `--deemphasis`，默认 50 us |
| $\alpha$ | 去加重 IIR 系数 | `1 / (1 + tau * 48000)` |
| $s[m]$ | 去加重后的音频 | de-emphasis output |
| $G$ | 音量增益 | `--volume`，默认 0.8 |
| $a[m]$ | 音量缩放后的音频 | volume-scaled sample |
| $c[m]$ | 限幅后的音频 | clamped sample in $[-1,1]$ |
| $p[m]$ | signed 16-bit PCM 样本 | 写入 stdout 或 ALSA default device 的音频样本 |

## 11. 当前实现的边界

当前示例用于说明和收听普通 FM 广播，重点是算法清晰和依赖简单。它已经实现 `L+R` 单声道兼容解调和基于 19 kHz 导频的立体声 `L-R` 恢复，但没有实现 RDS/RBDS、自动频偏估计或自动增益优化。

## 12. Debug IQ 采集与离线分析

如果实时解调听到的是噪声，可以开启 debug 模式：

```bash
build/fm_broadcast_player --freq 97.7 --gain 30 --audio on --debug on
```

程序会在当前目录保存原始 RTL-SDR unsigned 8-bit IQ 文件，文件名格式为：

```text
YYYYMMDDHHMMSS_FM.iq
```

同时日志会每秒输出 RF、鉴频、复合基带、音频和 PCM 的 RMS/peak 指标。保存的 IQ 可以用 Python 脚本离线分析：

```bash
python3 python/analyze_fm_iq.py 20260610143000_FM.iq --sample-rate 1200000 --station-offset -250000 --plot fm_spectrum.png
```

默认 `--offset on` 时，RTL-SDR 实际调谐到目标电台上方 250 kHz，因此目标电台理论上位于捕获 IQ 的 `-250 kHz` 附近。
