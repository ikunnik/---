# 离线女声语音包

这些 WAV 是**预生成的自然女声**，供 `workers/voice.py` 在运行时直接读取：

- 音色：`zh-CN-XiaoxiaoNeural`（Microsoft Edge TTS 自然女声）
- 格式：24 kHz / 单声道 / 16 bit PCM
- 运行时不需要网络、edge-tts、ffmpeg、sox
- 如果某个文件缺失，只报错静音，**不会回退机器声**

## 文件对应关系

| 事件 | 文件 |
|---|---|
| 左/右小于 60° 绕行（当前 45°） | `bypass_left.wav` / `bypass_right.wav` |
| 左/右大于等于 60° 绕行 | `bypass_left_front.wav` / `bypass_right_front.wav` |
| 绕行完成 | `bypass_done.wav` |
| 动态障碍 | `dynamic.wav` |
| 开始盲道 | `lane.wav`（女声“已识别前方盲道，请跟随我。”） |
| 盲道停车呼救 | `help.wav`（女声“快来人，帮帮我，快来人，帮帮我”） |
| 红灯/黄灯/绿灯 | `traffic_red.wav` / `traffic_yellow.wav` / `traffic_green.wav` |
| 具体停止原因 | `blocked_<reason>.wav` |
| 等待原因 | `wait_<reason>.wav` |

## 重新生成

生成脚本只在开发阶段使用，需要联网安装依赖：

```bash
distrobox enter roverx1-dev
pip install --user edge-tts miniaudio
cd /home/admin/rover_x1_v7.5
python3 tools/voice_generate.py --force
```

默认语速为 `+0%`；可以加 `--rate +10%` 让播报更短。  
生成后请重新运行 `python3 tools/selftest.py`，确认全部短句都存在（当前 38 条）。

## 导盲提示用语

面向使用者的语音采用步行用语：停止时说“已停下”，等待时说“停步等待”，紧急停止时说“已紧急停止”。需要人工处理的锁存故障提示“已停下，请寻求帮助”。修改 `workers/voice.py` 中的台词后，必须同步重新生成对应 WAV，运行时不会自动合成新台词。
